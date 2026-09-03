#!/usr/bin/env python3
"""Replay TRIAD's final timing-admission rule from one complete run log.

The tool uses values logged by ``[GlobalPlanCost]`` and
``[GlobalPlanTimingAdmissibility]``. It does not modify the controller or
rerun planning.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re
import sys

NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

GLOBAL_COST_RE = re.compile(
    rf"\[GlobalPlanCost\] hypothesis=(\d+) eventLead=({NUMBER})s "
    rf"presentationTime=({NUMBER})s candidate=(\S+) route=(\S+) "
    rf"valid=(true|false) motionJ=({NUMBER}) scheduleWait=({NUMBER})s "
    rf"searchToCompletion=({NUMBER})s globalJ=({NUMBER})"
)
TIMING_RE = re.compile(
    rf"\[GlobalPlanTimingAdmissibility\] hypothesis=(\d+) candidate=(\S+) "
    rf"route=(\S+) costValid=(true|false) globalJ=({NUMBER}) "
    rf"eventPresentationTime=({NUMBER})s now=({NUMBER})s "
    rf"remaining=({NUMBER})s minimumSafeCommitLead=({NUMBER})s "
    rf"predictedPresentationDuration=({NUMBER})s "
    rf"minimumReachEntryLead=({NUMBER})s "
    rf"eventWindowAdmissible=(true|false) timingAdmissible=(true|false)"
)
SELECTION_RE = re.compile(
    rf"\[GlobalTimePlanSelection\] success=true .*?tieTolerance=({NUMBER})"
)


@dataclass(frozen=True)
class Record:
    hypothesis: int
    candidate: str
    route: str
    event_lead: float
    event_presentation_time: float
    final_now: float
    logged_remaining: float
    minimum_safe_commit_lead: float
    predicted_presentation_duration: float
    minimum_reach_entry_lead: float
    global_j: float
    logged_cost_valid: bool
    logged_timing_admissible: bool

    @property
    def key(self) -> tuple[int, str, str]:
        return (self.hypothesis, self.candidate, self.route)

    @property
    def plan_start(self) -> float:
        return self.event_presentation_time - self.event_lead

    @property
    def fail_closed_breakpoint(self) -> float:
        required = max(
            self.minimum_safe_commit_lead,
            self.predicted_presentation_duration + self.minimum_reach_entry_lead,
        )
        return self.event_lead + 1e-12 - required


def _parse_costs(text: str) -> dict[tuple[int, str, str], dict[str, float | bool]]:
    out: dict[tuple[int, str, str], dict[str, float | bool]] = {}
    for match in GLOBAL_COST_RE.finditer(text):
        key = (int(match.group(1)), match.group(4), match.group(5))
        if key in out:
            raise ValueError(f"duplicate GlobalPlanCost record for {key}")
        out[key] = {
            "event_lead": float(match.group(2)),
            "presentation": float(match.group(3)),
            "valid": match.group(6) == "true",
            "global_j": float(match.group(10)),
        }
    return out


def parse_records(text: str) -> tuple[list[Record], float]:
    costs = _parse_costs(text)
    if not costs:
        raise ValueError("no [GlobalPlanCost] records found")

    records: list[Record] = []
    seen: set[tuple[int, str, str]] = set()
    for match in TIMING_RE.finditer(text):
        key = (int(match.group(1)), match.group(2), match.group(3))
        if key in seen:
            raise ValueError(
                f"duplicate GlobalPlanTimingAdmissibility record for {key}"
            )
        seen.add(key)
        if key not in costs:
            raise ValueError(
                f"timing record has no matching GlobalPlanCost record: {key}"
            )
        cost = costs[key]
        presentation = float(match.group(6))
        if not math.isclose(
            presentation, float(cost["presentation"]), abs_tol=1e-9
        ):
            raise ValueError(f"presentation time mismatch for {key}")
        if (match.group(4) == "true") != bool(cost["valid"]):
            raise ValueError(f"cost-valid flag mismatch for {key}")
        if not math.isclose(
            float(match.group(5)), float(cost["global_j"]), abs_tol=1e-9
        ):
            raise ValueError(f"global J mismatch for {key}")

        records.append(
            Record(
                hypothesis=key[0],
                candidate=key[1],
                route=key[2],
                event_lead=float(cost["event_lead"]),
                event_presentation_time=presentation,
                final_now=float(match.group(7)),
                logged_remaining=float(match.group(8)),
                minimum_safe_commit_lead=float(match.group(9)),
                predicted_presentation_duration=float(match.group(10)),
                minimum_reach_entry_lead=float(match.group(11)),
                global_j=float(match.group(5)),
                logged_cost_valid=match.group(4) == "true",
                logged_timing_admissible=match.group(13) == "true",
            )
        )

    if not records:
        raise ValueError(
            "no [GlobalPlanTimingAdmissibility] records found; "
            "this replay requires timing-diagnostic logging"
        )

    valid_cost_keys = {key for key, cost in costs.items() if bool(cost["valid"])}
    if valid_cost_keys != seen:
        missing = sorted(valid_cost_keys - seen)
        extra = sorted(seen - valid_cost_keys)
        raise ValueError(
            "timing-record coverage does not match the cost-valid pooled set "
            f"(missing={len(missing)}, extra={len(extra)})"
        )

    selection = SELECTION_RE.search(text)
    tie_tolerance = 0.0 if selection is None else max(
        0.0, float(selection.group(1))
    )
    return records, tie_tolerance


def timing_admissible(record: Record, planner_time: float) -> bool:
    if not record.logged_cost_valid:
        return False
    remaining = record.event_lead - planner_time
    event_window = remaining + 1e-12 >= record.minimum_safe_commit_lead
    return event_window and (
        record.predicted_presentation_duration + record.minimum_reach_entry_lead
        <= remaining + 1e-12
    )


def validate_logged_state(records: list[Record]) -> tuple[float, float]:
    starts = [record.plan_start for record in records]
    nows = [record.final_now for record in records]
    if max(starts) - min(starts) > 1e-9:
        raise ValueError("records do not share one common search epoch")
    if max(nows) - min(nows) > 1e-9:
        raise ValueError("records do not share one final selection time")

    plan_start = starts[0]
    logged_planner_time = nows[0] - plan_start
    remaining_mismatches = 0
    flag_mismatches = 0
    for record in records:
        expected_remaining = record.event_lead - logged_planner_time
        if not math.isclose(
            expected_remaining, record.logged_remaining, abs_tol=2e-6
        ):
            remaining_mismatches += 1
        if (
            timing_admissible(record, logged_planner_time)
            != record.logged_timing_admissible
        ):
            flag_mismatches += 1

    if remaining_mismatches:
        raise ValueError(
            f"{remaining_mismatches} logged remaining value(s) do not match "
            "eventLead - logged planner duration"
        )
    if flag_mismatches:
        raise ValueError(
            f"{flag_mismatches} timing-admissibility flag(s) do not replay exactly"
        )
    return plan_start, logged_planner_time


def evaluate(
    records: list[Record], planner_time: float, tie_tolerance: float
) -> tuple[str, float | None, int]:
    if planner_time < 0.0:
        raise ValueError("planner time must be non-negative")

    admissible = [
        record for record in records if timing_admissible(record, planner_time)
    ]
    if not admissible:
        return ("FAIL_CLOSED", None, 0)

    minimum = min(record.global_j for record in admissible)
    tie_set = [
        record
        for record in admissible
        if record.global_j <= minimum + tie_tolerance
    ]
    if len(tie_set) != 1:
        return ("UNRESOLVED_TIE_SET", minimum, len(admissible))

    record = tie_set[0]
    winner = f"h{record.hypothesis}:{record.candidate}:{record.route}"
    return (winner, minimum, len(admissible))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Replay TRIAD scenario-specific timing admission from a run log."
    )
    parser.add_argument("log", type=Path)
    parser.add_argument(
        "--planner-time",
        action="append",
        type=float,
        default=[],
        help="counterfactual external planner duration in seconds; may be repeated",
    )
    args = parser.parse_args(argv)

    try:
        text = args.log.read_text(encoding="utf-8", errors="replace")
        records, tie_tolerance = parse_records(text)
        plan_start, logged_planner_time = validate_logged_state(records)
    except (OSError, ValueError) as exc:
        print(f"timing frontier replay: FAIL: {exc}", file=sys.stderr)
        return 1

    breakpoints = [
        record.fail_closed_breakpoint
        for record in records
        if record.logged_cost_valid
    ]
    if not breakpoints:
        print("timing frontier replay: FAIL: no cost-valid records", file=sys.stderr)
        return 1

    frontier = max(breakpoints)
    logged_admissible = sum(
        timing_admissible(record, logged_planner_time) for record in records
    )

    print("timing frontier replay: PASS")
    print(f"records={len(records)}")
    print(f"plan_start={plan_start:.9f}")
    print(f"logged_planner_time={logged_planner_time:.9f}")
    print(f"logged_timing_admissible={logged_admissible}")
    print(f"tie_tolerance={tie_tolerance:.12g}")
    print(f"fail_closed_boundary={frontier:.12f}")
    print(
        "boundary_semantics=admissible_at_boundary_due_to_non_strict_inequality;"
        "fail_closed_for_Tp_greater_than_boundary"
    )

    for planner_time in args.planner_time:
        try:
            winner, minimum, count = evaluate(records, planner_time, tie_tolerance)
        except ValueError as exc:
            print(f"timing frontier replay: FAIL: {exc}", file=sys.stderr)
            return 1
        minimum_text = "NA" if minimum is None else f"{minimum:.9f}"
        print(
            f"Tp={planner_time:.9f} admissible={count} "
            f"winner={winner} minimumGlobalJ={minimum_text}"
        )
        if winner == "UNRESOLVED_TIE_SET":
            print(
                "  note=counterfactual minimum falls within the configured "
                "cost tie set; the log does not contain every secondary-order "
                "field needed to reproduce the deterministic tie break"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
