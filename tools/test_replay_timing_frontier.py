#!/usr/bin/env python3
"""Dependency-free regression test for replay_timing_frontier.py."""
from __future__ import annotations

import math
import tempfile
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import replay_timing_frontier as replay  # noqa: E402

SYNTHETIC_LOG = """
[GlobalPlanCost] hypothesis=0 eventLead=4.000000s presentationTime=14.000000s candidate=A route=direct valid=true motionJ=0.500000000 scheduleWait=0.0s searchToCompletion=4.0s globalJ=0.600000000
[GlobalPlanCost] hypothesis=9 eventLead=6.000000s presentationTime=16.000000s candidate=INVALID route=direct valid=false motionJ=9.000000000 scheduleWait=0.0s searchToCompletion=6.0s globalJ=9.000000000
[GlobalPlanCost] hypothesis=1 eventLead=5.000000s presentationTime=15.000000s candidate=B route=ring valid=true motionJ=0.600000000 scheduleWait=0.0s searchToCompletion=5.0s globalJ=0.700000000
[GlobalPlanTimingAdmissibility] hypothesis=0 candidate=A route=direct costValid=true globalJ=0.600000000 eventPresentationTime=14.000000s now=13.000000s remaining=1.000000s minimumSafeCommitLead=0.500000s predictedPresentationDuration=0.900000s minimumReachEntryLead=0.200000s eventWindowAdmissible=true timingAdmissible=false
[GlobalPlanTimingAdmissibility] hypothesis=1 candidate=B route=ring costValid=true globalJ=0.700000000 eventPresentationTime=15.000000s now=13.000000s remaining=2.000000s minimumSafeCommitLead=0.500000s predictedPresentationDuration=1.300000s minimumReachEntryLead=0.200000s eventWindowAdmissible=true timingAdmissible=true
[GlobalTimePlanSelection] success=true scheduleComplete=true evaluatedHypotheses=2 configuredHypotheses=2 completePlans=2 costValidPlans=2 timingAdmissiblePlans=1 hypothesis=1 eventLead=5.000000s presentationTime=15.000000s candidate=B route=ring motionJ=0.600000000 scheduleWait=0.0s selectedGlobalJ=0.700000000 minimumGlobalJ=0.700000000 selectedWithinMinimumTolerance=true tieTolerance=0.000000001
"""


def _expect_error(fragment: str, text: str) -> None:
    """The replay must reject `text`, and say why."""
    try:
        replay.parse_records(text)
    except ValueError as exc:
        assert fragment in str(exc), f"expected {fragment!r} in {exc!r}"
        return
    raise AssertionError(f"expected ValueError containing {fragment!r}")


def _cost_line(hypothesis: int, lead: float, presentation: float,
               candidate: str, route: str, valid: str, global_j: float) -> str:
    return (
        f"[GlobalPlanCost] hypothesis={hypothesis} eventLead={lead:.6f}s "
        f"presentationTime={presentation:.6f}s candidate={candidate} route={route} "
        f"valid={valid} motionJ=0.500000000 scheduleWait=0.0s "
        f"searchToCompletion={lead:.1f}s globalJ={global_j:.9f}"
    )


def _timing_line(hypothesis: int, candidate: str, route: str, presentation: float,
                 now: float, remaining: float, global_j: float,
                 mscl: float = 0.500000, ppd: float = 0.900000,
                 mrel: float = 0.200000, window: str = "true",
                 admissible: str = "false", cost_valid: str = "true") -> str:
    return (
        f"[GlobalPlanTimingAdmissibility] hypothesis={hypothesis} candidate={candidate} "
        f"route={route} costValid={cost_valid} globalJ={global_j:.9f} "
        f"eventPresentationTime={presentation:.6f}s now={now:.6f}s "
        f"remaining={remaining:.6f}s minimumSafeCommitLead={mscl:.6f}s "
        f"predictedPresentationDuration={ppd:.6f}s "
        f"minimumReachEntryLead={mrel:.6f}s "
        f"eventWindowAdmissible={window} timingAdmissible={admissible}"
    )


def check_fail_closed(records, tie_tolerance) -> None:
    """Beyond every analytical boundary nothing remains admissible."""
    frontier = max(record.fail_closed_breakpoint for record in records)
    winner, minimum, count = replay.evaluate(records, frontier + 1.0, tie_tolerance)
    assert winner == "FAIL_CLOSED"
    assert minimum is None
    assert count == 0


def check_unresolved_tie() -> None:
    """Two admissible plans inside the tie tolerance must not be guessed."""
    text = "\n".join(
        [
            _cost_line(0, 4.0, 14.0, "A", "direct", "true", 0.600000000),
            _cost_line(1, 4.0, 14.0, "B", "ring", "true", 0.600000000),
            _timing_line(0, "A", "direct", 14.0, 13.0, 1.0, 0.600000000,
                         ppd=0.100000, admissible="true"),
            _timing_line(1, "B", "ring", 14.0, 13.0, 1.0, 0.600000000,
                         ppd=0.100000, admissible="true"),
        ]
    )
    records, tie_tolerance = replay.parse_records(text)
    assert tie_tolerance == 0.0
    replay.validate_logged_state(records)
    winner, minimum, count = replay.evaluate(records, 3.0, tie_tolerance)
    assert winner == "UNRESOLVED_TIE_SET"
    assert math.isclose(minimum, 0.6)
    assert count == 2


def check_negative_planner_time(records, tie_tolerance) -> None:
    try:
        replay.evaluate(records, -0.001, tie_tolerance)
    except ValueError as exc:
        assert "non-negative" in str(exc)
        return
    raise AssertionError("expected ValueError for negative planner time")


def check_parse_guards() -> None:
    lines = SYNTHETIC_LOG.strip().splitlines()
    cost_a, cost_invalid, cost_b, timing_a, timing_b, selection = lines

    _expect_error("no [GlobalPlanCost] records found", "\n".join([timing_a, timing_b]))
    _expect_error(
        "no [GlobalPlanTimingAdmissibility] records found",
        "\n".join([cost_a, cost_invalid, cost_b, selection]),
    )
    _expect_error("duplicate GlobalPlanCost record",
                  "\n".join([cost_a, cost_a, cost_b, timing_a, timing_b]))
    _expect_error("duplicate GlobalPlanTimingAdmissibility record",
                  "\n".join([cost_a, cost_b, timing_a, timing_a, timing_b]))
    # timing record whose (hypothesis, candidate, route) key has no cost record
    orphan = _timing_line(5, "Z", "direct", 14.0, 13.0, 1.0, 0.600000000)
    _expect_error("no matching GlobalPlanCost record",
                  "\n".join([cost_a, cost_b, timing_a, timing_b, orphan]))
    # cost-valid pooled set not fully covered by timing records
    _expect_error("timing-record coverage does not match",
                  "\n".join([cost_a, cost_b, timing_a]))
    # presentation time disagreement between the two record families
    _expect_error(
        "presentation time mismatch",
        "\n".join([cost_a, cost_b,
                    _timing_line(0, "A", "direct", 14.5, 13.0, 1.0, 0.600000000),
                    timing_b]),
    )
    # globalJ disagreement between the two record families
    _expect_error(
        "global J mismatch",
        "\n".join([cost_a, cost_b,
                    _timing_line(0, "A", "direct", 14.0, 13.0, 1.0, 0.640000000),
                    timing_b]),
    )
    # costValid flag disagreement between the two record families
    _expect_error(
        "cost-valid flag mismatch",
        "\n".join([cost_a, cost_b,
                    _timing_line(0, "A", "direct", 14.0, 13.0, 1.0, 0.600000000,
                                 cost_valid="false"),
                    timing_b]),
    )


def _expect_state_error(fragment: str, text: str) -> None:
    records, _ = replay.parse_records(text)
    try:
        replay.validate_logged_state(records)
    except ValueError as exc:
        assert fragment in str(exc), f"expected {fragment!r} in {exc!r}"
        return
    raise AssertionError(f"expected ValueError containing {fragment!r}")


def check_state_guards() -> None:
    lines = SYNTHETIC_LOG.strip().splitlines()
    cost_a, _, cost_b, timing_a, timing_b, _ = lines

    # records disagreeing about the common search epoch (presentation - lead)
    _expect_state_error(
        "one common search epoch",
        "\n".join([cost_a,
                    _cost_line(1, 4.000000, 15.000000, "B", "ring", "true", 0.700000000),
                    timing_a,
                    _timing_line(1, "B", "ring", 15.0, 13.0, 2.0, 0.700000000,
                                 ppd=1.300000, admissible="true")]),
    )
    # records disagreeing about the final selector time
    _expect_state_error(
        "one final selection time",
        "\n".join([cost_a, cost_b, timing_a,
                    _timing_line(1, "B", "ring", 15.0, 13.5, 2.0, 0.700000000,
                                 ppd=1.300000, admissible="true")]),
    )
    # logged remaining inconsistent with eventLead - logged planner duration
    _expect_state_error(
        "do not match",
        "\n".join([cost_a, cost_b, timing_a,
                    _timing_line(1, "B", "ring", 15.0, 13.0, 1.5, 0.700000000,
                                 ppd=1.300000, admissible="true")]),
    )
    # logged admissibility flag that the exact rule does not reproduce
    _expect_state_error(
        "do not replay exactly",
        "\n".join([cost_a, cost_b, timing_a,
                    _timing_line(1, "B", "ring", 15.0, 13.0, 2.0, 0.700000000,
                                 ppd=1.300000, admissible="false")]),
    )


def check_commit_lead_boundary() -> None:
    """`remaining` exactly equal to minimumSafeCommitLead must stay admissible.

    The selector's event-window test is non-strict (`remaining + 1e-12 >=
    minimumSafeCommitLead`). Without a record sitting exactly on that boundary,
    a regression to a strict comparison would go unnoticed.
    """
    text = "\n".join(
        [
            _cost_line(2, 4.0, 14.0, "C", "direct", "true", 0.650000000),
            _timing_line(2, "C", "direct", 14.0, 13.0, 1.0, 0.650000000,
                         mscl=1.000000, ppd=0.700000, mrel=0.200000,
                         window="true", admissible="true"),
        ]
    )
    records, tie_tolerance = replay.parse_records(text)
    replay.validate_logged_state(records)
    record = records[0]
    assert math.isclose(
        record.event_lead - 3.0, record.minimum_safe_commit_lead, abs_tol=1e-12
    )
    assert replay.timing_admissible(record, 3.0)
    winner, _, count = replay.evaluate(records, 3.0, tie_tolerance)
    assert winner == "h2:C:direct"
    assert count == 1


def check_main_failure_path() -> None:
    """A malformed log must fail loudly with a non-zero exit status."""
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "broken.log"
        path.write_text("nothing useful here\n")
        assert replay.main([str(path)]) == 1
        missing = Path(directory) / "absent.log"
        assert replay.main([str(missing)]) == 1


def main() -> int:
    records, tie_tolerance = replay.parse_records(SYNTHETIC_LOG)
    start, logged_tp = replay.validate_logged_state(records)

    assert len(records) == 2
    assert math.isclose(start, 10.0, abs_tol=1e-12)
    assert math.isclose(logged_tp, 3.0, abs_tol=1e-12)
    assert math.isclose(tie_tolerance, 1e-9, abs_tol=1e-15)
    assert [record.logged_timing_admissible for record in records] == [False, True]

    frontier = max(record.fail_closed_breakpoint for record in records)
    assert math.isclose(frontier, 3.5 + 1e-12, abs_tol=1e-12)

    winner, minimum, count = replay.evaluate(records, 2.0, tie_tolerance)
    assert winner == "h0:A:direct"
    assert math.isclose(minimum, 0.6)
    assert count == 2

    winner, minimum, count = replay.evaluate(records, 3.0, tie_tolerance)
    assert winner == "h1:B:ring"
    assert math.isclose(minimum, 0.7)
    assert count == 1

    assert replay.timing_admissible(records[1], frontier)
    assert not replay.timing_admissible(records[1], frontier + 1e-6)

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "run.log"
        path.write_text(SYNTHETIC_LOG)
        assert replay.main([str(path), "--planner-time", "3.0", "--planner-time", "3.6"]) == 0

    check_fail_closed(records, tie_tolerance)
    check_unresolved_tie()
    check_negative_planner_time(records, tie_tolerance)
    check_parse_guards()
    check_state_guards()
    check_commit_lead_boundary()
    check_main_failure_path()

    print("timing-frontier replay tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
