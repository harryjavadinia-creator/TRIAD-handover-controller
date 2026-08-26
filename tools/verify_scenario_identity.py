#!/usr/bin/env python3
"""Verify which named scenario a simulation log actually reproduces.

`check_global_time_plan_log.py` independently verifies selector/runtime
consistency (that the committed plan really is the argmin, etc.) but does
not check that a run used a particular initial object pose or velocity.
This script is a separate, narrower check: it confirms the log's own
recorded initial position and settled velocity match a named scenario's
configuration, and reports the completion class (completed vs. fail-safe).
It does not re-verify anything already checked by
check_global_time_plan_log.py, and does not import or modify it.

Tolerances below are not invented, and are not physical-accuracy claims.
POSITION_TOLERANCE is set from the log's own numeric display precision (the
controller prints p0 to 3 decimal places). VELOCITY_TOLERANCE is a
scenario-identity tolerance covering both the log's 4-decimal velocity
display rounding and the small settled-velocity estimation variation
actually observed across validated runs -- not merely half a display digit.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# Mirrors the scenario table in scripts/run_scenario.sh. Duplicated rather
# than imported so this checker has no dependency on the shell script.
SCENARIOS = {
    "near-ground": ((0.25, 0.62, 0.15), (0.0, -0.08, 0.0)),
    "longitudinal": ((0.92, 0.00, 0.55), (-0.08, 0.0, 0.0)),
    "lateral-low": ((0.55, -0.56, 0.15), (0.0, 0.08, 0.0)),
    "diagonal": ((0.90, 0.00, 0.30), (-0.0565685, 0.0, 0.0565685)),
}

P0_RE = re.compile(
    r"\[ObserveObject\] unified static/moving observation started .*?"
    r"p0=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
)
# [ObserveObject] t=.../... ... v=[vx,vy,vz] ... -- the per-sample observed
# velocity during the observation window. The *last* occurrence (the final
# sample) is the settled estimate. Deliberately not the moving-object-only
# "[PresentationSolve] ... v=[...]" line, which would be absent for any
# zero-velocity scenario (see tools/verify_latency_matrix_cell.py, which
# uses this same source for the same reason).
VELOCITY_RE = re.compile(
    r"\[ObserveObject\] t=[\d.]+/[\d.]+s .*?v=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
)
COMPLETED_RE = re.compile(r"\[Completed\] full plan-once handover completed")
FAILSAFE_RE = re.compile(r"\[GlobalTimePlanSelection\] success=false reason=(\S+)")

POSITION_TOLERANCE = 0.0005  # half the log's displayed 3-decimal precision
# A scenario-identity tolerance, not a physical-accuracy claim: covers both
# the log's 4-decimal velocity display rounding and the small settled-
# velocity estimation variation observed across validated runs (see
# tools/verify_latency_matrix_cell.py, which uses the same value for the
# same reason).
VELOCITY_TOLERANCE = 0.0005


def parse_triplet(match: "re.Match[str]") -> tuple[float, float, float]:
    return tuple(float(x) for x in match.groups())  # type: ignore[return-value]


def close(a: tuple[float, float, float], b: tuple[float, float, float], tol: float) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument(
        "--expect-scenario",
        choices=sorted(SCENARIOS),
        required=True,
        help="scenario name the log is expected to reproduce",
    )
    parser.add_argument(
        "--expect-failsafe",
        action="store_true",
        help="expect a fail-safe (no committed plan) outcome instead of completion",
    )
    args = parser.parse_args()
    text = args.log.read_text(encoding="utf-8", errors="replace")

    errors: list[str] = []
    expected_p0, expected_v = SCENARIOS[args.expect_scenario]

    p0_match = P0_RE.search(text)
    observed_p0 = parse_triplet(p0_match) if p0_match else None
    if observed_p0 is None:
        errors.append("no [ObserveObject] p0= line found -- cannot verify initial position")
        position_ok = False
    else:
        position_ok = close(observed_p0, expected_p0, POSITION_TOLERANCE)
        if not position_ok:
            errors.append(
                f"initial position {observed_p0} does not match scenario "
                f"'{args.expect_scenario}' expected {expected_p0}"
            )

    v_matches = list(VELOCITY_RE.finditer(text))
    observed_v = parse_triplet(v_matches[-1]) if v_matches else None
    if observed_v is None:
        errors.append("no [ObserveObject] v= line found -- cannot verify velocity")
        velocity_ok = False
    else:
        velocity_ok = close(observed_v, expected_v, VELOCITY_TOLERANCE)
        if not velocity_ok:
            errors.append(
                f"velocity {observed_v} does not match scenario "
                f"'{args.expect_scenario}' expected {expected_v}"
            )

    completed = COMPLETED_RE.search(text) is not None
    failsafe_match = FAILSAFE_RE.search(text)

    if args.expect_failsafe:
        if completed:
            errors.append("expected a fail-safe outcome but the log shows a completed handover")
        if not failsafe_match:
            errors.append("expected a fail-safe outcome but no fail-safe reason was found")
    else:
        if not completed:
            errors.append("expected a completed handover but [Completed] was not found")
        if failsafe_match:
            errors.append(
                f"unexpected fail-safe reason '{failsafe_match.group(1)}' in a run "
                "expected to complete"
            )

    scenario_identity_result = "PASS" if (not errors and position_ok and velocity_ok) else "FAIL"

    print(f"observed_position={list(observed_p0) if observed_p0 else None}")
    print(f"observed_velocity={list(observed_v) if observed_v else None}")
    print(f"position_ok={position_ok}")
    print(f"velocity_ok={velocity_ok}")
    for error in errors:
        print(f"FAIL: {error}", file=sys.stderr)
    print(f"scenario_identity_result={scenario_identity_result}")

    if scenario_identity_result != "PASS":
        return 1
    print(f"scenario identity verified: {args.expect_scenario}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
