#!/usr/bin/env python3
"""Experiment-level reproduction verification for one cell of the July-19
perception-latency matrix (docs/experiments.md, Dataset A).

This is deliberately separate from tools/check_latency_log.py's raw
controller metrics. An expected FAILURE is a successful reproduction of
that matrix cell: REPRODUCTION_RESULT is PASS when the observed
completion/failure class matches the historically expected class for this
scenario/condition AND the requested scenario/condition identity is
confirmed from the log -- not when the raw controller metrics happen to
pass. check_latency_log.py's own process exit status is not used as
reproduction evidence; its "pass" field is read directly and reported
separately, as LATENCY_METRICS_PASS.

Scenario identity requires BOTH the initial object position (p0, from
[ObserveObject]'s "unified static/moving observation started" line) and the
object's settled linear velocity (v, from the last [ObserveObject] t=.../...
sample line) to match -- position alone cannot distinguish two scenarios
that happen to start from a similar point but move differently. The last
[ObserveObject] sample (rather than the "[PresentationSolve] robot
stationary, object approaching..." line) is used because that
PresentationSolve line is only emitted for a moving object and would leave
static_nominal (zero object velocity) unverifiable. Condition identity
requires BOTH the observed latency mode AND the configured delay to match
(0.220s for the two delayed conditions; 0.000s for ideal, since perception-
latency simulation is disabled there) -- mode alone does not catch a log run
with the right mode but a different configured delay value.

A log containing both a [Completed] and a Failure-state terminal marker is
rejected as ambiguous rather than resolved either way.

Expected outcomes, scenario positions, and scenario velocities are
recovered exactly from the 15 historical per-run metadata snapshots
(docs/experiments.md); not inferred.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

EXPECTED_OUTCOME = {
    ("static_nominal", "ideal"): "COMPLETED",
    ("static_nominal", "compensated220"): "COMPLETED",
    ("static_nominal", "uncompensated220"): "COMPLETED",
    ("canonical_yz", "ideal"): "COMPLETED",
    ("canonical_yz", "compensated220"): "COMPLETED",
    ("canonical_yz", "uncompensated220"): "FAILURE",
    ("pure_x", "ideal"): "COMPLETED",
    ("pure_x", "compensated220"): "COMPLETED",
    ("pure_x", "uncompensated220"): "FAILURE",
    ("diagonal_xz", "ideal"): "COMPLETED",
    ("diagonal_xz", "compensated220"): "COMPLETED",
    ("diagonal_xz", "uncompensated220"): "FAILURE",
    ("ground_near", "ideal"): "COMPLETED",
    ("ground_near", "compensated220"): "COMPLETED",
    ("ground_near", "uncompensated220"): "FAILURE",
}

SCENARIO_POSITION = {
    "static_nominal": (0.55, 0.0, 0.55),
    "canonical_yz": (0.55, -0.56, 0.15),
    "pure_x": (0.92, 0.0, 0.55),
    "diagonal_xz": (0.9, 0.0, 0.3),
    "ground_near": (0.25, 0.62, 0.15),
}
SCENARIO_VELOCITY = {
    "static_nominal": (0.0, 0.0, 0.0),
    "canonical_yz": (0.0, 0.08, 0.0),
    "pure_x": (-0.08, 0.0, 0.0),
    "diagonal_xz": (-0.0565685, 0.0, 0.0565685),
    "ground_near": (0.0, -0.08, 0.0),
}
CONDITION_MODE = {
    "ideal": "IDEAL",
    "compensated220": "DELAYED_COMPENSATED",
    "uncompensated220": "DELAYED_UNCOMPENSATED",
}
# The controller reports configuredDelay=0.000s in IDEAL mode (perception
# latency simulation disabled) regardless of the override YAML's dormant
# delay: 0.220 value; both delayed conditions report the configured 0.220s.
CONDITION_DELAY_S = {
    "ideal": 0.0,
    "compensated220": 0.220,
    "uncompensated220": 0.220,
}

P0_RE = re.compile(
    r"\[ObserveObject\] unified static/moving observation started .*?"
    r"p0=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
)
# [ObserveObject] t=.../... ... v=[vx,vy,vz] ... -- the per-sample observed
# velocity during the observation window. The *last* occurrence (the final
# sample, t equal to the window duration) is the settled estimate, printed
# at 4-decimal display precision. This field is present for every scenario,
# including static_nominal (v=[0,0,0], zero object velocity) -- unlike the
# "[PresentationSolve] robot stationary, object approaching..." line, which
# is only emitted for a moving object and would make static_nominal
# unverifiable if used as the sole source.
V_RE = re.compile(
    r"\[ObserveObject\] t=[\d.]+/[\d.]+s .*?"
    r"v=\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]"
)
COMPLETED_RE = re.compile(r"\[Completed\] full plan-once handover completed")
FAILURE_RE = re.compile(r"Starting state HandoverInterceptionController_Failure")

POSITION_TOLERANCE = 0.0005
# This is a scenario-identity tolerance, not a physical-accuracy claim: it
# covers both the log's 4-decimal display rounding (e.g. diagonal_xz's true
# value -0.0565685 rounds to -0.0566, a 0.0000315 delta) and the small
# settled-velocity estimation variation actually observed across validated
# runs (e.g. pure_x settling at -0.0799 rather than exactly -0.08). It is
# loose enough to absorb both of those, while remaining tight enough to
# catch a materially wrong velocity (a magnitude or sign error).
VELOCITY_TOLERANCE = 0.0005
DELAY_TOLERANCE = 1e-9


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--scenario", required=True, choices=sorted(SCENARIO_POSITION))
    parser.add_argument("--condition", required=True, choices=sorted(CONDITION_MODE))
    parser.add_argument(
        "--checker",
        type=Path,
        default=Path(__file__).parent / "check_latency_log.py",
        help="path to check_latency_log.py",
    )
    args = parser.parse_args()

    expected_outcome = EXPECTED_OUTCOME[(args.scenario, args.condition)]
    text = args.log.read_text(encoding="utf-8", errors="replace")

    completed_found = bool(COMPLETED_RE.search(text))
    failure_found = bool(FAILURE_RE.search(text))
    if completed_found and failure_found:
        observed_outcome = "AMBIGUOUS"
    elif completed_found:
        observed_outcome = "COMPLETED"
    elif failure_found:
        observed_outcome = "FAILURE"
    else:
        observed_outcome = "UNKNOWN"

    proc = subprocess.run(
        [sys.executable, str(args.checker), str(args.log)],
        capture_output=True,
        text=True,
    )
    try:
        metrics = json.loads(proc.stdout) if proc.stdout.strip() else {}
    except json.JSONDecodeError:
        metrics = {}
    latency_metrics_pass = bool(metrics.get("pass", False))

    p0_match = P0_RE.search(text)
    observed_position = tuple(float(x) for x in p0_match.groups()) if p0_match else None
    expected_position = SCENARIO_POSITION[args.scenario]
    scenario_position_ok = observed_position is not None and all(
        abs(a - b) <= POSITION_TOLERANCE for a, b in zip(observed_position, expected_position)
    )

    v_matches = list(V_RE.finditer(text))
    observed_velocity = tuple(float(x) for x in v_matches[-1].groups()) if v_matches else None
    expected_velocity = SCENARIO_VELOCITY[args.scenario]
    scenario_velocity_ok = observed_velocity is not None and all(
        abs(a - b) <= VELOCITY_TOLERANCE for a, b in zip(observed_velocity, expected_velocity)
    )

    expected_mode = CONDITION_MODE[args.condition]
    observed_mode = metrics.get("mode")
    condition_mode_ok = observed_mode == expected_mode

    configured_delay_s = metrics.get("configured_delay_s")
    expected_delay_s = CONDITION_DELAY_S[args.condition]
    condition_delay_ok = (
        configured_delay_s is not None
        and abs(float(configured_delay_s) - expected_delay_s) <= DELAY_TOLERANCE
    )

    reproduction_result = (
        "PASS"
        if (
            observed_outcome == expected_outcome
            and scenario_position_ok
            and scenario_velocity_ok
            and condition_mode_ok
            and condition_delay_ok
        )
        else "FAIL"
    )

    result = {
        "scenario": args.scenario,
        "condition": args.condition,
        "log": str(args.log),
        "expected_outcome": expected_outcome,
        "observed_outcome": observed_outcome,
        "latency_metrics_pass": latency_metrics_pass,
        "observed_position": observed_position,
        "observed_velocity": observed_velocity,
        "configured_delay_s": configured_delay_s,
        "scenario_position_ok": scenario_position_ok,
        "scenario_velocity_ok": scenario_velocity_ok,
        "condition_mode_ok": condition_mode_ok,
        "condition_delay_ok": condition_delay_ok,
        "observed_mode": observed_mode,
        "reproduction_result": reproduction_result,
    }
    print(json.dumps(result, indent=2, sort_keys=True, default=list))
    return 0 if reproduction_result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
