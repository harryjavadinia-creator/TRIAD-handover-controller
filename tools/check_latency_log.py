#!/usr/bin/env python3
"""Extract perception-latency condition metrics from a run log."""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("log", type=Path)
args = parser.parse_args()
text = args.log.read_text(errors="replace")

def find(pattern: str, cast=float, default=None):
    match = re.search(pattern, text)
    return cast(match.group(1)) if match else default

mode_match = re.search(r"\[PerceptionLatencySummary\] mode=([A-Z_]+)", text)
metrics = {
    "log": str(args.log),
    "mode": mode_match.group(1) if mode_match else None,
    "configured_delay_s": find(r"\[PerceptionLatencySummary\].*configuredDelay=([0-9.]+)s"),
    "measurement_age_s": find(r"\[PerceptionLatencySummary\].*measurementAge=([0-9.]+)s"),
    "raw_position_error_m": find(r"\[PerceptionLatencySummary\].*rawPositionError=([0-9.]+)m"),
    "estimate_position_error_m": find(r"\[PerceptionLatencySummary\].*compensatedPositionError=([0-9.]+)m"),
    "planning_s": find(r"\[TimingSummary\] planning=([0-9.]+)s"),
    "execution_s": find(r"\[TimingSummary\].*execution=([0-9.]+)s"),
    "completed": "[Completed] full plan-once handover completed" in text,
    "one_commit": "oneCommit=true" in text,
    "no_retiming": "noRetiming=true" in text,
    "no_replanning": "noReplanning=true" in text,
    "hard_geometry_failure": "hard closure geometry violation" in text,
    "failure_state": "Starting state HandoverInterceptionController_Failure" in text,
}
metrics["pass"] = all(
    [metrics["completed"], metrics["one_commit"], metrics["no_retiming"], metrics["no_replanning"]]
) and not metrics["hard_geometry_failure"] and not metrics["failure_state"]
print(json.dumps(metrics, indent=2, sort_keys=True))
