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

    print("timing-frontier replay tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
