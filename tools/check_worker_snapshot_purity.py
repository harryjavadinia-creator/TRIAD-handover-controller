#!/usr/bin/env python3
"""Worker snapshot purity check for the background TRIAD planner.

TRIAD V2 moves the robot while the background planner works. Every value the
worker consumes must therefore come from the immutable planning snapshot
(``planningSnapshot_``), the frozen planner configuration mirror
(``plannerConfig_``) or the worker-owned scratch context (``plannerContext_``),
never from the live robot, the live object estimator or the controller world.

The check builds the member call graph from the worker entry points and scans
every reachable function body. A forbidden live access is accepted only when
it sits in a ``plannerWorkerThreadActive() ? <snapshot> : <live>`` selection
(same line or the line before), which reads the snapshot on the worker thread
and preserves the original control-thread behaviour, or when it is listed in
ALLOWED with a reason.

Usage: python3 tools/check_worker_snapshot_purity.py [--list]
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_planner_core_purity as core  # noqa: E402

WORKER_ROOTS = [
    "runPlannerWorker",
    # V2 job entry points (present only on the V2 branch).
    "runReceiverWorkerJobV2",
]

FORBIDDEN = {
    "robot()": r"\brobot\(\)",
    "robots()": r"\brobots\(\)",
    "live mouth gap": r"\bliveMouth(?:Half)?Gap\(",
    "live pad centers": r"\blivePadCenters\(",
    "live mouth/base pose": r"\bactual(?:Mouth|Base)Pose\(",
    "live mouth-to-base": r"\bliveMouthToBaseTransform\(",
    "estimator twist": r"\bobject(?:Linear|Angular)VelocityEstimate_\b",
    "estimator validity": r"\bobjectMotionEstimateValid_\b",
    "observation mode": r"\bobservedObjectMode_\b",
    "controller world": r"(?<![.\w])(?:W_T_O_|W_T_H_|planningM_T_O_)\b",
    "controller clock": r"\bcontrollerTime_\b",
    "presentation mode member": r"\bpresentationMode_\b",
    "moving interception member": r"\bpreviewMovingInterception_\b",
    "V2 provisional state": r"\bprovisionalReceiverPlan_\b",
    "committed plan": r"\bcommittedInterceptionPlan_\b",
}

GUARD = re.compile(r"plannerWorkerThreadActive\(\)")

# (function, label): reason. Each entry is a documented, non-worker path that
# shares a function name or a pre-thread freeze step.
ALLOWED = {
    ("wholeRobotGroundSafe", "robot()"):
        "live overload shares the name; worker calls the MultiBodyConfig overload",
    ("carriedObjectArmSafe", "robot()"):
        "live overload shares the name; worker calls the MultiBodyConfig overload",
    ("beginPredictiveRouteCandidate", "robot()"):
        "seed fallback when no frozen state exists; runPlannerWorker rejects that case",
    ("startNextPlanningCandidate", "robot()"):
        "seed fallback when no frozen state exists; runPlannerWorker rejects that case",
    ("shutdownPlannerWorker", "robot()"): "not reachable from the worker body",
    # commitCandidate is reachable only through finalizeCapturePlanning with
    # capturePlanningCommitOnSuccess_ == true, which beginFiniteTriadSearch
    # forces false before the worker starts.
    ("commitCandidate", "*"): "unreachable on worker: commit disabled for frozen search",
    ("finalizeCapturePlanning", "committed plan"): "commit branch disabled for frozen search",
    ("selectPlanningBestForCommit", "*"): "diagnostic local selection; reads no live state",
}


def main() -> int:
    src, bounds, calls = core.build()
    roots = [r for r in WORKER_ROOTS if r in bounds]
    if "runPlannerWorker" not in roots:
        print("worker snapshot purity: FAIL runPlannerWorker not found")
        return 1
    reach = core.reachable(calls, roots)
    failures = []
    for name in sorted(reach):
        for start, end in bounds.get(name, []):
            for off, raw in enumerate(src[start:end]):
                line = re.sub(r"//.*", "", raw)
                prev = re.sub(r"//.*", "", src[start + off - 1]) if off else ""
                for label, pat in FORBIDDEN.items():
                    if not re.search(pat, line):
                        continue
                    if GUARD.search(line) or GUARD.search(prev):
                        continue
                    if (name, label) in ALLOWED or (name, "*") in ALLOWED:
                        continue
                    failures.append(
                        f"{name}() {label} at HandoverInterceptionController.cpp:"
                        f"{start + off + 1}: {raw.strip()}")
    if "--list" in sys.argv:
        print("\n".join(sorted(reach)))
    if failures:
        print(f"worker snapshot purity: FAIL ({len(failures)} live access(es))")
        for f in failures:
            print("  " + f)
        return 1
    print(f"worker snapshot purity: PASS ({len(reach)} worker-reachable functions, "
          f"{len(roots)} root(s), no unguarded live robot/estimator/world access)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
