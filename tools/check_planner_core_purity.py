#!/usr/bin/env python3
"""Live-state purity checks for the copied-state TRIAD planner core.

The planner is a copied-state certification model: from the frozen decision
epoch onward it must decide only from the frozen MultiBodyConfig, the immutable
robot model and immutable configuration. Reading the live robot inside the
rollout would make the decision depend on when the planner happened to run, and
would be a data race once the planner is moved off the control thread.

This check builds the call graph of the rollout core from the source and
asserts that the live-state helpers are unreachable from it, and that no
forbidden live symbol appears in any core function body.

Two deliberate exceptions are encoded:

  * ``wholeRobotGroundSafe`` and ``carriedObjectArmSafe`` each have a live
    overload and a ``(MultiBodyConfig, ...)`` overload. The planner calls the
    copied-state overload; the live overload shares the name and so lands in
    the same call-graph node. Line-level allowances below pin the live
    overloads so a new live read cannot hide behind the shared name.

  * ``beginPredictiveRouteCandidate`` reads ``robot().mbc()`` only as the seed
    fallback when no frozen robot state exists. That is the freeze point
    itself, not a live read during the rollout.
"""

from collections import defaultdict, deque
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "src" / "HandoverInterceptionController.cpp"
HDR = ROOT / "src" / "HandoverInterceptionController.h"

# Entry points of the copied-state rollout. The commit path is deliberately
# excluded: it runs on the control thread and is allowed to read live state.
ROOTS = [
    "stepTransitRouteCertification",
    "beginPredictiveRouteCandidate",
    "stepPredictiveRouteCandidate",
    "finalizePredictiveRouteCandidate",
    "previewReachStep",
]

# Entry points of the finite search *after* the snapshot has been created.
# prepareCapturePlanningSession() is deliberately excluded: it is the snapshot
# creation step itself, runs once on the control thread, and keeps a live
# fallback for the legacy non-global mode that has no frozen robot state.
SEARCH_ROOTS = [
    "stepFiniteTriadSearch",
    "beginCapturePlanningCore",
]

# Helpers that read the live robot. None may be reachable from the rollout.
LIVE_HELPERS = [
    "actualBasePose",
    "actualMouthPose",
    "livePadCenters",
    "mouthPoseFromBasePose",
    "liveMouthToBaseTransform",
    "evaluateCurrentPoseSafety",
    "evaluateCurrentClosureSafety",
    "evaluateAttachedRetreatSafety",
    "sweptGripperPoseSafe",
]

FORBIDDEN = {
    "robot().mb()": r"robot\(\)\.mb\(\)",
    "robot().mbc()": r"robot\(\)\.mbc\(\)",
    "robot().frame(": r"robot\(\)\.frame\(",
    "robot().hasFrame(": r"robot\(\)\.hasFrame\(",
    "robots()": r"\brobots\(\)",
    "realRobot": r"\brealRobot",
    "logger()": r"\blogger\(\)",
    "gui()": r"\bgui\(\)",
    "datastore": r"\bdatastore\b",
    "solver()": r"\bsolver\(\)",
    "controllerTime_": r"\bcontrollerTime_\b",
}

# (function, symbol) pairs allowed, with the reason. See the module docstring.
# Controller state the planner must never write: execution, commit and
# selected-plan state belong to the control thread, not to a relocatable search.
FORBIDDEN_WRITES = [
    r"\bcandidateSelected_\s*=",
    r"\binterceptionCommitted_\s*=",
    r"\bcommitted[A-Z]\w*_\s*=",
    r"\bselectedCandidate\w*_\s*=",
    r"\bselectedArmPosture_\s*=",
    r"\bW_T_M_(?:standoff|pre|retreat|transit|acquired)_\s*=",
]

ALLOWED = {
    ("wholeRobotGroundSafe", "robot().frame("): "live overload",
    ("wholeRobotGroundSafe", "robot().hasFrame("): "live overload",
    ("carriedObjectArmSafe", "robot().mbc()"): "live overload",
    ("beginPredictiveRouteCandidate", "robot().mbc()"): "frozen-state seed",
    ("refreshPlannerModel", "robot().mb()"): "takes the planner's model copy",
}

DEF_RE = re.compile(
    r"^(?:[A-Za-z_~][\w:<>,\s\*&]*\s)?HandoverInterceptionController::"
    r"([A-Za-z_]\w*)\s*\(")
# A self-call is not preceded by '.', '->' or '::'.
CALL_RE = re.compile(r"(?<![.\w>:])\b([a-z][A-Za-z0-9_]*)\s*\(")


def build():
    src = CPP.read_text(encoding="utf-8").split("\n")
    members = set(re.findall(r"\b([a-z][A-Za-z0-9_]*)\s*\(",
                             HDR.read_text(encoding="utf-8")))
    order = []
    for i, line in enumerate(src, 1):
        m = DEF_RE.match(line)
        if m:
            order.append((m.group(1), i))
    bounds = defaultdict(list)
    for k, (name, start) in enumerate(order):
        end = order[k + 1][1] - 1 if k + 1 < len(order) else len(src)
        bounds[name].append((start, end))
    calls = defaultdict(set)
    for name, ranges in bounds.items():
        for start, end in ranges:
            for line in src[start:end]:
                line = re.sub(r"//.*", "", line)
                line = re.sub(r"\w+(?:\.|->)\w+\s*\(", " ", line)
                for callee in CALL_RE.findall(line):
                    if callee in members and callee != name and callee in bounds:
                        calls[name].add(callee)
    return src, bounds, calls


def reachable(calls, roots):
    seen, stack = set(), list(roots)
    while stack:
        f = stack.pop()
        if f in seen:
            continue
        seen.add(f)
        stack.extend(calls.get(f, ()))
    return seen


def path_to(calls, roots, target):
    queue = deque((r, [r]) for r in roots)
    seen = set(roots)
    while queue:
        f, p = queue.popleft()
        if f == target:
            return p
        for c in calls.get(f, ()):
            if c not in seen:
                seen.add(c)
                queue.append((c, p + [c]))
    return None


def main() -> int:
    src, bounds, calls = build()
    for name in ROOTS:
        if name not in bounds:
            print(f"planner core purity: FAIL missing root {name}")
            return 1
    core = reachable(calls, ROOTS)
    failures = []

    # Once the snapshot exists, no part of the finite search may sample the live
    # robot again. The mouth-to-base transform in particular is derived from the
    # frozen robot state, so every event hypothesis consumes one common value.
    missing_search = [r for r in SEARCH_ROOTS if r not in bounds]
    if missing_search:
        print(f"planner core purity: FAIL missing search root {missing_search}")
        return 1
    search = reachable(calls, SEARCH_ROOTS)
    # The search owns its world. Touching the controller's W_T_O_/W_T_H_/
    # planningM_T_O_ would race control-thread logging once the search runs off
    # the control thread.
    for name in sorted(search):
        for start, end in bounds.get(name, []):
            for offset, line in enumerate(src[start:end]):
                stripped = re.sub(r"//.*", "", line)
                if re.search(r"(?<!\.)\b(W_T_O_|W_T_H_|planningM_T_O_)\b",
                             stripped):
                    failures.append(
                        f"{name}() touches the controller world at "
                        f"HandoverInterceptionController.cpp:"
                        f"{start + offset + 1}")
    for helper in LIVE_HELPERS + ["liveMouthToBaseTransform"]:
        if helper in search:
            chain = path_to(calls, SEARCH_ROOTS, helper)
            failures.append(
                f"live helper {helper}() is reachable from the finite search"
                f" after snapshot creation via {' -> '.join(chain)}")

    for helper in LIVE_HELPERS:
        if helper in core:
            chain = path_to(calls, ROOTS, helper)
            failures.append(
                f"live helper {helper}() is reachable from the rollout core"
                f" via {' -> '.join(chain)}")

    for name in sorted(core):
        for start, end in bounds.get(name, []):
            for offset, line in enumerate(src[start:end]):
                stripped = re.sub(r"//.*", "", line)
                for symbol, pattern in FORBIDDEN.items():
                    if not re.search(pattern, stripped):
                        continue
                    if (name, symbol) in ALLOWED:
                        continue
                    failures.append(
                        f"{name}() reads {symbol} at "
                        f"HandoverInterceptionController.cpp:{start + offset + 1}")

    # Ownership: the rollout must not write control-thread execution state.
    for name in sorted(core):
        for start, end in bounds.get(name, []):
            for offset, line in enumerate(src[start:end]):
                stripped = re.sub(r"//.*", "", line)
                for pattern in FORBIDDEN_WRITES:
                    if re.search(pattern, stripped):
                        failures.append(
                            f"{name}() writes control-thread execution state at "
                            f"HandoverInterceptionController.cpp:"
                            f"{start + offset + 1}")

    if failures:
        print("planner core purity: FAIL")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(f"planner core purity: PASS "
          f"({len(core)} rollout-core functions, {len(search)} post-snapshot "
          f"search functions, no live-state dependency, no control-thread "
          f"state written)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
