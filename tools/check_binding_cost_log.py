#!/usr/bin/env python3
"""Verify that a runtime log proves a binding-cost plan was committed."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
SELECTION_RE = re.compile(
    rf"\[BindingCostSelection\] success=true commitAdmissible=true "
    rf"completePlans=(\d+) costValidPlans=(\d+) timingAdmissiblePlans=(\d+) "
    rf"candidate=(\S+) route=(\S+) selectedJ=({NUMBER}) "
    rf"minimumAdmissibleJ=({NUMBER}) selectedWithinMinimumTolerance=true "
    rf"tieTolerance=({NUMBER})"
)
COMMIT_RE = re.compile(
    rf"\[BindingCostCommitProof\] committed=true candidate=(\S+) "
    rf"route=(\S+) selectedJ=({NUMBER}) minimumAdmissibleJ=({NUMBER}) "
    rf"selectedWithinMinimumTolerance=true tieTolerance=({NUMBER}) "
    rf"completePlans=(\d+) costValidPlans=(\d+) timingAdmissiblePlans=(\d+)"
)


def verify_text(text: str) -> list[str]:
    errors: list[str] = []
    if not re.search(
        r"\[PlanSelectionConfiguration\] mode=binding_cost "
        r"costConfigurationValid=true",
        text,
    ):
        errors.append("missing valid binding_cost configuration record")

    if "[BindingCostCommitProof] committed=false" in text:
        errors.append("log contains a rejected binding-cost commit")

    selections = list(SELECTION_RE.finditer(text))
    commits = list(COMMIT_RE.finditer(text))
    if not selections:
        errors.append("missing commit-admissible binding-cost selection")
    if len(commits) != 1:
        errors.append(f"expected exactly one binding-cost commit, found {len(commits)}")
    if errors or not commits or not selections:
        return errors

    commit = commits[0]
    c_candidate, c_route = commit.group(1), commit.group(2)
    c_selected = float(commit.group(3))
    c_minimum = float(commit.group(4))
    c_tolerance = float(commit.group(5))
    c_complete = int(commit.group(6))
    c_valid = int(commit.group(7))
    c_admissible = int(commit.group(8))

    if c_complete <= 0:
        errors.append("commit reports no complete plans")
    if c_valid != c_complete:
        errors.append(
            f"cost set is incomplete: {c_valid} valid of {c_complete} complete plans"
        )
    if c_admissible <= 0:
        errors.append("commit reports no timing-admissible plans")
    if abs(c_selected - c_minimum) > c_tolerance + 1e-12:
        errors.append("committed J is outside the logged minimum tolerance")

    matching = []
    for selection in selections:
        if selection.group(4) != c_candidate or selection.group(5) != c_route:
            continue
        s_selected = float(selection.group(6))
        s_minimum = float(selection.group(7))
        s_tolerance = float(selection.group(8))
        if (
            abs(s_selected - c_selected) <= max(s_tolerance, c_tolerance) + 1e-12
            and abs(s_minimum - c_minimum)
            <= max(s_tolerance, c_tolerance) + 1e-12
        ):
            matching.append(selection)
    if not matching:
        errors.append("committed action has no matching admissible argmin selection")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    text = args.log.read_text(encoding="utf-8", errors="replace")
    errors = verify_text(text)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("binding-cost runtime log proof: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
