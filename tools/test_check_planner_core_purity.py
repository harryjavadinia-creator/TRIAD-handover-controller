#!/usr/bin/env python3
"""Mutation tests for tools/check_planner_core_purity.py.

A purity checker that cannot fail proves nothing, so each mutation below
reintroduces a live-state dependency that the real refactor removed and asserts
the checker rejects it.
"""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def run_checker(tree: Path) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(tree / "tools" / "check_planner_core_purity.py")],
        capture_output=True, text=True)


def make_tree(tmp: Path) -> Path:
    tree = tmp / "repo"
    (tree / "src").mkdir(parents=True)
    (tree / "tools").mkdir(parents=True)
    for name in ("HandoverInterceptionController.cpp",
                 "HandoverInterceptionController.h"):
        shutil.copy(ROOT / "src" / name, tree / "src" / name)
    shutil.copy(ROOT / "tools" / "check_planner_core_purity.py",
                tree / "tools" / "check_planner_core_purity.py")
    return tree


def patch(tree: Path, old: str, new: str) -> None:
    path = tree / "src" / "HandoverInterceptionController.cpp"
    text = path.read_text(encoding="utf-8")
    if text.count(old) != 1:
        raise AssertionError(
            f"mutation anchor not unique ({text.count(old)}): {old[:60]!r}")
    path.write_text(text.replace(old, new), encoding="utf-8")


MUTATIONS = [
    (
        "reinstate the live mouth-to-base read in the swept certification",
        """               planningSnapshot_.mouthToBaseTransform, plannerContext_.W_T_O,
               plannerContext_.W_T_H, sweptReport, false))""",
        """               liveMouthToBaseTransform(), plannerContext_.W_T_O,
               plannerContext_.W_T_H, sweptReport, false))""",
    ),
    (
        "reinstate the live base-pose fallback in previewBasePose",
        """  W_T_B = mbc.bodyPosW[static_cast<size_t>(plannerContext_.previewToolBodyIndex)];
  return true;""",
        """  W_T_B = actualBasePose();
  return true;""",
    ),
    (
        "reinstate the live mouth fallback in previewMouthPose",
        """    // Previously mouthPoseFromBasePose(), which reads livePadCenters(),
    // actualBasePose() and actualMouthPose(). A copied-state preview that
    // cannot resolve its own pad frames fails instead.
    return false;""",
        """    W_T_M = mouthPoseFromBasePose(W_T_B);
    return true;""",
    ),
    (
        "read the controller clock inside the rollout",
        """  const int budget = std::max(1, workUnits);""",
        """  const int budget = std::max(1, workUnits + (controllerTime_ > 1e300));""",
    ),
    (
        "share the mc_rtc multibody instead of the planner's copy",
        """  const auto & mb = plannerModel();
  // Which joints the preview skips.""",
        """  const auto & mb = robot().mb();
  // Which joints the preview skips.""",
    ),
    (
        "write the controller world from the search instead of the planner world",
        """  plannerContext_.W_T_O = W_T_O_virtual;
  plannerContext_.W_T_H = compose(plannerContext_.W_T_O, O_T_H_);""",
        """  W_T_O_ = W_T_O_virtual;
  W_T_H_ = compose(W_T_O_, O_T_H_);""",
    ),
    (
        "write control-thread commit state from the rollout",
        """      plannerContext_.routeStepCandidate.plannedTransitArmPosture =
              armPostureFromMbc(plannerContext_.routeStepMbc);""",
        """      candidateSelected_ = true;
          plannerContext_.routeStepCandidate.plannedTransitArmPosture =
              armPostureFromMbc(plannerContext_.routeStepMbc);""",
    ),
    (
        "resample the live mouth-to-base transform inside the search",
        """  plannerContext_.planningM_T_O = sva::PTransformd::Identity();""",
        """  plannerContext_.planningM_T_O = sva::PTransformd::Identity();
  planningSnapshot_.mouthToBaseTransform = liveMouthToBaseTransform();""",
    ),
    (
        "reinstate the live start-mouth-pose fallback in the search",
        """  else if(!plannerContext_.planningStartMouthPoseValid)""",
        """  else if(!plannerContext_.planningStartMouthPoseValid
          && actualMouthPose().translation().x() > 1e300)""",
    ),
    (
        "emit an mc_rtc log from a pure preview helper",
        """  W_T_M = fromWorldPose(R_W_M, pM);
  return true;""",
        """  W_T_M = fromWorldPose(R_W_M, pM);
  logger();
  return true;""",
    ),
]


def main() -> int:
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        baseline = make_tree(tmp / "baseline")
        result = run_checker(baseline)
        if result.returncode != 0:
            print("unmutated source must PASS, got:")
            print(result.stdout + result.stderr)
            return 1
        print("unmutated source: PASS")

        caught = 0
        for index, (label, old, new) in enumerate(MUTATIONS):
            tree = make_tree(tmp / f"mut{index}")
            patch(tree, old, new)
            result = run_checker(tree)
            if result.returncode == 0:
                print(f"  NOT CAUGHT: {label}")
                print(result.stdout)
            else:
                caught += 1
                print(f"  caught: {label}")

        print(f"\nmutations caught: {caught}/{len(MUTATIONS)}")
        if caught != len(MUTATIONS):
            return 1
    print("planner core purity checker tests: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
