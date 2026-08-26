#!/usr/bin/env python3
"""Source consistency checks for the global binding-cost selector."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def main() -> int:
    controller = read("src/HandoverInterceptionController.cpp")
    header = read("src/HandoverInterceptionController.h")
    solve = read(
        "src/states/HandoverInterceptionController_SolveInterception.cpp"
    )
    config = read("etc/HandoverInterceptionController.in.yaml")
    cmake = read("CMakeLists.txt")

    require("selectionMode: binding_cost" in config,
            "development config is not in binding_cost mode")
    require("eventSelectionMode: global_time_plan" in config,
            "development config is not in global time-plan mode")
    require("allowPhysicalExecution: false" in config,
            "uncalibrated physical binding execution is not blocked")
    require("physicalBridge:\n    enabled: false" in config,
            "development configuration is not simulation-safe by default")
    require("set(CMAKE_CXX_STANDARD 14)" in cmake,
            "declared C++ standard does not match std::make_unique usage")
    require("FinitePlanSelector.h" in controller,
            "controller does not use the finite-plan selector")
    require("FiniteEventPlanSelector.h" in controller,
            "controller does not use the global finite-event selector")
    require(
        re.search(
            r"planningBestCandidate_\s*=\s*\n?\s*"
            r"planningCompletePlanAuditCandidates_\[selection\.selectedRecord\]",
            controller,
        ) is not None,
        "minimum-cost selector does not write the committed candidate",
    )
    require("[BindingCostCommitProof] committed=true" in controller,
            "commit path does not emit the binding-cost proof")
    require("selectedWithinMinimumTolerance=true" in controller,
            "commit path does not assert selected J matches minimum J")
    require("no fallback permitted" in controller,
            "binding selector lacks an explicit no-fallback failure policy")
    require("remainingToHypothesis, minimumReachEntryLead_" in solve,
            "moving event does not apply candidate-specific timing admission")
    require("selectPlanningBestForCommit()" in solve,
            "static event does not finalize the binding selector")
    require("planningCostSelectionCommitAdmissible_" in header,
            "commit-admission state is not represented in the controller")
    require("globalEventPlanAlternatives_" in header,
            "controller does not retain the complete cross-event plan set")
    require(
        "eventSearchStartTime_ + guessLead_" in solve,
        "global event times are not frozen to one common search epoch",
    )
    require(
        "boundedEventPresentationPoses_" in solve
        and "predictionModelFrozen=true" in solve,
        "global event poses are not frozen from one prediction snapshot",
    )
    require(
        "attemptedEventLeads_.size() == boundedEventLeads_.size()" in solve,
        "global commit does not require complete hypothesis coverage",
    )
    require(
        "captureCurrentEventPlanAlternatives" in solve,
        "complete event alternatives are not deferred for global selection",
    )
    require(
        "selectGlobalTimePlanForCommit" in solve,
        "state does not invoke the exhaustive time-plan argmin",
    )
    require(
        "commitGlobalTimePlanSelection" in solve,
        "state does not bind the global winner to commit",
    )
    require(
        "extendMotionCostToSearchEpoch" in controller,
        "global objective does not extend the existing time term",
    )
    require("[GlobalTimePlanCommitProof] committed=true" in controller,
            "commit path lacks a global time-plan proof")
    require("global_winner_expired_before_commit" in controller,
            "global winner is not timing-readmitted immediately before commit")
    require("incomplete_bounded_event_schedule" in controller,
            "incomplete global searches do not fail closed")

    retreat = function_body(
        controller,
        "bool HandoverInterceptionController::previewAttachedRetreatSafe(",
        "bool HandoverInterceptionController::evaluateAttachedRetreatSafety(",
    )
    require("sampleWorldPoint(sample, mbc)" in retreat,
            "retreat preview does not use copied closed-gripper transforms")
    require("basePoseFromMouthPose" not in retreat,
            "retreat preview still reconstructs cached live/open geometry")

    closure = function_body(
        controller,
        "HandoverInterceptionController::previewClosureStep(",
        "bool HandoverInterceptionController::previewTerminalCaptureDwell(",
    )
    require("previewDynamicClosureSafety(mbc, report, false)" in closure,
            "preview closure does not begin with the strict contact contract")
    require("previewDynamicClosureSafety(\n        mbc, designatedContactReport, true)"
            in closure,
            "preview closure lacks bounded designated-contact re-evaluation")
    require("designatedContactReport.bilateralPadContact" in closure,
            "preview contact relaxation is not bilateral-contact gated")

    print("global binding-cost source contract checks: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, ValueError) as error:
        print(f"global binding-cost source contract checks: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
