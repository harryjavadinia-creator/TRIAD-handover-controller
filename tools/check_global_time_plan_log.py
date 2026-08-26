#!/usr/bin/env python3
"""Runtime result checker for the TRIAD finite complete-plan selector.

Verifies one complete global time-grasp-route simulation run, or its
fail-safe counterpart when no cost-valid admissible alternative exists
globally.

A cost-invalid GlobalPlanCost record is not by itself a failure: it is only
a defect if it was not individually excluded from the pooled/selected set,
or if it conflates with a genuine geometry/IK rejection.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

SELECTION_RE = re.compile(
    rf"\[GlobalTimePlanSelection\] success=true scheduleComplete=true "
    rf"evaluatedHypotheses=(\d+) configuredHypotheses=(\d+) "
    rf"completePlans=(\d+) costValidPlans=(\d+) "
    rf"timingAdmissiblePlans=(\d+) hypothesis=(\d+) "
    rf"eventLead=({NUMBER})s presentationTime=({NUMBER})s "
    rf"candidate=(\S+) route=(\S+) motionJ=({NUMBER}) "
    rf"scheduleWait=({NUMBER})s selectedGlobalJ=({NUMBER}) "
    rf"minimumGlobalJ=({NUMBER}) selectedWithinMinimumTolerance=true "
    rf"tieTolerance=({NUMBER})"
)
COMMIT_RE = re.compile(
    rf"\[GlobalTimePlanCommitProof\] committed=true scheduleComplete=true "
    rf"evaluatedHypotheses=(\d+) configuredHypotheses=(\d+) "
    rf"completePlans=(\d+) costValidPlans=(\d+) "
    rf"timingAdmissiblePlans=(\d+) hypothesis=(\d+) "
    rf"eventLead=({NUMBER})s presentationTime=({NUMBER})s "
    rf"candidate=(\S+) route=(\S+) motionJ=({NUMBER}) "
    rf"scheduleWait=({NUMBER})s selectedGlobalJ=({NUMBER}) "
    rf"minimumGlobalJ=({NUMBER}) selectedWithinMinimumTolerance=true "
    rf"tieTolerance=({NUMBER})"
)
FAILSAFE_SELECTION_RE = re.compile(
    rf"\[GlobalTimePlanSelection\] success=false reason=(\S+) "
    rf"scheduleComplete=(true|false) evaluatedHypotheses=(\d+) "
    rf"configuredHypotheses=(\d+) completePlans=(\d+) costValidPlans=(\d+) "
    rf"timingAdmissiblePlans=(\d+) no fallback permitted"
)
GLOBAL_COST_RE = re.compile(
    rf"\[GlobalPlanCost\] hypothesis=(\d+) eventLead=({NUMBER})s "
    rf"presentationTime=({NUMBER})s candidate=(\S+) route=(\S+) "
    rf"valid=(true|false) motionJ=({NUMBER}) scheduleWait=({NUMBER})s "
    rf"searchToCompletion=({NUMBER})s globalJ=({NUMBER})"
)
CAPTURE_RE = re.compile(
    rf"\[GlobalTimePlanCapture\] success=true hypothesis=(\d+) "
    rf"eventLead=({NUMBER})s completePlansAdded=(\d+) "
    rf"cumulativeCompletePlans=(\d+) deferredCommit=true robotMotion=false"
)
COST_AUDIT_REJECT_RE = re.compile(
    rf"\[GlobalTimePlanCostAuditReject\] hypothesis=(\d+) "
    rf"eventLead=({NUMBER})s completePlans=(\d+) costInvalidPlans=(\d+) "
    rf"reason=all_candidates_cost_invalid deferredCommit=true; "
    rf"searching another future event"
)
GEOMETRY_REJECT_RE = re.compile(
    rf"\[BoundedEventGeometryReject\] hypothesis=(\d+) lead=({NUMBER})s "
    rf"source=(\S+) completePlan=false geometryFailures=(\d+) "
    rf"robotStationary=true; searching another future event"
)
TIMING_ADMISSIBILITY_RE = re.compile(
    rf"\[GlobalPlanTimingAdmissibility\] hypothesis=(\d+) candidate=(\S+) "
    rf"route=(\S+) costValid=(true|false) globalJ=({NUMBER}) "
    rf"eventPresentationTime=({NUMBER})s now=({NUMBER})s "
    rf"remaining=({NUMBER})s minimumSafeCommitLead=({NUMBER})s "
    rf"predictedPresentationDuration=({NUMBER})s "
    rf"minimumReachEntryLead=({NUMBER})s "
    rf"eventWindowAdmissible=(true|false) timingAdmissible=(true|false)"
)

ALLOWED_FAILSAFE_REASONS = {
    "no_complete_time_plan_alternatives",
    "incomplete_or_nonfinite_global_cost_set",
    "no_final_timing_admissible_time_plan",
}

# Frozen seven-term binding preference objective (T,E,L,C,Q,K,V). Must match
# HandoverInterceptionController.h's decision*Weight_ defaults exactly. R is
# intentionally absent: it is diagnostic-only and must never enter the
# binding sum. Kept here (not imported) so this check is a genuinely
# independent reconstruction, not a re-use of the controller's own weights.
FROZEN_WEIGHTS = {
    "T": 0.4210526,
    "E": 0.1052632,
    "L": 0.1052632,
    "C": 0.1578947,
    "Q": 0.0842105,
    "K": 0.0736842,
    "V": 0.0526316,
}

COMPLETE_PLAN_COST_RE = re.compile(
    rf"\[CompletePlanCost\] candidate=(\S+) route=(\S+) valid=(true|false) "
    rf"J=({NUMBER}) weightsNormalized=true .*?"
    rf"velocityUtil=({NUMBER}) velocityLaw=rho4 .*?"
    rf"terms=\[T:({NUMBER}),E:({NUMBER}),L:({NUMBER}),R:({NUMBER}),"
    rf"C:({NUMBER}),Q:({NUMBER}),K:({NUMBER}),V:({NUMBER})\]"
)


def _parse_complete_plan_costs(text: str) -> list[dict]:
    out = []
    for match in COMPLETE_PLAN_COST_RE.finditer(text):
        out.append(
            {
                "candidate": match.group(1),
                "route": match.group(2),
                "valid": match.group(3) == "true",
                "J": float(match.group(4)),
                "velocityUtil": float(match.group(5)),
                "T": float(match.group(6)),
                "E": float(match.group(7)),
                "L": float(match.group(8)),
                "R": float(match.group(9)),
                "C": float(match.group(10)),
                "Q": float(match.group(11)),
                "K": float(match.group(12)),
                "V": float(match.group(13)),
            }
        )
    return out


def verify_frozen_objective(text: str) -> list[str]:
    """Independently reconstruct the binding cost from logged per-candidate
    raw terms using ONLY the frozen seven-term weight vector (R excluded),
    and cross-check V against the quartic joint-velocity-only law. This
    proves, from real log data: R does not affect the binding cost (its
    logged value is free to vary; it is simply never summed); the binding
    weights match the frozen vector (any other vector would fail the
    reconstruction); and V follows clamp01(velocityUtil)**4 exactly.
    """
    errors: list[str] = []
    records = _parse_complete_plan_costs(text)
    if not records:
        errors.append(
            "missing per-candidate CompletePlanCost records needed to "
            "verify the frozen seven-term objective"
        )
        return errors

    # Loose per-term tolerance accounts for the log's own 4-decimal-place
    # rounding of each of the seven terms before this script re-sums them;
    # it is far tighter than the ~0.05 error an accidentally-reinstated R
    # term would produce, so it remains a meaningful check.
    tol = 2e-3
    j_mismatches = 0
    v_mismatches = 0
    for rec in records:
        if not rec["valid"]:
            continue
        j_check = sum(FROZEN_WEIGHTS[k] * rec[k] for k in FROZEN_WEIGHTS)
        if abs(j_check - rec["J"]) > tol:
            j_mismatches += 1
        v_check = min(1.0, max(0.0, rec["velocityUtil"])) ** 4
        if abs(v_check - rec["V"]) > tol:
            v_mismatches += 1

    if j_mismatches:
        errors.append(
            f"{j_mismatches} CompletePlanCost record(s) do not reconstruct "
            "from the frozen seven-term weight vector (R excluded) within "
            "tolerance -- binding weights or R-exclusion may have regressed"
        )
    if v_mismatches:
        errors.append(
            f"{v_mismatches} CompletePlanCost record(s) have a V term that "
            "does not match clamp01(velocityUtil)**4 -- V may no longer be "
            "joint-velocity-only"
        )
    return errors


def _parse_costs(text: str) -> list[dict]:
    out = []
    for match in GLOBAL_COST_RE.finditer(text):
        out.append(
            {
                "hypothesis": int(match.group(1)),
                "candidate": match.group(4),
                "route": match.group(5),
                "valid": match.group(6) == "true",
                "global": float(match.group(10)),
            }
        )
    return out


def _parse_captures(text: str) -> dict[int, dict]:
    out: dict[int, dict] = {}
    for match in CAPTURE_RE.finditer(text):
        out[int(match.group(1))] = {
            "added": int(match.group(3)),
            "cumulative": int(match.group(4)),
        }
    return out


def _parse_cost_audit_rejects(text: str) -> dict[int, dict]:
    out: dict[int, dict] = {}
    for match in COST_AUDIT_REJECT_RE.finditer(text):
        out[int(match.group(1))] = {
            "complete": int(match.group(3)),
            "invalid": int(match.group(4)),
        }
    return out


def _parse_geometry_rejects(text: str) -> dict[int, dict]:
    out: dict[int, dict] = {}
    for match in GEOMETRY_REJECT_RE.finditer(text):
        out[int(match.group(1))] = {"geometry_failures": int(match.group(4))}
    return out


def _parse_timing_admissibility(text: str) -> list[dict]:
    """Independently reconstructs, for every pooled alternative, the exact
    predicate selectFiniteEventPlan() applies for the final timing gate.
    Present only in logs produced after the timing-admissibility logging
    patch; callers must treat an empty result as "not available", not as
    "no admissible alternatives".
    """
    out = []
    for match in TIMING_ADMISSIBILITY_RE.finditer(text):
        out.append(
            {
                "hypothesis": int(match.group(1)),
                "candidate": match.group(2),
                "route": match.group(3),
                "cost_valid": match.group(4) == "true",
                "global_j": float(match.group(5)),
                "timing_admissible": match.group(13) == "true",
            }
        )
    return out


def _reconcile_hypotheses(
    costs: list[dict],
    captures: dict[int, dict],
    cost_audit_rejects: dict[int, dict],
    geometry_rejects: dict[int, dict],
) -> tuple[list[str], dict[int, int], dict[int, int]]:
    """Independently rebuild, per hypothesis, what happened to every
    generated complete plan: pooled (valid), individually excluded
    (cost-invalid), or never generated (genuine geometry/IK rejection).
    Flags any conflation between a cost-audit rejection and a geometry
    rejection, and any mismatch between the per-candidate records and the
    hypothesis-level counters that summarize them.
    """
    errors: list[str] = []
    valid_by_hyp: dict[int, int] = {}
    invalid_by_hyp: dict[int, int] = {}
    for cost in costs:
        bucket = valid_by_hyp if cost["valid"] else invalid_by_hyp
        bucket[cost["hypothesis"]] = bucket.get(cost["hypothesis"], 0) + 1

    hypotheses = (
        set(valid_by_hyp)
        | set(invalid_by_hyp)
        | set(captures)
        | set(cost_audit_rejects)
        | set(geometry_rejects)
    )
    for h in sorted(hypotheses):
        valid_count = valid_by_hyp.get(h, 0)
        invalid_count = invalid_by_hyp.get(h, 0)
        in_capture = h in captures
        in_cost_reject = h in cost_audit_rejects
        in_geometry_reject = h in geometry_rejects

        if in_geometry_reject:
            if valid_count or invalid_count or in_capture or in_cost_reject:
                errors.append(
                    f"hypothesis {h} is tagged BoundedEventGeometryReject "
                    "but also has complete-plan cost data "
                    "(geometry/cost-audit rejection conflated)"
                )
            continue

        if in_cost_reject:
            reject = cost_audit_rejects[h]
            if valid_count != 0:
                errors.append(
                    f"hypothesis {h} has a GlobalTimePlanCostAuditReject "
                    f"record but reports {valid_count} valid candidates"
                )
            if invalid_count != reject["invalid"]:
                errors.append(
                    f"hypothesis {h} cost-audit reject count mismatch: "
                    f"{invalid_count} logged invalid records vs "
                    f"costInvalidPlans={reject['invalid']}"
                )
            if valid_count + invalid_count != reject["complete"]:
                errors.append(
                    f"hypothesis {h} cost-audit reject completePlans="
                    f"{reject['complete']} does not match "
                    f"{valid_count + invalid_count} logged GlobalPlanCost "
                    "records"
                )
            if in_capture and captures[h]["added"] != 0:
                errors.append(
                    f"hypothesis {h} has a cost-audit rejection but also "
                    f"reports {captures[h]['added']} pooled alternatives"
                )
            continue

        if not in_capture:
            errors.append(
                f"hypothesis {h} has complete-plan cost data but no "
                "GlobalTimePlanCapture record"
            )
            continue

        if captures[h]["added"] != valid_count:
            errors.append(
                f"hypothesis {h} pooled {captures[h]['added']} alternatives "
                f"but {valid_count} valid GlobalPlanCost records were "
                "logged"
            )
        if captures[h]["added"] == 0:
            errors.append(
                f"hypothesis {h} pooled zero alternatives without a "
                "GlobalTimePlanCostAuditReject record"
            )

    return errors, valid_by_hyp, invalid_by_hyp


def verify_text(text: str) -> list[str]:
    errors: list[str] = []
    if not re.search(
        r"\[PlanSelectionConfiguration\] mode=binding_cost "
        r"costConfigurationValid=true.*"
        r"eventTimePolicy=global_time_plan timeTerm=search_to_completion",
        text,
    ):
        errors.append("missing valid global binding-cost configuration")
    if "[GlobalTimePlanSearchConfiguration] enabled=true" not in text:
        errors.append("missing fixed global event-set configuration")
    if "[GlobalTimePlanCommitProof] committed=false" in text:
        errors.append("log contains a rejected global commit")

    costs = _parse_costs(text)
    captures = _parse_captures(text)
    cost_audit_rejects = _parse_cost_audit_rejects(text)
    geometry_rejects = _parse_geometry_rejects(text)
    timing_records = _parse_timing_admissibility(text)

    if not costs:
        errors.append("missing per-alternative global costs")

    reconcile_errors, valid_by_hyp, _invalid_by_hyp = _reconcile_hypotheses(
        costs, captures, cost_audit_rejects, geometry_rejects
    )
    errors.extend(reconcile_errors)

    valid_total = sum(valid_by_hyp.values())
    pooled_total = sum(capture["added"] for capture in captures.values())
    if pooled_total != valid_total:
        errors.append(
            f"pooled valid alternatives ({pooled_total}) do not match the "
            f"generated valid-cost count ({valid_total})"
        )

    selections = list(SELECTION_RE.finditer(text))
    commits = list(COMMIT_RE.finditer(text))
    if len(selections) != 1:
        errors.append(
            f"expected exactly one global argmin selection, found {len(selections)}"
        )
    if len(commits) != 1:
        errors.append(f"expected exactly one global commit, found {len(commits)}")
    if errors or len(selections) != 1 or len(commits) != 1:
        return errors

    selection = selections[0]
    commit = commits[0]

    def parsed(match: re.Match[str]) -> dict[str, object]:
        return {
            "evaluated": int(match.group(1)),
            "configured": int(match.group(2)),
            "complete": int(match.group(3)),
            "valid": int(match.group(4)),
            "admissible": int(match.group(5)),
            "hypothesis": int(match.group(6)),
            "event_lead": float(match.group(7)),
            "presentation": float(match.group(8)),
            "candidate": match.group(9),
            "route": match.group(10),
            "motion": float(match.group(11)),
            "wait": float(match.group(12)),
            "selected": float(match.group(13)),
            "minimum": float(match.group(14)),
            "tolerance": float(match.group(15)),
        }

    selected = parsed(selection)
    committed = parsed(commit)
    if selected["configured"] <= 0:
        errors.append("global schedule contains no hypotheses")
    if selected["evaluated"] != selected["configured"]:
        errors.append("global event schedule was not completely evaluated")
    if selected["complete"] <= 0:
        errors.append("global selector reports no complete alternatives")
    if selected["valid"] != selected["complete"]:
        errors.append("global selector cost set is incomplete")
    if selected["admissible"] <= 0:
        errors.append("global selector reports no final timing-admissible plan")
    if valid_total != selected["complete"]:
        errors.append(
            "independently counted valid GlobalPlanCost records do not "
            "match the selector's reported complete-plan count"
        )
    if pooled_total != selected["valid"]:
        errors.append(
            "final costValidPlans does not match the independently "
            "reconstructed pooled-alternative count"
        )

    valid_costs = [cost["global"] for cost in costs if cost["valid"]]
    if not valid_costs:
        errors.append(
            "no valid global-cost alternative available to independently "
            "recompute the minimum"
        )
    else:
        # The reported minimum is taken over the valid AND final-timing-
        # admissible subset. Per-candidate timing admissibility is not
        # logged in GlobalPlanCost (it depends on minimumReachEntryLead,
        # minimumSafeCommitLead and "now", none of which appear per
        # record), so it cannot be independently reconstructed here. A
        # cost-valid candidate with a cheaper globalJ that is simply not
        # timing-admissible is normal and must not fail the proof.
        # What is never legitimate, and IS soundly checkable without
        # admissibility data, is a reported minimum lower than every
        # logged valid record: admissible records are a subset of valid
        # records, so the true minimum can only be >= the unconstrained
        # valid minimum.
        valid_minimum = min(valid_costs)
        tolerance = max(float(selected["tolerance"]), 1e-12)
        if selected["minimum"] < valid_minimum - tolerance:
            errors.append(
                "reported minimum global J is lower than every logged "
                "valid alternative, which is impossible for a subset argmin"
            )

    winner_identity = (
        selected["hypothesis"],
        selected["candidate"],
        selected["route"],
    )
    winner_is_logged_invalid = any(
        (cost["hypothesis"], cost["candidate"], cost["route"]) == winner_identity
        and not cost["valid"]
        for cost in costs
    )
    winner_records = [
        cost["global"]
        for cost in costs
        if (cost["hypothesis"], cost["candidate"], cost["route"]) == winner_identity
        and cost["valid"]
    ]
    if winner_is_logged_invalid:
        errors.append("selected plan matches a record logged as cost-invalid")
    elif not winner_records:
        errors.append(
            "selected plan has no corresponding valid GlobalPlanCost record"
        )
    else:
        # This IS soundly verifiable without timing data: whatever the
        # committed candidate's own logged cost is, it must be the value
        # actually reported as selected/minimum, not a fabricated number.
        tolerance = max(float(selected["tolerance"]), 1e-12)
        if any(
            abs(cost_value - selected["selected"]) > tolerance
            for cost_value in winner_records
        ):
            errors.append(
                "selected global J does not match the winning candidate's "
                "own logged GlobalPlanCost record"
            )

    if timing_records:
        # Strong verification, only possible when the timing-admissibility
        # log is present: independently reconstruct F_J (pooled records are
        # cost-valid by construction) and F_timing from the exact quantities
        # selectFiniteEventPlan() used, then confirm the committed candidate
        # is the exact argmin over F_J intersect F_timing.
        non_cost_valid = [r for r in timing_records if not r["cost_valid"]]
        if non_cost_valid:
            errors.append(
                f"{len(non_cost_valid)} GlobalPlanTimingAdmissibility record(s) "
                "are cost-invalid; the pooled set must contain only cost-valid "
                "alternatives"
            )

        admissible_set = [r for r in timing_records if r["timing_admissible"]]
        if len(timing_records) != selected["complete"]:
            errors.append(
                "GlobalPlanTimingAdmissibility record count does not match "
                "the selector's reported complete-plan count"
            )
        if len(admissible_set) != selected["admissible"]:
            errors.append(
                "independently reconstructed timing-admissible count does "
                "not match the selector's reported timingAdmissiblePlans"
            )
        if not admissible_set:
            errors.append(
                "no timing-admissible alternative found in the "
                "GlobalPlanTimingAdmissibility records, but a plan was "
                "committed"
            )
        else:
            argmin_j = min(r["global_j"] for r in admissible_set)
            argmin_records = [
                r for r in admissible_set if abs(r["global_j"] - argmin_j) <= (
                    max(float(selected["tolerance"]), 1e-12)
                )
            ]
            argmin_identities = {
                (r["hypothesis"], r["candidate"], r["route"])
                for r in argmin_records
            }
            if winner_identity not in argmin_identities:
                errors.append(
                    "committed candidate is not the exact minimum-J "
                    "candidate over the independently reconstructed "
                    "cost-valid AND timing-admissible set"
                )
            tolerance = max(float(selected["tolerance"]), 1e-12)
            if abs(argmin_j - selected["selected"]) > tolerance:
                errors.append(
                    "independently reconstructed argmin over F_J intersect "
                    "F_timing does not match the selected global J"
                )

    if abs(selected["selected"] - selected["minimum"]) > (
        selected["tolerance"] + 1e-12
    ):
        errors.append("selected global J is outside the exact minimum tolerance")

    exact_fields = (
        "evaluated",
        "configured",
        "complete",
        "valid",
        "admissible",
        "hypothesis",
        "candidate",
        "route",
    )
    if any(selected[key] != committed[key] for key in exact_fields):
        errors.append("global selection and commit identities/counts differ")
    numeric_fields = (
        "event_lead",
        "presentation",
        "motion",
        "wait",
        "selected",
        "minimum",
    )
    tolerance = max(
        float(selected["tolerance"]), float(committed["tolerance"]), 1e-12
    )
    if any(
        abs(float(selected[key]) - float(committed[key])) > tolerance
        for key in numeric_fields
    ):
        errors.append("global selection and commit numerical proofs differ")

    if not re.search(
        r"\[GlobalTimePlanSearchSummary\] committed=true "
        r"scheduleComplete=true.*globalArgmin=true",
        text,
    ):
        errors.append("missing successful complete global-search summary")
    if "[Completed] full plan-once handover completed" not in text:
        errors.append("global plan committed but full simulated handover did not complete")

    # Only applies when the log carries per-candidate CompletePlanCost detail
    # (every real simulation run does; minimal synthetic GlobalPlanCost-only
    # fixtures used elsewhere in this test suite intentionally do not, and
    # must not be forced to grow this unrelated detail).
    if "[CompletePlanCost]" in text:
        errors.extend(verify_frozen_objective(text))
    return errors


def verify_failsafe_text(text: str) -> list[str]:
    """Verify a log in which no cost-valid admissible alternative existed
    anywhere in the bounded schedule: no commit, no completed handover, and
    a genuine fail-closed reason, independently corroborated by the
    per-hypothesis records.
    """
    errors: list[str] = []
    if "[GlobalTimePlanCommitProof] committed=true" in text:
        errors.append("fail-safe log unexpectedly contains a global commit")
    if "[Completed] full plan-once handover completed" in text:
        errors.append("fail-safe log unexpectedly reports a completed handover")

    failures = list(FAILSAFE_SELECTION_RE.finditer(text))
    if len(failures) != 1:
        errors.append(
            f"expected exactly one failed global selection, found {len(failures)}"
        )
        return errors

    failure = failures[0]
    reason = failure.group(1)
    cost_valid_plans = int(failure.group(6))
    if reason not in ALLOWED_FAILSAFE_REASONS:
        errors.append(f"unexpected global fail-safe reason '{reason}'")
    if cost_valid_plans != 0:
        errors.append(
            f"fail-safe selection reports {cost_valid_plans} cost-valid "
            "alternatives instead of zero"
        )

    costs = _parse_costs(text)
    captures = _parse_captures(text)
    cost_audit_rejects = _parse_cost_audit_rejects(text)
    geometry_rejects = _parse_geometry_rejects(text)

    reconcile_errors, valid_by_hyp, _invalid_by_hyp = _reconcile_hypotheses(
        costs, captures, cost_audit_rejects, geometry_rejects
    )
    errors.extend(reconcile_errors)

    if sum(valid_by_hyp.values()) != 0:
        errors.append("fail-safe run contains a cost-valid GlobalPlanCost record")
    for h, capture in captures.items():
        if capture["added"] != 0:
            errors.append(
                f"hypothesis {h} pooled {capture['added']} alternatives in "
                "a fail-safe run"
            )

    if "[CompletePlanCost]" in text:
        errors.extend(verify_frozen_objective(text))
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument(
        "--expect-failsafe",
        action="store_true",
        help="verify a log expected to fail closed with no valid global alternative",
    )
    args = parser.parse_args()
    text = args.log.read_text(encoding="utf-8", errors="replace")
    errors = (
        verify_failsafe_text(text) if args.expect_failsafe else verify_text(text)
    )
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("global time-grasp-route runtime log proof: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
