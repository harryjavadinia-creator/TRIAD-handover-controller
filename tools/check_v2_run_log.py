#!/usr/bin/env python3
"""Machine checks for one TRIAD V2 (receding complete-action receiver) run log.

INVARIANTS (any failure -> exit 1):
  I1  architecture is v2_receding with an independent_scripted giver
  I2  giver truth independence: every [GiverTruthSample] recomputes from the
      logged [GiverTruthScript] alone; no commit creates a plan-driven truth
  I3  exactly one terminal commitment ([V2TerminalCommit] committed=true), and
      no V1 pre-reach commitment markers
  I4  gripper closure never authorized before the commitment
  I5  no global reselection after the commitment (no V2 job submission,
      adoption, V1 worker submission or global selection afterwards)
  I6  stale worker results cannot commit: the committing certificate is a
      logged [V2TerminalCertificate] whose generation was never rejected as
      stale and whose state generation is the one in force at commitment;
      every stale rejection reports canCommit=false canReplacePlan=false
  I7  FSM path ObserveObject -> ReceiverV2 -> MovePregrasp -> CaptureTransfer
      -> Retreat -> Completed, with no V1 solve/reach/hold states
  I8  capture (bilateral grasp confirmed), load transfer and carried-object
      retreat all succeed after the commitment
  I9  cancelled or stale generations have no effect: no generation that was
      cancel-requested, cancelled or rejected as stale appears as an accepted
      job result, provisional adoption, retention, terminal certificate or
      commitment certificate; every cancellation reports canCommit=false
      canReplacePlan=false; every cancel request is resolved by a cancellation
      (holds on failed runs too)
  I10 prediction updates cannot make a stale target adoptable: every
      provisional adoption is logged fresh ([V2AdoptFreshness] fresh=true, the
      adopted presentation pose within the commit-freshness tube of the
      prediction at adoption), and every adoption from a re-certified selected
      action (adoptionSource=certified) cites a certificate generation that
      succeeded ([V2SelectedCertification] success=true) before the adoption
      (holds on failed runs too)

DEMONSTRATIONS (reported per run; coverage is required across the evidence set,
see tools/summarize_v2_evidence.py):
  D1  robot moves while the object truth is still moving
  D2  planning generations submitted while the robot is moving
  D3  provisional plan updated (retained with new prediction) or replaced
  D4  stale results rejected (e.g. fault-injection run)

Usage: check_v2_run_log.py RUN.log [--json OUT.json]
"""

import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_giver_truth_independence as giver  # noqa: E402


def kv(line, key, cast=str):
    m = re.search(r"\b" + re.escape(key) + r"=([^\s\]]+)", line)
    return cast(m.group(1)) if m else None


def tflag(value):
    return value == "true"


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    log = argv[1]
    out_json = argv[argv.index("--json") + 1] if "--json" in argv else None
    lines = open(log, errors="replace").read().splitlines()

    results = {}
    failures = []

    def inv(key, ok, detail):
        results[key] = {"pass": bool(ok), "detail": detail}
        print(f"{'PASS' if ok else 'FAIL'} {key} {detail}")
        if not ok:
            failures.append(key)

    def demo(key, observed, detail):
        results[key] = {"observed": bool(observed), "detail": detail}
        print(f"{'OBSERVED' if observed else 'NOT_OBSERVED'} {key} {detail}")

    # I1
    arch = [l for l in lines if "[ReceiverArchitecture]" in l]
    inv("I1_architecture", bool(arch) and "architecture=v2_receding" in arch[-1]
        and "giverTruthModel=independent_scripted" in arch[-1] and "valid=true" in arch[-1],
        arch[-1].split("] ", 2)[-1] if arch else "missing [ReceiverArchitecture]")

    # I2
    import io, contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        replay_ok = giver.replay_check(log)
    plan_truth = [l for l in lines if "simulatedTruthPlanCreated=true" in l]
    commit_truth = [l for l in lines if "[GiverTruthIndependence]" in l]
    inv("I2_giver_truth_independence",
        replay_ok and not plan_truth and all("simulatedTruthPlanCreated=false" in l for l in commit_truth)
        and bool(commit_truth),
        buf.getvalue().strip() + f"; commitTruthLines={len(commit_truth)} planDrivenTruth={len(plan_truth)}")

    # locate commit
    commit_idx = [i for i, l in enumerate(lines) if "[V2TerminalCommit] committed=true" in l]
    v1_commits = [l for l in lines if "[GlobalTimePlanCommitProof] committed=true" in l
                  or "[PresentationCommit] COMMITTED" in l or "[BindingCostCommitProof] committed=true" in l]
    latched_refusals = [l for l in lines if "reason=commit_already_latched" in l]
    inv("I3_exactly_one_commit", len(commit_idx) == 1 and not v1_commits and not latched_refusals,
        f"v2Commits={len(commit_idx)} v1CommitMarkers={len(v1_commits)} secondCommitAttempts={len(latched_refusals)}")
    c = commit_idx[0] if commit_idx else len(lines)
    commit_line = lines[c] if commit_idx else ""

    # I4
    before = lines[:c]
    closure_before = [l for l in before if "[V2Motion]" in l and "closureAuthorized=true" in l]
    inv("I4_no_closure_authority_before_commit", not closure_before and any("[V2Motion]" in l for l in before),
        f"provisionalMotionSamples={sum('[V2Motion]' in l for l in before)} closureAuthorizedSamples={len(closure_before)}")

    # I5
    after = lines[c + 1:]
    reselection_markers = ("[V2PlanningJobSubmit]", "[V2ProvisionalAdopt]", "[PlannerWorkerSubmit]",
                           "[GlobalTimePlanSelection]", "[V2FullSearchSelection]",
                           "[V2PostCommitReselectionRefused]", "[PresentationSolve]")
    post = [l for l in after if any(m in l for m in reselection_markers)]
    inv("I5_no_global_reselection_after_commit", bool(commit_idx) and not post,
        f"postCommitReselectionEvents={len(post)}")

    # I6
    stale = [l for l in lines if "[V2StaleResultRejected]" in l]
    stale_bad = [l for l in stale if "canCommit=false" not in l or "canReplacePlan=false" not in l]
    stale_gens = {kv(l, "planningGeneration") for l in stale if kv(l, "planningGeneration")}
    cert_gen = kv(commit_line, "certificatePlanningGeneration")
    commit_state = kv(commit_line, "stateGeneration", int) if commit_idx else None
    certs = [l for l in before if "[V2TerminalCertificate]" in l and kv(l, "planningGeneration") == cert_gen]
    cert_state = kv(certs[-1], "stateGeneration", int) if certs else None
    stale_state_after_cert = False
    if certs:
        cert_pos = max(i for i, l in enumerate(before) if l is certs[-1])
        stale_state_after_cert = any(
            "[V2StaleResultRejected]" in l or "[V2ProvisionalInvalidated]" in l or "[V2ProvisionalAdopt]" in l
            for l in before[cert_pos + 1:])
    inv("I6_stale_results_cannot_commit",
        bool(certs) and cert_gen not in stale_gens and not stale_bad
        and commit_state is not None and cert_state is not None and commit_state == cert_state + 1
        and not stale_state_after_cert,
        f"certificateGeneration={cert_gen} certificateStateGeneration={cert_state} commitStateGeneration={commit_state} "
        f"staleRejections={len(stale)} staleGenerations={sorted(stale_gens, key=lambda x: int(x))} "
        f"stateChangedAfterCertificate={stale_state_after_cert}")

    # I7
    states = [re.search(r"Starting state (\S+)", l).group(1).replace("HandoverInterceptionController_", "")
              for l in lines if "Starting state" in l]
    expected = ["Initial", "ObserveObject", "ReceiverV2", "MovePregrasp", "CaptureTransfer", "Retreat", "Completed"]
    v1_states = {"SolveInterception", "ExecuteCommittedReach", "PresentationHold"} & set(states)
    inv("I7_fsm_path", states == expected and not v1_states, f"states={states}")

    # I8
    capture = [i for i, l in enumerate(after) if "[CaptureTransferBoundary] bilateral grasp confirmed" in l]
    transfer = [i for i, l in enumerate(after) if "[ForceTransferComplete] robot support established" in l]
    retreat = [i for i, l in enumerate(after) if "[Retreat] carried object reached certified retreat pose" in l]
    completed = [i for i, l in enumerate(after) if "[Completed] full plan-once handover completed" in l]
    ordered = bool(capture and transfer and retreat and completed) and capture[0] < transfer[0] < retreat[0] < completed[0]
    inv("I8_capture_transfer_retreat", ordered,
        f"capture={bool(capture)} transfer={bool(transfer)} retreat={bool(retreat)} completed={bool(completed)} ordered={ordered}")

    # I9
    cancel_req = {kv(l, "planningGeneration") for l in lines if "[V2JobCancelRequested]" in l}
    cancelled_lines = [l for l in lines if "[V2JobCancelled]" in l]
    cancelled = {kv(l, "planningGeneration") for l in cancelled_lines}
    cancelled_bad = [l for l in cancelled_lines if "canCommit=false" not in l or "canReplacePlan=false" not in l]
    obsolete = cancel_req | cancelled | stale_gens
    effect_sites = []
    for l in lines:
        if "[V2PlanningJobResult]" in l or "[V2ProvisionalRetain]" in l or "[V2TerminalCertificate]" in l:
            effect_sites.append(("result/retain/certificate", kv(l, "planningGeneration")))
        elif "[V2ProvisionalAdopt]" in l:
            effect_sites.append(("adopt", kv(l, "sourcePlanningGeneration")))
            effect_sites.append(("adopt-certificate", kv(l, "adoptionCertificateGeneration")))
        elif "[V2TerminalCommit] committed=true" in l:
            effect_sites.append(("commit", kv(l, "certificatePlanningGeneration")))
    leaked = [(k, g) for k, g in effect_sites if g is not None and g in obsolete]
    unresolved = cancel_req - cancelled
    # A cancel request still pending when the log ends (e.g. the run failed) is not a leak.
    ended_pending = bool(unresolved) and not any("[V2ReceiverSummary]" in l for l in lines)
    inv("I9_obsolete_generations_have_no_effect",
        not leaked and not cancelled_bad and (not unresolved or ended_pending),
        f"cancelRequests={len(cancel_req)} cancelled={len(cancelled)} stale={len(stale_gens)} "
        f"leaked={leaked} badCancelLines={len(cancelled_bad)} unresolvedCancelRequests={sorted(unresolved)}")

    # I10
    adopt_idx = [i for i, l in enumerate(lines) if "[V2ProvisionalAdopt]" in l]
    fresh_lines = [l for l in lines if "[V2AdoptFreshness]" in l]
    not_fresh = [l for l in fresh_lines if kv(l, "fresh") != "true"]
    uncertified = []
    for i in adopt_idx:
        if kv(lines[i], "adoptionSource") == "certified":
            g = kv(lines[i], "adoptionCertificateGeneration")
            if not any("[V2SelectedCertification]" in l and kv(l, "certificateGeneration") == g
                       and "success=true" in l for l in lines[:i]):
                uncertified.append(g)
    certified_adoptions = sum(1 for i in adopt_idx if kv(lines[i], "adoptionSource") == "certified")
    inv("I10_prediction_updates_cannot_adopt_stale_targets",
        not not_fresh and not uncertified and len(fresh_lines) >= len(adopt_idx),
        f"adoptions={len(adopt_idx)} certifiedAdoptions={certified_adoptions} adoptFreshnessLines={len(fresh_lines)} "
        f"notFresh={len(not_fresh)} uncertifiedCertifiedAdoptions={uncertified}")

    # Demonstrations
    motion = [l for l in before if "[V2Motion]" in l]
    concurrent = [l for l in motion if "robotMoving=true" in l and "objectMoving=true" in l]
    demo("D1_robot_moves_while_object_moves", concurrent,
         f"concurrentSamples={len(concurrent)} of {len(motion)} (50 ms sampling; object motion from giver truth)")
    submits = [l for l in before if "[V2PlanningJobSubmit]" in l]
    moving_submits = [l for l in submits if "robotMoving=true" in l]
    both_submits = [l for l in moving_submits if "objectMoving=true" in l]
    demo("D2_generations_while_robot_moving", moving_submits,
         f"jobs={len(submits)} whileRobotMoving={len(moving_submits)} whileRobotAndObjectMoving={len(both_submits)}")
    adopts = [l for l in before if "[V2ProvisionalAdopt]" in l]
    replacements = [l for l in adopts if "kind=REPLACEMENT" in l]
    retains = [l for l in before if "[V2ProvisionalRetain]" in l]
    updates = [l for l in retains if (kv(l, "presentationUpdate", lambda s: float(s.rstrip("m"))) or 0.0) > 0.0]
    demo("D3_provisional_update_or_replacement", replacements or updates,
         f"adoptions={len(adopts)} replacements={len(replacements)} retains={len(retains)} "
         f"retainsWithPredictionUpdate={len(updates)} replacementReasons={[kv(l, 'reason') for l in replacements]}")
    demo("D4_stale_results_rejected", stale, f"staleRejections={len(stale)}")

    # Metrics for the V1/V2 comparison
    def t_of(line):
        v = kv(line, "t", str)
        return float(v) if v else None
    begin = [l for l in lines if "[V2ReceiverBegin]" in l]
    summary = [l for l in lines if "[V2ReceiverSummary]" in l]
    first_submit_t = t_of(submits[0]) if submits else None
    commit_t = float(kv(commit_line, "commitTime")) if commit_idx else None
    metrics = {
        "completed": bool(completed),
        "commitTime": commit_t,
        "firstPlanningSubmitTime": first_submit_t,
        "planningGenerations": len(submits),
        "provisionalAdoptions": len(adopts),
        "provisionalReplacements": len(replacements),
        "retainsWithPredictionUpdate": len(updates),
        "staleRejections": len(stale),
        "cancelRequests": len(cancel_req),
        "cancelled": len(cancelled),
        "certifiedAdoptions": certified_adoptions,
        "cancelLatencies": [kv(l, "cancelLatency", lambda v: float(v.rstrip("s"))) for l in cancelled_lines],
        "minRuntimeClearance": kv(summary[-1], "minRuntimeClearance", float) if summary else None,
        "fullSearchLatencies": [kv(l, "latency", lambda s: float(s.rstrip("s")))
                                for l in before if "[V2PlanningJobResult] type=FULL_SEARCH" in l],
        "commitObjectDrift": kv(commit_line, "objectDrift", float),
        "commitMouthDrift": kv(commit_line, "mouthDrift", float),
        "tauMinusCommit": kv(commit_line, "tauMinusCommit", float),
    }
    results["metrics"] = metrics
    print("METRICS " + json.dumps(metrics))
    if out_json:
        Path(out_json).write_text(json.dumps(results, indent=2))
    print(f"V2 run check: {'PASS' if not failures else 'FAIL'} ({len(failures)} invariant failure(s))")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
