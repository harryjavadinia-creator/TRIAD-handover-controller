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
