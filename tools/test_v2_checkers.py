#!/usr/bin/env python3
"""Mutation tests: the V2 checkers must fail on injected violations.

  * worker snapshot purity fails when the V2 worker reads the live world
  * giver independence (static) fails when the truth path reads a commit
  * V2 run-log checker fails on a second commitment, on post-commit
    reselection, on closure authority before commitment, and on a stale or
    cancelled generation affecting adoption or commitment, and on adoption
    from an uncertified re-certification or outside the freshness tube
"""
import contextlib
import io
import re
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import check_giver_truth_independence as giver  # noqa: E402
import check_v2_run_log as runlog  # noqa: E402
import check_worker_snapshot_purity as purity  # noqa: E402

failures = 0


def expect(name, ok):
    global failures
    print(f"{'PASS' if ok else 'FAIL'} {name}")
    failures += 0 if ok else 1


def quiet(fn, *args):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*args)


with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    # 1. worker purity mutation
    v2 = tmp / "ReceiverV2.cpp"
    text = (ROOT / "src" / "ReceiverV2.cpp").read_text()
    mutated = text.replace(
        "  const sva::PTransformd objectPose = request.terminalObjectPose;",
        "  const sva::PTransformd objectPose = W_T_O_;", 1)
    assert mutated != text
    v2.write_text(mutated)
    purity.SOURCES = [purity.core.CPP, v2]
    expect("worker purity rejects a live world read in the V2 worker", quiet(purity.main) == 1)
    purity.SOURCES = [purity.core.CPP, ROOT / "src" / "ReceiverV2.cpp"]
    expect("worker purity passes on the real source", quiet(purity.main) == 0)

    # 2. giver independence static mutation
    cpp = tmp / "HandoverInterceptionController.cpp"
    ctext = (ROOT / "src" / "HandoverInterceptionController.cpp").read_text()
    cmut = ctext.replace(
        "const double elapsed = controllerTime_ - independentGiverStartTime_;",
        "const double elapsed = committedInterceptionPlan_.presentationTime - independentGiverStartTime_;", 1)
    assert cmut != ctext
    cpp.write_text(cmut)
    giver.CPP = cpp
    expect("giver independence rejects a commit-dependent truth", quiet(giver.static_check) is False)
    giver.CPP = ROOT / "src" / "HandoverInterceptionController.cpp"
    expect("giver independence passes on the real source", quiet(giver.static_check) is True)

    # 3. run-log checker mutations on a real passing log, if available
    import os
    reference = os.environ.get("TRIAD_V2_REFERENCE_LOG", "")
    base = Path(reference) if reference and Path(reference).is_file() else None
    if base is None:
        print("SKIP run-log mutations (set TRIAD_V2_REFERENCE_LOG to a passing V2 run log)")
    else:
        lines = base.read_text(errors="replace").splitlines()
        ci = next(i for i, l in enumerate(lines) if "[V2TerminalCommit] committed=true" in l)
        expect("run-log checker passes on the unmodified log",
               quiet(runlog.main, ["x", str(base)]) == 0)
        cases = {
            "second commitment": lines[:ci + 1] + [lines[ci]] + lines[ci + 1:],
            "post-commit reselection": lines[:ci + 1] + [
                "[success] [V2ProvisionalAdopt] planId=9 kind=REPLACEMENT t=99.0"] + lines[ci + 1:],
            "closure authority before commit": [
                re.sub("closureAuthorized=false", "closureAuthorized=true", l) if "[V2Motion]" in l and i < ci else l
                for i, l in enumerate(lines)],
            "stale certificate used for commit": [
                l if i != ci else l + "" for i, l in enumerate(lines)] [:ci] + [
                "[warning] [V2StaleResultRejected] type=TERMINAL_CERTIFY planningGeneration=" +
                re.search(r"certificatePlanningGeneration=(\d+)", lines[ci]).group(1) +
                " canCommit=false canReplacePlan=false"] + lines[ci:],
            "cancelled generation adopted": lines[:ci] + [
                "[warning] [V2JobCancelRequested] type=FULL_SEARCH planningGeneration=999999 reason=prediction_superseded",
                "[warning] [V2JobCancelled] type=FULL_SEARCH planningGeneration=999999 effect=none canCommit=false canReplacePlan=false",
                "[success] [V2ProvisionalAdopt] planId=8 kind=REPLACEMENT sourcePlanningGeneration=999999 t=1.0"] + lines[ci:],
            "cancelled certificate used for commit": lines[:ci] + [
                "[warning] [V2JobCancelled] type=TERMINAL_CERTIFY planningGeneration=" +
                re.search(r"certificatePlanningGeneration=(\d+)", lines[ci]).group(1) +
                " effect=none canCommit=false canReplacePlan=false"] + lines[ci:],
            "certified adoption without a successful certificate": lines[:ci] + [
                "[info] [V2AdoptFreshness] planningGeneration=1 hypothesis=1 source=certified certificateGeneration=999998 fresh=true",
                "[success] [V2ProvisionalAdopt] planId=7 kind=REPLACEMENT sourcePlanningGeneration=1 adoptionSource=certified adoptionCertificateGeneration=999998 t=1.0"] + lines[ci:],
            "re-selection of a record whose certificate failed": lines[:ci] + [
                "[success] [V2PlanningJobResult] type=FULL_SEARCH planningGeneration=999990 failed=false",
                "[info] [V2SelectedTargetMoved] searchGeneration=999990 record=5 action=certify_selected",
                "[info] [V2SelectedCertification] certificateGeneration=999991 searchGeneration=999990 record=5 success=false reason=x",
                "[info] [V2SelectedTargetMoved] searchGeneration=999990 record=5 action=certify_selected"] + lines[ci:],
            "adoption outside the freshness tube": [
                l.replace("fresh=true", "fresh=false") if "[V2AdoptFreshness]" in l else l for l in lines],
        }
        for name, content in cases.items():
            path = tmp / (name.replace(" ", "_") + ".log")
            path.write_text("\n".join(content))
            expect(f"run-log checker rejects {name}", quiet(runlog.main, ["x", str(path)]) == 1)

print(f"V2 checker mutation tests: {'PASS' if failures == 0 else 'FAIL'} ({failures} failure(s))")
sys.exit(1 if failures else 0)
