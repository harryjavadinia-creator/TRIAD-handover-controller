#!/usr/bin/env python3
"""Summarize a TRIAD V1/V2 evidence directory.

Layout expected (produced by the evidence batch):
  <dir>/v1/<scenario>/<scenario>.log              V1, giver follows commit (regression)
  <dir>/v1_independent/<scenario>/<scenario>.log  V1 against the independent giver
  <dir>/v2/<scenario>/<scenario>.log              V2 receding receiver
  <dir>/v2_repeat/<scenario>/<scenario>.log       V2 repeat
  <dir>/v2_inject_stale/<scenario>/<scenario>.log V2 with injected stale terminal results
  <dir>/v2_inject_supersede/<scenario>/<scenario>.log V2 with an injected supersession of the first search

Outputs markdown to stdout: V1 regression, V2 invariant/coverage table,
cross-run giver independence, and the V1-independent vs V2 comparison. Exit 1
when a V2 invariant fails on any completed V2 run, when required coverage is
missing, when cross-run giver truth differs, or when a V1 FrozenPlanSet hash
differs from the recorded evidence.

Metrics that a log does not contain are reported as n/a; nothing is estimated.
"""

import contextlib
import hashlib
import io
import json
import re
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import check_giver_truth_independence as giver  # noqa: E402
import check_v2_run_log as v2check  # noqa: E402

SCENARIOS = ["longitudinal", "near-ground", "lateral-low", "diagonal"]


def frozen_hash(log):
    rows = [l.rstrip("\n") for l in open(log, errors="replace")
            if "[FrozenPlanRecord]" in l or "[FrozenPlanSetSummary]" in l]
    body = "\n".join(rows) + ("\n" if rows else "")
    return hashlib.sha256(body.encode()).hexdigest(), sum("[FrozenPlanRecord]" in r for r in rows)


def num(line, key):
    m = re.search(r"\b" + re.escape(key) + r"=([-+0-9.eE]+)", line or "")
    return float(m.group(1)) if m else None


def giver_elapsed_before(lines, marker):
    last = None
    for l in lines:
        m = re.search(r"\[GiverTruthSample\] elapsed=([-0-9.]+)", l)
        if m:
            last = float(m.group(1))
        if marker in l:
            return last
    return None


def failure_reason(lines):
    """Last [error] line before the FSM enters Failure (the cause), not the
    first diagnostic error, which can be a non-fatal within-event message."""
    last = None
    for l in lines:
        if "Starting state HandoverInterceptionController_Failure" in l:
            break
        if "[error]" in l:
            last = l
    if last is None:
        return None
    m = re.search(r"\[error\] \[(\w+)\](.*?)(?:reason=(\S+))?(?:;|$)", last)
    if m and m.group(3):
        return f"{m.group(1)}:{m.group(3)}"
    body = last.split("] ", 2)[-1]
    return re.sub(r"\s+", " ", body)[:90]


def run_metrics(log, arch):
    lines = open(log, errors="replace").read().splitlines()
    completed = any("[Completed] full plan-once handover completed" in l for l in lines)
    if arch == "v1":
        commit = any("[GlobalTimePlanCommitProof] committed=true" in l for l in lines)
        latency = [num(l, "workerWall") for l in lines if "[PlannerWorkerResult]" in l]
        gens = len(latency)
        switches = 0
        clearance = [num(l, "minimumClearance") for l in lines if "[PredictiveReachGovernor] completed" in l]
        drift = [num(l, "translationDrift") for l in lines if "[GlobalTimePlanCommitFreshness] accepted=true" in l]
        model_err = [num(l, "objectModelError") for l in lines if "[PredictiveReach]" in l and "objectModelError=" in l]
        deviation = max(model_err) if model_err else (drift[0] if drift else None)
        concurrent = None
    else:
        commit = any("[V2TerminalCommit] committed=true" in l for l in lines)
        latency = [num(l, "latency") for l in lines if "[V2PlanningJobResult] type=FULL_SEARCH" in l]
        gens = sum("[V2PlanningJobSubmit]" in l for l in lines)
        switches = sum("[V2ProvisionalAdopt]" in l and "kind=REPLACEMENT" in l for l in lines)
        summary = [l for l in lines if "[V2ReceiverSummary]" in l]
        clearance = [num(summary[-1], "minRuntimeClearance")] if summary else []
        updates = [num(l, "presentationUpdate") for l in lines if "[V2ProvisionalRetain]" in l]
        commit_line = next((l for l in lines if "[V2TerminalCommit] committed=true" in l), "")
        deviation = max(updates) if updates else None
        concurrent = sum("[V2Motion]" in l and "robotMoving=true" in l and "objectMoving=true" in l for l in lines)
    capture_t = giver_elapsed_before(lines, "[CaptureTransferBoundary] bilateral grasp confirmed")
    capture = any("[CaptureTransferBoundary] bilateral grasp confirmed" in l for l in lines)
    transfer = any("[ForceTransferComplete] robot support established" in l for l in lines)
    retreat = any("[Retreat] carried object reached certified retreat pose" in l for l in lines)
    return {
        "completed": completed,
        "commit": commit,
        "failure": None if completed else failure_reason(lines),
        "planningLatency": latency,
        "planningGenerations": gens,
        "provisionalSwitches": switches,
        "minClearance": min([c for c in clearance if c is not None], default=None),
        "deviationTolerated": deviation,
        "timeToCaptureSinceGiverStart": capture_t,
        "postCommitFailure": commit and not completed,
        "capture": capture,
        "transfer": transfer,
        "retreat": retreat,
        "concurrentMotionSamples": concurrent,
    }


def fmt(v, digits=3):
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    if isinstance(v, list):
        return "[" + ", ".join(fmt(x, 2) for x in v if x is not None) + "]" if v else "n/a"
    return str(v)


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    base = Path(argv[1])
    bad = []
    out = []

    out.append("## V1 regression (FrozenPlanSet sha256, default configuration)\n")
    out.append("| scenario | recorded evidence | this run | records | identical | completed |")
    out.append("|---|---|---|---:|---|---|")
    for s in SCENARIOS:
        log = base / "v1" / s / f"{s}.log"
        if not log.is_file():
            out.append(f"| {s} | | missing | | | |")
            bad.append(f"v1 {s} missing")
            continue
        rec = (ROOT / "evidence" / "async" / s / "frozen_plan_set_sha.txt").read_text().split()[0]
        h, n = frozen_hash(log)
        same = h == rec
        if not same:
            bad.append(f"v1 regression {s}")
        done = any("[Completed] full plan-once" in l for l in open(log, errors="replace"))
        out.append(f"| {s} | `{rec[:16]}…` | `{h[:16]}…` | {n} | {'YES' if same else 'NO'} | {fmt(done)} |")

    out.append("\n## V2 invariants and demonstrations\n")
    out.append("| run | completed | I1 | I2 | I3 | I4 | I5 | I6 | I7 | I8 | I9 | D1 concurrent motion | D2 gens while moving | D3 update/replacement | D4 stale rejected |")
    out.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    coverage = {"D1": [], "D2": [], "D3_replacement": [], "D3_update": [], "D4": [], "cancel": []}
    for tag in ("v2", "v2_repeat", "v2_inject_stale", "v2_inject_supersede"):
        for s in SCENARIOS:
            log = base / tag / s / f"{s}.log"
            if not log.is_file():
                continue
            js = base / tag / s / "v2_check.json"
            with contextlib.redirect_stdout(io.StringIO()):
                v2check.main(["x", str(log), "--json", str(js)])
            r = json.loads(js.read_text())
            completed = r["metrics"]["completed"]
            inv = {k: r[k]["pass"] for k in r if k.startswith("I")}
            if completed and not all(inv.values()):
                bad.append(f"{tag} {s} invariant failure on completed run: "
                           + ",".join(k for k, v in inv.items() if not v))
            # Invariants that must hold even on failed runs.
            for k in ("I1_architecture", "I4_no_closure_authority_before_commit"):
                if not r[k]["pass"]:
                    bad.append(f"{tag} {s} {k}")
            if not completed and "planDrivenTruth=0" not in r["I2_giver_truth_independence"]["detail"]:
                bad.append(f"{tag} {s} giver truth")
            d = {k: r[k]["observed"] for k in r if k.startswith("D")}
            name = f"{tag}/{s}"
            if d["D1_robot_moves_while_object_moves"]:
                coverage["D1"].append(name)
            if d["D2_generations_while_robot_moving"]:
                coverage["D2"].append(name)
            det3 = r["D3_provisional_update_or_replacement"]["detail"]
            if not re.search(r"replacements=0\b", det3):
                coverage["D3_replacement"].append(name)
            if not re.search(r"retainsWithPredictionUpdate=0\b", det3):
                coverage["D3_update"].append(name)
            if d["D4_stale_results_rejected"]:
                coverage["D4"].append(name)
            if r["metrics"].get("cancelled"):
                coverage["cancel"].append(name)
            cells = ["PASS" if inv[k] else ("FAIL" if completed else "—") for k in sorted(inv)]
            out.append(f"| {name} | {fmt(completed)} | " + " | ".join(cells) + " | "
                       + " | ".join("yes" if d[k] else "no" for k in sorted(d)) + " |")
    out.append("\n(— = invariant not applicable because the run failed closed before commitment.)\n")
    out.append("### Required coverage across the V2 evidence set\n")
    for key, label in (("D1", "robot moves while object moves"),
                       ("D2", "planning generations while robot moves"),
                       ("D3_replacement", "provisional plan replaced before commitment"),
                       ("D3_update", "provisional plan retained with prediction update"),
                       ("D4", "stale worker results rejected"),
                       ("cancel", "superseded generations cancelled without effect (I9)")):
        ok = bool(coverage[key])
        if not ok:
            bad.append(f"coverage {key}")
        out.append(f"- {'COVERED' if ok else 'MISSING'}: {label} — {', '.join(coverage[key]) or 'none'}")

    out.append("\n## Cross-run giver truth independence (V1-independent vs V2, same scenario)\n")
    for s in SCENARIOS:
        a = base / "v1_independent" / s / f"{s}.log"
        b = base / "v2" / s / f"{s}.log"
        if a.is_file() and b.is_file():
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                ok = giver.cross_check(str(a), str(b))
            if not ok:
                bad.append(f"cross giver {s}")
            out.append(f"- {buf.getvalue().strip()}")

    out.append("\n## First architectural comparison: V1 frozen pre-reach vs V2 receding (independent giver)\n")
    cols = ["completed", "commit", "failure", "planningLatency", "planningGenerations", "provisionalSwitches",
            "minClearance", "deviationTolerated", "timeToCaptureSinceGiverStart", "postCommitFailure",
            "capture", "transfer", "retreat", "concurrentMotionSamples"]
    out.append("| run | " + " | ".join(cols) + " |")
    out.append("|---|" + "---|" * len(cols))
    for tag, arch in (("v1_independent", "v1"), ("v2", "v2"), ("v2_repeat", "v2")):
        for s in SCENARIOS:
            log = base / tag / s / f"{s}.log"
            if not log.is_file():
                continue
            m = run_metrics(log, arch)
            out.append(f"| {tag}/{s} | " + " | ".join(fmt(m[c]) for c in cols) + " |")
    out.append("\nUnits: latency s (V1: worker wall of the single search; V2: latency of each full search), "
               "clearance m (live gripper clearance during reach/provisional motion), deviation m (V1: largest "
               "object-model error tolerated during committed reach; V2: largest retained prediction update), "
               "time to capture s since giver start (last truth sample before bilateral grasp confirmation).")

    print("\n".join(out))
    print(f"\nEVIDENCE SUMMARY: {'PASS' if not bad else 'FAIL'}")
    for b in bad:
        print(f"  - {b}")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
