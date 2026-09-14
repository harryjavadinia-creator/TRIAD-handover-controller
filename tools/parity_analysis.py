#!/usr/bin/env python3
"""Final TRIAD Phase 2B: preview/runtime parity from parityTrace runs.

Usage: parity_analysis.py OUT.json LOG [LOG ...]   (then: parity_analysis.py report OUT.json)

Per executed plan (V2 log with ReceiverV2 parityTrace: true):

REACH (copied-state rollout of the last RECERTIFY_ACTIVE from reach start,
logged at reach start, versus the 100 Hz runtime trace of that plan):
  - time-aligned path deviation |p_run(t) - p_prev(t)| and orientation deviation
    over [reachStart, min(standoffTime, end of the plan's execution)]
  - pose error to the certified standoff at standoffTime (preview vs runtime)
  - arrival time inside the reach tolerances (0.012 m / 0.050 rad), preview vs runtime
  - minimum clearance (preview samples vs runtime current-pose clearance)
  - how the plan ended (reached standoff / invalidated / replaced) and the largest
    presentation update the plan received while executing (prediction confound)

TERMINAL (terminal certificate used for commitment, logged at commit, versus the
post-commit runtime):
  - predicted insertion / acquire / carried retreat / total durations vs the
    runtime [TimingSummary] phase durations
  - spatial deviation of the runtime insertion and retreat paths from the preview
    polyline, end-pose errors, retreat clearance
"""

import json
import lzma
import math
import re
import sys
from collections import defaultdict

import numpy as np

KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")
POS_TOL, ORI_TOL = 0.012, 0.050


def kv(line):
    i = line.find("] [")
    return dict(KV.findall(line[i:] if i >= 0 else line))


def vec(s):
    return np.array([float(x) for x in s.strip("[]").split(",")])


def fl(s):
    return float(str(s).rstrip("sm"))


def qangle(qa, qb):
    d = abs(float(np.dot(qa / np.linalg.norm(qa), qb / np.linalg.norm(qb))))
    return 2 * math.acos(min(1.0, d))


def read(path):
    op = lzma.open if path.endswith(".xz") else open
    with op(path, "rt", errors="replace") as f:
        return f.read().splitlines()


def polyline_distance(points, poly):
    if len(poly) < 2:
        return [float(np.linalg.norm(p - poly[0])) for p in points] if len(poly) else []
    A, B = poly[:-1], poly[1:]
    AB = B - A
    L2 = np.maximum((AB * AB).sum(1), 1e-18)
    out = []
    for p in points:
        u = np.clip(((p - A) * AB).sum(1) / L2, 0, 1)
        proj = A + AB * u[:, None]
        out.append(float(np.min(np.linalg.norm(proj - p, axis=1))))
    return out


def analyse(path):
    lines = read(path)
    runtime = []
    preview = defaultdict(list)
    terminal = defaultdict(list)
    starts, commits, adopts, invalid, retains = {}, {}, [], [], defaultdict(list)
    timing = None
    completed = any("[Completed] full plan-once" in l for l in lines)
    for l in lines:
        if "[V2ParityRuntime]" in l:
            d = kv(l)
            runtime.append((fl(d["t"]), d["state"], d["v2phase"], int(d["planId"]), vec(d["mouth"]), vec(d["mouthQ"]),
                            vec(d["object"]), vec(d["truth"]), fl(d["clear"]), d["clearSource"], d["attached"] == "true"))
        elif "[V2ParityPreviewReach]" in l:
            d = kv(l)
            if d["phase"] == "0":  # reach samples; the rest of the rollout follows in the same trace
                preview[int(d["planId"])].append((fl(d["t"]), vec(d["p"]), vec(d["q"]), fl(d["clear"])))
        elif "[V2ParityPreviewTerminal]" in l:
            d = kv(l)
            terminal[int(d["planId"])].append((int(d["phase"]), fl(d["t"]), vec(d["p"]), vec(d["q"]), fl(d["clear"])))
        elif "[V2ParityReachStart]" in l:
            d = kv(l)
            starts[int(d["planId"])] = d
        elif "[V2ParityCommitPreview]" in l:
            d = kv(l)
            commits[int(d["planId"])] = d
        elif "[V2ProvisionalAdopt]" in l:
            adopts.append(kv(l))
        elif "[V2ProvisionalInvalidated]" in l:
            invalid.append(kv(l))
        elif "[V2ProvisionalRetain]" in l:
            d = kv(l)
            retains[int(d["planId"])].append((fl(d["t"]), fl(d["presentationUpdate"])))
        elif "[TimingSummary]" in l:
            timing = kv(l)
    run = dict(log=path, completed=completed, reach=[], terminal=None)
    rt = np.array([r[0] for r in runtime])
    for pid, st in starts.items():
        if st["available"] != "true" or pid not in preview:
            run["reach"].append(dict(planId=pid, available=False))
            continue
        prev = preview[pid]
        pt = np.array([x[0] for x in prev])
        pp = np.array([x[1] for x in prev])
        pq = np.array([x[2] for x in prev])
        rs, so = fl(st["reachStart"]), fl(st["standoffTime"])
        target_p, target_q = vec(st["standoff"]), vec(st["standoffQ"])
        # end of this plan's execution: invalidation, replacement, or commit
        end = None
        how = "reached_standoff_window"
        for iv in invalid:
            if int(iv["planId"]) == pid and fl(iv["t"]) >= rs:
                end, how = fl(iv["t"]), "invalidated:" + iv["reason"]
                break
        for a in adopts:
            if int(a["previousPlanId"]) == pid and fl(a["t"]) >= rs and (end is None or fl(a["t"]) < end):
                end, how = fl(a["t"]), "replaced"
        win_end = min(so, end) if end is not None else so
        sel = [r for r in runtime if r[3] == pid and rs - 1e-9 <= r[0] <= win_end + 1e-9
               and r[2] in ("PROVISIONAL_REACH", "TERMINAL_TRACK")]
        dev_p, dev_q, rclear = [], [], []
        for r in sel:
            j = np.searchsorted(pt, r[0])
            j = min(max(j, 1), len(pt) - 1)
            u = (r[0] - pt[j - 1]) / max(1e-9, pt[j] - pt[j - 1])
            u = min(1.0, max(0.0, u))
            p_int = pp[j - 1] + u * (pp[j] - pp[j - 1])
            q_int = pq[j - 1] if u < 0.5 else pq[j]
            dev_p.append(float(np.linalg.norm(r[4] - p_int)))
            dev_q.append(qangle(r[5], q_int))
            if r[9] == "pose" and math.isfinite(r[8]):
                rclear.append(r[8])

        def arrival(ts, ps, qs):
            for t, p, q in zip(ts, ps, qs):
                if np.linalg.norm(p - target_p) <= POS_TOL and qangle(q, target_q) <= ORI_TOL:
                    return t
            return None

        spatial = polyline_distance([r[4] for r in sel], pp) if sel else []
        prev_arr = arrival(pt, pp, pq)
        run_all = [r for r in runtime if r[3] == pid and r[0] >= rs - 1e-9 and (end is None or r[0] <= end + 1e-9)
                   and r[2] in ("PROVISIONAL_REACH", "TERMINAL_TRACK", "COMMITTED")]
        run_arr = arrival([r[0] for r in run_all], [r[4] for r in run_all], [r[5] for r in run_all])
        at_so = [r for r in runtime if r[3] == pid and abs(r[0] - so) <= 0.0051]
        upd = max([u for (t, u) in retains[pid] if t >= rs] or [0.0])
        run["reach"].append(dict(
            planId=pid, available=True, candidate=st["candidate"], route=st["route"], reachDuration=fl(st["reachDuration"]),
            ended=how, executedFraction=(win_end - rs) / max(1e-9, so - rs), samples=len(sel),
            maxPathDev=max(dev_p) if dev_p else None, rmsPathDev=float(np.sqrt(np.mean(np.square(dev_p)))) if dev_p else None,
            maxOriDev=max(dev_q) if dev_q else None,
            maxSpatialDev=max(spatial) if spatial else None,
            previewStandoffErr=float(np.linalg.norm(pp[-1] - target_p)), previewStandoffOri=qangle(pq[-1], target_q),
            runtimeStandoffErr=float(np.linalg.norm(at_so[0][4] - target_p)) if at_so and (end is None or end >= so) else None,
            runtimeStandoffOri=qangle(at_so[0][5], target_q) if at_so and (end is None or end >= so) else None,
            previewArrival=(prev_arr - rs) if prev_arr is not None else None,
            runtimeArrival=(run_arr - rs) if run_arr is not None else None,
            previewMinClear=float(min(x[3] for x in prev)), runtimeMinClear=min(rclear) if rclear else None,
            predictedReachClear=fl(st["predictedReachClear"]), maxPresentationUpdate=upd,
            objectDisplacementDuringWindow=float(np.linalg.norm(sel[-1][7] - sel[0][7])) if sel else None))
    for pid, cm in commits.items():
        tr = terminal.get(pid, [])
        ins = np.array([x[2] for x in tr if x[0] == 1])
        ret = np.array([x[2] for x in tr if x[0] == 4])
        mp = [r for r in runtime if r[1].endswith("MovePregrasp")]
        rr = [r for r in runtime if r[1].endswith("Retreat")]
        ct = [r for r in runtime if r[1].endswith("CaptureTransfer")]
        term = dict(planId=pid, predictedApproach=fl(cm["predictedApproach"]), auditDuration=fl(cm["auditDuration"]),
                    legacyApproach=fl(cm["legacyApproach"]), predictedAcquire=fl(cm["predictedAcquire"]),
                    predictedRetreat=fl(cm["predictedRetreat"]), estimated=fl(cm["estimated"]),
                    predictedRetreatClear=fl(cm["retreatClear"]),
                    previewRetreatMinClear=float(min([x[4] for x in tr if x[0] == 4] or [float("nan")])))
        if timing:
            for k in ("approach", "acquire", "transfer", "retreat", "execution"):
                term["runtime_" + k] = fl(timing[k])
        term["runtimeStateDurations"] = {name: (seq[-1][0] - seq[0][0]) if seq else None
                                         for name, seq in (("MovePregrasp", mp), ("CaptureTransfer", ct), ("Retreat", rr))}
        if len(ins) and mp:
            d = polyline_distance([r[4] for r in mp], ins)
            term["insertionMaxPathDev"] = max(d)
            term["insertionEndErr"] = float(np.linalg.norm(mp[-1][4] - ins[-1]))
        if len(ret) and rr:
            d = polyline_distance([r[4] for r in rr], ret)
            term["retreatMaxPathDev"] = max(d)
            term["retreatEndErr"] = float(np.linalg.norm(rr[-1][4] - ret[-1]))
            rc = [r[8] for r in rr if r[9] == "attached_retreat" and math.isfinite(r[8])]
            term["runtimeRetreatMinClear"] = min(rc) if rc else None
        run["terminal"] = term
    return run


def fmt(x, d=3, scale=1.0):
    return "—" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x*scale:.{d}f}"


def report(path):
    runs = json.load(open(path))
    print("### Reach: copied-state preview vs runtime (per executed plan)\n")
    print("| run | plan | candidate / route | ended | executed fraction | time-aligned max / RMS path dev (mm) | spatial max dev (mm) | max ori dev (rad) | standoff err preview / runtime (mm) | arrival preview / runtime (s after reach start) | min clearance preview / runtime (mm) | max presentation update while executing (mm) |")
    print("|---|---:|---|---|---:|---|---:|---:|---|---|---|---:|")
    for r in runs:
        name = "/".join(r["log"].split("/")[-3:-1])
        for p in r["reach"]:
            if not p.get("available"):
                print(f"| {name} | {p['planId']} | trace unavailable | | | | | | | | | |")
                continue
            print(f"| {name} | {p['planId']} | {p['candidate']} / {p['route']} | {p['ended'][:60]} | {p['executedFraction']:.2f} | "
                  f"{fmt(p['maxPathDev'],1,1000)} / {fmt(p['rmsPathDev'],1,1000)} | {fmt(p.get('maxSpatialDev'),1,1000)} | {fmt(p['maxOriDev'],3)} | "
                  f"{fmt(p['previewStandoffErr'],1,1000)} / {fmt(p['runtimeStandoffErr'],1,1000)} | "
                  f"{fmt(p['previewArrival'],2)} / {fmt(p['runtimeArrival'],2)} | {fmt(p['previewMinClear'],1,1000)} / {fmt(p['runtimeMinClear'],1,1000)} | "
                  f"{fmt(p['maxPresentationUpdate'],1,1000)} |")
    print("\n### Terminal: certificate used at commitment vs post-commit runtime\n")
    print("| run | completed | insertion predicted (audit) / runtime (s) | acquire predicted / runtime (s) | retreat predicted / runtime (s) | total predicted / runtime (s) | insertion max path dev / end err (mm) | retreat max path dev / end err (mm) | retreat clearance predicted / runtime min (mm) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in runs:
        t = r.get("terminal")
        name = "/".join(r["log"].split("/")[-3:-1])
        if not t:
            print(f"| {name} | {r['completed']} | no commit | | | | | | |")
            continue
        print(f"| {name} | {r['completed']} | {t['predictedApproach']:.3f} / {fmt(t.get('runtime_approach'))} | "
              f"{t['predictedAcquire']:.3f} / {fmt(t.get('runtime_acquire'))} | {t['predictedRetreat']:.3f} / {fmt(t.get('runtime_retreat'))} | "
              f"{t['estimated']:.3f} / {fmt(t.get('runtime_execution'))} | {fmt(t.get('insertionMaxPathDev'),1,1000)} / {fmt(t.get('insertionEndErr'),1,1000)} | "
              f"{fmt(t.get('retreatMaxPathDev'),1,1000)} / {fmt(t.get('retreatEndErr'),1,1000)} | {fmt(t['predictedRetreatClear'],1,1000)} / {fmt(t.get('runtimeRetreatMinClear'),1,1000)} |")


def main(argv):
    if len(argv) == 3 and argv[1] == "report":
        report(argv[2])
        return 0
    if len(argv) >= 3:
        runs = [analyse(p) for p in argv[2:]]
        json.dump(runs, open(argv[1], "w"), indent=1, default=float)
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
