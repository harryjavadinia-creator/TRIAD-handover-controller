#!/usr/bin/env python3
"""Authority demand vs executed command: rollout peak commanded body linear speed of each adopted
encounter ([TriadLiteInterception] pathPeakLinearSpeed in the adopting job) vs the executed commanded
body speed ([TriadLiteInterceptTrack] bodyLinearSpeed, 50 ms samples) during that plan's intercept phase.
Plans shorter than 1 s or with < 80 % sample coverage are excluded.
usage: demand_vs_execution.py EVIDENCE_DIR"""
import glob, lzma, re, statistics as st, sys
KV = re.compile(r"(\w+)=(\[[^\]]*\]|[^ ]+)")
rat = []
for f in sorted(glob.glob(sys.argv[1] + "/logs/r*/*/*/*.log*")):
    v = f.split("/")[-3]
    if v == "reactive":
        continue
    fh = lzma.open(f, "rt", errors="ignore") if f.endswith(".xz") else open(f, errors="ignore")
    cand, plans, lastgen = {}, [], None
    for l in fh:
        if "[TriadLiteInterception]" in l:
            d = dict(KV.findall(l)); cand[(d["planningGeneration"], d["graspId"])] = float(d["pathPeakLinearSpeed"])
        elif "[TriadLiteSelection]" in l:
            lastgen = dict(KV.findall(l))["planningGeneration"]
        elif "[TriadLiteEvent] type=interception_plan" in l:
            d = dict(KV.findall(l)); plans.append([d["graspId"], float(d["t"]), float(d["tRendezvous"]), cand.get((lastgen, d["graspId"])), []])
        elif "[TriadLiteInterceptTrack]" in l and "phase=intercept" in l and plans:
            d = dict(KV.findall(l)); t = float(d["t"]); p = plans[-1]
            if d["graspId"] == p[0] and p[1] <= t <= p[2]:
                p[4].append(float(d["bodyLinearSpeed"]))
    for g, t0, tR, peak, ex in plans:
        if peak and ex and tR - t0 > 1.0 and len(ex) >= 0.8 * (tR - t0) / 0.05:
            rat.append(max(ex) / peak)
            print(f"{'/'.join(f.split('/')[-4:-2])} {f.split('/')[-2]} grasp={g} rolloutPeak={peak:.3f} executedPeak={max(ex):.3f} ratio={max(ex) / peak:.2f}")
if rat:
    print(f"ratio executed/rollout peak: n={len(rat)} median={st.median(rat):.2f} min={min(rat):.2f} max={max(rat):.2f}")
