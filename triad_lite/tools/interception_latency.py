#!/usr/bin/env python3
"""Selection-job latency split by object motion (L_calc characterization).
usage: interception_latency.py LOG[.xz] ..."""
import lzma, re, statistics as st, sys
KV = re.compile(r"(\w+)=(\[[^\]]*\]|[^ ]+)")
q = lambda v, p: sorted(v)[min(len(v) - 1, int(p * len(v)))]
sweep = {"moving": [], "rest": []}
wall = {"moving": [], "rest": []}
for f in sys.argv[1:]:
    fh = lzma.open(f, "rt", errors="ignore") if f.endswith(".xz") else open(f, errors="ignore")
    moving = {}
    for l in fh:
        if "[TriadLiteInterceptionJob]" in l:
            d = dict(KV.findall(l))
            sweep["rest" if d["step"] == "inf" else "moving"].append(float(d["sweepMs"]))
        m = re.search(r"V2PlanningJobSubmit\] type=CONTROL_AWARE_SELECT planningGeneration=(\d+).*objectMoving=(\w+)", l)
        if m: moving[m.group(1)] = m.group(2) == "true"
        m = re.search(r"V2JobProfile\] type=CONTROL_AWARE_SELECT planningGeneration=(\d+).*jobWall=([0-9.]+)s", l)
        if m: wall["moving" if moving.get(m.group(1)) else "rest"].append(float(m.group(2)))
for k in ("moving", "rest"):
    s, w = sweep[k], wall[k]
    if s: print(f"{k}: sweepMs n={len(s)} median={st.median(s):.0f} p90={q(s, .9):.0f} max={max(s):.0f}")
    if w: print(f"{k}: jobWall s n={len(w)} median={st.median(w):.3f} p90={q(w, .9):.3f} p95={q(w, .95):.3f} max={max(w):.3f}")
