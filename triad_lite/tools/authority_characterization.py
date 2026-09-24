#!/usr/bin/env python3
"""Phase D authority characterization from [TriadLiteInterception*] lines.

For each configuration directory (log / filter / stride1):
  * kappa of encounters in F_I and its limiting phase (path / synchronization / insertion);
  * fraction of F_I encounters with kappa < kappaMin (would leave F_C);
  * filter runs: grasps whose F_C encounter is later than their F_I encounter,
    grasps in F_I with no F_C encounter, jobs whose F_C is empty while F_I is not;
  * H2 information check: Spearman rank correlation between kappa and the generic
    capability measure (condition index at the rendezvous) among feasible grasps,
    and how often the FULL and B2 tie-break selections differ;
  * cost: rollout and sweep wall.
usage: authority_characterization.py LOG[.xz] ...   (grouped by parent-of-scenario directory)
"""
import lzma, re, statistics as st, sys
from collections import Counter, defaultdict

KV = re.compile(r"(\w+)=(\[[^\]]*\]|[^ ]+)")
KAPPA_MIN = 1.0


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")


def ranks(v):
    order = sorted(range(len(v)), key=lambda i: v[i])
    r = [0.0] * len(v)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
            j += 1
        for k in range(i, j + 1):
            r[order[k]] = (i + j) / 2.0
        i = j + 1
    return r


def spearman(a, b):
    if len(a) < 3:
        return float("nan")
    ra, rb = ranks(a), ranks(b)
    ma, mb = st.mean(ra), st.mean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    den = (sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb)) ** 0.5
    return num / den if den > 0 else float("nan")


def fnum(x):
    try:
        return float(x)
    except ValueError:
        return float("nan")


# Held-arm characterization repeats near-identical at-rest jobs until the
# presentation window expires; statistics use every moving-prediction job and
# only the FIRST at-rest job of each run (SCOPE=all keeps every job).
import os
SCOPE = os.environ.get("SCOPE", "dedup")
groups = defaultdict(list)
for p in sys.argv[1:]:
    groups[p.split("/")[-3]].append(p)

for cfg, paths in sorted(groups.items()):
    feas, limiting, jobs = [], Counter(), []
    later = inFInoFC = emptyFC = jobsFI = 0
    per_job_rho = []
    full_ne_b2 = full_ne_b1 = feasible_jobs = 0
    authority_attempts = 0
    for p in paths:
        fh = lzma.open(p, "rt", errors="ignore") if p.endswith(".xz") else open(p, errors="ignore")
        cands = defaultdict(list)
        keep = set()
        rest_seen = False
        lines = fh.readlines()
        for line in lines:
            if "[TriadLiteInterceptionJob]" in line:
                d = dict(KV.findall(line))
                if SCOPE == "all" or d["step"] != "inf" or not rest_seen:
                    keep.add(d["planningGeneration"])
                    jobs.append(d)
                rest_seen = rest_seen or d["step"] == "inf"
        for line in lines:
            if "[TriadLiteInterception]" in line:
                d = dict(KV.findall(line))
                if d["planningGeneration"] in keep:
                    cands[d["planningGeneration"]].append(d)
            elif "[TriadLiteInterceptAttempt]" in line and "stage=authority" in line:
                if dict(KV.findall(line))["planningGeneration"] in keep:
                    authority_attempts += 1
        for g, cs in cands.items():
            f = [c for c in cs if c["status"] == "feasible"]
            feas += f
            for c in f:
                limiting[c["kappaLimiting"]] += 1
            fi = [c for c in cs if fnum(c["tauStarInterception"]) != float("inf") and c["tauStarInterception"] != "inf"]
            if fi:
                jobsFI += 1
                if not f:
                    emptyFC += 1
            for c in fi:
                if c["status"] != "feasible":
                    inFInoFC += 1
                elif fnum(c["tauStar"]) > fnum(c["tauStarInterception"]) + 1e-9:
                    later += 1
            if len(f) >= 3:
                per_job_rho.append(spearman([fnum(c["kappa"]) for c in f], [fnum(c["conditionIndex"]) for c in f]))
    for j in jobs:
        if int(j["feasible"]) > 0:
            feasible_jobs += 1
            full_ne_b2 += j["selectedFull"] != j["selectedB2"]
            full_ne_b1 += j["selectedFull"] != j["selectedB1"]
    kap = [fnum(c["kappa"]) for c in feas]
    print(f"== {cfg}: runs={len(paths)} jobs={len(jobs)} feasible encounters={len(feas)}")
    if kap:
        print(f"  kappa: median={st.median(kap):.3f} p10={q(kap, .1):.3f} min={min(kap):.3f}; below kappaMin={sum(k < KAPPA_MIN for k in kap)}"
              f" ({100.0 * sum(k < KAPPA_MIN for k in kap) / len(kap):.1f}%)")
        for name in ("pathReserve", "syncReserve", "insertionReserve"):
            v = [fnum(c[name]) for c in feas if fnum(c[name]) == fnum(c[name])]
            if v:
                print(f"  {name}: median={st.median(v):.3f} p10={q(v, .1):.3f} min={min(v):.3f}")
        print(f"  limiting phase: {dict(limiting)}")
        ps = [fnum(c["pathSamples"]) for c in feas]
        pk = [fnum(c["pathPeakLinearSpeed"]) for c in feas]
        print(f"  path samples median={st.median(ps):.0f}; peak commanded body speed median={st.median(pk):.3f} max={max(pk):.3f} m/s")
    print(f"  authority attempts rejected (filter)={authority_attempts}; grasps with F_C later than F_I={later}; "
          f"grasps in F_I without F_C encounter={inFInoFC}; jobs with F_I nonempty={jobsFI}, F_C empty={emptyFC}")
    rho = [r for r in per_job_rho if r == r]
    if rho:
        print(f"  H2: per-job Spearman(kappa, conditionIndex) over feasible grasps (jobs with >=3): n={len(rho)} "
              f"median={st.median(rho):.2f} p10={q(rho, .1):.2f} p90={q(rho, .9):.2f}")
    if feasible_jobs:
        print(f"  selections over jobs with a feasible encounter ({feasible_jobs}): FULL!=B1 {full_ne_b1}, FULL!=B2 {full_ne_b2}")
    ro = [float(j["rolloutMs"]) for j in jobs if int(j["rollouts"]) > 0]
    rr = [float(j["rolloutMs"]) / int(j["rollouts"]) for j in jobs if int(j["rollouts"]) > 0]
    sw = [float(j["sweepMs"]) for j in jobs if j["step"] != "inf"]
    if rr:
        print(f"  cost: rollout per (g,tau) median={st.median(rr):.1f} ms p90={q(rr, .9):.1f}; moving-job sweep median={st.median(sw) if sw else float('nan'):.0f} ms"
              f" p90={q(sw, .9):.0f}")
