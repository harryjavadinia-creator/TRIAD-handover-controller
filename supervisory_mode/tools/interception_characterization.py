#!/usr/bin/env python3
"""Phase C interception-solver characterization from [TriadLiteInterception*] log lines.

Per log: job counts, stage timing (front end / exact / rollout / sweep), layer
counts G0 -> Gmech -> GR -> GK -> exact-feasible -> F_I, attempt stages, the
budget needed to reach the first feasible encounter, and the selections of the
B1/B2/FULL tie-break rules. With two directories (skip, noskip) it compares
tauBest per job matched by planningGeneration order (the runs are different
simulations, so the comparison is distributional, not paired).
usage: interception_characterization.py LOG [LOG ...]
"""
import lzma, re, statistics as st, sys
from collections import Counter, defaultdict

ARM_SCALE = 1.25  # executionTiming.armScale
V_FAR = 0.38      # predictiveReachPolicy.farLinearSpeed
KV = re.compile(r"(\w+)=(\[[^\]]*\]|[^ ]+)")


def kv(line):
    return dict(KV.findall(line))


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")


def load(path):
    fh = lzma.open(path, "rt", errors="ignore") if path.endswith(".xz") else open(path, errors="ignore")
    jobs, cands, attempts, funnels = {}, defaultdict(list), defaultdict(list), {}
    for line in fh:
        if "[TriadLiteInterceptionJob]" in line:
            d = kv(line); jobs[d["planningGeneration"]] = d
        elif "[TriadLiteInterception]" in line:
            d = kv(line); cands[d["planningGeneration"]].append(d)
        elif "[TriadLiteInterceptAttempt]" in line:
            d = kv(line); attempts[d["planningGeneration"]].append(d)
        elif "[TriadLiteInterceptionFunnel]" in line:
            d = kv(line); funnels[d["planningGeneration"]] = d
    return jobs, cands, attempts, funnels


def summarize(path):
    jobs, cands, attempts, funnels = load(path)
    name = "/".join(path.split("/")[-3:-1])
    print(f"== {name}: jobs={len(jobs)}")
    if not jobs:
        return {}
    f = lambda k: [float(j[k]) for j in jobs.values()]
    for k in ("frontEndMs", "exactMs", "rolloutMs", "sweepMs"):
        v = f(k)
        print(f"  {k}: median={st.median(v):.1f} p90={q(v, .9):.1f} max={max(v):.1f}")
    ex = [int(j["exactEvaluations"]) for j in jobs.values()]
    ro = [int(j["rollouts"]) for j in jobs.values()]
    print(f"  exactEvaluations median={st.median(ex)} max={max(ex)}; rollouts median={st.median(ro)} max={max(ro)}; "
          f"budgetExhausted={sum(j['budgetExhausted'] == 'true' for j in jobs.values())}")
    moving = [j for j in jobs.values() if j["step"] not in ("inf", "nan")]
    feas = [j for j in jobs.values() if int(j["feasible"]) > 0]
    print(f"  moving-prediction jobs={len(moving)} jobs with >=1 feasible encounter={len(feas)} "
          f"(moving {sum(int(j['feasible']) > 0 for j in moving)})")
    # layer counts
    layers = defaultdict(list)
    for g, j in jobs.items():
        fu = funnels.get(g, {})
        cs = cands.get(g, [])
        layers["G0"].append(int(fu.get("G0", 0)))
        layers["Gmech"].append(int(fu.get("Gmech", 0)))
        layers["GR"].append(int(fu.get("GR", 0)))
        layers["GK"].append(int(fu.get("GK", 0)))
        layers["exactFeasibleSomeTau"].append(len({a["graspId"] for a in attempts.get(g, [])
                                                   if a["stage"] in ("timing", "rollout", "feasible")}))
        layers["F_I (feasible)"].append(sum(c["status"] == "feasible" for c in cs))
        layers["dominated"].append(sum(c["status"] == "dominated" for c in cs))
    print("  layers (median per job): " + "  ".join(f"{k}={st.median(v)}" for k, v in layers.items()))
    stages = Counter()
    for g in jobs:
        for a in attempts.get(g, []):
            r = re.sub(r"=[-0-9.]+", "", a["reason"]).split("/")
            stages[a["stage"] + ":" + "/".join(r[:2])] += 1
    print("  attempt stages: " + ", ".join(f"{k}={v}" for k, v in stages.most_common(12)))
    # exact evaluations spent before the first feasible attempt (budget needed)
    need = []
    for g, j in jobs.items():
        n = 0
        for a in attempts.get(g, []):
            if a["stage"] in ("exact", "timing", "rollout", "feasible", "dominated"):
                n += 1
            if a["stage"] == "feasible":
                need.append(n)
                break
    if need:
        print(f"  exact evaluations until first feasible encounter: median={st.median(need)} p90={q(need, .9)} max={max(need)}")
    sel = Counter()
    for j in feas:
        sel["B1==FULL" if j["selectedB1"] == j["selectedFull"] else "B1!=FULL"] += 1
        sel["B1==B2" if j["selectedB1"] == j["selectedB2"] else "B1!=B2"] += 1
    print(f"  tie-break selections over feasible jobs: {dict(sel)}")
    rel = [float(c["relativeLinearSpeed"]) for cs in cands.values() for c in cs if c["status"] == "feasible"]
    if rel:
        print(f"  feasible terminal relative speed median={st.median(rel):.4f} max={max(rel):.4f} m/s")
    # Paired losslessness of the timing skip (meaningful on exact-scan logs):
    # after a timing failure at tau with required time R, the skip rule jumps
    # past events < tau + (R - tau) / (1 + r), r = ARM_SCALE * pointSpeed / V_FAR.
    # Count grasps whose timing test PASSED at an event the rule would skip.
    violations = checked = 0
    for g, j in jobs.items():
        r = ARM_SCALE * float(j["pointSpeed"]) / V_FAR
        per = defaultdict(list)
        for a in attempts.get(g, []):
            per[a["graspId"]].append(a)
        for gid, seq in per.items():
            for i, a in enumerate(seq):
                if a["stage"] != "timing":
                    continue
                tau, req = float(a["tau"]), float(a["requiredTime"])
                skip_to = tau + (req - tau) / (1.0 + r)
                for b in seq[i + 1:]:
                    tb = float(b["tau"])
                    if tb + 1e-9 >= skip_to:
                        break
                    if b["stage"] in ("exact", "mechanical", "pursuit", "surrogate"):
                        continue
                    checked += 1
                    if b["stage"] != "timing":
                        violations += 1
    print(f"  timing-skip check: skipped-event timing outcomes observed={checked} passed-timing-inside-skip={violations}")
    return {"tauBest": [float(j["tauBest"]) for j in moving if j["tauBest"] != "inf"], "exact": ex,
            "sweep": f("sweepMs")}


if __name__ == "__main__":
    groups = defaultdict(dict)
    for p in sys.argv[1:]:
        r = summarize(p)
        parts = p.split("/")
        groups[parts[-3]][parts[-2]] = r
    if "hold_skip" in groups and "hold_noskip" in groups:
        print("== timing skip vs exact scan (distributional; different simulations)")
        for s in sorted(set(groups["hold_skip"]) & set(groups["hold_noskip"])):
            a, b = groups["hold_skip"][s], groups["hold_noskip"][s]
            if a.get("tauBest") and b.get("tauBest"):
                print(f"  {s}: moving tauBest median skip={st.median(a['tauBest']):.3f} noskip={st.median(b['tauBest']):.3f}; "
                      f"exact median skip={st.median(a['exact'])} noskip={st.median(b['exact'])}; "
                      f"sweepMs median skip={st.median(a['sweep']):.0f} noskip={st.median(b['sweep']):.0f}")
