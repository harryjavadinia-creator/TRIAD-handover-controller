#!/usr/bin/env python3
"""Phase B funnel characterization from characterizeAll logs.

For every selection generation, all mechanically valid hypotheses were evaluated
exactly. Reports: surrogate false negatives (pruned but exactly feasible), ranking
quality (AUC) of candidate cheap scores for predicting exact robot feasibility and
admissibility, and recall@K (some admissible grasp inside the top K when one exists).
usage: funnel_characterization.py LOG [LOG ...]
"""
import math
import lzma, random, re, sys
from collections import defaultdict

KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")
def kv(line): return dict(KV.findall(line))

def auc(scores_pos, scores_neg):
    if not scores_pos or not scores_neg: return float("nan")
    allv = sorted([(s, 1) for s in scores_pos] + [(s, 0) for s in scores_neg])
    rank_sum, i = 0.0, 0
    while i < len(allv):
        j = i
        while j < len(allv) and allv[j][0] == allv[i][0]: j += 1
        avg = (i + j + 1) / 2.0
        rank_sum += avg * sum(1 for k in range(i, j) if allv[k][1] == 1)
        i = j
    n1, n0 = len(scores_pos), len(scores_neg)
    return (rank_sum - n1 * (n1 + 1) / 2.0) / (n1 * n0)

PRUNE_TOLERANCE = 0.02            # reachabilityFunnel.pruneTolerance
THETA_BIN_WIDTH = math.radians(13.6)  # reachabilityFunnel.thetaBinWidthDeg
CLEARANCE_TIE_BAND = 0.005        # controlAware.clearanceTieBand

def shortlist(g, K):
    """Python mirror of reachabilityShortlist() in src/ReceivingGraspFamily.h (prune, score order, diversity, fill)."""
    kept = sorted((r for r in g if r["score"] >= -PRUNE_TOLERANCE), key=lambda r: -r["score"])
    if K <= 0 or K >= len(kept): return kept
    out, seen, deferred = [], set(), []
    for r in kept:
        key = (r["sign"], round(r["s"], 6), int(math.floor(r["theta"] / THETA_BIN_WIDTH)))
        if key in seen: deferred.append(r); continue
        seen.add(key); out.append(r)
        if len(out) == K: return out
    return out + deferred[:K - len(out)]

def main(paths):
    gens = defaultdict(list)
    for p in paths:
        fh = lzma.open(p, "rt", errors="ignore") if p.endswith(".xz") else open(p, errors="ignore")
        for line in fh:
            if "[TriadLiteCandidate]" in line and "family=receiving" in line:
                d = kv(line)
                gens[(p, d["planningGeneration"])].append(dict(
                    score=float(d["reachability"]), reach=float(d["reachDistance"]),
                    robot=d["robotFeasible"] == "true", adm=d["admissible"] == "true",
                    layer=d["rejectionLayer"], sign=d["sign"], s=float(d["s"]),
                    theta=math.radians(float(d["thetaDeg"])),
                    clearance=float(d["clearance"]) if d["clearance"] not in ("-inf", "inf", "nan") else -math.inf))
    recs = [r for g in gens.values() for r in g]
    print(f"generations={len(gens)} evaluated hypotheses={len(recs)} robotFeasible={sum(r['robot'] for r in recs)} admissible={sum(r['adm'] for r in recs)}")
    for tol in (0.0, 0.02, 0.05):
        pruned = [r for r in recs if r["score"] < -tol]
        print(f"  pruned at -{tol:.2f}: {len(pruned)}; of those robotFeasible={sum(r['robot'] for r in pruned)} admissible={sum(r['adm'] for r in pruned)}")
    for name, key, sign in (("wristSDF", "score", 1), ("-reachDistance", "reach", -1)):
        a_robot = auc([sign * r[key] for r in recs if r["robot"]], [sign * r[key] for r in recs if not r["robot"]])
        a_adm = auc([sign * r[key] for r in recs if r["adm"]], [sign * r[key] for r in recs if not r["adm"]])
        print(f"  AUC {name}: robotFeasible={a_robot:.3f} admissible={a_adm:.3f}")
    rng = random.Random(3)
    with_adm = [g for g in gens.values() if any(r["adm"] for r in g)]
    print(f"generations with >=1 admissible: {len(with_adm)}")
    n = max(1, len(with_adm))
    for K in (4, 8, 12, 16, 20, 24, 32, 40, 48, 64, 96, 200):
        hit = {"shortlist": 0, "wristSDF": 0, "-reachDistance": 0, "random": 0}
        best = 0  # shortlist contains an admissible grasp within the clearance tie band of the full-set best
        for g in with_adm:
            for name, order in (("shortlist", shortlist(g, K)),
                                ("wristSDF", sorted(g, key=lambda r: -r["score"])),
                                ("-reachDistance", sorted(g, key=lambda r: r["reach"])),
                                ("random", rng.sample(g, len(g)))):
                if any(r["adm"] for r in order[:K]): hit[name] += 1
            full_best = max(r["clearance"] for r in g if r["adm"])
            if any(r["adm"] and r["clearance"] >= full_best - CLEARANCE_TIE_BAND for r in shortlist(g, K)): best += 1
        print(f"  recall@{K:>3}: " + "  ".join(f"{k}={v / n:.3f}" for k, v in hit.items())
              + f"  shortlistContainsClearanceBest={best / n:.3f}")

    # sampling-resolution convergence (sub-families of the characterized family; finer grids need new runs)
    thetas = sorted({round(r["theta"], 6) for r in recs})
    n_theta = len(thetas)
    axials = sorted({round(r["s"], 6) for r in recs})
    print(f"family: thetaSamples={n_theta} axialValues={axials}")
    subsets = []
    for stride in (1, 2, 4, 8):
        for label, keep_s in (("all", set(axials)), ("3", {axials[0], 0.0, axials[-1]}), ("centred", {0.0})):
            subsets.append((stride, label, keep_s))
    for stride, label, keep_s in subsets:
        found = 0; losses = []
        for g in with_adm:
            full_best = max(r["clearance"] for r in g if r["adm"])
            sub = [r for r in g if round(r["s"], 6) in keep_s and round(r["theta"] / (2 * math.pi / n_theta)) % stride == 0]
            adm = [r["clearance"] for r in sub if r["adm"]]
            if adm: found += 1; losses.append(full_best - max(adm))
        losses.sort()
        med = losses[len(losses) // 2] if losses else float("nan")
        worst = losses[-1] if losses else float("nan")
        print(f"  thetaStride={stride} (N={n_theta // stride if stride > 1 else n_theta}) axial={label}: generationsWithAdmissible={found}/{len(with_adm)}"
              f" bestClearanceLoss median={1000 * med:.2f}mm worst={1000 * worst:.2f}mm")
    # smallest K reaching full recall and clearance-best containment, per log file (scenario)
    for path in paths:
        gs = [g for (pp, _), g in gens.items() if pp == path and any(r["adm"] for r in g)]
        def kmin(pred):
            for K in range(1, 531):
                if all(pred(g, K) for g in gs): return K
            return None
        k_rec = kmin(lambda g, K: any(r["adm"] for r in shortlist(g, K)))
        k_best = kmin(lambda g, K: any(r["adm"] and r["clearance"] >= max(x["clearance"] for x in g if x["adm"]) - CLEARANCE_TIE_BAND
                                        for r in shortlist(g, K)))
        print(f"  {path.split('/')[-1]}: generations={len(gs)} minK(recall=1)={k_rec} minK(clearanceBest=1)={k_best}")

if __name__ == "__main__":
    main(sys.argv[1:])
