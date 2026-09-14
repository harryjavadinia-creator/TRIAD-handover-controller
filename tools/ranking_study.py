#!/usr/bin/env python3
"""Final TRIAD Phase 6: ranking of certified complete actions, offline comparison.

Joins every complete record ([V2CompleteRecord]) with its seven cost terms
([CompletePlanCost] logged immediately before the route certification of the same
candidate/route; memoized rest records reuse the first hypothesis's terms) and
compares rankers over the SAME certified, timing-admissible set:

  J7                current binding objective (recomputed from terms and checked)
  EARLIEST_TAU      min interception time, then min completion time, then max clearance
  EARLIEST_DONE     min completion time C = lead - T_pres + T_exec, then max clearance
  LEX(d)            completion within d s of the minimum, then max min-clearance, then min effort
  SAFE_EARLIEST     earliest completion among records with min-clearance >= soft clearance
                    (0.080 m), else max min-clearance
  J_T+C             time and clearance terms only (weights renormalized)
  J7 without V / without E+L / without Q+K     reduced objectives
  weight sensitivity: each weight x0.5 and x2 (renormalized)

Rest epochs use the Phase 3 rest law (one pose, tau per record at the epoch).
Moving epochs use zero-latency admission at the epoch over the logged leads.

Usage: ranking_study.py OUT.json LOG [LOG ...]  |  ranking_study.py report OUT.json
"""

import json
import lzma
import math
import re
import sys
from collections import Counter, defaultdict

KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")
W = dict(T=0.4210526, E=0.1052632, L=0.1052632, C=0.1578947, Q=0.0842105, K=0.0736842, V=0.0526316)
T_REF = 8.0
L_COMMIT, L_ENTRY = 1.6, 0.05
SOFT_CLEAR = 0.080


def kv(line):
    i = line.find("] [")
    return dict(KV.findall(line[i:] if i >= 0 else line))


def fl(s):
    try:
        return float(str(s).rstrip("smx"))
    except ValueError:
        return float("nan")


def read(path):
    op = lzma.open if path.endswith(".xz") else open
    with op(path, "rt", errors="replace") as f:
        return f.read().splitlines()


def parse_terms(s):
    return {k: float(v) for k, v in (x.split(":") for x in s.strip("[]").split(","))}


def load(path):
    gens = {}
    last_cost = {}
    terms_by_pair = defaultdict(dict)  # gen -> (cand, route) -> terms (first seen; for memo)
    for l in read(path):
        if "[V2PlanningJobSubmit] type=FULL_SEARCH" in l:
            d = kv(l)
            gens[d["planningGeneration"]] = dict(epoch=fl(d["t"]), records=[], terms={})
            cur = d["planningGeneration"]
        elif "[CompletePlanCost]" in l:
            d = kv(l)
            last_cost[(d["candidate"], d["route"])] = dict(terms=parse_terms(d["terms"]), effort=fl(d["effort"]),
                                                          J=fl(d["J"]), valid=d["valid"] == "true")
        elif "[CertStage] job=FULL_SEARCH" in l and "path=route" in l and "feasible=true" in l:
            d = kv(l)
            c = last_cost.get((d["candidate"], d["route"]))
            if c is not None and d["planningGeneration"] in gens:
                gens[d["planningGeneration"]]["terms"][(round(fl(d["lead"]), 2), d["candidate"], d["route"])] = c
                terms_by_pair[d["planningGeneration"]].setdefault((d["candidate"], d["route"]), c)
        elif "[V2CompleteRecord]" in l:
            d = kv(l)
            g = gens.get(d["planningGeneration"])
            if g is None or d["costValid"] != "true":
                continue
            key = (round(fl(d["lead"]), 2), d["candidate"], d["route"])
            c = g["terms"].get(key) or terms_by_pair[d["planningGeneration"]].get((d["candidate"], d["route"]))
            if c is None:
                continue
            g["records"].append(dict(lead=fl(d["lead"]), eventTime=fl(d["eventTime"]), cand=d["candidate"], route=d["route"],
                                     motionJ=fl(d["motionJ"]), J=fl(d["globalJ"]), Tpres=fl(d["presentationDuration"]),
                                     Texec=fl(d["executionDuration"]), reachClear=fl(d["reachClear"]),
                                     retreatClear=fl(d["retreatClear"]), terms=c["terms"], effort=c["effort"]))
    return gens


def admissible_set(g, rest):
    epoch = g["epoch"]
    out = []
    if rest:
        lead0 = min(round(r["lead"], 2) for r in g["records"])
        seen = set()
        for r in g["records"]:
            if round(r["lead"], 2) != lead0 or (r["cand"], r["route"]) in seen:
                continue
            seen.add((r["cand"], r["route"]))
            lead = max(L_COMMIT, r["Tpres"] + L_ENTRY)
            out.append(dict(r, lead=lead, eventTime=epoch + lead))
    else:
        for r in g["records"]:
            rem = r["eventTime"] - epoch
            if rem + 1e-12 >= L_COMMIT and r["Tpres"] + L_ENTRY <= rem + 1e-12:
                out.append(dict(r))
    for r in out:
        r["C"] = (r["eventTime"] - epoch) - r["Tpres"] + r["Texec"]   # time from epoch to completion
        r["minClear"] = min(r["reachClear"], r["retreatClear"])
    return out


def objective(r, w):
    s = sum(w.values())
    wn = {k: v / s for k, v in w.items()}
    motion = sum(wn[k] * r["terms"][k] for k in wn)
    return motion + wn.get("T", 0.0) * (r["lead"] - r["Tpres"]) / T_REF


def pick(rs, key):
    return min(rs, key=key) if rs else None


def rankers():
    R = {}
    R["J7"] = lambda rs: pick(rs, lambda r: (round(objective(r, W), 9), r["eventTime"], r["cand"], r["route"]))
    R["EARLIEST_TAU"] = lambda rs: pick(rs, lambda r: (round(r["eventTime"], 6), round(r["C"], 6), -r["minClear"], r["cand"], r["route"]))
    R["EARLIEST_DONE"] = lambda rs: pick(rs, lambda r: (round(r["C"], 6), -r["minClear"], r["cand"], r["route"]))
    for d in (0.2, 0.5, 1.0):
        def lex(rs, d=d):
            if not rs:
                return None
            cmin = min(r["C"] for r in rs)
            band = [r for r in rs if r["C"] <= cmin + d + 1e-9]
            return pick(band, lambda r: (-round(r["minClear"], 4), r["effort"], r["C"], r["cand"], r["route"]))
        R[f"LEX(done+{d}s -> clearance -> effort)"] = lex

    def safe(rs):
        if not rs:
            return None
        ok = [r for r in rs if r["minClear"] >= SOFT_CLEAR]
        return pick(ok, lambda r: (round(r["C"], 6), -r["minClear"], r["cand"], r["route"])) if ok else \
            pick(rs, lambda r: (-r["minClear"], r["C"]))
    R["SAFE_EARLIEST (clear >= 80 mm)"] = safe
    R["J_T+C"] = lambda rs: pick(rs, lambda r: (round(objective(r, {"T": W["T"], "C": W["C"]}), 9), r["cand"], r["route"]))
    R["J7 - V"] = lambda rs: pick(rs, lambda r: (round(objective(r, {k: v for k, v in W.items() if k != "V"}), 9), r["cand"], r["route"]))
    R["J7 - E,L"] = lambda rs: pick(rs, lambda r: (round(objective(r, {k: v for k, v in W.items() if k not in "EL"}), 9), r["cand"], r["route"]))
    R["J7 - Q,K"] = lambda rs: pick(rs, lambda r: (round(objective(r, {k: v for k, v in W.items() if k not in "QK"}), 9), r["cand"], r["route"]))
    return R


def summarize(sel, rs):
    if sel is None:
        return None
    cmin = min(r["C"] for r in rs)
    tmin = min(r["eventTime"] for r in rs)
    clear_best_band = max(r["minClear"] for r in rs if r["C"] <= cmin + 0.5 + 1e-9)
    jmin = min(objective(r, W) for r in rs)
    return dict(tuple=(round(sel["lead"], 3), sel["cand"], sel["route"]), C=sel["C"], tau=sel["eventTime"], minClear=sel["minClear"],
                reachClear=sel["reachClear"], retreatClear=sel["retreatClear"], effort=sel["effort"],
                regretDone=sel["C"] - cmin, regretTau=sel["eventTime"] - tmin, clearDeficitVsBand=clear_best_band - sel["minClear"],
                regretJ=objective(sel, W) - jmin)


def evaluate(g, rest):
    rs = admissible_set(g, rest)
    out = dict(n=len(rs))
    # J recomputation check against logged motionJ
    if rs:
        out["maxMotionJError"] = max(abs(sum(W[k] * r["terms"][k] for k in W) / sum(W.values()) - r["motionJ"]) for r in rs)
    R = rankers()
    out["rankers"] = {name: summarize(f(rs), rs) for name, f in R.items()}
    # weight sensitivity of J7
    base = R["J7"](rs)
    sens = {}
    for k in W:
        for fct in (0.5, 2.0):
            w = dict(W)
            w[k] = W[k] * fct
            s = pick(rs, lambda r: (round(objective(r, w), 9), r["eventTime"], r["cand"], r["route"]))
            sens[f"{k}x{fct}"] = None if s is None or base is None else ((s["cand"], s["route"], round(s["lead"], 3)) != (base["cand"], base["route"], round(base["lead"], 3)), s["C"] - base["C"], s["minClear"] - base["minClear"])
    out["sensitivity"] = sens
    return out


def main(argv):
    if len(argv) == 3 and argv[1] == "report":
        return report(argv[2])
    res = []
    for path in argv[2:]:
        gens = load(path)
        order = sorted(gens, key=int)
        res.append(dict(log=path, moving=evaluate(gens[order[0]], False), rest=evaluate(gens[order[-1]], True)))
    json.dump(res, open(argv[1], "w"), indent=1, default=str)
    return 0


def report(path):
    res = json.load(open(path))
    names = list(next(r for r in res)["moving"]["rankers"].keys())
    agg = {n: dict(done=[], tau=[], clear=[], def_=[], rJ=[], same=0, n=0) for n in names}
    print("### Per search\n")
    print("| dataset / scenario / epoch | admissible | ranker | tuple | completion (s) | min clearance (mm) | regret completion (s) | clearance deficit vs best within +0.5 s (mm) | J regret |")
    print("|---|---:|---|---|---:|---:|---:|---:|---:|")
    for r in res:
        label = "/".join(r["log"].split("/")[-4:-1])
        for ep in ("moving", "rest"):
            e = r[ep]
            base = e["rankers"]["J7"]
            for n in names:
                s = e["rankers"][n]
                if s is None:
                    continue
                a = agg[n]
                a["n"] += 1
                a["done"].append(s["regretDone"]); a["tau"].append(s["regretTau"]); a["clear"].append(s["minClear"])
                a["def_"].append(s["clearDeficitVsBand"]); a["rJ"].append(s["regretJ"])
                a["same"] += int(base is not None and tuple(s["tuple"]) == tuple(base["tuple"]))
                print(f"| {label}/{ep} | {e['n']} | {n} | {tuple(s['tuple'])} | {s['C']:.2f} | {1000*s['minClear']:.1f} | {s['regretDone']:.2f} | {1000*s['clearDeficitVsBand']:.1f} | {s['regretJ']:.4f} |")
    print("\n### Pooled over all searches\n")
    print("| ranker | searches | same tuple as J7 | completion regret mean / max (s) | min clearance min / median (mm) | clearance deficit vs best within +0.5 s: mean / max (mm) | J regret mean / max |")
    print("|---|---:|---:|---|---|---|---|")
    import statistics as st
    for n in names:
        a = agg[n]
        if not a["n"]:
            continue
        print(f"| {n} | {a['n']} | {a['same']} | {st.mean(a['done']):.2f} / {max(a['done']):.2f} | {1000*min(a['clear']):.1f} / {1000*st.median(a['clear']):.1f} | "
              f"{1000*st.mean(a['def_']):.1f} / {1000*max(a['def_']):.1f} | {st.mean(a['rJ']):.4f} / {max(a['rJ']):.4f} |")
    print("\n### J7 weight sensitivity (selection changes when one weight is halved or doubled and weights renormalized)\n")
    cnt = Counter(); tot = Counter(); dC = defaultdict(list); dClr = defaultdict(list)
    for r in res:
        for ep in ("moving", "rest"):
            for k, v in r[ep]["sensitivity"].items():
                if v is None:
                    continue
                tot[k] += 1
                if v[0]:
                    cnt[k] += 1
                    dC[k].append(v[1]); dClr[k].append(v[2])
    print("| perturbation | searches changed | completion change when changed (s) | min-clearance change when changed (mm) |")
    print("|---|---:|---|---|")
    for k in sorted(tot):
        rng = lambda xs, s=1.0: f"{s*min(xs):+.2f} … {s*max(xs):+.2f}" if xs else "—"
        print(f"| {k} | {cnt[k]}/{tot[k]} | {rng(dC[k])} | {rng(dClr[k], 1000)} |")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
