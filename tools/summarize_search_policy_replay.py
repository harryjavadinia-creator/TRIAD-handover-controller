#!/usr/bin/env python3
"""Markdown tables from replay_search_policies.py JSON output(s).

Usage: summarize_search_policy_replay.py REPLAY.json [REPLAY.json ...]
"""

import json
import statistics
import sys
from collections import defaultdict

POLICIES = ["FULL_ARGMIN", "FIRST_COMPLETE_FEASIBLE", "FIRST_ADMISSIBLE_COMPLETE",
            "EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR", "ANYTIME_INCUMBENT"]


def f(v, d=3):
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:+.{d}f}" if d and v < 0 else f"{v:.{d}f}"
    return str(v)


def med(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def frac(xs):
    xs = [x for x in xs if x is not None]
    return (sum(1 for x in xs if x) / len(xs), len(xs)) if xs else (None, 0)


def scen(run):
    return run.split("/")[-1]


def main(paths):
    res = []
    for p in paths:
        res += json.load(open(p))
    out = []

    # ---------------- concise policy table
    for kind in ("moving", "rest"):
        rs = [r for r in res if r["kind"] == kind and r["outcome"] == "accepted"]
        if not rs:
            continue
        out.append(f"\n### Policy comparison — {kind}-epoch searches that completed ({len(rs)} searches)\n")
        out.append("| policy | plan found | admissible & before reach-start deadline | median wall to plan (s) | median work units | median fraction of full work avoided | median t_plan − t_rest (s) | τ = FULL_ARGMIN τ | tuple = FULL_ARGMIN | median / max ΔJ vs FULL_ARGMIN | τ = zero-latency argmin τ | tuple = zero-latency argmin | median / max ΔJ vs zero-latency argmin | median Δ reach clearance (m) | median Δ retreat clearance (m) |")
        out.append("|---|---|---|---:|---:|---:|---:|---|---|---|---|---|---|---:|---:|")
        for pol in POLICIES:
            os_ = [r["policies"].get(pol) for r in rs]
            av = [o for o in os_ if o and o.get("available")]
            ok = [o["commitLeadOk"] and o["beforeReachStartDeadline"] for o in av]
            def rate(key):
                v, n = frac([o.get(key) for o in av])
                return "n/a" if v is None else f"{v*100:.0f}% ({n})"
            dj = [o.get("dJ") for o in av if o.get("dJ") is not None]
            dje = [o.get("dJVsEpochFull") for o in av if o.get("dJVsEpochFull") is not None]
            out.append(f"| {pol} | {len(av)}/{len(rs)} | {sum(ok)}/{len(av)} | {f(med([o['wallToPlan'] for o in av]))} | "
                       f"{f(med([o['unitsToPlan'] for o in av]),0)} | {f(med([o['fractionUnitsAvoided'] for o in av]),2)} | "
                       f"{f(med([o['tMinusRest'] for o in av]),2)} | {rate('sameTau')} | {rate('sameTuple')} | "
                       f"{f(med(dj),4)} / {f(max(dj) if dj else None,4)} | {rate('sameTauVsEpochFull')} | {rate('sameTupleVsEpochFull')} | "
                       f"{f(med(dje),4)} / {f(max(dje) if dje else None,4)} | {f(med([o.get('dReachClear') for o in av]),4)} | {f(med([o.get('dRetreatClear') for o in av]),4)} |")

    # ---------------- per-search detail
    out.append("\n### Per search: plan availability, tuple, cost and clearance\n")
    out.append("ΔJ: vs FULL_ARGMIN at its own receipt / vs the zero-latency exhaustive argmin (all records admitted at the search epoch). "
               "H/G/R: temporal hypotheses touched / grasp screens / route rollouts up to the stop. Deadline: available with ≥1.6 s to τ and reach duration + 0.05 s ≤ τ − t.\n")
    out.append("| search | kind | outcome | policy | wall to plan (s) | work units | t − t_rest (s) | deadline ok | τ lead (s) | g | r | J | ΔJ vs FULL / vs zero-latency | same τ,g,r as FULL | same τ,g,r as zero-latency | reach clr (m) | retreat clr (m) | H/G/R | full work avoided |")
    out.append("|---|---|---|---|---:|---:|---:|---|---:|---|---|---:|---|---|---|---:|---:|---|---:|")
    for r in sorted(res, key=lambda r: (scen(r["run"]), r["run"], int(r["gen"]))):
        for pol in POLICIES:
            o = r["policies"].get(pol)
            if o is None:
                continue
            name = f"{r['run']} g{r['gen']}"
            if not o.get("available"):
                out.append(f"| {name} | {r['kind']} | {r['outcome']} | {pol} | none | | | | | | | | | | | | | | |")
                continue
            same = lambda s: "/".join("τ" if o.get("sameTau" + s) else "·" for _ in [0]) + ("g" if o.get("sameG" + s) else "·") + ("r" if o.get("sameR" + s) else "·") if o.get("sameTau" + s) is not None else "n/a"
            out.append(f"| {name} | {r['kind']} | {r['outcome']} | {pol} | {o['wallToPlan']:.3f} | {o['unitsToPlan']} | {o['tMinusRest']:+.2f} | "
                       f"{f(o['commitLeadOk'] and o['beforeReachStartDeadline'])} | {o['lead']:.2f} | {o['g']} | {o['r']} | {o['J']:.4f} | "
                       f"{f(o.get('dJ'),4)} / {f(o.get('dJVsEpochFull'),4)} | {same('')} | {same('VsEpochFull')} | {o['reachClear']:.3f} | {o['retreatClear']:.3f} | "
                       f"{o['hypothesesEvaluated']}/{o['graspScreens']}/{o['routeRollouts']} | {f(o['fractionUnitsAvoided'],2)} |")

    # ---------------- structural question D
    out.append("\n### Does exhaustive argmin behave like 'earliest timing-admissible τ, then best (g,r)'?\n")
    for label, key_b, key_f in (("zero latency (both evaluated with every record admitted at the search epoch)", "earliestTauAtEpoch", "fullArgminAtEpoch"),):
        rows = [(r, r.get(key_b), r.get(key_f)) for r in res if r["outcome"] == "accepted" and r.get(key_f) is not None]
        for kind in ("moving", "rest", "all"):
            sub = [(r, b, fa) for r, b, fa in rows if kind == "all" or r["kind"] == kind]
            if not sub:
                continue
            n = len(sub)
            tau = sum(1 for _, b, fa in sub if b and abs(b["eventTime"] - fa["eventTime"]) < 1e-9)
            g = sum(1 for _, b, fa in sub if b and b["cand"] == fa["cand"])
            rr = sum(1 for _, b, fa in sub if b and b["route"] == fa["route"])
            tup = sum(1 for _, b, fa in sub if b and abs(b["eventTime"] - fa["eventTime"]) < 1e-9 and b["cand"] == fa["cand"] and b["route"] == fa["route"])
            out.append(f"- {label}, {kind}: n={n}; P(τ equal)={tau}/{n}, P(g equal)={g}/{n}, P(r equal)={rr}/{n}, P(tuple equal)={tup}/{n}")
    for kind in ("moving", "rest", "all"):
        sub = [r for r in res if r["outcome"] == "accepted" and (kind == "all" or r["kind"] == kind)
               and r["policies"]["FULL_ARGMIN"].get("available") and r["policies"]["EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR"].get("available")]
        n = len(sub)
        if not n:
            continue
        o = [r["policies"]["EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR"] for r in sub]
        out.append(f"- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), {kind}: n={n}; "
                   f"P(τ equal)={sum(x['sameTau'] for x in o)}/{n}, P(g equal)={sum(x['sameG'] for x in o)}/{n}, "
                   f"P(r equal)={sum(x['sameR'] for x in o)}/{n}, P(tuple equal)={sum(x['sameTuple'] for x in o)}/{n}")
    nofull = [r for r in res if r["outcome"] == "accepted" and not r["policies"]["FULL_ARGMIN"].get("available")]
    out.append(f"- completed searches in which FULL_ARGMIN had no admissible plan at receipt: {len(nofull)} "
               f"({', '.join(r['run'] + ' g' + r['gen'] for r in nofull) or 'none'}); bounded policies with an admissible plan in those: "
               + ", ".join(f"{p}={sum(1 for r in nofull if r['policies'][p].get('available') and r['policies'][p]['commitLeadOk'] and r['policies'][p]['beforeReachStartDeadline'])}"
                           for p in POLICIES[1:]))

    # ---------------- anytime evolution
    out.append("\n### ANYTIME_INCUMBENT evolution (moving-epoch searches)\n")
    out.append("| search | first incumbent wall (s) | incumbent changes | incumbent lost to expiry | incumbent J (ΔJ vs zero-latency argmin) at 10% / 25% / 50% / 75% / 100% of the full search wall |")
    out.append("|---|---:|---:|---:|---|")
    for r in sorted([r for r in res if r["kind"] == "moving"], key=lambda r: r["run"]):
        a = r["anytime"]
        fe = r.get("fullArgminAtEpoch")
        cells = []
        for k in ("0.1", "0.25", "0.5", "0.75", "1.0"):
            x = a["atFractionOfFullWall"][k]
            cells.append("none" if x is None else f"{x['J']:.3f} ({f(x['J'] - fe['J'], 3) if fe else 'n/a'}) τ={x['lead']:.2f}")
        out.append(f"| {r['run']} g{r['gen']} ({r['outcome']}) | {f(a['firstIncumbentWall'])} | {a['changes']} | {a['losses']} | {' / '.join(cells)} |")

    # ---------------- computation breakdown
    out.append("\n### Computation before the stop (moving-epoch searches; worker wall in s, bounded work units in parentheses)\n")
    cats = None
    for r in res:
        o = r["policies"]["FIRST_COMPLETE_FEASIBLE"]
        if o.get("available") and o.get("breakdown"):
            cats = list(o["breakdown"].keys())
            break
    if cats:
        out.append("| search | policy | " + " | ".join(cats) + " |")
        out.append("|---|---|" + "---:|" * len(cats))
        for r in sorted([r for r in res if r["kind"] == "moving"], key=lambda r: r["run"]):
            for pol in POLICIES:
                o = r["policies"].get(pol)
                if not o or not o.get("available") or not o.get("breakdown"):
                    continue
                b = o["breakdown"]
                out.append(f"| {r['run']} g{r['gen']} | {pol} | " + " | ".join(f"{b[c][1]:.3f} ({b[c][0]})" for c in cats) + " |")
    out.append("\n### Per-candidate certification cost and work before the first complete certified record\n")
    out.append("| search | kind | full wall (s) | static screens rejected / passed (median ms) | route rollouts rejected / complete (median ms) | before first complete: rejected screens, passed screens, rejected routes, hypotheses | wall of rejected screens / passed screens / rejected routes / the successful route (s) |")
    out.append("|---|---|---:|---|---|---|---|")
    for r in sorted(res, key=lambda r: (scen(r["run"]), r["run"], int(r["gen"]))):
        pc = r["perCandidateWall"]
        bf = r.get("beforeFirstComplete") or {}
        g = lambda k: pc.get(k, {"n": 0, "median": float("nan")})
        out.append(f"| {r['run']} g{r['gen']} | {r['kind']} | {r['fullWall']:.3f} | {g('static/rejected')['n']} ({g('static/rejected')['median']*1000:.2f}) / {g('static/feasible')['n']} ({g('static/feasible')['median']*1000:.2f}) | "
                   f"{g('route/rejected')['n']} ({g('route/rejected')['median']*1000:.2f}) / {g('route/feasible')['n']} ({g('route/feasible')['median']*1000:.2f}) | "
                   f"{bf.get('staticScreensFailed','n/a')}, {bf.get('staticScreensPassed','n/a')}, {bf.get('routeRolloutsFailed','n/a')}, {bf.get('hypothesesTouched','n/a')} | "
                   f"{f(bf.get('wallStaticFailed'))} / {f(bf.get('wallStaticPassed'))} / {f(bf.get('wallRouteFailed'))} / {f(bf.get('wallSuccessfulRoute'))} |")
    print("\n".join(out))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
