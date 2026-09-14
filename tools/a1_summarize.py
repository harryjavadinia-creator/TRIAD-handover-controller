#!/usr/bin/env python3
"""Markdown tables for A1 from a1_certification_depth.py JSON plus in-situ logs.

Usage: a1_summarize.py A1.json [--insitu LOG ...]
"""

import json
import re
import sys
from collections import Counter, defaultdict

KV = re.compile(r"(\w+)=(\S+)")
LEVEL_NAMES = {"1": "F1", "2": "F1+F2", "3": "F1+F2+F3", "4": "F1+F2+F3+F5", "C5": "full (C5)"}
STAGE = {"1": "F1 reach", "2": "F2 insertion", "3": "F3 closure/contact", "4": "F5 carried retreat", "audit": "terminal timing audit / cost validity"}


def pct(a, b):
    return "n/a" if not b else f"{100.0 * a / b:.1f}%"


def main(argv):
    res = json.load(open(argv[1]))
    insitu = argv[argv.index("--insitu") + 1:] if "--insitu" in argv else []
    acc = [r for r in res if r["outcome"] == "accepted"]
    out = []

    def pooled(kind):
        rs = [r for r in acc if kind == "all" or r["kind"] == kind]
        S = dict(total=0, passF=Counter(), newly=Counter(), reasons=defaultdict(Counter), fullRej=Counter(), fullAcc=0)
        R = dict(total=0, passF=Counter(), newly=Counter(), reasons=defaultdict(Counter), fullRej=Counter())
        walls = Counter()
        for r in rs:
            S["total"] += r["static"]["total"]; S["fullAcc"] += r["static"]["fullAccepted"]
            for k, v in r["static"]["passF"].items(): S["passF"][k] += v
            for k, v in r["static"]["newly"].items(): S["newly"][k] += v
            for k, v in r["static"]["fullRejGiven"].items(): S["fullRej"][k] += v
            for k, d in r["static"]["reasons"].items():
                for rr, c in d.items(): S["reasons"][k][rr] += c
            R["total"] += r["route"]["total"]
            for k, v in r["route"]["passF"].items(): R["passF"][k] += v
            for k, v in r["route"]["newly"].items(): R["newly"][k] += v
            for k, v in r["route"]["fullRejGiven"].items(): R["fullRej"][k] += v
            for k, d in r["route"]["reasons"].items():
                for rr, c in d.items(): R["reasons"][k][rr] += c
            if r.get("stageWalls"):
                for k, v in r["stageWalls"].items(): walls[k] += v
        return rs, S, R, walls

    for kind in ("moving", "rest"):
        rs, S, R, walls = pooled(kind)
        out.append(f"\n### {kind.upper()}-epoch completed searches: {len(rs)} unique ({sum(len(r['occurrences']) for r in rs)} occurrences across runs)\n")
        out.append("**Candidate survival and conditional full rejection**\n")
        out.append("| level | STATIC (τ,g) surviving | % | P(full rejection \\| STATIC level accepted) | ROUTE (τ,g,r) surviving | % | P(full rejection \\| ROUTE level accepted) |")
        out.append("|---|---:|---:|---:|---:|---:|---:|")
        for k in ("1", "2", "3", "4"):
            out.append(f"| {LEVEL_NAMES[k]} | {S['passF'][k]} | {pct(S['passF'][k], S['total'])} | {S['fullRej'][k]}/{S['passF'][k]} = {pct(S['fullRej'][k], S['passF'][k])} | "
                       f"{R['passF'][k]} | {pct(R['passF'][k], R['total'])} | {R['fullRej'][k]}/{R['passF'][k]} = {pct(R['fullRej'][k], R['passF'][k])} |")
        out.append(f"| full (C5) | {S['fullAcc']} (with ≥1 complete route) | {pct(S['fullAcc'], S['total'])} | 0 | {R['passF']['C5']} | {pct(R['passF']['C5'], R['total'])} | 0 |")
        out.append(f"\nTotals evaluated: STATIC {S['total']}, ROUTE {R['total']} (route rollouts exist only for STATIC F5 passes).\n")
        out.append("**Newly rejected by each stage and top reasons**\n")
        out.append("| population | stage | newly rejected | % of population | top reasons (count) |")
        out.append("|---|---|---:|---:|---|")
        for pop, D in (("STATIC", S), ("ROUTE", R)):
            for k in ("1", "2", "3", "4", "audit"):
                n = D["newly"].get(k, 0)
                if pop == "STATIC" and k == "audit":
                    continue
                reasons = "; ".join(f"`{rr}` {c}" for rr, c in D["reasons"][k].most_common(4)) if n else "—"
                out.append(f"| {pop} | {STAGE[k]} | {n} | {pct(n, D['total'])} | {reasons} |")
        if walls:
            out.append("\n**Worker wall by stage (sum over these searches)**\n")
            keys = ["static F1", "static F2", "static F3", "static F5", "route F1", "route F2", "route F3", "route F5", "audit+cost", "total"]
            out.append("| " + " | ".join(keys) + " |")
            out.append("|" + "---:|" * len(keys))
            out.append("| " + " | ".join(f"{walls[k]:.2f}s ({100*walls[k]/walls['total']:.0f}%)" if k != "total" else f"{walls[k]:.2f}s" for k in keys) + " |")

    out.append("\n### Selection-level counterfactual per unique completed search\n")
    out.append("Status: DETERMINED_SAME = no shallow-only candidate can be admitted at the decision time with J lower bound ≤ winner J, so the shallower policy provably selects the same tuple; NOT_DETERMINED = at least one admissible shallow-only candidate whose J is undefined could outrank the full winner (count, stages). PROXY (C4 only) = selector over full records plus audit-rejected records ranked by the logged pre-audit objective; not a policy.\n")
    out.append("| scenario | epoch | searches | decision at | C5 winner (lead, g, r) | J | reach / retreat clr (m) | C4 | C3 | C2 | C1 | C4 PROXY winner |")
    out.append("|---|---|---:|---|---|---:|---|---|---|---|---|---|")
    counts = defaultdict(Counter)
    rows = {}
    for r in sorted(acc, key=lambda r: (r["scenario"], r["kind"], -len(r["occurrences"]))):
        for lab in ("receipt", "epoch"):
            s = r["selection"][lab]
            w = s["C5"]["winner"]
            cells = []
            for lvl in ("C4", "C3", "C2", "C1"):
                e = s[lvl]
                counts[(lab, r["kind"], lvl)][e["status"]] += 1
                st = ", ".join(f"{k} {v}" for k, v in sorted(e["contenderStages"].items()))
                if e["status"] == "NOT_DETERMINED":
                    cells.append(f"NOT DET. ({e['contenders']}: {st})")
                elif e["status"] == "DETERMINED_INVALID":
                    cells.append(f"**INVALID** ({e['admissibleShallowOnly']} admissible shallow-only records: {st})")
                elif e["status"] == "POTENTIALLY_INVALID_UNOBSERVED_ROUTES":
                    cells.append(f"potentially invalid ({e['admissibleUnrolledGraspFamilies']} unrolled grasps)")
                elif e["status"] == "DETERMINED_SAME":
                    cells.append("same")
                else:
                    cells.append("none")
            pw = s["C4"].get("proxyWinner")
            ptxt = "n/a (no audit-rejected record)" if not s["C4"].get("proxyAvailable") else (
                ("same" if pw and w and tuple(pw) == tuple(w) else f"{pw} J={s['C4'].get('proxyJ', float('nan')):.4f} deepest={s['C4'].get('proxyWinnerDeepest')} audit={s['C4'].get('proxyWinnerAuditReason')}"))
            body = (f"{lab} | {tuple(w) if w else 'none'} | "
                    f"{s['C5'].get('J', float('nan')):.4f} | {s['C5'].get('reachClear', float('nan')):.3f} / {s['C5'].get('retreatClear', float('nan')):.3f} | "
                    + " | ".join(cells) + f" | {ptxt} |")
            key = (r["scenario"], r["kind"], body)
            rows.setdefault(key, [0, 0])
            rows[key][0] += len(r["occurrences"]); rows[key][1] += 1
    for (sc, kind, body), (occ, uniq) in rows.items():
        out.append(f"| {sc} | {kind} | {uniq} unique / {occ} occ. | {body}")
    out.append("\n**Determinacy counts (unique completed searches)**\n")
    cols = ["DETERMINED_SAME", "DETERMINED_INVALID", "POTENTIALLY_INVALID_UNOBSERVED_ROUTES", "DETERMINED_NONE", "NOT_DETERMINED"]
    out.append("| decision at | epoch | level | " + " | ".join(cols) + " |")
    out.append("|---|---|---|" + "---:|" * len(cols))
    for (lab, kind, lvl), c in sorted(counts.items()):
        out.append(f"| {lab} | {kind} | {lvl} | " + " | ".join(str(c[k]) for k in cols) + " |")

    if insitu:
        out.append("\n### Runtime connection: executed (full-certificate) actions in situ\n")
        out.append("| run | adoptions (kind: lead, g, r) | runtime invalidations (phase: stage/reason) | commit | completed |")
        out.append("|---|---|---|---|---|")
        for log in insitu:
            lines = open(log, errors="replace").read().splitlines()
            ad = [dict(KV.findall(l[l.find('] ['):])) for l in lines if "[V2ProvisionalAdopt]" in l]
            inv = [dict(KV.findall(l[l.find('] ['):])) for l in lines if "[V2ProvisionalInvalidated]" in l]
            com = next((dict(KV.findall(l[l.find('] ['):])) for l in lines if "[V2TerminalCommit] committed=true" in l), None)
            done = any("[Completed] full plan-once" in l for l in lines)
            run = "/".join(log.split("/")[-3:-1])
            out.append(f"| {run} | " + "; ".join(f"{a['kind']}: {a['eventLead']}, {a['candidate']}, {a['route']}" for a in ad) + " | "
                       + "; ".join(f"{i['phase']}: {'/'.join(i['reason'].split('/')[:4])}" for i in inv) + f" | {'yes' if com else 'no'} | {'yes' if done else 'no'} |")
    print("\n".join(out))


if __name__ == "__main__":
    sys.exit(main(sys.argv))
