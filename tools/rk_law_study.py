#!/usr/bin/env python3
"""Final TRIAD Phase 5: route candidate law R_k(tau, g), offline evaluation.

Inputs: characterization logs (every grasp statically screened; every static
F5-feasible (tau, g) route-certified over the configured route bank):
  - runs_ae1e824/char_R: direct + 16 directions x {40, 80, 140, 200} mm (65)
  - runs_phase5_routes/char_stretch: direct + 8 directions x {80, 140} mm (17)
    + direct routes with only the reach duration stretched (directx105 ... directx170)

  failures  why direct fails (reason), and for each direct failure which
            alternatives succeed (spatial ring by radius/direction, pure time stretch)
  laws      route subsets and lazy/failure-conditioned generators with the
            unchanged selector at the epoch (zero latency; rest epoch uses the
            Phase 3 rest law): route rollouts, selected tuple, dJ, earliest
            admissible tau, (tau, g) pairs with a complete action

Usage: rk_law_study.py OUT.json LOG [LOG ...]   |   rk_law_study.py report OUT.json
"""

import json
import lzma
import os
import re
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_search_policies as rp  # noqa: E402

KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")
W_T, T_REF = 0.4210526, 8.0


def kv(line):
    i = line.find("] [")
    return dict(KV.findall(line[i:] if i >= 0 else line))


def fl(s):
    try:
        return float(str(s).rstrip("sm"))
    except ValueError:
        return float("nan")


def read(path):
    op = lzma.open if path.endswith(".xz") else open
    with op(path, "rt", errors="replace") as f:
        return f.read().splitlines()


def route_kind(name):
    if name == "direct":
        return ("direct", 0, 0, 1.0)
    if name.startswith("directx"):
        return ("stretch", 0, 0, int(name[7:]) / 100.0)
    m = re.match(r"ring(\d+)mm_(\d+)of(\d+)", name)
    return ("ring", int(m.group(1)), (int(m.group(2)), int(m.group(3))), None)


def load(path):
    gens = {}
    for l in read(path):
        if "[V2PlanningJobSubmit] type=FULL_SEARCH" in l:
            d = kv(l)
            gens[d["planningGeneration"]] = dict(epoch=fl(d["t"]), pairs=defaultdict(dict), records=[])
        elif "[CertStage] job=FULL_SEARCH" in l and "path=route" in l:
            d = kv(l)
            g = gens.get(d["planningGeneration"])
            if g is not None:
                g["pairs"][(round(fl(d["lead"]), 2), d["candidate"])][d["route"]] = (
                    d["feasible"] == "true" and d.get("costValid") == "true", d["reason"], fl(d["routeReachDuration"]))
        elif "[V2CompleteRecord]" in l:
            d = kv(l)
            g = gens.get(d["planningGeneration"])
            if g is not None and d["costValid"] == "true":
                g["records"].append(dict(source=len(g["records"]), hyp=int(d["hypothesis"]), lead=fl(d["lead"]),
                                         eventTime=fl(d["eventTime"]), cand=d["candidate"], route=d["route"],
                                         J=fl(d["globalJ"]), motionJ=fl(d["motionJ"]), Tpres=fl(d["presentationDuration"]),
                                         Texec=fl(d["executionDuration"]), reachClear=fl(d["reachClear"]),
                                         retreatClear=fl(d["retreatClear"]), costValid=True))
    return gens


def rest_records(g):
    lead0 = min(round(r["lead"], 2) for r in g["records"])
    out = []
    for r in g["records"]:
        if round(r["lead"], 2) != lead0:
            continue
        lead = max(rp.MIN_SAFE_COMMIT_LEAD, r["Tpres"] + rp.MIN_REACH_ENTRY_LEAD)
        out.append(dict(r, eventTime=g["epoch"] + lead, J=r["motionJ"] + W_T * (lead - r["Tpres"]) / T_REF))
    return out


def failures(g):
    out = dict(directOutcomes=Counter(), afterDirectFailure=defaultdict(Counter))
    for key, routes in g["pairs"].items():
        if "direct" not in routes:
            continue
        ok, reason, dur = routes["direct"]
        cls = "success" if ok else "/".join(reason.split("/")[:2])
        out["directOutcomes"][cls] += 1
        if ok:
            continue
        for name, (rok, rreason, rdur) in routes.items():
            kind, radius, _, stretch = route_kind(name)
            if kind == "direct":
                continue
            label = f"ring{radius}" if kind == "ring" else f"x{stretch:.2f}"
            out["afterDirectFailure"][cls][label + (":ok" if rok else ":fail")] += 1
        # any stretch success / any ring success per failure
        any_ring = any(v[0] for n, v in routes.items() if n.startswith("ring"))
        any_stretch = any(v[0] for n, v in routes.items() if n.startswith("directx"))
        out["afterDirectFailure"][cls][f"anyRing={any_ring},anyStretch={any_stretch}"] += 1
    out["directOutcomes"] = dict(out["directOutcomes"])
    out["afterDirectFailure"] = {k: dict(v) for k, v in out["afterDirectFailure"].items()}
    return out


def law_subset(g, allowed_for_pair):
    """allowed_for_pair(routes_dict) -> set of route names that the law certifies for this pair"""
    certified = {}
    n = 0
    for key, routes in g["pairs"].items():
        names = allowed_for_pair(routes)
        n += len(names)
        certified[key] = names
    return certified, n


def evaluate_laws(g, rest):
    recs = rest_records(g) if rest else g["records"]
    epoch = g["epoch"]
    ref = rp.select(recs, epoch)
    all_routes = sorted({n for routes in g["pairs"].values() for n in routes})
    has_ring65 = any(n.endswith("of16") for n in all_routes)
    has_stretch = any(n.startswith("directx") for n in all_routes)

    def ring(radii, dirs):
        def f(n):
            k, r, d, _ = route_kind(n)
            return k == "ring" and r in radii and d[1] // dirs * 0 == 0 and d[0] % (d[1] // dirs) == 0
        return f

    laws = {"DIRECT_ONLY": lambda routes: {"direct"} & set(routes)}
    laws["ALL"] = lambda routes: set(routes)
    if has_ring65:
        laws["V1_17 (8 dirs x 80,140)"] = lambda routes: {n for n in routes if n == "direct" or ring((80, 140), 8)(n)}
        laws["9 (4 dirs x 80,140)"] = lambda routes: {n for n in routes if n == "direct" or ring((80, 140), 4)(n)}
        laws["5 (4 dirs x 140)"] = lambda routes: {n for n in routes if n == "direct" or ring((140,), 4)(n)}
        laws["LAZY: direct, then 8 dirs x 80,140 if direct fails"] = lambda routes: (
            {"direct"} if routes.get("direct", (False,))[0] else {n for n in routes if n == "direct" or ring((80, 140), 8)(n)})
        laws["LAZY: direct, then all 64 if direct fails"] = lambda routes: (
            {"direct"} if routes.get("direct", (False,))[0] else set(routes))
    if has_stretch:
        stretches = sorted((route_kind(n)[3], n) for n in all_routes if n.startswith("directx"))
        laws["DIRECT + stretch ladder (no rings)"] = lambda routes: {n for n in routes if n == "direct" or n.startswith("directx")}
        laws["RINGS only (17, current)"] = lambda routes: {n for n in routes if n == "direct" or n.startswith("ring")}

        def failure_conditioned(routes):
            d = routes.get("direct")
            if d is None or d[0]:
                return {"direct"} & set(routes)
            chosen = {"direct"}
            if "reach_tracking" in d[1]:
                for _, n in stretches:          # ascending stretch until one succeeds
                    if n in routes:
                        chosen.add(n)
                        if routes[n][0]:
                            return chosen
                return chosen | {n for n in routes if n.startswith("ring")}
            return chosen | {n for n in routes if n.startswith("ring")}
        laws["FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings"] = failure_conditioned

    out = dict(reference=None if ref is None else (ref["lead"], ref["cand"], ref["route"], ref["J"]), laws={})
    total = sum(len(r) for r in g["pairs"].values())
    for name, f in laws.items():
        cert, n = law_subset(g, f)
        sub = [r for r in recs if r["route"] in cert.get((round(r["lead"], 2), r["cand"]), set())]
        sel = rp.select(sub, epoch)
        adm = [r for r in sub if rp.admissible(r, epoch)]
        pairs_complete = len({(round(r["lead"], 2), r["cand"]) for r in sub})
        out["laws"][name] = dict(routeRollouts=n, fraction=n / max(1, total), completePairs=pairs_complete,
                                 selected=None if sel is None else (sel["lead"], sel["cand"], sel["route"]),
                                 dJ=None if (sel is None or ref is None) else sel["J"] - ref["J"],
                                 earliest=min((r["lead"] for r in adm), default=None) if not rest else (
                                     None if sel is None else sel["eventTime"] - epoch),
                                 clearance=None if sel is None else (sel["reachClear"], sel["retreatClear"]))
    ref_pairs = len({(round(r["lead"], 2), r["cand"]) for r in recs})
    out["referenceCompletePairs"] = ref_pairs
    return out


def main(argv):
    if len(argv) == 3 and argv[1] == "report":
        return report(argv[2])
    res = []
    for path in argv[2:]:
        gens = load(path)
        order = sorted(gens, key=int)
        mv, rs = gens[order[0]], gens[order[-1]]
        res.append(dict(log=path, movingFailures=failures(mv), restFailures=failures(rs),
                        moving=evaluate_laws(mv, False), rest=evaluate_laws(rs, True)))
    json.dump(res, open(argv[1], "w"), indent=1, default=str)
    return 0


def report(path):
    res = json.load(open(path))
    for r in res:
        name = "/".join(r["log"].split("/")[-4:-1])
        print(f"\n### {name}\n")
        for ep in ("moving", "rest"):
            f = r[ep + "Failures"]
            print(f"- {ep}: direct outcomes {f['directOutcomes']}")
            for cls, c in f["afterDirectFailure"].items():
                keys = sorted(k for k in c if k.startswith("anyRing"))
                print(f"  - after direct `{cls}`: " + "; ".join(f"{k} {c[k]}" for k in keys))
                oks = {k.split(":")[0]: (c.get(k.split(":")[0] + ":ok", 0), c.get(k.split(":")[0] + ":ok", 0) + c.get(k.split(":")[0] + ":fail", 0))
                       for k in c if ":" in k}
                print("    success/attempts by alternative: " + ", ".join(f"{k} {a}/{b}" for k, (a, b) in sorted(oks.items())))
        print()
        print("| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |")
        print("|---|---|---|---|---|---:|---|")
        for ep in ("moving", "rest"):
            e = r[ep]
            for law, v in e["laws"].items():
                sel = v["selected"]
                print(f"| {ep} | {law} | {v['routeRollouts']} ({v['fraction']:.2f}) | {v['completePairs']}/{e['referenceCompletePairs']} | "
                      f"{tuple(sel) if sel else 'none'} | {'' if v['dJ'] is None else format(v['dJ'], '+.4f')} | "
                      f"{'' if v['earliest'] is None else format(v['earliest'], '.2f')} |")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
