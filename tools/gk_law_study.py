#!/usr/bin/env python3
"""Final TRIAD Phase 4: grasp candidate law G_k(tau), offline evaluation.

Input: runs_ae1e824/char_G characterization logs (64 approach angles per handle
axis sign = 5.625 deg, 14 bank leads, every grasp statically screened and every
static-feasible grasp route-certified, moving and rest epochs). Coarser angle
sets are exact subsamples.

  arcs      contiguous feasible arcs around the handle axis (static screen pass
            through carried retreat; complete action with any route) per lead
            and sign; widths
  laws      UNIFORM(n) angle subsets and ADAPT(n0) coarse-to-fine refinement
            (refine around statically feasible samples down to the 64 grid)
            evaluated with the unchanged selector at the epoch (zero latency):
            static screens, route rollouts, selected tuple, dJ and earliest
            admissible tau against the 64-angle reference

Usage: gk_law_study.py OUT.json LOG [LOG ...]   |   gk_law_study.py report OUT.json
"""

import json
import lzma
import math
import os
import re
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_search_policies as rp  # noqa: E402

KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")
NANG = 64


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


def parse_name(name):
    sign = "P" if name.startswith("axisP") else "N"
    deg = int(name.split("_")[-1].replace("deg", ""))
    return sign, deg


def load(path):
    gens = {}
    for l in read(path):
        if "[V2PlanningJobSubmit] type=FULL_SEARCH" in l:
            d = kv(l)
            gens[d["planningGeneration"]] = dict(epoch=fl(d["t"]), speed=fl(d["objectEstimateSpeed"]),
                                                 static={}, routes=defaultdict(int), records=[])
        elif "[CertStage] job=FULL_SEARCH" in l:
            d = kv(l)
            g = gens.get(d["planningGeneration"])
            if g is None:
                continue
            lead = round(fl(d["lead"]), 2)
            if d["path"] == "static":
                idx = int(d["grasp"].split("/")[0])
                g["static"][(lead, d["candidate"])] = (idx, d["deepest"], fl(d["staticReachTime"]))
            elif d["path"] == "route":
                g["routes"][(lead, d["candidate"])] += 1
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


def angle_index(g):
    # candidate name -> (sign, index on the 64 grid) from the static records
    out = {}
    for (lead, cand), (idx, deepest, ts) in g["static"].items():
        sign, _ = parse_name(cand)
        out[cand] = (sign, idx % NANG)
    return out


def arcs_of(indices):
    """circular contiguous runs on the 64 grid; returns widths in samples"""
    s = sorted(set(indices))
    if not s:
        return []
    if len(s) == NANG:
        return [NANG]
    runs, start, prev = [], s[0], s[0]
    for x in s[1:]:
        if x == prev + 1:
            prev = x
            continue
        runs.append((start, prev))
        start = prev = x
    runs.append((start, prev))
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][1] == NANG - 1:
        runs[0] = (runs[-1][0], runs[0][1] + NANG)
        runs.pop()
    return [b - a + 1 for a, b in runs]


def arcs_members(sorted_idx):
    s = sorted(set(sorted_idx))
    if not s:
        return []
    runs, cur = [], [s[0]]
    for x in s[1:]:
        if x == cur[-1] + 1:
            cur.append(x)
        else:
            runs.append(cur)
            cur = [x]
    runs.append(cur)
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == NANG - 1:
        runs[0] = runs[-1] + runs[0]
        runs.pop()
    return runs


def arc_stats(g, idxmap):
    leads = sorted({l for (l, c) in g["static"]})
    complete_cands = defaultdict(set)
    for r in g["records"]:
        complete_cands[round(r["lead"], 2)].add(r["cand"])
    stat = dict(staticArcs=[], completeArcs=[])
    for lead in leads:
        for sign in ("P", "N"):
            sp = [idxmap[c][1] for (l, c), (i, dp, ts) in g["static"].items() if l == lead and idxmap[c][0] == sign and dp == "CARRIED_RETREAT"]
            cp = [idxmap[c][1] for c in complete_cands[lead] if c in idxmap and idxmap[c][0] == sign]
            stat["staticArcs"] += arcs_of(sp)
            stat["completeArcs"] += arcs_of(cp)
    return stat


W_T, T_REF = 0.4210526, 8.0


def rest_records(g):
    lead0 = min(round(r["lead"], 2) for r in g["records"])
    out = []
    for r in g["records"]:
        if round(r["lead"], 2) != lead0:
            continue
        lead = max(rp.MIN_SAFE_COMMIT_LEAD, r["Tpres"] + rp.MIN_REACH_ENTRY_LEAD)
        out.append(dict(r, lead=lead0, eventTime=g["epoch"] + lead, J=r["motionJ"] + W_T * (lead - r["Tpres"]) / T_REF))
    return out


def evaluate(g, laws, rest=False):
    idxmap = angle_index(g)
    epoch = g["epoch"]
    if rest:
        g = dict(g, records=rest_records(g))
    ref = rp.select(g["records"], epoch)
    ref_adm = [r for r in g["records"] if rp.admissible(r, epoch)]
    ref_earliest = min((r["lead"] for r in ref_adm), default=None)
    leads = sorted({l for (l, c) in g["static"]})
    out = dict(epoch=epoch, speed=g["speed"], reference=None if ref is None else (ref["lead"], ref["cand"], ref["route"], ref["J"]),
               referenceEarliest=ref_earliest, arcs=arc_stats(g, idxmap), laws={})
    total_static = len(g["static"])
    total_routes = sum(g["routes"].values())
    for name, chooser in laws.items():
        chosen = set()
        for lead in leads:
            for sign in ("P", "N"):
                passing = lambda i: (lead, f"axis{sign}_side_" + name_deg(g, idxmap, sign, i)) in g["static"] and \
                    g["static"][(lead, f"axis{sign}_side_" + name_deg(g, idxmap, sign, i))][1] == "CARRIED_RETREAT"
                for i in chooser(passing):
                    chosen.add((lead, sign, i))
        # map back to candidate names
        inv = {(s, i): c for c, (s, i) in idxmap.items()}
        chosen_names = {(lead, inv[(s, i)]) for (lead, s, i) in chosen if (s, i) in inv}
        n_static = len(chosen_names)
        route_names = chosen_names
        if name.startswith("ARCREP"):
            k = int(name[len("ARCREP("):-1])
            route_names = set()
            for lead in leads:
                for sign in ("P", "N"):
                    feas = sorted(i for i in range(NANG) if (sign, i) in inv and (lead, inv[(sign, i)]) in g["static"]
                                  and g["static"][(lead, inv[(sign, i)])][1] == "CARRIED_RETREAT")
                    for arc in arcs_members(feas):
                        m = len(arc)
                        picks = {arc[m // 2]} if k == 1 else {arc[0], arc[m // 2], arc[-1]}
                        route_names |= {(lead, inv[(sign, i)]) for i in picks}
        n_routes = sum(g["routes"].get(k, 0) for k in route_names
                       if g["static"].get(k, (0, "", 0))[1] == "CARRIED_RETREAT")
        sub = [r for r in g["records"] if (round(r["lead"], 2), r["cand"]) in route_names]
        sel = rp.select(sub, epoch)
        adm = [r for r in sub if rp.admissible(r, epoch)]
        earliest = min((r["lead"] for r in adm), default=None)
        out["laws"][name] = dict(
            staticScreens=n_static, staticFraction=n_static / total_static, routeRollouts=n_routes,
            routeFraction=n_routes / max(1, total_routes),
            selected=None if sel is None else (sel["lead"], sel["cand"], sel["route"]),
            dJ=None if (sel is None or ref is None) else sel["J"] - ref["J"],
            earliestAdmissible=earliest,
            dEarliest=None if (earliest is None or ref_earliest is None) else earliest - ref_earliest,
            clearance=None if sel is None else (sel["reachClear"], sel["retreatClear"]))
    return out


_deg_cache = {}


def name_deg(g, idxmap, sign, i):
    key = (id(g), sign, i)
    if key not in _deg_cache:
        for c, (s, j) in idxmap.items():
            if s == sign and j == i:
                _deg_cache[key] = c.split("_side_")[1]
                break
        else:
            _deg_cache[key] = "missing"
    return _deg_cache[key]


def uniform(n):
    stride = NANG // n
    return lambda passing: [i for i in range(0, NANG, stride)]


def adaptive(n0, radius_samples=None):
    """coarse n0 per sign; then, for every statically feasible sample, recursively
    refine the neighbouring half-intervals down to the 64 grid (both sides), and
    refine every coarse interval whose one end passes. radius_samples limits the
    refinement around a pass to +-radius (in 64-grid samples)."""
    stride = NANG // n0

    def chooser(passing):
        chosen = set(range(0, NANG, stride))
        frontier = [i for i in chosen if passing(i)]
        step = stride
        while step > 1:
            half = step // 2
            new = []
            for i in frontier:
                for j in ((i + half) % NANG, (i - half) % NANG):
                    if j not in chosen:
                        chosen.add(j)
                        if passing(j):
                            new.append(j)
            frontier = list(set(frontier + new))
            step = half
        return sorted(chosen)
    return chooser


def main(argv):
    if len(argv) == 3 and argv[1] == "report":
        return report(argv[2])
    laws = {f"UNIFORM({n})": uniform(n) for n in (4, 8, 16, 32, 64)}
    laws.update({f"ADAPT({n})": adaptive(n) for n in (4, 8, 16)})
    laws.update({"ARCREP(1)": uniform(64), "ARCREP(3)": uniform(64)})
    res = []
    for path in argv[2:]:
        gens = load(path)
        order = sorted(gens, key=int)
        res.append(dict(log=path, moving=evaluate(gens[order[0]], laws), rest=evaluate(gens[order[-1]], laws, rest=True)))
    json.dump(res, open(argv[1], "w"), indent=1, default=str)
    return 0


def report(path):
    res = json.load(open(path))
    print("### Feasible arc widths (samples of 5.625 deg; per lead and axis sign)\n")
    print("| scenario / epoch | static-feasible arcs: count, min, median, singletons | complete-action arcs: count, min, median, singletons |")
    print("|---|---|---|")
    import statistics
    for r in res:
        for ep in ("moving", "rest"):
            a = r[ep]["arcs"]
            def st(x):
                return f"{len(x)}, {min(x) if x else '—'}, {statistics.median(x) if x else '—'}, {sum(1 for w in x if w == 1)}"
            print(f"| {r['log'].split('/')[-2]} / {ep} | {st(a['staticArcs'])} | {st(a['completeArcs'])} |")
    print("\n### Laws (zero-latency selection at the search epoch; reference = 64 angles per sign)\n")
    print("| scenario / epoch | law | static screens (fraction) | route rollouts (fraction) | selected (lead, grasp, route) | ΔJ | earliest admissible lead (Δ) |")
    print("|---|---|---|---|---|---:|---|")
    for r in res:
        for ep in ("moving", "rest"):
            e = r[ep]
            for name, v in e["laws"].items():
                sel = v["selected"]
                print(f"| {r['log'].split('/')[-2]} / {ep} | {name} | {v['staticScreens']} ({v['staticFraction']:.2f}) | {v['routeRollouts']} ({v['routeFraction']:.2f}) | "
                      f"{tuple(sel) if sel else 'none'} | {'' if v['dJ'] is None else f'{v[chr(100)+chr(74)]:+.4f}'} | "
                      f"{v['earliestAdmissible']} ({'' if v['dEarliest'] is None else f'{v[chr(100)+chr(69)+chr(97)+chr(114)+chr(108)+chr(105)+chr(101)+chr(115)+chr(116)]:+.2f}'}) |")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
