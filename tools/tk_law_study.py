#!/usr/bin/env python3
"""Final TRIAD Phase 3: temporal candidate-set law T_k, offline evaluation.

Input: characterization logs with a dense lead grid (runs_ae1e824/char_T: 191
leads on [0.50, 10.00] s at 0.05 s, every grasp and route certified at both the
moving epoch and the rest epoch). Every coarser lead set is an exact subsample
of the same search (checked in V2_COMPUTATION_AND_CANDIDATE_SPACE §5).

For each epoch it evaluates lead-set laws with the unchanged selector replica
(tools/replay_search_policies.py: timing admission, cost, tie-break) at a
decision time equal to the epoch plus the modelled worker wall of the leads the
law evaluates (per-lead cost from the job profile: static screens and route
rollouts at that lead times the measured mean unit cost).

  FULL        all 191 leads
  BANK14      the V1/V2 fixed bank
  GRID(d)     every lead on a d-spaced grid from 1.60 s (no stopping)
  ASC(d, W)   ascending leads on a d-spaced grid from L_commit + c_hat; stop
              when the leads evaluated so far contain a record admissible at the
              current decision time, then continue for a window W beyond that
              lead, then select at the final decision time
  REST        (rest epoch) a single pose, each certified (g, r) assigned
              tau = t_dec + max(L_commit, T_pres + L_entry)

Usage: tk_law_study.py OUT.json LOG [LOG ...]   |   tk_law_study.py report OUT.json
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
L_COMMIT, L_ENTRY = rp.MIN_SAFE_COMMIT_LEAD, rp.MIN_REACH_ENTRY_LEAD
BANK14 = [1.80, 1.90, 2.35, 2.80, 3.25, 3.70, 4.15, 4.60, 5.05, 5.50, 5.95, 6.40, 6.85, 8.00]
STATIC_BUCKETS = ["staticReachStandoff", "staticReachCapture", "staticClosure", "staticRetreat"]
ROUTE_BUCKETS = ["routeSetup", "routeReach", "routeApproach", "routeDwell", "routeClosure", "routeRetreat", "routeFinalize"]


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


def key(x):
    return round(x + 1e-9, 2)


def load(path):
    lines = read(path)
    gens = {}
    for l in lines:
        if "[V2PlanningJobSubmit] type=FULL_SEARCH" in l:
            d = kv(l)
            gens[d["planningGeneration"]] = dict(epoch=fl(d["t"]), speed=fl(d["objectEstimateSpeed"]), records=[],
                                                 static=defaultdict(int), route=defaultdict(int), statics=defaultdict(list),
                                                 routeByGrasp=defaultdict(int),
                                                 profile=None)
        elif "[V2CompleteRecord]" in l:
            d = kv(l)
            g = gens.get(d["planningGeneration"])
            if g is not None:
                g["records"].append(dict(source=len(g["records"]), hyp=int(d["hypothesis"]), lead=fl(d["lead"]),
                                         eventTime=fl(d["eventTime"]), cand=d["candidate"], route=d["route"],
                                         J=fl(d["globalJ"]), motionJ=fl(d["motionJ"]), Tpres=fl(d["presentationDuration"]),
                                         Texec=fl(d["executionDuration"]), reachClear=fl(d["reachClear"]),
                                         retreatClear=fl(d["retreatClear"]), costValid=d["costValid"] == "true"))
        elif "[CertStage] job=FULL_SEARCH" in l:
            d = kv(l)
            g = gens.get(d["planningGeneration"])
            if g is None:
                continue
            lead = key(fl(d["lead"]))
            if d["path"] == "static":
                g["static"][lead] += 1
                g["statics"][lead].append((d["candidate"], d["deepest"], fl(d["staticReachTime"])))
            elif d["path"] == "route":
                g["route"][lead] += 1
                g["routeByGrasp"][(lead, d["candidate"])] += 1
        elif "[V2JobProfile] type=FULL_SEARCH" in l:
            d = kv(l)
            if d["planningGeneration"] in gens:
                gens[d["planningGeneration"]]["profile"] = d
    return gens


def unit_costs(g):
    p = g["profile"]
    sw = sum(fl(p[b].split("/")[0]) for b in STATIC_BUCKETS)
    rw = sum(fl(p[b].split("/")[0]) for b in ROUTE_BUCKETS)
    ns = int(p["staticRecords"]) or 1
    nr = int(p["routeRecords"]) or 1
    return sw / ns, rw / nr


def lead_cost(g, lead, cs, cr, memo_first=None):
    # memoized (rest) searches: only the first hypothesis pays certification
    return g["static"].get(key(lead), 0) * cs + g["route"].get(key(lead), 0) * cr


def select_at(records, leads, now):
    S = {key(l) for l in leads}
    sub = [r for r in records if key(r["lead"]) in S]
    return rp.select(sub, now)


def summary(sel, ref):
    if sel is None:
        return dict(selected=None)
    return dict(selected=(sel["lead"], sel["cand"], sel["route"]), J=sel["J"], tau=sel["eventTime"],
                dJ=None if ref is None else sel["J"] - ref["J"],
                dTau=None if ref is None else sel["eventTime"] - ref["eventTime"],
                reachClear=sel["reachClear"], retreatClear=sel["retreatClear"])


def evaluate_moving(g):
    recs = [r for r in g["records"] if r["costValid"]]
    grid = sorted({key(l) for l in g["static"].keys()})
    cs, cr = unit_costs(g)
    epoch = g["epoch"]
    ref0 = select_at(recs, grid, epoch)  # zero-latency full-grid reference
    out = dict(epoch=epoch, speed=g["speed"], unitCostStatic=cs, unitCostRoute=cr, leadsTotal=len(grid),
               referenceZeroLatency=summary(ref0, None))
    # earliest admissible tau on the full grid at zero latency
    adm = [r for r in recs if rp.admissible(r, epoch)]
    out["earliestAdmissibleLead0"] = min((r["lead"] for r in adm), default=None)
    laws = {}

    def run_fixed(name, leads):
        leads = [l for l in grid if any(abs(l - x) < 1e-6 for x in leads)]
        cost = sum(lead_cost(g, l, cs, cr) for l in leads)
        sel = select_at(recs, leads, epoch + cost)
        laws[name] = dict(leads=len(leads), cost=cost, **summary(sel, ref0))

    run_fixed("FULL", grid)
    run_fixed("BANK14", BANK14)
    for d in (0.05, 0.10, 0.15, 0.20, 0.45):
        run_fixed(f"GRID({d:.2f})", [x for x in grid if x >= 1.6 - 1e-9 and abs(((x - 1.6) / d) - round((x - 1.6) / d)) < 1e-6])

    def run_asc(name, d, W, start):
        ladder = [x for x in grid if x >= start - 1e-9 and abs(((x - start) / d) - round((x - start) / d)) < 1e-6]
        cost, evaluated, first = 0.0, [], None
        for x in ladder:
            if first is not None and x > first + W + 1e-9:
                break
            cost += lead_cost(g, x, cs, cr)
            evaluated.append(x)
            if first is None and select_at(recs, evaluated, epoch + cost) is not None:
                first = x
        sel = select_at(recs, evaluated, epoch + cost)
        laws[name] = dict(leads=len(evaluated), cost=cost, firstAdmissibleLead=first, **summary(sel, ref0))

    def run_pruned(name, ladder_leads, W=None):
        # Exact timing prune (Track 1) and dead-lead skipping: a lead whose remaining
        # time at the current decision time is below L_commit costs nothing; a grasp
        # whose static reach time already fails admission skips its route rollouts.
        cost, evaluated_recs, first, n_leads, n_routes = 0.0, [], None, 0, 0
        for x in ladder_leads:
            if W is not None and first is not None and x > first + W + 1e-9:
                break
            now = epoch + cost
            if epoch + x - now + 1e-12 < L_COMMIT:
                continue
            n_leads += 1
            cost += g["static"].get(key(x), 0) * cs
            now = epoch + cost
            keep = set()
            for cand, deepest, ts in g["statics"].get(key(x), []):
                if deepest != "CARRIED_RETREAT":
                    continue
                if ts + L_ENTRY > epoch + x - now + 1e-12:
                    continue
                keep.add(cand)
                nr = g["routeByGrasp"].get((key(x), cand), 0)
                cost += nr * cr
                n_routes += nr
            evaluated_recs += [r for r in recs if key(r["lead"]) == key(x) and r["cand"] in keep]
            if first is None and rp.select(evaluated_recs, epoch + cost) is not None:
                first = x
        sel = rp.select(evaluated_recs, epoch + cost)
        laws[name] = dict(leads=n_leads, routeRollouts=n_routes, cost=cost, firstAdmissibleLead=first, **summary(sel, ref0))

    run_pruned("FULL+prune", grid)
    run_pruned("BANK14+prune", [x for x in grid if any(abs(x - b) < 1e-6 for b in BANK14)])

    speed = g["speed"]
    d_phys = 0.015 / speed if speed > 1e-6 else float("inf")
    d_grid = max(0.05, math.floor(d_phys / 0.05 + 1e-9) * 0.05)
    out["deltaTauPhysical"] = d_phys
    out["deltaTauOnGrid"] = d_grid
    for W in (0.0, 0.5, 1.0, 2.0):
        run_asc(f"ASC(dphys,W={W})", d_grid, W, 1.6)
        run_asc(f"ASC(0.45,W={W})", 0.45, W, 1.6)
    for d in sorted({d_grid, 0.05, 0.10, 0.20, 0.45}):
        ladder = [x for x in grid if x >= 1.6 - 1e-9 and abs(((x - 1.6) / d) - round((x - 1.6) / d)) < 1e-6]
        for W in (0.0, 0.5, 1.0, 2.0, None):
            run_pruned(f"ASC+prune(d={d:.2f},W={'inf' if W is None else W})", ladder, W)
    # Latency-anchored ladder (proposed law): probe the static screen at the first
    # lead that can still be admissible after the latency allowance A, take
    # h_lo = max(L_commit, min_g T_s + L_entry), then evaluate tau_j = A + h_lo + j*d
    # (rounded up to the 0.05 s grid) with the exact prune and window W.
    per_lead_units = {}
    def probe_hlo(A):
        x0 = min((x for x in grid if x >= L_COMMIT + A - 1e-9), default=None)
        if x0 is None:
            return None, 0.0
        ts = [t for (c, dpst, t) in g["statics"].get(key(x0), []) if dpst == "CARRIED_RETREAT" and math.isfinite(t)]
        cost = g["static"].get(key(x0), 0) * cs
        if not ts:
            return L_COMMIT, cost
        return max(L_COMMIT, min(ts) + L_ENTRY), cost
    anchored = {}
    for A in (0.5, 1.0, 1.5, 2.0):
        hlo, probe_cost = probe_hlo(A)
        if hlo is None:
            continue
        for d in sorted({d_grid, 0.20, 0.30, 0.45}):
            for W in (0.0, 0.5):
                ladder, j = [], 0
                while True:
                    x = A + hlo + j * d
                    if x > grid[-1] + 1e-9:
                        break
                    xg = min((y for y in grid if y >= x - 1e-9), default=None)
                    if xg is not None and (not ladder or xg > ladder[-1] + 1e-9):
                        ladder.append(xg)
                    j += 1
                name = f"ANCHOR(A={A},d={d:.2f},W={W})"
                run_pruned(name, ladder, W)
                laws[name]["cost"] += probe_cost
                laws[name]["hlo"] = hlo
                laws[name]["consistent"] = laws[name]["cost"] <= A + 1e-9
                sel = None
    out["laws"] = laws
    return out


def evaluate_rest(g):
    recs = [r for r in g["records"] if r["costValid"]]
    grid = sorted({key(l) for l in g["static"].keys()})
    cs, cr = unit_costs(g)
    epoch = g["epoch"]
    # memoized: all leads share the first hypothesis's certification; wall ~ profile
    first_lead = grid[0]
    one = [r for r in recs if key(r["lead"]) == first_lead]
    poses = {r.get("pose") for r in recs}
    cost_one = lead_cost(g, first_lead, cs, cr)
    out = dict(epoch=epoch, speed=g["speed"], leadsTotal=len({key(r["lead"]) for r in recs}), recordsPerLead=len(one))
    # identical-record check across leads (memoization exactness): same (cand, route) -> same motionJ, Tpres
    by = defaultdict(set)
    for r in recs:
        by[(r["cand"], r["route"])].add((round(r["motionJ"], 9), round(r["Tpres"], 6), round(r["reachClear"], 5)))
    out["recordsIdenticalAcrossLeads"] = all(len(v) == 1 for v in by.values())
    ref_full = rp.select(recs, epoch)
    ref_full_latency = rp.select(recs, epoch + fl(g["profile"]["jobWall"]))
    bank = [r for r in recs if any(abs(r["lead"] - x) < 1e-6 for x in BANK14)]
    ref_bank = rp.select(bank, epoch + fl(g["profile"]["jobWall"]))
    # REST law: one pose, continuous tau per record at decision time
    t_dec = epoch + cost_one
    law = []
    w_T, T_ref = 0.4210526, 8.0
    for r in one:
        lead = max(L_COMMIT, r["Tpres"] + L_ENTRY) + (t_dec - epoch)
        J = r["motionJ"] + w_T * (lead - r["Tpres"]) / T_ref
        law.append(dict(r, lead=lead, eventTime=epoch + lead, J=J, source=r["source"]))
    sel = rp.select(law, t_dec)
    out.update(fullZeroLatency=summary(ref_full, None), fullAtOwnLatency=summary(ref_full_latency, ref_full),
               bank14AtFullLatency=summary(ref_bank, ref_full), restLaw=summary(sel, ref_full),
               restLawCost=cost_one, fullCost=fl(g["profile"]["jobWall"]))
    # check of the dominance argument: for every (g, r) the earliest admissible grid lead
    # gives the minimum J among that pair's admissible grid leads
    viol = 0
    for k, _ in by.items():
        rs = [r for r in recs if (r["cand"], r["route"]) == k and rp.admissible(r, epoch)]
        if rs:
            e = min(rs, key=lambda r: r["lead"])
            if any(r["J"] < e["J"] - 1e-9 for r in rs):
                viol += 1
    out["monotoneInLeadViolations"] = viol
    return out


def main(argv):
    if len(argv) == 3 and argv[1] == "report":
        return report(argv[2])
    res = []
    for path in argv[2:]:
        gens = load(path)
        order = sorted(gens, key=int)
        res.append(dict(log=path, moving=evaluate_moving(gens[order[0]]), rest=evaluate_rest(gens[order[-1]])))
    json.dump(res, open(argv[1], "w"), indent=1, default=str)
    return 0


def fmt(x, d=3):
    return "—" if x is None else (f"{x:+.{d}f}" if isinstance(x, float) else str(x))


def report(path):
    res = json.load(open(path))
    for r in res:
        m = r["moving"]
        name = r["log"].split("/")[-2]
        print(f"\n### {name} — moving epoch (speed {m['speed']:.4f} m/s; Δτ_phys = 0.015/v = {m['deltaTauPhysical']:.3f} s → grid {m['deltaTauOnGrid']:.2f} s; earliest admissible lead at zero latency {m['earliestAdmissibleLead0']})\n")
        ref = m["referenceZeroLatency"]
        print(f"Zero-latency full-grid reference: {ref['selected']} J={ref.get('J'):.4f}\n")
        print("| law | leads certified | modelled worker wall (s) | first admissible lead | selected (lead, g, r) at t_epoch + wall | ΔJ vs reference | Δτ vs reference (s) | reach / retreat clearance (mm) |")
        print("|---|---:|---:|---:|---|---:|---:|---|")
        for k, v in m["laws"].items():
            sel = v.get("selected")
            print(f"| {k} | {v['leads']} | {v['cost']:.2f} | {v.get('firstAdmissibleLead', '')} | {tuple(sel) if sel else 'none admissible'} | "
                  f"{fmt(v.get('dJ'),4)} | {fmt(v.get('dTau'),2)} | "
                  f"{(str(round(v['reachClear']*1000,1)) + ' / ' + str(round(v['retreatClear']*1000,1))) if sel else ''} |")
        s = r["rest"]
        print(f"\n{name} — rest epoch: records identical across all {s['leadsTotal']} leads: {s['recordsIdenticalAcrossLeads']}; "
              f"J monotone in lead per (g,r) violations: {s['monotoneInLeadViolations']}\n")
        print("| rest-epoch policy | cost (s) | selected (lead, g, r) | J | ΔJ vs full zero latency | τ − t_epoch (s) |")
        print("|---|---:|---|---:|---:|---:|")
        for lab, keyname, cost in (("FULL grid, zero latency", "fullZeroLatency", 0.0), ("FULL grid at its own latency", "fullAtOwnLatency", s["fullCost"]),
                                   ("BANK14 at full latency", "bank14AtFullLatency", s["fullCost"]), ("REST law (one pose, τ per record)", "restLaw", s["restLawCost"])):
            v = s[keyname]
            if not v.get("selected"):
                print(f"| {lab} | {cost:.2f} | none | | | |")
                continue
            print(f"| {lab} | {cost:.2f} | {tuple(v['selected'])} | {v['J']:.4f} | {fmt(v.get('dJ'),4)} | {v['tau'] - s['epoch']:.3f} |")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
