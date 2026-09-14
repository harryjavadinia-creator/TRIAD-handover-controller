#!/usr/bin/env python3
"""TRIAD V2 Phase D: offline replay of search stopping policies.

Reads V2 run logs produced with the Phase D logging ([CertStage] with jobWall,
workUnits, globalJ, presentationDuration, executionDuration and prof=...). For
every FULL_SEARCH job it reconstructs the order in which complete-action-
certified records became available in the unchanged exhaustive search and
replays these policies over exactly those records, with the same hard
certification and the same cost:

  FULL_ARGMIN                          evaluate the whole bank, select at receipt
  FIRST_COMPLETE_FEASIBLE              stop at the first complete certified record
  FIRST_ADMISSIBLE_COMPLETE (extra)    stop at the first complete certified record
                                       that is timing-admissible when it appears
  EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR evaluate tau in the existing (ascending)
                                       order; after finishing a tau, stop if it has
                                       an admissible certified record and select
                                       the best record of that tau
  ANYTIME_INCUMBENT                    best admissible certified record found so
                                       far, tracked at every record event

The selector is a line-by-line replica of call_handover::selectFiniteEventPlan
(timing admission, 1e-9 cost tie tolerance, deterministicEventSecondaryOrder);
the replayed FULL_ARGMIN is checked against the controller's logged selection.
Decision times are controller times: job submission time + worker wall at the
record's availability (the worker runs in wall time; latency and worker wall
agree to about 1 ms in these logs).

No enumeration order is changed. The existing bank is evaluated in ascending
lead, grasp index, then route order, which is already temporal order.

Usage: replay_search_policies.py [--json OUT] LOG...
"""

import json
import math
import re
import statistics
import sys
from collections import defaultdict

KV = re.compile(r"(\w+)=(\S+)")
MIN_SAFE_COMMIT_LEAD = 1.6      # max(minimumCommitRemainingTime 1.6, decel 0.85 + 0.25)
MIN_REACH_ENTRY_LEAD = 0.05
COST_TIE = 1e-9
STAGE_NAMES = ["staticReachStandoff", "staticReachCapture", "staticClosure", "staticRetreat",
               "routeSetup", "routeReach", "routeApproach", "routeDwell", "routeClosure", "routeRetreat",
               "routeFinalize", "hypothesisSetup", "terminalStandoff",
               "nestedIkStep", "nestedSweptQuery", "nestedConfigurationSafety", "nestedClosureSafety"]
CATEGORIES = {
    "temporal hypothesis setup": ["hypothesisSetup"],
    "grasp/static reach screening": ["staticReachStandoff"],
    "insertion/closure certification": ["staticReachCapture", "staticClosure", "routeApproach", "routeDwell", "routeClosure"],
    "route (transit reach) certification": ["routeSetup", "routeReach"],
    "carried-retreat certification": ["staticRetreat", "routeRetreat"],
    "cost / terminal timing audit": ["routeFinalize"],
}


def kvs(line):
    i = line.find("] [")
    return dict(KV.findall(line[i:] if i >= 0 else line))


def fl(v):
    try:
        return float(str(v).rstrip("sm"))
    except (TypeError, ValueError):
        return float("nan")


def parse_prof(s):
    out = {}
    for name, part in zip(STAGE_NAMES, (s or "").split(",")):
        c, w = part.split(":")
        out[name] = (int(c), float(w))
    return out


# ---------------------------------------------------------------------------
# selector replica
# ---------------------------------------------------------------------------

def admissible(r, now):
    remaining = r["eventTime"] - now
    return remaining + 1e-12 >= MIN_SAFE_COMMIT_LEAD and r["Tpres"] + MIN_REACH_ENTRY_LEAD <= remaining + 1e-12


def secondary_before(a, b):
    tol = 1e-12
    ca = a["lead"] - a["Tpres"] + a["Texec"]
    cb = b["lead"] - b["Tpres"] + b["Texec"]
    for x, y, less in ((ca, cb, True), (a["eventTime"], b["eventTime"], True), (a["Tpres"], b["Tpres"], True),
                       (a["reachClear"], b["reachClear"], False)):
        if less:
            if x < y - tol: return True
            if y < x - tol: return False
        else:
            if x > y + tol: return True
            if y > x + tol: return False
    for key in ("cand", "route", "hyp", "source"):
        if a[key] != b[key]:
            return a[key] < b[key]
    return False


def select(records, now):
    valid = [r for r in records if r["costValid"] and all(math.isfinite(r[k]) for k in ("J", "lead", "eventTime", "Tpres", "Texec", "reachClear"))]
    adm = [r for r in valid if admissible(r, now)]
    if not adm:
        return None
    jmin = min(r["J"] for r in adm)
    best = None
    for r in adm:
        if r["J"] > jmin + COST_TIE:
            continue
        if best is None or secondary_before(r, best):
            best = r
    return best


# ---------------------------------------------------------------------------
# log parsing
# ---------------------------------------------------------------------------

def load_jobs(log):
    jobs = {}
    giver = None
    selections = {}
    current_result_gen = None
    for line in open(log, errors="replace"):
        if "[GiverTruthScript]" in line:
            giver = kvs(line)
        elif "[V2PlanningJobSubmit] type=FULL_SEARCH" in line:
            d = kvs(line)
            jobs[d["planningGeneration"]] = dict(gen=d["planningGeneration"], submit=fl(d["t"]),
                                                  estSpeed=fl(d.get("objectEstimateSpeed")), events=[],
                                                  outcome="pending", receipt=None, profile=None, selection=None)
        elif "[CertStage] job=FULL_SEARCH" in line:
            d = kvs(line)
            j = jobs.get(d["planningGeneration"])
            if j is not None and "jobWall" in d:
                j["events"].append(d)
        elif "[V2JobProfile] type=FULL_SEARCH" in line:
            d = kvs(line)
            if d["planningGeneration"] in jobs:
                jobs[d["planningGeneration"]]["profile"] = d
        elif "[V2JobCancelled] type=FULL_SEARCH" in line:
            d = kvs(line)
            if d["planningGeneration"] in jobs:
                jobs[d["planningGeneration"]]["outcome"] = "cancelled"
                jobs[d["planningGeneration"]]["receipt"] = fl(d["t"])
        elif "[V2PlanningJobResult] type=FULL_SEARCH" in line:
            d = kvs(line)
            if d["planningGeneration"] in jobs:
                jobs[d["planningGeneration"]]["outcome"] = "accepted" if d["failed"] == "false" else "failed"
                jobs[d["planningGeneration"]]["receipt"] = fl(d["t"])
                current_result_gen = d["planningGeneration"]
        elif "[V2FullSearchSelection]" in line and "selectedCandidate=" in line and current_result_gen:
            d = kvs(line)
            if jobs[current_result_gen]["selection"] is None:
                jobs[current_result_gen]["selection"] = d
        elif "[V2CharacterizationSelection]" in line and "evaluatedAt=receipt" in line:
            d = kvs(line)
            if d["planningGeneration"] in jobs:
                jobs[d["planningGeneration"]]["charSelection"] = d
    rest = None
    if giver:
        rest = fl(giver["startTime"]) + fl(giver["cruiseDuration"]) + fl(giver["stopDuration"])
    return jobs, rest


def build(job):
    records, seq = [], []
    for i, e in enumerate(job["events"]):
        t = job["submit"] + fl(e["jobWall"])
        ev = dict(idx=i, path=e["path"], hyp=int(e["hypothesis"]), t=t, wall=fl(e["jobWall"]),
                  units=int(e["workUnits"]), prof=e.get("prof"))
        seq.append(ev)
        if e["path"] in ("route", "memo") and e["feasible"] == "true":
            r = dict(source=len(records), hyp=int(e["hypothesis"]), lead=fl(e["lead"]), eventTime=fl(e["eventTime"]),
                     cand=e["candidate"], route=e["route"], J=fl(e["globalJ"]), motionJ=fl(e["motionJ"]),
                     Tpres=fl(e["presentationDuration"]), Texec=fl(e["executionDuration"]),
                     reachClear=fl(e["reachClear"]), retreatClear=fl(e["retreatClear"]),
                     costValid=e["costValid"] == "true", deepest=e["deepest"], t=t, wall=ev["wall"],
                     units=ev["units"], eventIdx=i)
            records.append(r)
    return records, seq


def counts_until(seq, idx):
    hyps = {s["hyp"] for s in seq[: idx + 1]}
    grasps = sum(1 for s in seq[: idx + 1] if s["path"] == "static")
    routes = sum(1 for s in seq[: idx + 1] if s["path"] == "route")
    memo = sum(1 for s in seq[: idx + 1] if s["path"] == "memo")
    return len(hyps), grasps, routes, memo


def breakdown(prof_str):
    if not prof_str:
        return None
    p = parse_prof(prof_str)
    out = {cat: (sum(p[b][0] for b in bs), sum(p[b][1] for b in bs)) for cat, bs in CATEGORIES.items()}
    out["nested: IK steps"] = p["nestedIkStep"]
    out["nested: swept-volume queries"] = p["nestedSweptQuery"]
    out["nested: configuration safety"] = p["nestedConfigurationSafety"]
    out["nested: closure/contact safety"] = p["nestedClosureSafety"]
    return out


def compare(o, rec, ref, suffix):
    if ref is None or rec is None:
        return
    o["dJ" + suffix] = rec["J"] - ref["J"]
    o["sameTau" + suffix] = abs(rec["eventTime"] - ref["eventTime"]) < 1e-9
    o["sameG" + suffix] = rec["cand"] == ref["cand"]
    o["sameR" + suffix] = rec["route"] == ref["route"]
    o["sameTuple" + suffix] = o["sameTau" + suffix] and o["sameG" + suffix] and o["sameR" + suffix]
    o["dReachClear" + suffix] = rec["reachClear"] - ref["reachClear"]
    o["dRetreatClear" + suffix] = rec["retreatClear"] - ref["retreatClear"]


def outcome(policy, rec, decision_t, stop_idx, seq, full_units, full_wall, rest, full, full_epoch=None):
    if rec is None:
        return dict(policy=policy, available=False, decisionT=decision_t,
                    wall=(seq[stop_idx]["wall"] if stop_idx is not None and seq else None))
    hyps, grasps, routes, memo = counts_until(seq, stop_idx)
    units = seq[stop_idx]["units"]
    wall = seq[stop_idx]["wall"]
    remaining = rec["eventTime"] - decision_t
    o = dict(policy=policy, available=True, decisionT=decision_t, wallToPlan=wall, unitsToPlan=units,
             tMinusRest=(decision_t - rest) if rest is not None else None,
             commitLeadOk=remaining + 1e-12 >= MIN_SAFE_COMMIT_LEAD,
             beforeReachStartDeadline=rec["Tpres"] + MIN_REACH_ENTRY_LEAD <= remaining + 1e-12,
             reachStart=rec["eventTime"] - rec["Tpres"],
             lead=rec["lead"], tau=rec["eventTime"], g=rec["cand"], r=rec["route"], J=rec["J"],
             reachClear=rec["reachClear"], retreatClear=rec["retreatClear"], deepest=rec["deepest"],
             hypothesesEvaluated=hyps, graspScreens=grasps, routeRollouts=routes, memoRecords=memo,
             fractionUnitsAvoided=(1.0 - units / full_units) if full_units else None,
             fractionWallAvoided=(1.0 - wall / full_wall) if full_wall else None,
             breakdown=breakdown(seq[stop_idx]["prof"]))
    compare(o, rec, full, "")
    compare(o, rec, full_epoch, "VsEpochFull")
    return o


def replay_job(job, rest, name, epoch_reference=None):
    records, seq = build(job)
    if not seq:
        return None
    last_idx = len(seq) - 1
    complete = job["outcome"] == "accepted"
    full_wall = fl(job["profile"]["jobWall"]) if job["profile"] else seq[-1]["wall"]
    full_units = int(job["profile"]["workUnits"]) if job["profile"] and "workUnits" in job["profile"] else seq[-1]["units"]
    t_full = job["receipt"] if job["receipt"] is not None else job["submit"] + full_wall
    kind = "moving" if (job["estSpeed"] or 0) > 0.004 else "rest"
    res = dict(run=name, gen=job["gen"], kind=kind, outcome=job["outcome"], submit=job["submit"], rest=rest,
               records=len(records), fullWall=full_wall, fullUnits=full_units, policies={})

    # FULL_ARGMIN (only for completed searches)
    full = select(records, t_full) if complete else None
    full_epoch = select(records, job["submit"]) if complete else None
    if epoch_reference is not None:
        full_epoch = epoch_reference  # exact-timing-prune replay: compare with the unpruned zero-latency argmin
    if complete:
        res["policies"]["FULL_ARGMIN"] = outcome("FULL_ARGMIN", full, t_full, last_idx, seq, full_units, full_wall, rest, full, full_epoch)
        logged = job.get("selection") or job.get("charSelection")
        if logged is not None:
            lc = logged.get("selectedCandidate", logged.get("candidate"))
            lr = logged.get("selectedRoute", logged.get("route"))
            res["fullReplayMatchesController"] = (full is None and lc == "none") or (
                full is not None and full["cand"] == lc and full["route"] == lr)
    # zero-latency reference at the search epoch (decision-structure question)
    res["fullArgminAtEpoch"] = full_epoch

    # FIRST_COMPLETE_FEASIBLE
    first = next((r for r in records if r["costValid"]), None)
    res["policies"]["FIRST_COMPLETE_FEASIBLE"] = outcome(
        "FIRST_COMPLETE_FEASIBLE", first, first["t"] if first else None, first["eventIdx"] if first else last_idx,
        seq, full_units, full_wall, rest, full, full_epoch)
    # FIRST_ADMISSIBLE_COMPLETE
    fa = next((r for r in records if r["costValid"] and admissible(r, r["t"])), None)
    res["policies"]["FIRST_ADMISSIBLE_COMPLETE"] = outcome(
        "FIRST_ADMISSIBLE_COMPLETE", fa, fa["t"] if fa else None, fa["eventIdx"] if fa else last_idx,
        seq, full_units, full_wall, rest, full, full_epoch)
    # EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR
    hyp_order = []
    for s in seq:
        if not hyp_order or hyp_order[-1] != s["hyp"]:
            hyp_order.append(s["hyp"])
    chosen = None
    for h in hyp_order:
        idxs = [s["idx"] for s in seq if s["hyp"] == h]
        end_idx = max(idxs)
        if not complete and end_idx == last_idx:
            break  # hypothesis unfinished when the job was cancelled
        t_end = seq[end_idx]["t"]
        best = select([r for r in records if r["hyp"] == h], t_end)
        if best is not None:
            chosen = (best, t_end, end_idx)
            break
    res["policies"]["EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR"] = outcome(
        "EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR", chosen[0] if chosen else None, chosen[1] if chosen else None,
        chosen[2] if chosen else last_idx, seq, full_units, full_wall, rest, full, full_epoch)
    # latency-free structural comparison at the epoch
    if complete:
        ep_best = None
        for h in hyp_order:
            b = select([r for r in records if r["hyp"] == h], job["submit"])
            if b is not None:
                ep_best = b
                break
        res["earliestTauAtEpoch"] = ep_best

    # ANYTIME_INCUMBENT trajectory
    traj = []
    last_key = None
    for r in records:
        inc = select([x for x in records if x["source"] <= r["source"]], r["t"])
        key = None if inc is None else (inc["hyp"], inc["cand"], inc["route"])
        if key != last_key:
            traj.append(dict(t=r["t"], wall=r["wall"], units=r["units"], record=inc, eventIdx=r["eventIdx"],
                             incumbent=None if inc is None else dict(lead=inc["lead"], g=inc["cand"], r=inc["route"], J=inc["J"],
                                                                     reachClear=inc["reachClear"], retreatClear=inc["retreatClear"])))
            last_key = key
    first_inc = next((x for x in traj if x["incumbent"] is not None), None)
    anytime = dict(firstIncumbentWall=first_inc["wall"] if first_inc else None,
                   firstIncumbentT=first_inc["t"] if first_inc else None,
                   firstIncumbentMinusRest=(first_inc["t"] - rest) if first_inc and rest is not None else None,
                   firstIncumbentUnits=first_inc["units"] if first_inc else None,
                   changes=len([x for x in traj if x["incumbent"] is not None]),
                   losses=len([x for x in traj[1:] if x["incumbent"] is None]),
                   trajectory=[{k: v for k, v in x.items() if k != "record"} for x in traj])
    # incumbent at fractions of the full wall
    fr = {}
    for frac in (0.1, 0.25, 0.5, 0.75, 1.0):
        tw = frac * full_wall
        avail = [x for x in records if x["wall"] <= tw + 1e-9]
        inc = select(avail, job["submit"] + tw)
        fr[str(frac)] = None if inc is None else dict(lead=inc["lead"], g=inc["cand"], r=inc["route"], J=inc["J"],
                                                     dJ=(inc["J"] - full["J"]) if full else None,
                                                     reachClear=inc["reachClear"], retreatClear=inc["retreatClear"])
    anytime["atFractionOfFullWall"] = fr
    if first_inc is not None:
        o = outcome("ANYTIME_INCUMBENT", first_inc["record"], first_inc["t"], first_inc["eventIdx"],
                    seq, full_units, full_wall, rest, full, full_epoch)
        inc0 = first_inc["incumbent"]
        res["policies"]["ANYTIME_INCUMBENT"] = o
    else:
        res["policies"]["ANYTIME_INCUMBENT"] = dict(policy="ANYTIME_INCUMBENT", available=False)
    res["anytime"] = anytime

    # where computation goes before the first complete certified record
    if first is not None:
        pre = seq[: first["eventIdx"] + 1]
        static_ok = static_fail = route_fail = 0
        w_static_ok = w_static_fail = w_route_fail = w_route_ok = 0.0
        prev = 0.0
        for s, e in zip(pre, job["events"]):
            dw = s["wall"] - prev
            prev = s["wall"]
            if s["path"] == "static":
                if e["feasible"] == "true":
                    static_ok += 1; w_static_ok += dw
                else:
                    static_fail += 1; w_static_fail += dw
            elif s["path"] == "route":
                if s["idx"] == first["eventIdx"]:
                    w_route_ok += dw
                else:
                    route_fail += 1; w_route_fail += dw
        res["beforeFirstComplete"] = dict(staticScreensFailed=static_fail, wallStaticFailed=w_static_fail,
                                          staticScreensPassed=static_ok, wallStaticPassed=w_static_ok,
                                          routeRolloutsFailed=route_fail, wallRouteFailed=w_route_fail,
                                          wallSuccessfulRoute=w_route_ok, hypothesesTouched=len({s["hyp"] for s in pre}))
    # per-candidate certification cost over the whole search
    prev = 0.0
    costs = defaultdict(list)
    for s, e in zip(seq, job["events"]):
        dw = s["wall"] - prev
        prev = s["wall"]
        if s["path"] in ("static", "route"):
            costs[(s["path"], e["feasible"])].append(dw)
    res["perCandidateWall"] = {f"{p}/{'feasible' if f == 'true' else 'rejected'}": dict(n=len(v), median=statistics.median(v), total=sum(v))
                               for (p, f), v in costs.items()}
    return res


def replay_exact_timing_prune(job, rest, name, full_epoch):
    """EXPLORATORY, reported separately. Same enumeration order, same records;
    route rollouts of a (tau, grasp) are skipped when its static screen finished
    at a time t at which tau - t < minimumSafeCommitLead or
    staticReachTime + minimumReachEntryLead > tau - t. Because every route's
    presentation duration is >= the static reach time (route reach duration =
    max(2 dt, static) x max(1, stretch); checked on every logged record) and
    admissibility only decreases with time, no skipped route could be admitted
    at any later decision time. Later events are shifted earlier by the measured
    wall time and work units of the skipped rollouts. Applied only to searches
    without memo reuse (moving epochs)."""
    if any(e["path"] == "memo" for e in job["events"]):
        return None
    saved_wall, saved_units, prev_wall = 0.0, 0, 0.0
    prev_units = 0
    pruned = set()
    events = []
    for e in job["events"]:
        wall, units = fl(e["jobWall"]), int(e["workUnits"])
        dw, du = wall - prev_wall, units - prev_units
        prev_wall, prev_units = wall, units
        key = (int(e["hypothesis"]), e["grasp"])
        t_adj = job["submit"] + wall - saved_wall
        if e["path"] == "route" and key in pruned:
            saved_wall += dw
            saved_units += du
            continue
        if e["path"] == "static" and e["feasible"] == "true":
            remaining = fl(e["eventTime"]) - t_adj
            if remaining + 1e-12 < MIN_SAFE_COMMIT_LEAD or fl(e["staticReachTime"]) + MIN_REACH_ENTRY_LEAD > remaining + 1e-12:
                pruned.add(key)
        e2 = dict(e)
        e2["jobWall"] = str(wall - saved_wall)
        e2["workUnits"] = str(units - saved_units)
        events.append(e2)
    j2 = dict(job)
    j2["events"] = events
    total_wall = (fl(job["profile"]["jobWall"]) if job["profile"] else prev_wall) - saved_wall
    j2["profile"] = dict(job["profile"] or {}, jobWall=str(total_wall),
                         workUnits=str(int((job["profile"] or {}).get("workUnits", prev_units)) - saved_units))
    j2["receipt"] = job["submit"] + total_wall if job["outcome"] == "accepted" else (job["receipt"] - saved_wall if job["receipt"] else None)
    j2["selection"] = None
    j2.pop("charSelection", None)
    r = replay_job(j2, rest, name, epoch_reference=full_epoch)
    if r is None:
        return None
    r["exactTimingPrune"] = dict(prunedGraspTau=len(pruned), savedWall=saved_wall, savedUnits=saved_units,
                                 originalFullWall=fl(job["profile"]["jobWall"]) if job["profile"] else None)
    # compare against the unpruned zero-latency argmin (the prune never removes an epoch-admissible record? report it)
    r["fullArgminAtEpochUnpruned"] = full_epoch
    return r


def main(argv):
    out_json = None
    if "--json" in argv:
        i = argv.index("--json")
        out_json = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    logs = [a for a in argv[1:] if not a.startswith("--")]
    if not logs:
        print(__doc__)
        return 2
    results = []
    for log in logs:
        jobs, rest = load_jobs(log)
        name = "/".join(log.split("/")[-3:-1])
        for gen in sorted(jobs, key=int):
            r = replay_job(jobs[gen], rest, name)
            if r is not None and r["records"] + len(jobs[gen]["events"]) > 0:
                if "--exact-timing-prune" in sys.argv:
                    pr = replay_exact_timing_prune(jobs[gen], rest, name + " [exact timing prune]", r.get("fullArgminAtEpoch"))
                    if pr is not None:
                        pr["kind"] = r["kind"]
                        results.append(pr)
                    continue
                results.append(r)
    if out_json:
        with open(out_json, "w") as fh:
            json.dump(results, fh, indent=1, default=str)
    mism = [(r["run"], r["gen"]) for r in results if r.get("fullReplayMatchesController") is False]
    print(f"jobs replayed: {len(results)}; FULL_ARGMIN replay vs controller selection mismatches: {mism}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
