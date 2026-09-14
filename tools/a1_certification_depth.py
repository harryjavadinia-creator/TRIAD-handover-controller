#!/usr/bin/env python3
"""A1: complete-action certification depth ablation (offline, frozen bank).

Usage: a1_certification_depth.py [--json OUT] LOG...

Inputs: V2 logs with Phase-D [CertStage] fields (jobWall, globalJ,
presentationDuration, reachClear/retreatClear); A1 audit fields are used when
present (auditSuccess, auditReason, proxyGlobalJPreAuditTime).

Certification levels (same bank, enumeration, timing admission, objective,
selector and tie logic):
  C1 = F1            C2 = F1+F2          C3 = F1+F2+F3
  C4 = F1+F2+F3+F5   C5 = C4 + terminal timing audit (+ cost-audit validity) = current
Stage ranks in [CertStage] deepest: NONE < REACH(F1) < INSERTION(F2) <
CLOSURE_CONTACT(F3) < CARRIED_RETREAT(F5); C5 = route record feasible and costValid.

Populations are kept separate:
  STATIC  (tau, g)      every evaluated grasp-event static screen
  ROUTE   (tau, g, r)   route rollouts, which exist only for STATIC F5 passes

Selection counterfactual: the unchanged objective J is defined only for
C5 records (the seven-term cost needs whole-action effort, retreat clearance,
joint metrics and the audited execution time; computeCompletePlanAuditCost
requires the audit to succeed). A shallower level's winner is therefore
DETERMINED only when no shallow-only candidate can be admitted at the decision
time or can reach the winner's J (lower bound J >= w_T (lead + 0.70) / T_ref,
from auditEstimatedTime >= T_pres + priorityBlend + captureLock + bilateralDwell
+ confirmationDwell and non-negative terms); otherwise NOT DETERMINED. For
statically rejected grasps whose routes were never rolled out, admissibility is
tested with the exact necessary condition of the timing prune (route T_pres >=
static reach time). A PROXY winner for C4 uses the logged pre-audit objective and
is reported separately; it is not a policy.
"""

import json
import math
import re
import statistics
import sys
from collections import Counter, defaultdict

KV = re.compile(r"(\w+)=(\S+)")
RANK = {"NONE": 0, "REACH": 1, "INSERTION": 2, "CLOSURE_CONTACT": 3, "CARRIED_RETREAT": 4}
STAGE_OF_RANK = {1: "F1 reach", 2: "F2 insertion", 3: "F3 closure/contact", 4: "F5 carried retreat"}
L_COMMIT, L_ENTRY, TIE = 1.6, 0.05, 1e-9
W_T, T_REF, FIXED = 0.4210526, 8.0, 0.20 + 0.15 + 0.10 + 0.25
LEVELS = ["C1", "C2", "C3", "C4", "C5"]


def kvs(line):
    i = line.find("] [")
    return dict(KV.findall(line[i:] if i >= 0 else line))


def fl(v):
    try:
        return float(str(v).rstrip("sm"))
    except (TypeError, ValueError):
        return float("nan")


def norm_reason(r):
    r = r or "none"
    r = re.sub(r"^predictive_static/", "", r)
    parts = r.split("/")
    return "/".join(parts[:3])


def admissible(event_time, t_pres, now):
    rem = event_time - now
    return rem + 1e-12 >= L_COMMIT and t_pres + L_ENTRY <= rem + 1e-12


def secondary_before(a, b):
    tol = 1e-12
    for x, y, less in ((a["lead"] - a["Tpres"] + a["Texec"], b["lead"] - b["Tpres"] + b["Texec"], True),
                       (a["eventTime"], b["eventTime"], True), (a["Tpres"], b["Tpres"], True),
                       (a["reachClear"], b["reachClear"], False)):
        if less:
            if x < y - tol: return True
            if y < x - tol: return False
        else:
            if x > y + tol: return True
            if y > x + tol: return False
    for k in ("cand", "route", "hyp", "source"):
        if a[k] != b[k]:
            return a[k] < b[k]
    return False


def select(recs, now, jkey="J"):
    adm = [r for r in recs if math.isfinite(r.get(jkey, float("nan"))) and admissible(r["eventTime"], r["Tpres"], now)]
    if not adm:
        return None
    jmin = min(r[jkey] for r in adm)
    best = None
    for r in adm:
        if r[jkey] > jmin + TIE:
            continue
        if best is None or secondary_before(r, best):
            best = r
    return best


def load(log):
    jobs = {}
    giver_rest = None
    current = None
    for line in open(log, errors="replace"):
        if "[GiverTruthScript]" in line:
            d = kvs(line)
            giver_rest = fl(d["startTime"]) + fl(d["cruiseDuration"]) + fl(d["stopDuration"])
        elif "[V2PlanningJobSubmit] type=FULL_SEARCH" in line:
            d = kvs(line)
            jobs[d["planningGeneration"]] = dict(gen=d["planningGeneration"], submit=fl(d["t"]),
                                                  kind="moving" if fl(d.get("objectEstimateSpeed")) > 0.004 else "rest",
                                                  static=[], route=[], memo=[], reuse=[], outcome="pending",
                                                  receipt=None, profile=None)
        elif "[CertStage] job=FULL_SEARCH" in line:
            d = kvs(line)
            j = jobs.get(d["planningGeneration"])
            if j is None or "jobWall" not in d:
                continue
            j[d["path"] if d["path"] in ("static", "route", "memo") else "route"].append(d)
        elif "[V2HypothesisCertificationReused]" in line:
            d = kvs(line)
            if d.get("planningGeneration") in jobs:
                jobs[d["planningGeneration"]]["reuse"].append(d)
        elif "[V2JobProfile] type=FULL_SEARCH" in line:
            d = kvs(line)
            if d["planningGeneration"] in jobs:
                jobs[d["planningGeneration"]]["profile"] = d
        elif "[V2PlanningJobResult] type=FULL_SEARCH" in line:
            d = kvs(line)
            if d["planningGeneration"] in jobs:
                jobs[d["planningGeneration"]].update(outcome="accepted" if d["failed"] == "false" else "failed", receipt=fl(d["t"]))
        elif "[V2JobCancelled] type=FULL_SEARCH" in line:
            d = kvs(line)
            if d["planningGeneration"] in jobs:
                jobs[d["planningGeneration"]].update(outcome="cancelled", receipt=fl(d["t"]))
    return jobs, giver_rest


def fingerprint(job):
    items = []
    for d in job["static"]:
        items.append(("s", d["hypothesis"], d["grasp"], d["deepest"]))
    for d in job["route"]:
        items.append(("r", d["hypothesis"], d["grasp"], d["route"], d["deepest"], d.get("costValid"), d.get("globalJ", "")[:12]))
    return (job["kind"], hash(tuple(sorted(items))), len(job["static"]), len(job["route"]))


def build(job):
    """Candidate records with memo expansion. Returns (full C5 records, extras per level, static/route tables)."""
    reuse_map = defaultdict(list)  # source hypothesis -> [(hyp, lead, eventTime)]
    for u in job["reuse"]:
        reuse_map[u["sourceHypothesis"]].append((u["hypothesis"], fl(u["lead"]), fl(u["eventTime"])))

    def expand(h, lead, et):
        out = [(h, lead, et)]
        out += reuse_map.get(h, [])
        return out

    full, extras = [], defaultdict(list)
    src = 0
    for d in job["route"]:
        rank = RANK.get(d["deepest"], 0)
        feasible = d["feasible"] == "true"
        cv = d.get("costValid") == "true"
        tpres = fl(d["presentationDuration"]) if feasible else fl(d["routeReachDuration"])
        for h, lead, et in expand(d["hypothesis"], fl(d["lead"]), fl(d["eventTime"])):
            rec = dict(source=src, hyp=int(h), lead=lead, eventTime=et, cand=d["candidate"], route=d["route"], grasp=d["grasp"],
                       deepest=d["deepest"], reason=d["reason"], Tpres=tpres, Texec=fl(d.get("executionDuration")),
                       reachClear=fl(d["reachClear"]), retreatClear=fl(d["retreatClear"]),
                       auditSuccess=d.get("auditSuccess"), auditReason=d.get("auditReason"), costInputsFinite=d.get("costInputsFinite"),
                       proxyJ=fl(d.get("proxyGlobalJPreAuditTime")), memoCopy=(h != d["hypothesis"]))
            src += 1
            if feasible and cv:
                # J is lead dependent: recompute the time extension for memo copies exactly as the certifier does
                jm = fl(d["motionJ"])
                rec["J"] = jm + W_T * (lead - tpres) / T_REF if rec["memoCopy"] else fl(d["globalJ"])
                full.append(rec)
            else:
                rec["J"] = float("nan")
                if feasible and not cv:
                    if math.isfinite(rec["proxyJ"]) and rec["memoCopy"]:
                        rec["proxyJ"] = rec["proxyJ"] - W_T * (fl(d["lead"]) - tpres) / T_REF + W_T * (lead - tpres) / T_REF
                    extras["C4"].append(rec)
                if rank >= 3 and not feasible:
                    extras["C3"].append(rec)
                if rank >= 2 and not feasible:
                    extras["C2"].append(rec)
                if rank >= 1 and not feasible:
                    extras["C1"].append(rec)
    # memo lines (complete records of reused hypotheses) are already covered by expansion of route lines;
    # use them only as a consistency check.
    static_only = defaultdict(list)
    for d in job["static"]:
        rank = RANK.get(d["deepest"], 0)
        if rank >= 4 or rank == 0:
            continue
        ts = fl(d["staticReachTime"])
        for h, lead, et in expand(d["hypothesis"], fl(d["lead"]), fl(d["eventTime"])):
            fam = dict(hyp=int(h), lead=lead, eventTime=et, cand=d["candidate"], grasp=d["grasp"], deepest=d["deepest"],
                       reason=d["reason"], Ts=ts, reachClear=fl(d["reachClear"]))
            if rank >= 3: static_only["C3"].append(fam)
            if rank >= 2: static_only["C2"].append(fam)
            if rank >= 1: static_only["C1"].append(fam)
    for lvl in ("C1", "C2", "C3"):
        extras[lvl] += extras["C4"]
    extras["C2"] += [r for r in extras["C3"] if r not in extras["C2"]]
    extras["C1"] += [r for r in extras["C2"] if r not in extras["C1"]]
    # de-duplicate by source
    for lvl in list(extras):
        seen, uniq = set(), []
        for r in extras[lvl]:
            if r["source"] not in seen:
                seen.add(r["source"]); uniq.append(r)
        extras[lvl] = uniq
    return full, extras, static_only


def candidate_tables(job):
    """Evaluated candidates only (no memo expansion)."""
    complete_tg = {(d["hypothesis"], d["grasp"]) for d in job["route"] if d["feasible"] == "true" and d.get("costValid") == "true"}
    st = dict(total=len(job["static"]), passF=Counter(), newly=Counter(), reasons=defaultdict(Counter),
              fullRejGiven=Counter(), acceptedGiven=Counter())
    for d in job["static"]:
        rank = RANK.get(d["deepest"], 0)
        full_ok = (d["hypothesis"], d["grasp"]) in complete_tg and rank == 4
        for lvl_rank in (1, 2, 3, 4):
            if rank >= lvl_rank:
                st["passF"][lvl_rank] += 1
                st["acceptedGiven"][lvl_rank] += 1
                if not full_ok:
                    st["fullRejGiven"][lvl_rank] += 1
        if rank < 4:
            st["newly"][rank + 1] += 1
            st["reasons"][rank + 1][norm_reason(d["reason"])] += 1
        st["acceptedGiven"]["C5"] += int(full_ok)
    rt = dict(total=len(job["route"]), passF=Counter(), newly=Counter(), reasons=defaultdict(Counter),
              fullRejGiven=Counter(), acceptedGiven=Counter(), audit=Counter(), auditReasons=Counter())
    for d in job["route"]:
        rank = RANK.get(d["deepest"], 0)
        feasible = d["feasible"] == "true"
        cv = d.get("costValid") == "true"
        full_ok = feasible and cv
        # route deepest semantics: feasible=true -> passed F5; else deepest = last passed stage
        passed = 4 if feasible else rank
        for lvl_rank in (1, 2, 3, 4):
            if passed >= lvl_rank:
                rt["passF"][lvl_rank] += 1
                if not full_ok:
                    rt["fullRejGiven"][lvl_rank] += 1
        if passed < 4:
            rt["newly"][passed + 1] += 1
            rt["reasons"][passed + 1][norm_reason(d["reason"])] += 1
        elif not cv:
            rt["newly"]["audit"] += 1
            key = "audit:" + (d.get("auditReason") or "unlogged") if d.get("auditSuccess") != "true" else "cost:non-finite input"
            rt["reasons"]["audit"][key] += 1
        rt["passF"]["C5"] += int(full_ok)
    return st, rt


def stage_walls(profile):
    if not profile:
        return None
    w = lambda n: fl(profile.get(n, "0s/0").split("/")[0])
    return {"static F1": w("staticReachStandoff"), "static F2": w("staticReachCapture"), "static F3": w("staticClosure"),
            "static F5": w("staticRetreat"), "route F1": w("routeSetup") + w("routeReach"),
            "route F2": w("routeApproach") + w("routeDwell"), "route F3": w("routeClosure"), "route F5": w("routeRetreat"),
            "audit+cost": w("routeFinalize"), "total": fl(profile["jobWall"]), "finalizeCount": int(profile.get("routeFinalize", "0s/0").split("/")[1])}


def selection(job, full, extras, static_only, now):
    out = {}
    win = select(full, now)
    out["C5"] = dict(status="SELECTED" if win else "NO_ADMISSIBLE", winner=win)
    jwin = win["J"] if win else float("inf")
    for lvl in ("C4", "C3", "C2", "C1"):
        adm = [r for r in extras[lvl] if admissible(r["eventTime"], r["Tpres"], now)]
        adm_fams = [f for f in static_only.get(lvl, []) if admissible(f["eventTime"], f["Ts"], now)]
        # ranking-free J lower bound: can this candidate reach the winner's J?
        contenders = [r for r in adm if W_T * (r["lead"] + FIXED) / T_REF <= jwin + TIE]
        fam_contenders = [f for f in adm_fams if W_T * (f["lead"] + FIXED) / T_REF <= jwin + TIE]
        if win is None:
            if adm:
                # every admissible candidate at this level is rejected by the full certificate
                status = "DETERMINED_INVALID"
            elif adm_fams:
                status = "POTENTIALLY_INVALID_UNOBSERVED_ROUTES"
            else:
                status = "DETERMINED_NONE"
        elif not contenders and not fam_contenders:
            status = "DETERMINED_SAME"
        else:
            status = "NOT_DETERMINED"
        entry = dict(status=status, admissibleShallowOnly=len(adm), admissibleUnrolledGraspFamilies=len(adm_fams),
                     contenders=len(contenders) + len(fam_contenders),
                     contenderStages=Counter([r["deepest"] + ("/audit_or_cost" if r["deepest"] == "CARRIED_RETREAT" else "") for r in contenders]
                                             + ["static " + f["deepest"] for f in fam_contenders]),
                     contenderReasons=Counter([norm_reason(r["reason"]) if r["deepest"] != "CARRIED_RETREAT" else
                                               "audit:" + str(r.get("auditReason")) for r in contenders]
                                              + ["static:" + norm_reason(f["reason"]) for f in fam_contenders]))
        if lvl == "C4":
            proxy_pool = full + [r for r in extras["C4"] if math.isfinite(r["proxyJ"])]
            for r in proxy_pool:
                r.setdefault("proxyJfull", r["J"] if math.isfinite(r["J"]) else r["proxyJ"])
            pw = select(proxy_pool, now, jkey="proxyJfull") if any(math.isfinite(r["proxyJ"]) for r in extras["C4"]) else None
            entry["proxyWinner"] = pw
            entry["proxyAvailable"] = any(math.isfinite(r["proxyJ"]) for r in extras["C4"])
        out[lvl] = entry
    return out


def tuple_of(r):
    return None if r is None else (r["lead"], r["cand"], r["route"])


def main(argv):
    out_json = None
    if "--json" in argv:
        i = argv.index("--json"); out_json = argv[i + 1]; argv = argv[:i] + argv[i + 2:]
    logs = argv[1:]
    unique = {}
    for log in logs:
        jobs, rest = load(log)
        run = "/".join(log.split("/")[-3:-1])
        for gen, j in sorted(jobs.items(), key=lambda kv: int(kv[0])):
            if not j["static"] and not j["route"]:
                continue
            fp = fingerprint(j)
            key = (run.split("/")[-1], fp)
            if key not in unique:
                unique[key] = dict(job=j, rest=rest, occurrences=[])
            unique[key]["occurrences"].append(f"{run} g{gen} ({j['outcome']})")
    results = []
    for (scenario, fp), u in unique.items():
        j = u["job"]
        st, rt = candidate_tables(j)
        full, extras, static_only = build(j)
        res = dict(scenario=scenario, kind=j["kind"], outcome=j["outcome"], occurrences=u["occurrences"],
                   static=dict(total=st["total"], passF={str(k): v for k, v in st["passF"].items()},
                               newly={str(k): v for k, v in st["newly"].items()},
                               reasons={str(k): dict(v.most_common(6)) for k, v in st["reasons"].items()},
                               fullRejGiven={str(k): v for k, v in st["fullRejGiven"].items()},
                               fullAccepted=st["acceptedGiven"]["C5"]),
                   route=dict(total=rt["total"], passF={str(k): v for k, v in rt["passF"].items()},
                              newly={str(k): v for k, v in rt["newly"].items()},
                              reasons={str(k): dict(v.most_common(6)) for k, v in rt["reasons"].items()},
                              fullRejGiven={str(k): v for k, v in rt["fullRejGiven"].items()}),
                   stageWalls=stage_walls(j["profile"]), fullRecords=len(full),
                   extras={k: len(v) for k, v in extras.items()}, staticOnlyFamilies={k: len(v) for k, v in static_only.items()})
        if j["outcome"] == "accepted" and j["receipt"] is not None:
            res["selection"] = {}
            for label, now in (("receipt", j["receipt"]), ("epoch", j["submit"])):
                sel = selection(j, full, extras, static_only, now)
                res["selection"][label] = {lvl: {k: (tuple_of(v) if k in ("winner", "proxyWinner") else
                                                     (dict(v) if isinstance(v, Counter) else v))
                                                 for k, v in e.items()} for lvl, e in sel.items()}
                w = sel["C5"]["winner"]
                if w:
                    res["selection"][label]["C5"].update(J=w["J"], reachClear=w["reachClear"], retreatClear=w["retreatClear"])
                pw = sel["C4"].get("proxyWinner")
                if pw:
                    res["selection"][label]["C4"].update(proxyJ=pw["proxyJfull"], proxyWinnerDeepest=pw["deepest"],
                                                         proxyWinnerAuditReason=pw.get("auditReason"),
                                                         proxyWinnerIsFull=math.isfinite(pw["J"]))
        results.append(res)
    if out_json:
        json.dump(results, open(out_json, "w"), indent=1, default=str)
        # per-candidate listing of every evaluated candidate of every unique completed search
        import csv
        with open(out_json.replace(".json", "_candidates.csv"), "w", newline="") as fh:
            wcsv = csv.writer(fh)
            wcsv.writerow(["scenario", "epochKind", "occurrence", "population", "hypothesis", "lead", "eventTime", "grasp", "candidate",
                           "route", "deepestPassed", "firstFailedStage", "reason", "costValid", "J", "reachClear", "retreatClear",
                           "presentationDuration", "auditSuccess", "auditReason", "proxyJPreAuditTime"])
            for (scenario, fp), u in unique.items():
                j = u["job"]
                if j["outcome"] != "accepted":
                    continue
                occ = u["occurrences"][0]
                for d in j["static"]:
                    rank = RANK.get(d["deepest"], 0)
                    wcsv.writerow([scenario, j["kind"], occ, "STATIC", d["hypothesis"], d["lead"], d["eventTime"], d["grasp"], d["candidate"], "-",
                                   d["deepest"], STAGE_OF_RANK.get(rank + 1, "none (passed static F5)"), d["reason"], "", "", d["reachClear"], "",
                                   d["staticReachTime"], "", "", ""])
                for d in j["route"]:
                    feasible = d["feasible"] == "true"
                    rank = 4 if feasible else RANK.get(d["deepest"], 0)
                    cv = d.get("costValid") == "true"
                    first_fail = STAGE_OF_RANK.get(rank + 1) if rank < 4 else ("none" if cv else "terminal timing audit / cost")
                    wcsv.writerow([scenario, j["kind"], occ, "ROUTE", d["hypothesis"], d["lead"], d["eventTime"], d["grasp"], d["candidate"], d["route"],
                                   "CARRIED_RETREAT" if feasible else d["deepest"], first_fail, d["reason"], d.get("costValid"),
                                   d.get("globalJ") if cv else "", d["reachClear"], d["retreatClear"] if feasible else "",
                                   d["presentationDuration"] if feasible else d["routeReachDuration"], d.get("auditSuccess", ""),
                                   d.get("auditReason", ""), d.get("proxyGlobalJPreAuditTime", "")])
    print(f"unique searches: {len(results)} from {len(logs)} logs")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
