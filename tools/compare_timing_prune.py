#!/usr/bin/env python3
"""Track 1: exact timing prune, before/after comparison.

  runs  OFF_DIR ON_DIR      in-situ V2 runs (per scenario/repeat directories)
  search OFF_LOG ON_LOG     characterization logs of one scenario (prune off/on):
                            search-level losslessness check

runs: per run, first provisional plan (time relative to giver rest), completion,
commit time, capture, concurrent robot/object motion, selected tuple (lead, g, r)
and objective of the first adoption and of the committed plan, reach/retreat
clearance of the adopted record, worker wall and bounded work units (all
FULL_SEARCH jobs up to the first adoption and up to commit), and the number of
T/G/R candidates reaching each stage in those FULL_SEARCH jobs (static screens by
deepest stage, route rollouts, complete records, pruned (tau,g)).

search: for each FULL_SEARCH generation present in both logs, the record set of the
prune-on run must equal the prune-off record set restricted to records that are
timing-admissible at the prune-on receipt time, and the unchanged selector must
pick the same record from both at that time (lossless by construction).
"""

import glob
import os
import re
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_v2_characterization as ac  # noqa: E402
import replay_search_policies as rp  # noqa: E402

KV = re.compile(r"(\w+)=(\S+)")
PHASE_BUCKETS = rp.STAGE_NAMES[:12]


def kvs(line):
    i = line.find("] [")
    return dict(KV.findall(line[i:] if i >= 0 else line))


def fl(v):
    try:
        return float(str(v).rstrip("sm"))
    except (TypeError, ValueError):
        return float("nan")


def run_metrics(log):
    m = ac.supersession_metrics(log)
    lines = open(log, errors="replace").read().splitlines()
    adopt = next((kvs(l) for l in lines if "[V2ProvisionalAdopt]" in l), None)
    commit = next((kvs(l) for l in lines if "[V2TerminalCommit] committed=true" in l), None)
    t_first = fl(adopt["t"]) if adopt else None
    t_commit = fl(commit["commitTime"]) if commit else None
    capture = any("[CaptureTransferBoundary] bilateral grasp confirmed" in l for l in lines)
    # stage counts and work for FULL_SEARCH jobs finished before first adoption / commit
    jobs = {}
    stage = defaultdict(Counter)
    for l in lines:
        if "[V2JobProfile] type=FULL_SEARCH" in l:
            d = kvs(l)
            jobs[d["planningGeneration"]] = d
    def horizon_sum(limit):
        wall, units = 0.0, 0
        gens = []
        for g, p in jobs.items():
            if limit is None or fl(p["t"]) <= limit + 1e-9:
                wall += fl(p["jobWall"])
                units += sum(int(p[b].split("/")[1]) for b in PHASE_BUCKETS)
                gens.append(g)
        return wall, units, set(gens)
    w_first, u_first, gens_first = horizon_sum(t_first)
    w_commit, u_commit, gens_commit = horizon_sum(t_commit)
    counts = {"toFirstPlan": Counter(), "toCommit": Counter()}
    adopted_rec = None
    for l in lines:
        if "[CertStage] job=FULL_SEARCH" in l or "[V2TimingPrune]" in l:
            d = kvs(l)
            g = d.get("planningGeneration")
            for key, gens in (("toFirstPlan", gens_first), ("toCommit", gens_commit)):
                if g not in gens:
                    continue
                c = counts[key]
                if "[V2TimingPrune]" in l:
                    c["prunedGraspTau"] += 1
                elif d["path"] == "static":
                    c["staticScreens"] += 1
                    c["static:" + d["deepest"]] += 1
                elif d["path"] == "route":
                    c["routeRollouts"] += 1
                    if d["feasible"] == "true" and d.get("costValid") == "true":
                        c["completeRouteRecords"] += 1
                elif d["path"] == "memo":
                    c["memoRecords"] += 1
            if adopt and "[CertStage]" in l and g == adopt["sourcePlanningGeneration"] and d.get("hypothesis") == adopt["hypothesis"] \
                    and d.get("candidate") == adopt["candidate"] and d.get("route") == adopt["route"] and d["path"] in ("route", "memo"):
                adopted_rec = d
    return dict(
        completed=m["completed"], firstAdoptMinusRest=m["firstAdoptMinusRest"], firstAdoptWhileMoving=m["firstAdoptWhileMoving"],
        timeToFirstPlan=m["timeToFirstPlan"], commitTime=t_commit, capture=capture, concurrentMotion=m["concurrentMotion"],
        firstTuple=(adopt["eventLead"].rstrip("s"), adopt["candidate"], adopt["route"]) if adopt else None,
        firstJ=fl(adopt["globalJ"]) if adopt else None,
        firstReachClear=fl(adopted_rec["reachClear"]) if adopted_rec else None,
        firstRetreatClear=fl(adopted_rec["retreatClear"]) if adopted_rec else None,
        commitTuple=(commit["candidate"], commit["route"], commit["tau"]) if commit else None,
        replacements=m["replacements"], fullSearches=m["fullSearches"], fullCancelled=m["fullCancelled"],
        workerWallToFirstPlan=w_first, unitsToFirstPlan=u_first, workerWallToCommit=w_commit, unitsToCommit=u_commit,
        allWorkerWallToCommit=m["allWorkerWallToCommit"],
        stagesToFirstPlan=dict(counts["toFirstPlan"]), stagesToCommit=dict(counts["toCommit"]),
        premiseViolations=sum("[V2TimingPrunePremiseViolation]" in l for l in lines),
        rest=m["giverRest"])


def fmt(v, d=3):
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:+.{d}f}" if v < 0 else f"{v:.{d}f}"
    if isinstance(v, tuple):
        return "/".join(str(x) for x in v)
    return str(v)


def runs(off_dirs, on_dirs):
    rows = []
    for label, dirs in (("off", off_dirs), ("on", on_dirs)):
        for d in dirs:
            for log in sorted(glob.glob(os.path.join(d, "*", "*.log"))):
                sc = os.path.basename(os.path.dirname(log))
                rows.append((sc, label, os.path.basename(d), run_metrics(log)))
    print("### Per run\n")
    cols = ["completed", "firstAdoptMinusRest", "timeToFirstPlan", "firstTuple", "firstJ", "firstReachClear", "firstRetreatClear",
            "commitTuple", "capture", "concurrentMotion", "replacements", "fullSearches", "fullCancelled",
            "workerWallToFirstPlan", "unitsToFirstPlan", "workerWallToCommit", "allWorkerWallToCommit", "premiseViolations"]
    print("| scenario | prune | run | " + " | ".join(cols) + " |")
    print("|---|---|---|" + "---|" * len(cols))
    for sc, label, run, m in sorted(rows):
        print(f"| {sc} | {label} | {run} | " + " | ".join(fmt(m[c]) for c in cols) + " |")
    print("\n### Candidates reaching each stage in FULL_SEARCH jobs up to the first provisional plan (sum over the run's jobs; median over repeats)\n")
    keys = ["staticScreens", "static:NONE", "static:REACH", "static:INSERTION", "static:CLOSURE_CONTACT", "static:CARRIED_RETREAT",
            "prunedGraspTau", "routeRollouts", "completeRouteRecords", "memoRecords"]
    print("| scenario | prune | " + " | ".join(keys) + " |")
    print("|---|---|" + "---:|" * len(keys))
    import statistics
    for sc in sorted({r[0] for r in rows}):
        for label in ("off", "on"):
            ms = [r[3] for r in rows if r[0] == sc and r[1] == label]
            if not ms:
                continue
            print(f"| {sc} | {label} | " + " | ".join(str(statistics.median([m['stagesToFirstPlan'].get(k, 0) for m in ms])) for k in keys) + " |")
    print("\n### Summary per scenario (median over repeats; counts are totals)\n")
    print("| scenario | prune | runs | completed | first plan before rest | median first adoption − rest (s) | median worker wall to first plan (s) | median units to first plan | median worker wall to commit (s) | same first tuple as prune-off (all repeats) | median J first plan | median concurrent motion (s) |")
    print("|---|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|")
    for sc in sorted({r[0] for r in rows}):
        off_tuples = {r[3]["firstTuple"] for r in rows if r[0] == sc and r[1] == "off"}
        for label in ("off", "on"):
            ms = [r[3] for r in rows if r[0] == sc and r[1] == label]
            if not ms:
                continue
            med = lambda k: statistics.median([m[k] for m in ms if m[k] is not None]) if any(m[k] is not None for m in ms) else None
            same = all(m["firstTuple"] in off_tuples for m in ms) if label == "on" else "—"
            print(f"| {sc} | {label} | {len(ms)} | {sum(m['completed'] for m in ms)} | {sum(m['firstAdoptWhileMoving'] for m in ms)} | "
                  f"{fmt(med('firstAdoptMinusRest'))} | {fmt(med('workerWallToFirstPlan'))} | {fmt(med('unitsToFirstPlan'),0)} | "
                  f"{fmt(med('workerWallToCommit'))} | {fmt(same)} | {fmt(med('firstJ'),4)} | {fmt(med('concurrentMotion'),2)} |")


def search(off_log, on_log):
    joff, _ = rp.load_jobs(off_log)
    jon, _ = rp.load_jobs(on_log)
    ok_all = True
    for gen in sorted(set(joff) & set(jon), key=int):
        a, b = joff[gen], jon[gen]
        if a["outcome"] != "accepted" or b["outcome"] != "accepted":
            continue
        ra, _ = rp.build(a)
        rb, _ = rp.build(b)
        t_on = b["receipt"]
        key = lambda r: (round(r["lead"], 6), r["cand"], r["route"])
        adm_off = {key(r): r["J"] for r in ra if r["costValid"] and rp.admissible(r, t_on)}
        adm_on = {key(r): r["J"] for r in rb if r["costValid"] and rp.admissible(r, t_on)}
        all_on = {key(r) for r in rb if r["costValid"]}
        missing = [k for k in adm_off if k not in adm_on]
        extra_on = [k for k in all_on if k not in {key(r) for r in ra if r["costValid"]}]
        jdiff = [k for k in adm_off if k in adm_on and abs(adm_off[k] - adm_on[k]) > 1e-9]
        sel_off = rp.select(ra, t_on)
        sel_on = rp.select(rb, t_on)
        same = (sel_off is None and sel_on is None) or (sel_off is not None and sel_on is not None and key(sel_off) == key(sel_on))
        ok = not missing and not extra_on and not jdiff and same
        ok_all &= ok
        print(f"gen {gen}: off records {len(ra)} (search wall {fl(a['profile']['jobWall']):.3f}s), on records {len(rb)} (wall {fl(b['profile']['jobWall']):.3f}s); "
              f"records admissible at on-receipt t={t_on:.3f}: off {len(adm_off)}, on {len(adm_on)}; missing in on {len(missing)}, "
              f"on-only records {len(extra_on)}, J differences {len(jdiff)}; selection at t_on off={sel_off and key(sel_off)} on={sel_on and key(sel_on)} -> {'LOSSLESS' if ok else 'DIFFERENT'}")
    return 0 if ok_all else 1


def main(argv):
    if len(argv) >= 4 and argv[1] == "runs":
        sep = argv.index("--on")
        runs(argv[2:sep], argv[sep + 1:])
        return 0
    if len(argv) == 4 and argv[1] == "search":
        return search(argv[2], argv[3])
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
