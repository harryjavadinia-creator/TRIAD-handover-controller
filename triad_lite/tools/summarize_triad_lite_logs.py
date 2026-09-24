#!/usr/bin/env python3
"""Summarize TRIAD-lite (supervisorMode: control_aware) and TRIAD V2 bank-search logs.

Sanity characterization only: does the directional authority layer discriminate
among grasps that pass geometry, robot and clearance layers? No outcome claim.

usage: summarize_triad_lite_logs.py LOG [LOG ...]
"""
import lzma, re, statistics, sys
from collections import Counter

KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")
def kv(line): return dict(KV.findall(line))

def summarize(path):
    sel = 0; layers = Counter(); passed = []; auth_rej = 0; kappas_by_gen = {}
    events = Counter(); commit_t = None; completed = False; fail = None
    fullsearch = 0; recert = 0; selected_names = []
    fh = lzma.open(path, "rt", errors="ignore") if path.endswith(".xz") else open(path, errors="ignore")
    for line in fh:
        if "[TriadLiteCandidate]" in line:
            d = kv(line)
            layers[d["rejectionLayer"]] += 1
            if d["robotFeasible"] == "true" and d["clearanceFeasible"] == "true":
                k = float(d["reserve"]); passed.append(k)
                kappas_by_gen.setdefault(d["planningGeneration"], []).append(k)
                if d["authorityFeasible"] != "true": auth_rej += 1
        elif "[TriadLiteSelection]" in line and "evaluated=" in line:
            sel += 1
        elif "[TriadLiteEvent]" in line:
            events[kv(line)["type"].split("/")[0]] += 1
        elif "[V2TerminalCommit] committed=true" in line:
            commit_t = float(kv(line)["commitTime"])
        elif "full plan-once handover completed" in line:
            completed = True
        elif "[V2Failure]" in line and fail is None:
            fail = kv(line).get("reason")
        elif "[V2PlanningJobSubmit] type=FULL_SEARCH" in line:
            fullsearch += 1
        elif "[V2PlanningJobSubmit] type=RECERTIFY_ACTIVE" in line:
            recert += 1
        elif "[V2FullSearchSelection] success=true" in line:
            selected_names.append(kv(line).get("selectedCandidate"))
    spreads = [max(v) - min(v) for v in kappas_by_gen.values() if len(v) >= 2]
    ratios = [max(v) / max(min(v), 1e-9) for v in kappas_by_gen.values() if len(v) >= 2 and min(v) > 0]
    out = dict(log=path, completed=completed, failure=fail, commitTime=commit_t, selections=sel,
               layers=dict(layers), passedGeomRobotClearance=len(passed), authorityRejected=auth_rej,
               events=dict(events), fullSearches=fullsearch, recertifications=recert,
               bankSelections=Counter(selected_names).most_common(3))
    if passed:
        out["reserve"] = dict(min=min(passed), median=statistics.median(passed), max=max(passed))
    if spreads:
        out["withinGenerationReserveSpread"] = dict(generations=len(spreads), medianSpread=statistics.median(spreads),
                                                    maxRatio=max(ratios) if ratios else None)
    return out

if __name__ == "__main__":
    for p in sys.argv[1:]:
        s = summarize(p)
        print(s)
