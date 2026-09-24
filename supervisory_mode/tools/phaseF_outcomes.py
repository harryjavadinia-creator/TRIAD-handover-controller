#!/usr/bin/env python3
"""Phase F in-situ outcomes of B0/B1/B2/FULL (descriptive; small samples).

Per run (logs/<repeat>/<variant>/<scenario>/<scenario>.log[.xz]):
  completed, failure reason, giver stop time (script start + first atRest sample),
  first plan adoption time (and whether the object was still moving),
  meeting: predictive = local_synchronization event (distance/angle to the standoff);
           reactive   = first TriadLiteTrack sample inside the terminal tube (12 mm, 0.05 rad),
  acquisition freeze time and freeze - giver stop,
  grasp switches, aborts, plan patches/retentions, refused/inconclusive results,
  selection-job latency.
usage: phaseF_outcomes.py EVIDENCE_DIR [EXTRA_DIR for variants absent from EVIDENCE_DIR]
"""
import glob, lzma, os, re, statistics as st, sys
from collections import defaultdict

KV = re.compile(r"(\w+)=(\[[^\]]*\]|[^ ]+)")
POS_TOL, ANG_TOL = 0.012, 0.050  # ReceiverV2 terminalPositionTolerance / terminalOrientationTolerance


def opener(p):
    return lzma.open(p, "rt", errors="ignore") if p.endswith(".xz") else open(p, errors="ignore")


def parse(path):
    r = dict(completed=False, failure="", giverStop=float("nan"), firstPlan=float("nan"), firstPlanMoving=False,
             meetT=float("nan"), meetDist=float("nan"), meetAngle=float("nan"), freeze=float("nan"),
             switches=0, aborts=0, patched=0, retained=0, refused=0, inconclusive=0, latencies=[])
    start = None
    for line in opener(path):
        if "[GiverTruthScript]" in line:
            start = float(dict(KV.findall(line))["startTime"])
        elif "[GiverTruthSample]" in line and "atRest=true" in line and r["giverStop"] != r["giverStop"] and start is not None:
            r["giverStop"] = start + float(dict(KV.findall(line))["elapsed"])
        elif "[Completed] full plan-once" in line:
            r["completed"] = True
        elif "[V2Failure]" in line and not r["failure"]:
            r["failure"] = dict(KV.findall(line)).get("reason", "?")
        elif "[TriadLiteEvent]" in line:
            d = dict(KV.findall(line))
            t = float(d.get("t", "nan"))
            ty = d.get("type", "")
            if ty == "initial_select" and r["firstPlan"] != r["firstPlan"]:
                r["firstPlan"] = t
            elif ty.startswith("grasp_switch"):
                r["switches"] += 1
            elif ty == "abort_to_hold":
                r["aborts"] += 1
            elif ty == "plan_patched":
                r["patched"] += 1
            elif ty == "plan_retained":
                r["retained"] += 1
            elif ty == "result_refused":
                r["refused"] += 1
            elif ty == "result_inconclusive":
                r["inconclusive"] += 1
            elif ty == "local_synchronization" and r["meetT"] != r["meetT"]:
                r["meetT"], r["meetDist"], r["meetAngle"] = t, float(d["distToStandoff"]), float(d["angle"])
            elif ty == "grasp_freeze" and r["freeze"] != r["freeze"]:
                r["freeze"] = t
        elif "[TriadLiteTrack]" in line and r["meetT"] != r["meetT"] and "/reactive/" in path:
            d = dict(KV.findall(line))
            if float(d["dist"]) <= POS_TOL and float(d["angle"]) <= ANG_TOL:
                r["meetT"], r["meetDist"], r["meetAngle"] = float(d["t"]), float(d["dist"]), float(d["angle"])
        elif "[V2JobProfile] type=CONTROL_AWARE_SELECT" in line:
            m = re.search(r"latency=([0-9.]+)s", line)
            if m:
                r["latencies"].append(float(m.group(1)))
    if r["firstPlan"] == r["firstPlan"] and r["giverStop"] == r["giverStop"]:
        r["firstPlanMoving"] = r["firstPlan"] < r["giverStop"]
    return r


def fmt(v, spec=".2f"):
    return "—" if v != v else format(v, spec)


def main(E, extra=None):
    """E: evidence dir; extra: optional dir supplying variants absent from E (e.g. reactive, regression)."""
    rows = defaultdict(list)
    for p in sorted(glob.glob(os.path.join(E, "logs", "r*", "*", "*", "*.log*"))):
        parts = p.split("/")
        variant, scenario = parts[-3], parts[-2]
        rows[(scenario, variant)].append((parts[-4], parse(p)))
    if extra:
        present = {v for (_, v) in rows}
        for p in sorted(glob.glob(os.path.join(extra, "logs", "r*", "*", "*", "*.log*"))):
            parts = p.split("/")
            if parts[-3] not in present:
                rows[(parts[-2], parts[-3])].append((parts[-4], parse(p)))
    order = ["reactive", "predictive", "predictive_capability", "full"]
    scen = ["longitudinal", "near-ground", "lateral-low", "diagonal"]
    print("| scenario | variant | completed | first plan while moving | meeting (t − stop) s | meeting error mm / rad | freeze − giver stop s | switches | aborts | patched / retained | refused / inconclusive | failures |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    totals = defaultdict(lambda: [0, 0])
    for s in scen:
        for v in order:
            runs = rows.get((s, v), [])
            if not runs:
                continue
            rs = [r for _, r in runs]
            done = sum(r["completed"] for r in rs)
            totals[v][0] += done
            totals[v][1] += len(rs)
            mv = sum(r["firstPlanMoving"] for r in rs)
            meet = [r["meetT"] - r["giverStop"] for r in rs if r["meetT"] == r["meetT"]]
            merr = [(1000 * r["meetDist"], r["meetAngle"]) for r in rs if r["meetT"] == r["meetT"]]
            fz = [r["freeze"] - r["giverStop"] for r in rs if r["freeze"] == r["freeze"]]
            fails = ",".join(sorted({r["failure"] for r in rs if r["failure"] and not r["completed"]})) or "—"
            print(f"| {s} | {v} | {done}/{len(rs)} | {mv}/{len(rs)} | "
                  f"{'; '.join(fmt(x) for x in meet) or '—'} | {'; '.join(f'{a:.1f}/{b:.3f}' for a, b in merr) or '—'} | "
                  f"{'; '.join(fmt(x) for x in fz) or '—'} | {sum(r['switches'] for r in rs)} | {sum(r['aborts'] for r in rs)} | "
                  f"{sum(r['patched'] for r in rs)} / {sum(r['retained'] for r in rs)} | "
                  f"{sum(r['refused'] for r in rs)} / {sum(r['inconclusive'] for r in rs)} | {fails} |")
    print()
    print("completions per variant: " + ", ".join(f"{v} {totals[v][0]}/{totals[v][1]}" for v in order if totals[v][1]))
    for v in order:
        lat = [x for (s, vv), runs in rows.items() if vv == v for _, r in runs for x in r["latencies"]]
        if lat:
            lat.sort()
            print(f"selection-job latency {v}: median={st.median(lat):.3f}s p90={lat[int(.9 * (len(lat) - 1))]:.3f}s max={lat[-1]:.3f}s n={len(lat)}")
    reg = sorted(glob.glob(os.path.join(extra or E, "logs", "regression", "*", "*", "*.log*")))
    if reg:
        print("regression bank_search: " + ", ".join(f"{p.split('/')[-2]} completed={parse(p)['completed']}" for p in reg))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
