#!/usr/bin/env python3
"""TRIAD V2 computational and candidate-space characterization.

Reads V2 run logs and prints markdown. Nothing is estimated that the logs do
not contain; every number is a count or a logged value.

  timing LOG...           per job type (FULL_SEARCH, RECERTIFY_ACTIVE,
                          TERMINAL_CERTIFY): wall time, latency, per-stage wall
                          and counts ([V2JobProfile]); cancellation latency and
                          snapshot age/drift ([V2JobCancelled], [V2SnapshotAudit])
  stages LOG...           deepest certified stage of every evaluated candidate
                          ([CertStage], memoized hypotheses expanded), downstream-
                          only rejections, and the reach-only counterfactual
                          selectors versus complete-action certification
  subset --search moving|rest DEFAULT_LOG SUPERSET_LOG
                          reproducibility check: every complete record of the
                          default-bank characterization run appears with the same
                          lead, grasp, route and objective in the superset run
  supersession LOG...     prediction-update handling comparison per run: time to
                          the first certified provisional plan relative to the
                          search epoch and to the giver's rest time, discarded
                          work units, concurrent robot/object motion, and worker
                          load (FULL_SEARCH and all jobs) until commitment
  tgr --search moving|rest LOG
                          T/G/R resolution study on a characterization run
                          (characterizeOnly: true) whose bank is a superset grid:
                          leads on a 0.05 s grid, candidateCount 64,
                          routeDirections 16, apexOffsets [0.04,0.08,0.14,0.20]

Stage ladder (from the certifier's own phase order):
  NONE < REACH < INSERTION < CLOSURE_CONTACT < CARRIED_RETREAT
REACH: standoff reached within tracking tolerance and clearance reserve.
INSERTION: capture pose reached through the corridor (route: plus capture
dwell/velocity gate). CLOSURE_CONTACT: closure sweep reached bilateral pad
contact with all hard constraints. Transfer readiness has no separate,
candidate-dependent test in the certifier (it is implied by bilateral contact),
so it cannot discriminate candidates and is not a separate rung.
CARRIED_RETREAT: carried retreat certified; a route record is COMPLETE when it
also has a valid complete-plan cost audit.
"""

import math
import re
import statistics
import sys
from collections import Counter, defaultdict

STAGES = ["NONE", "REACH", "INSERTION", "CLOSURE_CONTACT", "CARRIED_RETREAT"]
RANK = {s: i for i, s in enumerate(STAGES)}
KV = re.compile(r"(\w+)=(\S+)")


def kvs(line):
    tag = line.find("] [")
    body = line[tag:] if tag >= 0 else line
    return {k: v for k, v in KV.findall(body)}


def fnum(v):
    if v is None:
        return None
    v = v.rstrip("sm")
    try:
        return float(v)
    except ValueError:
        return None


def q(values, p):
    values = sorted(v for v in values if v is not None)
    if not values:
        return None
    k = (len(values) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return values[lo] + (values[hi] - values[lo]) * (k - lo)


def f(v, d=3):
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.{d}f}"
    return str(v)


# ---------------------------------------------------------------------------
# parsing
# ---------------------------------------------------------------------------

def load(log):
    jobs = {}          # gen -> dict(type, submit line kv, outcome, profile kv)
    cert = []          # CertStage kv dicts
    reuse = []
    complete = []
    selections = []
    audits = []
    cancels = []
    for line in open(log, errors="replace"):
        if "[V2PlanningJobSubmit]" in line:
            d = kvs(line)
            jobs.setdefault(d["planningGeneration"], {}).update(type=d["type"], submit=d, outcome="pending")
        elif "[V2PlanningJobResult]" in line:
            d = kvs(line)
            jobs.setdefault(d["planningGeneration"], {}).update(outcome="accepted", result=d)
        elif "[V2StaleResultRejected]" in line:
            d = kvs(line)
            jobs.setdefault(d["planningGeneration"], {}).update(outcome="stale")
        elif "[V2JobCancelled]" in line:
            d = kvs(line)
            jobs.setdefault(d["planningGeneration"], {}).update(outcome="cancelled", cancel=d)
            cancels.append(d)
        elif "[V2JobProfile]" in line:
            d = kvs(line)
            jobs.setdefault(d["planningGeneration"], {})["profile"] = d
        elif "[V2SnapshotAudit]" in line:
            audits.append(kvs(line))
        elif "[CertStage]" in line:
            cert.append(kvs(line))
        elif "[V2HypothesisCertificationReused]" in line:
            reuse.append(kvs(line))
        elif "[V2CompleteRecord]" in line:
            complete.append(kvs(line))
        elif "[V2CharacterizationSelection]" in line:
            selections.append(kvs(line))
    return dict(jobs=jobs, cert=cert, reuse=reuse, complete=complete,
                selections=selections, audits=audits, cancels=cancels)


def expand_search_records(data, gen):
    """CertStage records of one FULL_SEARCH, with memoized hypotheses expanded
    from their bitwise-identical source hypothesis."""
    recs = [r for r in data["cert"] if r.get("job") == "FULL_SEARCH" and r.get("planningGeneration") == gen]
    by_hyp = defaultdict(list)
    for r in recs:
        by_hyp[r["hypothesis"]].append(r)
    out = list(recs)
    for u in data["reuse"]:
        if u.get("planningGeneration") != gen:
            continue
        for r in by_hyp.get(u["sourceHypothesis"], []):
            c = dict(r)
            c.update(hypothesis=u["hypothesis"], lead=u["lead"].rstrip("s"), eventTime=u["eventTime"], reused="true")
            out.append(c)
    return out


# ---------------------------------------------------------------------------
# timing
# ---------------------------------------------------------------------------

BUCKETS = ["staticReachStandoff", "staticReachCapture", "staticClosure", "staticRetreat",
           "routeSetup", "routeReach", "routeApproach", "routeDwell", "routeClosure", "routeRetreat",
           "routeFinalize", "hypothesisSetup", "terminalStandoff",
           "nestedIkStep", "nestedSweptQuery", "nestedConfigurationSafety", "nestedClosureSafety"]


def timing(logs):
    rows = defaultdict(list)
    cancel_lat = []
    audit = defaultdict(list)
    for log in logs:
        data = load(log)
        for gen, j in data["jobs"].items():
            p = j.get("profile")
            if not p or "type" not in j:
                continue
            rows[(j["type"], j["outcome"])].append(p)
        cancel_lat += [(c["type"], fnum(c["cancelLatency"]), c.get("workerStoppedByCancel")) for c in data["cancels"]]
        for a in data["audits"]:
            audit[(a["type"], a["outcome"])].append(a)
    print("## Worker timing per job type\n")
    print("Wall = worker wall time of the job; latency = controller time from submission to receipt.\n")
    print("| job | outcome | n | wall median | wall p95 | wall max | latency median | latency max | hypotheses (median) | static records (median) | route records (median) |")
    print("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for (t, o), ps in sorted(rows.items()):
        wall = [fnum(p["jobWall"]) for p in ps]
        lat = [fnum(p["latency"]) for p in ps]
        print(f"| {t} | {o} | {len(ps)} | {f(q(wall,.5),4)} | {f(q(wall,.95),4)} | {f(max(wall),4)} | "
              f"{f(q(lat,.5),3)} | {f(max(lat),3)} | {f(q([fnum(p['hypotheses']) for p in ps],.5),0)} | "
              f"{f(q([fnum(p['staticRecords']) for p in ps],.5),0)} | {f(q([fnum(p['routeRecords']) for p in ps],.5),0)} |")
    print("\n### Stage wall time (sum over jobs, completed or accepted jobs only)\n")
    print("Nested buckets are contained in the phase buckets; phase buckets partition the certification work.\n")
    header = "| job | " + " | ".join(BUCKETS) + " |"
    print(header)
    print("|---|" + "---:|" * len(BUCKETS))
    for t in ("FULL_SEARCH", "RECERTIFY_ACTIVE", "TERMINAL_CERTIFY"):
        ps = rows.get((t, "accepted"), [])
        if not ps:
            continue
        total = {b: 0.0 for b in BUCKETS}
        count = {b: 0 for b in BUCKETS}
        for p in ps:
            for b in BUCKETS:
                w, c = p.get(b, "0s/0").split("/")
                total[b] += fnum(w)
                count[b] += int(c)
        jw = sum(fnum(p["jobWall"]) for p in ps)
        print(f"| {t} (n={len(ps)}, wall {jw:.3f}s) | " + " | ".join(
            f"{total[b]:.3f}s ({100*total[b]/jw if jw else 0:.0f}%)/{count[b]}" for b in BUCKETS) + " |")
    if cancel_lat:
        print("\n### Cancellation\n")
        for t in sorted({c[0] for c in cancel_lat}):
            lat = [c[1] for c in cancel_lat if c[0] == t]
            stopped = sum(1 for c in cancel_lat if c[0] == t and c[2] == "true")
            print(f"- {t}: {len(lat)} cancelled; controller-time cancel latency median {f(q(lat,.5))} s, max {f(max(lat))} s; "
                  f"worker stopped by cancel in {stopped}, finished before the request was observed in {len(lat)-stopped}")
    print("\n### Planning-snapshot age and drift at result receipt\n")
    print("| job | outcome | n | age median s | age max s | robot drift max m | object displacement max m | snapshot prediction error max m |")
    print("|---|---|---:|---:|---:|---:|---:|---:|")
    for (t, o), a in sorted(audit.items()):
        g = lambda k: [fnum(x[k]) for x in a]
        print(f"| {t} | {o} | {len(a)} | {f(q(g('snapshotAge'),.5))} | {f(max(g('snapshotAge')))} | "
              f"{f(max(g('robotDrift')),4)} | {f(max(g('objectDisplacementSinceSnapshot')),4)} | {f(max(g('snapshotPredictionError')),4)} |")


# ---------------------------------------------------------------------------
# stages (goal C)
# ---------------------------------------------------------------------------

def admissible(lead, reach_time, min_commit, min_entry):
    return lead + 1e-12 >= min_commit and reach_time is not None and reach_time + min_entry <= lead + 1e-12


def stages(logs, min_commit=1.6, min_entry=0.05):
    tot = Counter()
    downstream_reasons = Counter()
    cf_rows = []
    for log in logs:
        data = load(log)
        for gen, j in sorted(data["jobs"].items(), key=lambda kv: int(kv[0])):
            if j.get("type") != "FULL_SEARCH" or j.get("outcome") not in ("accepted",):
                continue
            recs = expand_search_records(data, gen)
            if not recs:
                continue
            for r in recs:
                tot[(r["path"], r["deepest"], r["path"] == "route" and r["feasible"] == "true" and r["costValid"] == "true")] += 1
                if r["deepest"] in ("REACH", "INSERTION", "CLOSURE_CONTACT"):
                    reason = re.sub(r"/[^/]*$", "", r["reason"]) if r["reason"].count("/") > 2 else r["reason"]
                    downstream_reasons[(r["path"], r["deepest"], reason)] += 1
            cf_rows.append(counterfactual(log, gen, recs, min_commit, min_entry))
    print("## Deepest certified stage of every evaluated candidate (accepted FULL_SEARCH jobs, memoized hypotheses expanded)\n")
    for path in ("static", "route"):
        n = sum(v for k, v in tot.items() if k[0] == path)
        if not n:
            continue
        cells = []
        for s in STAGES:
            c = sum(v for k, v in tot.items() if k[0] == path and k[1] == s)
            cells.append(f"{s} {c} ({100*c/n:.1f}%)")
        extra = ""
        if path == "route":
            comp = sum(v for k, v in tot.items() if k[0] == "route" and k[2])
            extra = f"; COMPLETE (retreat + valid cost audit) {comp} ({100*comp/n:.1f}%)"
        down = sum(v for k, v in tot.items() if k[0] == path and k[1] in ("REACH", "INSERTION", "CLOSURE_CONTACT"))
        reach_ok = sum(v for k, v in tot.items() if k[0] == path and RANK[k[1]] >= 1)
        print(f"- **{path}** (n={n}): " + ", ".join(cells) + extra)
        print(f"  - rejected only downstream of reach: {down} = {100*down/n:.1f}% of evaluated, "
              f"{100*down/reach_ok if reach_ok else 0:.1f}% of reach-certified")
    print("\n### Most frequent downstream-only rejection reasons\n")
    print("| path | deepest | reason | count |")
    print("|---|---|---|---:|")
    for (p, d, why), c in downstream_reasons.most_common(15):
        print(f"| {p} | {d} | `{why}` | {c} |")
    print("\nRoute rollouts are run only for grasps that already passed the complete static screen (direct copied-state "
          "reach, insertion, closure, carried retreat), so route-level stage counts are conditional on that screen.")
    print("\n### Reach-only selection versus complete-action certification\n")
    print("Reach-only rule (lexicographic): earliest timing-admissible event, then shortest reach time, then largest reach "
          f"clearance; admissible at the search epoch when lead >= {min_commit} s and reach time + {min_entry} s <= lead. "
          "Static: grasp reach to standoff on the direct copied-state screen. Route: (grasp, route) whose route reach "
          "phase certified. 'complete?' = the chosen action has a COMPLETE record (static: any route of that grasp at "
          "that event).\n")
    print("| log | job | reach-only static choice | complete? | deepest | reach-only route choice | complete? | deepest (reason) | reach-feasible static actions at that event that are complete | reach-feasible route actions at that event that are complete |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for row in cf_rows:
        print("| " + " | ".join(row) + " |")


def counterfactual(log, gen, recs, min_commit, min_entry):
    completes = {(r["hypothesis"], r["candidate"], r["route"]) for r in recs
                 if r["path"] == "route" and r["feasible"] == "true" and r["costValid"] == "true"}
    complete_grasps = {(h, c) for h, c, _ in completes}
    static = [r for r in recs if r["path"] == "static" and RANK[r["deepest"]] >= 1]
    route = [r for r in recs if r["path"] == "route" and RANK[r["deepest"]] >= 1]

    def key_static(r):
        return (fnum(r["lead"]), fnum(r["staticReachTime"]), -(fnum(r["reachClear"]) or -1e9), r["candidate"])

    def key_route(r):
        return (fnum(r["lead"]), fnum(r["routeReachDuration"]), -(fnum(r["reachClear"]) or -1e9), r["candidate"], r["route"])

    s_adm = [r for r in static if admissible(fnum(r["lead"]), fnum(r["staticReachTime"]), min_commit, min_entry)]
    r_adm = [r for r in route if admissible(fnum(r["lead"]), fnum(r["routeReachDuration"]), min_commit, min_entry)]
    name = "/".join(log.split("/")[-3:-1])
    cells = [name, gen]
    if s_adm:
        s = min(s_adm, key=key_static)
        ok = (s["hypothesis"], s["candidate"]) in complete_grasps
        same = [r for r in s_adm if r["hypothesis"] == s["hypothesis"]]
        frac = sum((r["hypothesis"], r["candidate"]) in complete_grasps for r in same)
        cells += [f"τ={s['lead']} {s['candidate']}", "yes" if ok else "**NO**", s["deepest"]]
    else:
        cells += ["none", "", ""]
        same, frac = [], 0
    if r_adm:
        r = min(r_adm, key=key_route)
        ok = (r["hypothesis"], r["candidate"], r["route"]) in completes
        rs = [x for x in r_adm if x["hypothesis"] == r["hypothesis"]]
        rfrac = sum((x["hypothesis"], x["candidate"], x["route"]) in completes for x in rs)
        cells += [f"τ={r['lead']} {r['candidate']}/{r['route']}", "yes" if ok else "**NO**",
                  f"{r['deepest']} ({r['reason'][:60]})", f"{frac}/{len(same)}", f"{rfrac}/{len(rs)}"]
    else:
        cells += ["none", "", "", f"{frac}/{len(same)}", "0/0"]
    return cells


# ---------------------------------------------------------------------------
# T/G/R resolution (goal B)
# ---------------------------------------------------------------------------

def on_grid(x, step, origin, tol=1e-6):
    k = (x - origin) / step
    return abs(k - round(k)) < tol / step


def route_geometry(name):
    if name == "direct":
        return (0, 0.0)
    m = re.match(r"ring(\d+)mm_(\d+)of(\d+)", name)
    return (int(m.group(1)), int(m.group(2)) / int(m.group(3)))


def tgr(log, which):
    data = load(log)
    fulls = sorted((int(g), j) for g, j in data["jobs"].items()
                   if j.get("type") == "FULL_SEARCH" and j.get("outcome") == "accepted")
    if len(fulls) < 1:
        print("no accepted FULL_SEARCH in log")
        return 1
    gen, job = fulls[0] if which == "moving" else fulls[-1]
    gen = str(gen)
    submit = job["submit"]
    speed = fnum(submit.get("objectEstimateSpeed"))
    epoch = fnum(submit["t"])
    sels = [s for s in data["selections"] if s["planningGeneration"] == gen]
    sel_epoch = next((s for s in sels if s["evaluatedAt"] == "epoch"), None)
    min_commit = fnum(sel_epoch["minimumSafeCommitLead"]) if sel_epoch else 1.6
    min_entry = fnum(sel_epoch["minimumReachEntryLead"]) if sel_epoch else 0.05
    recs = expand_search_records(data, gen)
    comp = [c for c in data["complete"] if c["planningGeneration"] == gen]
    prof = job.get("profile", {})
    print(f"# T/G/R characterization — {log.split('/')[-1]}, {which} epoch search (generation {gen})\n")
    print(f"- search epoch t={epoch:.3f} s; object estimate speed at epoch {f(speed,4)} m/s; worker wall {prof.get('jobWall')}; "
          f"hypotheses {prof.get('hypotheses')}, memo reuses {prof.get('memoReuses')}, static records {prof.get('staticRecords')}, route records {prof.get('routeRecords')}")
    print(f"- timing admission applied here at the epoch: lead >= {min_commit} s and presentationDuration + {min_entry} s <= lead")
    if sel_epoch:
        print(f"- unchanged selector on the full grid at the epoch: {sel_epoch['candidate']}/{sel_epoch['route']} τ={sel_epoch['lead']} "
              f"(timing-admissible plans {sel_epoch['timingAdmissiblePlans']}, cost-valid {sel_epoch['costValidPlans']})")

    # complete records with timing admission
    R = []
    for c in comp:
        lead = fnum(c["lead"])
        R.append(dict(lead=lead, cand=c["candidate"], route=c["route"], valid=c["costValid"] == "true",
                      J=fnum(c["globalJ"]), Tp=fnum(c["presentationDuration"]),
                      adm=c["costValid"] == "true" and admissible(lead, fnum(c["presentationDuration"]), min_commit, min_entry)))
    leads = sorted({fnum(r["lead"]) for r in recs if r["path"] == "static"})
    grasp_index = {}
    grasp_count = 0
    for r in recs:
        if r["path"] == "static":
            k, n = r["grasp"].split("/")
            grasp_index[r["candidate"]] = int(k)
            grasp_count = int(n)
    ring = grasp_count // 2 if grasp_count else 0

    def best(subset):
        adm = [r for r in subset if r["adm"]]
        if not adm:
            return None
        return min(adm, key=lambda r: (r["J"], r["lead"], r["cand"], r["route"]))

    full_best = best(R)
    if sel_epoch and full_best:
        agree = (full_best["cand"] == sel_epoch["candidate"] and full_best["route"] == sel_epoch["route"]
                 and abs(full_best["lead"] - fnum(sel_epoch["lead"])) < 1e-6)
        print(f"- re-implemented argmin globalJ on admissible cost-valid records: {full_best['cand']}/{full_best['route']} τ={full_best['lead']:.2f} "
              f"J={full_best['J']:.6f} — {'matches' if agree else 'DIFFERS FROM'} the logged selector (tie-break differences possible)")

    # ---- T
    print("\n## T — temporal interception interval and resolution\n")
    per_lead = defaultdict(lambda: dict(complete=0, adm=0))
    for r in R:
        if r["valid"]:
            per_lead[r["lead"]]["complete"] += 1
        if r["adm"]:
            per_lead[r["lead"]]["adm"] += 1
    geo = [l for l in leads if per_lead[l]["complete"] > 0]
    adm = [l for l in leads if per_lead[l]["adm"] > 0]

    def intervals(xs, step):
        out = []
        for x in xs:
            if out and x - out[-1][1] <= step + 1e-6:
                out[-1][1] = x
            else:
                out.append([x, x])
        return out

    step0 = min((b - a for a, b in zip(leads, leads[1:])), default=0.05)
    print(f"- evaluated leads: {len(leads)} on [{f(leads[0],2) if leads else 'n/a'}, {f(leads[-1],2) if leads else 'n/a'}] s, finest step {step0:.3f} s")
    print(f"- geometrically complete (any cost-valid complete action): {len(geo)} leads, intervals "
          + ", ".join(f"[{a:.2f}, {b:.2f}]" for a, b in intervals(geo, step0)))
    print(f"- timing-admissible and complete: {len(adm)} leads, intervals "
          + ", ".join(f"[{a:.2f}, {b:.2f}]" for a, b in intervals(adm, step0)))
    if speed is not None:
        print(f"- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is "
              f"crossed per step when Δτ > {0.015/speed if speed > 1e-9 else float('inf'):.3f} s at {speed:.4f} m/s")
    print("\n| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |")
    print("|---:|---|---:|---:|---:|---:|---:|---:|---|---:|")
    v1_leads = [1.8, 1.9, 2.35, 2.8, 3.25, 3.7, 4.15, 4.6, 5.05, 5.5, 5.95, 6.4, 6.85, 8.0]
    configs = [(0.05, 0.5, None), (0.10, 0.5, None), (0.20, 0.5, None), (0.45, 3.25, None), (0.90, 3.25, None),
               (1.80, 3.25, None), (None, None, v1_leads)]
    for step, origin, explicit in configs:
        if explicit:
            sub = [l for l in leads if any(abs(l - e) < 1e-6 for e in explicit)]
            label, org = "V1 14", "V1 bank"
        else:
            sub = [l for l in leads if on_grid(l, step, origin)]
            label, org = f"{step:.2f}", f"{origin}"
        subset = [r for r in R if any(abs(r["lead"] - l) < 1e-6 for l in sub)]
        b = best(subset)
        ea = min((l for l in sub if per_lead[l]["adm"] > 0), default=None)
        disp = (speed * (step if step else 0.45) * 1000) if speed is not None else None
        print(f"| {label} | {org} | {len(sub)} | {sum(per_lead[l]['complete']>0 for l in sub)} | {sum(per_lead[l]['adm']>0 for l in sub)} | "
              f"{f(ea,2)} | {f(b['J'],6) if b else 'n/a'} | {f(b['J']-full_best['J'],6) if b and full_best else 'n/a'} | "
              f"{(b['cand']+'/'+b['route']+' τ='+format(b['lead'],'.2f')) if b else 'none'} | {f(disp,1)} |")

    # ---- G
    print("\n## G — receiver approach family and angular resolution\n")
    if ring:
        print(f"- grasp ring: {ring} angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)")
        feas = defaultdict(set)   # lead -> set of grasp idx complete (any route)
        for r in R:
            if r["valid"] and r["cand"] in grasp_index:
                feas[r["lead"]].add(grasp_index[r["cand"]])
        allg = set().union(*feas.values()) if feas else set()
        for sign, off in (("axisP", 0), ("axisN", ring)):
            ks = sorted(k - off for k in allg if off <= k < off + ring)
            arcs = []
            for k in ks:
                if arcs and k == arcs[-1][1] + 1:
                    arcs[-1][1] = k
                else:
                    arcs.append([k, k])
            if len(arcs) > 1 and arcs[0][0] == 0 and arcs[-1][1] == ring - 1:
                arcs[0][0] = arcs[-1][0] - ring
                arcs.pop()
            deg = 360.0 / ring
            print(f"- {sign}: complete at any evaluated lead for {len(ks)}/{ring} angles; arcs (deg) "
                  + (", ".join(f"samples {a*deg:.1f}..{b*deg:.1f} ({b-a+1} samples, spacing {deg:.3f})" for a, b in arcs) or "none"))
        per_lead_arcs = []
        for l in sorted(feas):
            per_lead_arcs.append(len(feas[l]))
        print(f"- complete grasps per complete lead: median {f(q(per_lead_arcs,.5),0)}, min {min(per_lead_arcs) if per_lead_arcs else 'n/a'}, max {max(per_lead_arcs) if per_lead_arcs else 'n/a'} (of {2*ring})")
        print("\n| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |")
        print("|---:|---:|---:|---:|---:|---:|---:|---|---:|")
        for n in (4, 8, 16, 32, 64, 128):
            if n > ring or ring % n:
                continue
            stride = ring // n
            keep = {k for k in range(2 * ring) if (k % ring) % stride == 0}
            subset = [r for r in R if grasp_index.get(r["cand"], -1) in keep]
            b = best(subset)
            ls = {r["lead"] for r in subset if r["valid"]}
            la = {r["lead"] for r in subset if r["adm"]}
            nstatic = sum(1 for r in recs if r["path"] == "static" and int(r["grasp"].split("/")[0]) in keep)
            print(f"| {n} | {2*n} | {len(ls)} | {len(la)} | {f(min(la),2) if la else 'n/a'} | {f(b['J'],6) if b else 'n/a'} | "
                  f"{f(b['J']-full_best['J'],6) if b and full_best else 'n/a'} | {(b['cand']+'/'+b['route']) if b else 'none'} | {nstatic} |")
        signs = Counter(r["cand"].split("_")[0] for r in R if r["valid"])
        print(f"\n- complete records by handle-axis sign: {dict(signs)}")

    # ---- R
    print("\n## R — direct motion versus route alternatives\n")
    route_recs = [r for r in recs if r["path"] == "route"]
    by_action = defaultdict(dict)
    for r in route_recs:
        by_action[(fnum(r["lead"]), r["candidate"])][r["route"]] = (
            r["feasible"] == "true" and r["costValid"] == "true", r["deepest"], r["reason"])
    cat = Counter()
    direct_fail = Counter()
    for key, routes in by_action.items():
        d = routes.get("direct", (False, "NONE", "missing"))
        ring_ok = any(v[0] for k, v in routes.items() if k != "direct")
        if d[0]:
            cat["direct sufficient"] += 1
        elif ring_ok:
            cat["route alternative required"] += 1
            direct_fail[(d[1], re.sub(r"(/[^/]+){2,}$", "", d[2]))] += 1
        else:
            cat["no complete route"] += 1
    n = sum(cat.values())
    print(f"- (event, grasp) pairs that passed the static screen and entered route certification: {n}")
    for k in ("direct sufficient", "route alternative required", "no complete route"):
        print(f"  - {k}: {cat[k]} ({100*cat[k]/n if n else 0:.1f}%)")
    if direct_fail:
        print("- where a route was required, the direct route failed at: " +
              "; ".join(f"{s} `{w}` ×{c}" for (s, w), c in direct_fail.most_common(6)))
    names = sorted({r["route"] for r in route_recs}, key=lambda s: route_geometry(s))
    radii = sorted({route_geometry(s)[0] for s in names if s != "direct"})
    dirs = max((int(re.match(r"ring\d+mm_\d+of(\d+)", s).group(1)) for s in names if s != "direct"), default=0)
    route_wall = sum(fnum(prof.get(b, "0s/0").split("/")[0]) for b in BUCKETS[4:11])
    per_route_time = route_wall / max(1, int(prof.get("routeRecords", "1") or 1))
    print(f"- route wall time in this search {route_wall:.3f} s over {prof.get('routeRecords')} route rollouts "
          f"(mean {1000*per_route_time:.2f} ms per rollout; memoized hypotheses cost nothing)")
    subsets = [("direct only", lambda g: g[0] == 0)]
    for rad_set in ([40], [80], [140], [200], [80, 140], [40, 80, 140, 200]):
        for nd in (4, 8, 16):
            if dirs and dirs % nd == 0 and all(x in radii for x in rad_set):
                subsets.append((f"direct + {nd} dirs × {rad_set} mm",
                                lambda g, rs=rad_set, nd=nd: g[0] == 0 or (g[0] in rs and abs(g[1] * nd - round(g[1] * nd)) < 1e-9)))
    print("\n| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |")
    print("|---|---:|---:|---:|---:|---:|---|---:|")
    for label, pred in subsets:
        keep = {s for s in names if pred(route_geometry(s))}
        ok_pairs = sum(1 for routes in by_action.values() if any(v[0] for k, v in routes.items() if k in keep))
        subset = [r for r in R if r["route"] in keep]
        b = best(subset)
        rollouts = sum(1 for r in route_recs if r["route"] in keep)
        print(f"| {label} | {len(keep)} | {ok_pairs} | {len({r['lead'] for r in subset if r['valid']})} | {f(b['J'],6) if b else 'n/a'} | "
              f"{f(b['J']-full_best['J'],6) if b and full_best else 'n/a'} | {(b['cand']+'/'+b['route']+' τ='+format(b['lead'],'.2f')) if b else 'none'} | {rollouts} |")
    return 0


def pick_search(data, which):
    fulls = sorted((int(g), j) for g, j in data["jobs"].items()
                   if j.get("type") == "FULL_SEARCH" and j.get("outcome") == "accepted")
    if not fulls:
        return None, None
    g, j = fulls[0] if which == "moving" else fulls[-1]
    return str(g), j


def subset_check(default_log, superset_log, which):
    a, b = load(default_log), load(superset_log)
    ga, ja = pick_search(a, which)
    gb, jb = pick_search(b, which)
    ta, tb = fnum(ja["submit"]["t"]), fnum(jb["submit"]["t"])
    def keyed(data, gen):
        grasp = {}
        for r in data["cert"]:
            if r.get("path") == "static" and r.get("planningGeneration") == gen:
                k, n = (int(x) for x in r["grasp"].split("/"))
                ring = n // 2
                grasp[r["candidate"]] = (k // ring, round((k % ring) / ring, 9))
        out = {}
        for c in data["complete"]:
            if c["planningGeneration"] != gen:
                continue
            rad, frac = route_geometry(c["route"])
            out[(round(fnum(c["lead"]), 3), grasp.get(c["candidate"], c["candidate"]), (rad, round(frac, 9)))] = fnum(c["globalJ"])
        return out
    ra = keyed(a, ga)
    rb = keyed(b, gb)
    missing = [k for k in ra if k not in rb]
    differ = [k for k in ra if k in rb and abs(ra[k] - rb[k]) > 1e-6]
    leads_a = {k[0] for k in ra}
    extra_same_leads = [k for k in rb if k[0] in leads_a and k not in ra]
    ok = not missing and not differ and abs(ta - tb) < 1e-9
    print(f"subset {which}: epoch {ta:.3f} vs {tb:.3f}; default complete records {len(ra)}, superset {len(rb)}; "
          f"missing {len(missing)}, objective differs {len(differ)}, superset-only records at default leads {len(extra_same_leads)} "
          f"-> {'PASS' if ok else 'FAIL'}")
    for k in (missing + differ)[:5]:
        print("   ", k, ra.get(k), rb.get(k))
    return 0 if ok else 1


PHASE_BUCKETS = BUCKETS[:12]


def units_of(profile):
    return sum(int(profile.get(b, "0s/0").split("/")[1]) for b in PHASE_BUCKETS)


def supersession_metrics(log):
    lines = open(log, errors="replace").read().splitlines()
    mode = "cancel_and_restart"
    giver = None
    submits, adopts, profiles, motion, moved, certs = [], [], [], [], [], {}
    commit_t = None
    completed = False
    for l in lines:
        if "[V2PredictionUpdateMode]" in l:
            mode = kvs(l)["fullSearchPredictionUpdate"]
        elif "[GiverTruthScript]" in l:
            giver = kvs(l)
        elif "[V2PlanningJobSubmit]" in l:
            submits.append(kvs(l))
        elif "[V2ProvisionalAdopt]" in l:
            adopts.append(kvs(l))
        elif "[V2JobProfile]" in l:
            profiles.append(kvs(l))
        elif "[V2Motion]" in l and commit_t is None:
            motion.append(kvs(l))
        elif "[V2SelectedTargetMoved]" in l:
            moved.append(kvs(l))
        elif "[V2SelectedCertification]" in l and "success=" in l:
            d = kvs(l)
            certs[d["certificateGeneration"]] = d["success"] == "true"
        elif "[V2TerminalCommit] committed=true" in l:
            commit_t = fnum(kvs(l)["commitTime"])
        elif "[Completed] full plan-once handover completed" in l:
            completed = True
    start = fnum(giver["startTime"])
    decel = start + fnum(giver["cruiseDuration"])
    rest = decel + fnum(giver["stopDuration"])
    full_submits = [x for x in submits if x["type"] == "FULL_SEARCH"]
    epoch0 = fnum(full_submits[0]["t"]) if full_submits else None
    first = adopts[0] if adopts else None
    first_t = fnum(first["t"]) if first else None
    horizon = commit_t if commit_t is not None else (fnum(profiles[-1]["t"]) if profiles else None)
    full = [p for p in profiles if p["type"] == "FULL_SEARCH"]
    full_cancel = [p for p in full if p["cancelRequested"] == "true"]
    failed_certs = [p for p in profiles if p["type"] == "CERTIFY_SELECTED"
                    and (p["cancelRequested"] == "true" or not certs.get(p["planningGeneration"], False))]
    discarded_units = sum(units_of(p) for p in full_cancel) + sum(units_of(p) for p in failed_certs)
    discarded_wall = sum(fnum(p["jobWall"]) for p in full_cancel) + sum(fnum(p["jobWall"]) for p in failed_certs)
    before_first = [p for p in full if first_t is None or fnum(p["t"]) <= first_t + 1e-9]
    source_gen = first.get("sourcePlanningGeneration") if first else None
    cert_gen = first.get("adoptionCertificateGeneration") if first else None
    useful_wall = sum(fnum(p["jobWall"]) for p in profiles
                      if p["planningGeneration"] in (source_gen, cert_gen) and (first_t is None or fnum(p["t"]) <= first_t + 1e-9))
    concurrent = sum(1 for m in motion if m.get("robotMoving") == "true" and m.get("objectMoving") == "true") * 0.05
    all_wall = sum(fnum(p["jobWall"]) for p in profiles if horizon is None or fnum(p["t"]) <= horizon + 1e-9)
    return dict(
        mode=mode, completed=completed, giverDecel=decel, giverRest=rest, epoch0=epoch0,
        firstAdopt=first_t,
        timeToFirstPlan=(first_t - epoch0) if first_t is not None and epoch0 is not None else None,
        firstAdoptMinusRest=(first_t - rest) if first_t is not None else None,
        firstAdoptWhileMoving=(first_t is not None and first_t < rest),
        firstAdoptLead=first.get("eventLead", "").rstrip("s") if first else None,
        commit=commit_t,
        fullSearches=len(full), fullCancelled=len(full_cancel),
        selectedTargetMoved=len(moved),
        selectedCertOk=sum(1 for v in certs.values() if v),
        selectedCertFailed=sum(1 for v in certs.values() if not v),
        firstAdoptSource=first.get("adoptionSource", "search") if first else None,
        discardedUnits=discarded_units, discardedWall=discarded_wall,
        fullUnits=sum(units_of(p) for p in full), fullWall=sum(fnum(p["jobWall"]) for p in full),
        fullWallToFirstPlan=sum(fnum(p["jobWall"]) for p in before_first),
        fullUnitsToFirstPlan=sum(units_of(p) for p in before_first),
        usefulWallToFirstPlan=useful_wall,
        unusedWallToFirstPlan=(sum(fnum(p["jobWall"]) for p in profiles if first_t is None or fnum(p["t"]) <= first_t + 1e-9) - useful_wall) if first_t is not None else None,
        allWorkerWallToCommit=all_wall,
        concurrentMotion=concurrent,
        replacements=sum(1 for a in adopts if a.get("kind") == "REPLACEMENT"),
    )


def supersession(logs):
    cols = ["mode", "completed", "timeToFirstPlan", "firstAdoptMinusRest", "firstAdoptWhileMoving", "firstAdoptLead",
            "firstAdoptSource", "fullSearches", "fullCancelled", "selectedTargetMoved", "selectedCertOk", "selectedCertFailed",
            "discardedUnits", "discardedWall", "fullUnits", "fullWall", "fullWallToFirstPlan", "usefulWallToFirstPlan", "unusedWallToFirstPlan",
            "allWorkerWallToCommit", "concurrentMotion", "replacements"]
    print("## Prediction-update handling: per run\n")
    print("Times in s. timeToFirstPlan: first provisional adoption minus first FULL_SEARCH submission. "
          "firstAdoptMinusRest: negative means the plan was obtained before the giver came to rest. "
          "discardedUnits: bounded planner work units (static preview steps, route work units, hypothesis setups) "
          "belonging to cancelled searches or to failed/cancelled selected-action certifications. concurrentMotion: 50 ms samples "
          "with robot and object truth both moving, before commitment. usefulWallToFirstPlan: worker wall of the search "
          "generation (and certificate) that produced the first plan; unusedWallToFirstPlan: all other worker wall before "
          "the first plan, whether the work was cancelled or completed but not used.\n")
    print("| run | " + " | ".join(cols) + " |")
    print("|---|" + "---|" * len(cols))
    rows = []
    for log in logs:
        m = supersession_metrics(log)
        rows.append(m)
        name = "/".join(log.split("/")[-3:-1])
        print(f"| {name} | " + " | ".join(f(m[c], 3) if not isinstance(m[c], bool) else ("yes" if m[c] else "no") for c in cols) + " |")
    print("\n## Per mode (median over runs; counts are totals)\n")
    print("| mode | runs | completed | first plan while moving | median timeToFirstPlan | median firstAdoptMinusRest | total discardedUnits | total discardedWall | median fullWall | median concurrentMotion |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for mode in sorted({r["mode"] for r in rows}):
        rs = [r for r in rows if r["mode"] == mode]
        print(f"| {mode} | {len(rs)} | {sum(r['completed'] for r in rs)} | {sum(r['firstAdoptWhileMoving'] for r in rs)} | "
              f"{f(q([r['timeToFirstPlan'] for r in rs], .5))} | {f(q([r['firstAdoptMinusRest'] for r in rs], .5))} | "
              f"{sum(r['discardedUnits'] for r in rs)} | {f(sum(r['discardedWall'] for r in rs))} | "
              f"{f(q([r['fullWall'] for r in rs], .5))} | {f(q([r['concurrentMotion'] for r in rs], .5))} |")


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    mode = argv[1]
    if mode == "timing":
        timing(argv[2:])
    elif mode == "stages":
        stages(argv[2:])
    elif mode == "supersession":
        supersession(argv[2:])
    elif mode == "subset":
        args = argv[2:]
        which = "moving"
        if args[0] == "--search":
            which = args[1]
            args = args[2:]
        return subset_check(args[0], args[1], which)
    elif mode == "tgr":
        which = "moving"
        args = argv[2:]
        if args[0] == "--search":
            which = args[1]
            args = args[2:]
        return tgr(args[0], which)
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
