#!/usr/bin/env python3
"""Dependency-free integrity and consistency check for the reduced evidence package.

Checks, in order:

1. every file listed in evidence/MANIFEST.sha256 hashes to its recorded digest;
2. the two pre-registered input sets still hash to the values recorded before
   their campaigns were executed;
3. the published frozen plan sets hash to the digests recorded alongside them;
4. the headline outcome counts quoted in evidence/README.md are exactly what the
   raw per-run records contain;
5. selected publication-facing interpretation guards remain in their corrected
   form.

Exit status is 0 only if every check passes.
"""
import hashlib
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EV = os.path.join(ROOT, "evidence")

failures = []
checks = 0


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def check(label, condition, detail=""):
    global checks
    checks += 1
    if condition:
        print("  PASS  %s" % label)
    else:
        print("  FAIL  %s%s" % (label, ("  --  " + detail) if detail else ""))
        failures.append(label)


def manifest_entries():
    path = os.path.join(EV, "MANIFEST.sha256")
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.rstrip("\n")
            if not line or line.startswith("#"):
                continue
            digest, _, rel = line.partition("  ")
            yield digest, rel


print("1. evidence manifest")
missing = 0
bad = 0
count = 0
for digest, rel in manifest_entries():
    target = os.path.join(EV, rel)
    count += 1
    if not os.path.isfile(target):
        missing += 1
        continue
    if sha256(target) != digest:
        bad += 1
check("all %d manifest entries present" % count, missing == 0, "%d missing" % missing)
check("all %d manifest digests match" % count, bad == 0, "%d mismatched" % bad)

print("2. pre-registered input sets")
for csv_rel, sha_rel in (
    ("generalization/held_out_scenarios.csv", "generalization/held_out_scenarios.csv.sha256"),
    ("robustness/perturbations.csv", "robustness/perturbations.csv.sha256"),
):
    csv_path = os.path.join(EV, csv_rel)
    sha_path = os.path.join(EV, sha_rel)
    recorded = ""
    if os.path.isfile(sha_path):
        with open(sha_path, encoding="utf-8") as handle:
            recorded = handle.read().split()[0]
    actual = sha256(csv_path) if os.path.isfile(csv_path) else ""
    check("%s matches its pre-execution hash" % os.path.basename(csv_rel),
          bool(recorded) and recorded == actual,
          "recorded %s actual %s" % (recorded[:16], actual[:16]))

print("3. published frozen plan sets")
for scenario in ("near-ground", "longitudinal", "lateral-low", "diagonal"):
    plan = os.path.join(EV, "async", scenario, "frozen_plan_set.txt")
    rec = os.path.join(EV, "async", scenario, "frozen_plan_set_sha.txt")
    if not (os.path.isfile(plan) and os.path.isfile(rec)):
        check("%s frozen plan set present" % scenario, False)
        continue
    with open(rec, encoding="utf-8") as handle:
        recorded = handle.read().split()[0]
    check("%s frozen plan set matches its recorded digest" % scenario,
          sha256(plan) == recorded)

print("4. headline counts recomputed from the raw records")
with open(os.path.join(EV, "generalization", "outcomes.json"), encoding="utf-8") as handle:
    gen = json.load(handle)
with open(os.path.join(EV, "robustness", "outcomes.json"), encoding="utf-8") as handle:
    rob = json.load(handle)
with open(os.path.join(EV, "README.md"), encoding="utf-8") as handle:
    readme = handle.read()

counts = {}
for row in gen:
    counts[row["outcome"]] = counts.get(row["outcome"], 0) + 1
expected = {
    "A_PLAN_FOUND_AND_COMPLETED": "A — plan found and completed",
    "B_PLAN_FOUND_BUT_EXECUTION_FAILED": "B — committed, then execution failed",
    "C_NO_PHYSICALLY_FEASIBLE_PLAN": "C — no physically feasible TRIAD plan",
    "D_NO_TIMING_ADMISSIBLE_PLAN": "D — no timing-admissible TRIAD plan",
    "E_COMMIT_FRESHNESS_REJECTED": "E — commit freshness rejected",
}
check("held-out record count is 62", len(gen) == 62, str(len(gen)))
for key, label in expected.items():
    n = counts.get(key, 0)
    pattern = r"\|\s*%s\s*\|\s*%d\s*\|" % (re.escape(label), n)
    check("README reports %s = %d" % (key, n), re.search(pattern, readme) is not None)

committed = [r for r in gen if r["committed"]]
completed = [r for r in gen if r["completed"]]
check("committed 38 of 62", len(committed) == 38, str(len(committed)))
check("completed given commitment 36 of 38", len(completed) == 36, str(len(completed)))
check("every held-out run used ideal sensing",
      all(r.get("mode") == "IDEAL" and float(r.get("configured_delay_s") or 0) == 0.0 for r in gen))

check("perturbation record count is 66", len(rob) == 66, str(len(rob)))
r_completed = [r for r in rob if r["completed"]]
r_committed = [r for r in rob if r["committed"]]
r_execfail = [r for r in rob if r["committed"] and not r["completed"]]
r_rejected = [r for r in rob if not r["committed"]]
check("perturbations completed 40", len(r_completed) == 40, str(len(r_completed)))
check("perturbations execution failure after commitment 12", len(r_execfail) == 12, str(len(r_execfail)))
check("perturbations safely rejected 14", len(r_rejected) == 14, str(len(r_rejected)))

h002_all = [r for r in rob if r.get("anchor_id") == "H002"]
h002_completed = [r for r in h002_all if r["completed"]]
h002_failed = [r for r in h002_all if not r["completed"]]
h002_execfail = [r for r in r_execfail if r.get("anchor_id") == "H002"]
check("H002 family contains 11 perturbations", len(h002_all) == 11, str(len(h002_all)))
check("H002 family contains 3 completions", len(h002_completed) == 3, str(len(h002_completed)))
check("H002 family contains 8 failures", len(h002_failed) == 8, str(len(h002_failed)))
check("eight of the twelve execution failures belong to H002", len(h002_execfail) == 8, str(len(h002_execfail)))
check("every perturbation run used ideal sensing",
      all(r.get("mode") == "IDEAL" and float(r.get("configured_delay_s") or 0) == 0.0 for r in rob))

print("5. publication interpretation guards")
check("README makes the full 66-case robustness result primary",
      "The left-hand column is the primary result" in readme)
check("README labels the H002 exclusion secondary and post-hoc",
      "secondary, post-hoc diagnostic" in readme)
check("README does not claim feasible-set coverage",
      "does **not** call them feasible-set coverage" in readme)
check("README records 0.60 s uncompensated AMBIGUOUS classification",
      "classification is `AMBIGUOUS`" in readme and "0.0241 m" in readme and "0.0760 m/s" in readme)
check("README states near-ground normalization by sourceIndex",
      "sourceIndex" in readme and "full raw records and full set hashes are **not** identical" in readme)
check("README scopes the <2 ms statement to in-planning ControllerRun",
      "restricted to the in-planning" in readme and "No hard-real-time/WCET claim" in readme)

print()
if failures:
    print("EVIDENCE CHECK: FAIL  (%d of %d checks failed)" % (len(failures), checks))
    for name in failures:
        print("   - %s" % name)
    sys.exit(1)
print("EVIDENCE CHECK: PASS  (%d checks)" % checks)
