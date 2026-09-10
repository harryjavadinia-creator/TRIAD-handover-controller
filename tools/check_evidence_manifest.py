#!/usr/bin/env python3
"""Dependency-free integrity checks for the reduced release evidence package."""
import hashlib
import json
import os
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

print("2. frozen plan sets")
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

print("3. corrected latency evidence")
with open(os.path.join(EV, "latency", "sweep_rows.json"), encoding="utf-8") as handle:
    rows = json.load(handle)
check("latency sweep is non-empty", bool(rows))
check("latency sweep contains 0.30 s compensated condition",
      any(float(r.get("delay", -1)) == 0.30 and r.get("comp") is True for r in rows))
check("latency sweep contains 0.30 s uncompensated condition",
      any(float(r.get("delay", -1)) == 0.30 and r.get("comp") is False for r in rows))
check("latency sweep contains 0.60 s conditions",
      sum(float(r.get("delay", -1)) == 0.60 for r in rows) >= 2)

print("4. publication interpretation guards")
with open(os.path.join(EV, "README.md"), encoding="utf-8") as handle:
    readme = handle.read()
check("0.60 s ambiguous observation is documented",
      "`AMBIGUOUS`" in readme and "0.0241 m" in readme and "0.0760 m/s" in readme)
check("near-ground normalization is documented",
      "sourceIndex" in readme and "full raw records and full set hashes are **not** identical" in readme)
check("<2 ms statement is scoped",
      "restricted to this in-planning" in readme and "No hard-real-time/WCET claim" in readme)

print()
if failures:
    print("EVIDENCE CHECK: FAIL  (%d of %d checks failed)" % (len(failures), checks))
    for name in failures:
        print("   - %s" % name)
    sys.exit(1)
print("EVIDENCE CHECK: PASS  (%d checks)" % checks)
