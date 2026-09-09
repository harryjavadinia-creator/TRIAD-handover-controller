#!/usr/bin/env python3
"""Deterministic held-out scenario set for frozen TRIAD.

Envelope derived from etc/HandoverInterceptionController.in.yaml and the four
development scenarios; see OPERATING_ENVELOPE.md. Development scenarios are
excluded by construction. Seed is fixed; the table is written and hashed before
any scenario is executed.
"""
import csv, random
SEED = 20260904
random.seed(SEED)

DEV = {(0.55,-0.56,0.15), (0.25,0.62,0.15), (0.92,0.0,0.55), (0.90,0.0,0.30), (0.55,0.0,0.55)}
RPY = (0.0, 1.5708, 0.0)          # orientation of every development scenario
S = 0.0565685                      # diagonal component used by diagonal_xz

# Velocity families, all at speeds the implementation has been exercised at or
# just inside. Directions are unit-ish vectors scaled by speed.
FAMILIES = {
    "minus_x":  (-1.0, 0.0, 0.0),
    "plus_y":   ( 0.0, 1.0, 0.0),
    "minus_y":  ( 0.0,-1.0, 0.0),
    "diag_xz":  (-0.7071, 0.0, 0.7071),
    "diag_xy":  (-0.7071, 0.7071, 0.0),
    "diag_xyz": (-0.5774, 0.5774, 0.5774),
}
SPEEDS = [0.05, 0.08, 0.11]

rows = []
def add(x, y, z, fam, speed, region, note=""):
    if (round(x,2), round(y,2), round(z,2)) in DEV: return
    d = FAMILIES[fam]
    vx, vy, vz = (round(speed*c, 6) for c in d)
    rows.append(dict(id=f"H{len(rows)+1:03d}", x=round(x,3), y=round(y,3), z=round(z,3),
                     vx=vx, vy=vy, vz=vz, speed=speed, family=fam,
                     rr=RPY[0], rp=RPY[1], ry=RPY[2], region=region, note=note))

# 1) Interior grid: mid workspace, moderate heights, all families at 0.08
for x in (0.50, 0.65):
    for y in (-0.30, 0.0, 0.30):
        for fam in ("minus_x", "plus_y", "minus_y", "diag_xz"):
            add(x, y, 0.35, fam, 0.08, "interior")

# 2) Height sweep at a central column
for z in (0.20, 0.35, 0.50):
    for fam in ("minus_x", "plus_y", "diag_xz"):
        add(0.60, -0.20, z, fam, 0.08, "height_sweep")

# 3) Speed sweep, interior
for sp in SPEEDS:
    for fam in ("minus_x", "minus_y", "diag_xz"):
        add(0.58, 0.25, 0.30, fam, sp, "speed_sweep")

# 4) Lateral extremes (inside the exercised |y| range, away from dev points)
for y in (-0.50, 0.50):
    for fam in ("minus_x", "plus_y", "diag_xy"):
        add(0.45, y, 0.25, fam, 0.08, "lateral")

# 5) Longitudinal near/far
for x in (0.35, 0.82):
    for fam in ("minus_x", "minus_y", "diag_xz"):
        add(x, 0.10, 0.40, fam, 0.08, "longitudinal")

# 6) Multi-axis / higher motion (stress, still inside exercised speed range)
for fam in ("diag_xyz", "diag_xy"):
    for sp in (0.08, 0.11):
        add(0.70, -0.25, 0.45, fam, sp, "multiaxis")

# 7) Boundary-ish: low height near ground, and high height
for z, tag in ((0.18, "low_height"), (0.55, "high_height")):
    for fam in ("minus_x", "plus_y"):
        add(0.55, 0.35, z, fam, 0.08, tag)

with open("held_out_scenarios.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    for r in rows: w.writerow(r)
print(f"generated {len(rows)} held-out scenarios (seed {SEED})")
from collections import Counter
print("by region:", dict(Counter(r["region"] for r in rows)))
print("by family:", dict(Counter(r["family"] for r in rows)))
