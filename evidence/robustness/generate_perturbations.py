#!/usr/bin/env python3
"""Deterministic robustness perturbation table.

Anchors are selected by ROBUSTNESS_ANCHOR_RULE.md, using scenario geometry and
input values only; no run outcome is consulted. Perturbation magnitudes are
EXPERIMENTAL values (see the rule document), not system specifications.

PUBLISHED VARIANT. This file is derived from the campaign script that produced
`perturbations.csv`. The only change is that the two hard-coded archive paths
were replaced by command-line arguments so the script is machine-independent;
the anchor rule and every perturbation magnitude are unchanged. Running

    python3 generate_perturbations.py ../generalization/held_out_scenarios.csv out.csv

reproduces `perturbations.csv` byte for byte, which
`tools/check_evidence_manifest.py` verifies against the pre-registered hash.

The original campaign script has sha256 5fa16a1786af2694dabb02e9fa685587fae39d793c8d2a84dd6c904b6ee52c86.
"""
import csv, math, sys
SEED = 20260904
IN  = sys.argv[1] if len(sys.argv) > 1 else "../generalization/held_out_scenarios.csv"
OUT = sys.argv[2] if len(sys.argv) > 2 else "perturbations.csv"
rows=[dict(r) for r in csv.DictReader(open(IN))]
for r in rows:
    for k in ('x','y','z','vx','vy','vz','speed','rr','rp','ry'): r[k]=float(r[k])

def first(pred):
    for r in rows:
        if pred(r): return r
    return None

anchors=[]
interior=[r for r in rows if r['region']=='interior' and r['speed']==0.08]
seen=set()
for r in interior:
    if r['family'] not in seen:
        anchors.append(('interior', r)); seen.add(r['family'])
    if len(anchors)==2: break
anchors.append(('low_height', first(lambda r: r['region']=='low_height')))
anchors.append(('lateral_neg_y', first(lambda r: r['region']=='lateral' and r['y']<0)))
anchors.append(('multiaxis_xyz', first(lambda r: r['family']=='diag_xyz')))
anchors.append(('speed_high', first(lambda r: r['region']=='speed_sweep' and r['speed']==0.11)))
anchors=[(t,a) for t,a in anchors if a is not None]

def rot_z(vx,vy,deg):
    a=math.radians(deg); return (vx*math.cos(a)-vy*math.sin(a), vx*math.sin(a)+vy*math.cos(a))

out=[]
for tag,a in anchors:
    base=dict(anchor_tag=tag, anchor_id=a['id'], rr=a['rr'], rp=a['rp'], ry=a['ry'])
    P=[("nominal",     0,0,0, 1.0, 0),
       ("x_plus_20mm", +0.020,0,0, 1.0, 0),
       ("x_minus_20mm",-0.020,0,0, 1.0, 0),
       ("y_plus_20mm", 0,+0.020,0, 1.0, 0),
       ("y_minus_20mm",0,-0.020,0, 1.0, 0),
       ("z_plus_20mm", 0,0,+0.020, 1.0, 0),
       ("z_minus_20mm",0,0,-0.020, 1.0, 0),
       ("speed_plus",  0,0,0, (a['speed']+0.02)/a['speed'] if a['speed'] else 1.0, 0),
       ("speed_minus", 0,0,0, (a['speed']-0.02)/a['speed'] if a['speed'] else 1.0, 0),
       ("dir_plus10",  0,0,0, 1.0, +10),
       ("combo_x40_spd",+0.040,0,0, (a['speed']+0.02)/a['speed'] if a['speed'] else 1.0, 0)]
    for name,dx,dy,dz,sf,deg in P:
        vx,vy,vz=a['vx']*sf, a['vy']*sf, a['vz']*sf
        if deg: vx,vy=rot_z(vx,vy,deg)
        out.append(dict(**base, pert=name,
                        x=round(a['x']+dx,3), y=round(a['y']+dy,3), z=round(a['z']+dz,3),
                        vx=round(vx,6), vy=round(vy,6), vz=round(vz,6),
                        id=f"{a['id']}_{name}"))
with open(OUT,'w',newline='') as f:
    w=csv.DictWriter(f, fieldnames=['id','anchor_id','anchor_tag','pert','x','y','z','vx','vy','vz','rr','rp','ry'])
    w.writeheader()
    for r in out: w.writerow({k:r[k] for k in w.fieldnames})
print(f"anchors ({len(anchors)}):")
for t,a in anchors: print(f"  {t:14s} {a['id']} pos=({a['x']},{a['y']},{a['z']}) fam={a['family']} speed={a['speed']}")
print(f"total perturbed cases: {len(out)} ({len(P)} per anchor, including nominal)")
