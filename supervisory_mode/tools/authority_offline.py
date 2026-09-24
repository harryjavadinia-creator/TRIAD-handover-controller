#!/usr/bin/env python3
"""Offline falsification test for a control-aware grasp supervisor.

Question: among IK-feasible receiving grasps g=(sigma,phi) of the TRIAD handle,
does the velocity-level authority of the actual mc_rtc hard constraints
(KinematicsConstraint: 0.95*velocity limits + position damper [0.1,0.01,0.5])
ever fail to realize the twist required to follow a moving object during
acquisition, within the configured CALL envelope?  If never, the criterion is
vacuous and should not be implemented as a selector.
"""
import sys, json, math, xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import linprog, lsq_linear

import os
URDF = os.path.join(os.environ.get("MAIN_ROBOT_MODULE_PATH", "gen3_2f85_module"), "urdf", "gen3_2f85.urdf")  # the module built in the README
TOOL = "gen3_robotiq_85_base_link"
VEL_PCT, D_INTER, D_SECUR, D_OFF = 0.95, 0.10, 0.01, 0.50   # etc: constraints[kinematics]
B_T_M_T = np.array([0.0, 0.0, 0.0983262]); B_T_M_RPY = (-1.57079632679, 0.0, 0.0)
O_T_H_T = np.array([0.0, 0.0, -0.0869])
OBJ_RPY = (0.0, 1.5708, 0.0)
CAPTURE_DEPTH, STANDOFF = 0.006, 0.120
TRAVEL, STOP_D = 0.40, 0.85
READY = [-0.044821, 0.784738, 2.757292, -1.694771, -0.390082, 1.026258, 1.899765]
SCEN = {"near-ground": ([0.25, 0.62, 0.15], [0.0, -0.08, 0.0]),
        "longitudinal": ([0.92, 0.00, 0.55], [-0.08, 0.0, 0.0]),
        "lateral-low": ([0.55, -0.56, 0.15], [0.0, 0.08, 0.0]),
        "diagonal": ([0.90, 0.00, 0.30], [-0.0565685, 0.0, 0.0565685])}
L_CHAR = 0.20   # repo convention: characteristic length for angular rows

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx
def axang(a, t):
    a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K
def T(R, p):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p; return M

# ---- URDF chain world -> TOOL
root = ET.parse(URDF).getroot()
joints = {}
for j in root.findall("joint"):
    o = j.find("origin"); ax = j.find("axis"); lim = j.find("limit")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else "0 0 0".split())]
    r = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else "0 0 0".split())]
    joints[j.find("child").get("link")] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
        T=T(rpy(*r), xyz), axis=np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None,
        lo=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
        hi=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
        vel=float(lim.get("velocity")) if lim is not None and lim.get("velocity") else None)
chain, link = [], TOOL
while link in joints:
    chain.append(joints[link]); link = joints[link]["parent"]
chain.reverse()
act = [j for j in chain if j["type"] in ("revolute", "continuous")]
assert len(act) == 7, [j["name"] for j in act]
VMAX = np.array([j["vel"] for j in act]) * VEL_PCT
LO = np.array([j["lo"] if j["lo"] is not None else -np.inf for j in act])
HI = np.array([j["hi"] if j["hi"] is not None else np.inf for j in act])

def fk(q):
    M = np.eye(4); origins, axes = [], []; k = 0
    for j in chain:
        M = M @ j["T"]
        if j["type"] in ("revolute", "continuous"):
            axes.append(M[:3, :3] @ j["axis"]); origins.append(M[:3, 3].copy())
            M = M @ T(axang(j["axis"], q[k]), [0, 0, 0]); k += 1
    return M, origins, axes
def jac(q):
    M, O, A = fk(q); p = M[:3, 3]
    J = np.zeros((6, 7))
    for i in range(7):
        J[:3, i] = np.cross(A[i], p - O[i]); J[3:, i] = A[i]
    return M, J
def rot_err(Rd, R):
    E = Rd @ R.T; ang = math.acos(max(-1, min(1, (np.trace(E) - 1) / 2)))
    if ang < 1e-9: return np.zeros(3)
    w = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]]) / (2 * math.sin(ang))
    return w * ang

B_T_M = T(rpy(*B_T_M_RPY), B_T_M_T); M_T_B = np.linalg.inv(B_T_M)

def ik(Td, q0, iters=600):
    q = np.array(q0, float)
    for _ in range(iters):
        M, J = jac(q)
        e = np.concatenate([Td[:3, 3] - M[:3, 3], rot_err(Td[:3, :3], M[:3, :3])])
        if np.linalg.norm(e[:3]) < 1e-3 and np.linalg.norm(e[3:]) < 1e-2:
            return q, True
        lam = 0.05
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), np.concatenate([e[:3], e[3:]]))
        n = np.linalg.norm(dq)
        if n > 0.2: dq *= 0.2 / n
        q = q + dq
        q = np.minimum(np.maximum(q, LO + 0.01), HI - 0.01)
    return q, False

def qdot_bounds(q):
    lo, hi = -VMAX.copy(), VMAX.copy(); active = []
    for i in range(7):
        if not np.isfinite(LO[i]): continue
        rng = HI[i] - LO[i]; iD, sD = D_INTER * rng, D_SECUR * rng
        ld, ud = q[i] - LO[i], HI[i] - q[i]
        if ld < iD:   # damper(dist) <= alpha ; xi >= offset (history-free lower bound)
            lo[i] = max(lo[i], -D_OFF * (ld - sD) / (iD - sD)); active.append((i + 1, "low", round(ld, 3)))
        elif ud < iD:
            hi[i] = min(hi[i], D_OFF * (ud - sD) / (iD - sD)); active.append((i + 1, "upp", round(ud, 3)))
    return lo, hi, active

W = np.diag([1, 1, 1, L_CHAR, L_CHAR, L_CHAR])
def residual(J, lo, hi, y):
    r = lsq_linear(W @ J, W @ y, bounds=(lo, hi), method="bvls")
    return float(np.linalg.norm(W @ J @ r.x - W @ y))
def reserve(J, lo, hi, d6):
    # max lambda s.t. J qd = lambda d6, lo<=qd<=hi  (vars: qd(7), lambda)
    c = np.zeros(8); c[-1] = -1
    Aeq = np.hstack([J, -d6.reshape(6, 1)]); beq = np.zeros(6)
    bnds = [(lo[i], hi[i]) for i in range(7)] + [(0, None)]
    r = linprog(c, A_eq=Aeq, b_eq=beq, bounds=bnds, method="highs")
    return float(r.x[-1]) if r.status == 0 else 0.0

M0, _ = jac(np.array(READY)); W_T_M0 = M0 @ B_T_M
out = {}
NPHI = 64
for name, (p0, v) in SCEN.items():
    v = np.array(v); vhat = v / np.linalg.norm(v)
    p_rest = np.array(p0) + vhat * (TRAVEL + 0.5 * np.linalg.norm(v) * STOP_D)
    W_T_O = T(rpy(*OBJ_RPY), p_rest); pH = (W_T_O @ np.append(O_T_H_T, 1))[:3]; zH = W_T_O[:3, 2]
    outward = W_T_M0[:3, 3] - pH; outward -= zH * zH.dot(outward); outward /= np.linalg.norm(outward)
    rows = []
    for sgn in (+1, -1):
        for k in range(NPHI):
            phi = 2 * math.pi * k / NPHI
            zM = sgn * zH; yM = axang(zM, phi) @ outward; yM -= zM * zM.dot(yM); yM /= np.linalg.norm(yM)
            xM = np.cross(yM, zM); xM /= np.linalg.norm(xM); yM = np.cross(zM, xM)
            R = np.column_stack([xM, yM, zM])
            pC = pH + yM * CAPTURE_DEPTH; pS = pC + yM * STANDOFF
            TS = T(R, pS) @ M_T_B; TC = T(R, pC) @ M_T_B
            qS, okS = ik(TS, READY)
            if not okS: rows.append(dict(sign=sgn, phi=round(math.degrees(phi), 1), ik=False)); continue
            qC, okC = ik(TC, qS)
            if not okC: rows.append(dict(sign=sgn, phi=round(math.degrees(phi), 1), ik=False)); continue
            Mc, J = jac(qC); lo, hi, active = qdot_bounds(qC)
            def inside_security(qq):
                for i in range(7):
                    if not np.isfinite(LO[i]): continue
                    sD = D_SECUR * (HI[i] - LO[i])
                    if qq[i] - LO[i] <= sD or HI[i] - qq[i] <= sD: return True
                return False
            if inside_security(qS) or inside_security(qC):
                rows.append(dict(sign=sgn, phi=round(math.degrees(phi), 1), ik=False, why="security_distance")); continue
            # ground screen (crude): wrist/tool origins above 3 cm
            _, O, _ = fk(qC)
            ground_ok = min(o[2] for o in O[3:]) > 0.03 and Mc[2, 3] > 0.03
            # required base-link twist: follow object (no rotation) + insertion along -yM
            d_track = np.concatenate([vhat, np.zeros(3)])
            d_ins = np.concatenate([-yM, np.zeros(3)])
            sv = np.linalg.svd(J[:3, :], compute_uv=False)
            rec = dict(sign=sgn, phi=round(math.degrees(phi), 1), ik=True, ground=bool(ground_ok),
                       damper_active=active, sigma_min_lin=float(sv[-1]),
                       reserve_track=reserve(J, lo, hi, d_track), reserve_ins=reserve(J, lo, hi, d_ins),
                       reserve_track_nodamper=reserve(J, -VMAX, VMAX, d_track))
            for vobj in (0.08, 0.16, 0.24):
                for vins in (0.14, 0.38):
                    y = np.concatenate([vobj * vhat + vins * (-yM), np.zeros(3)])
                    rec[f"res_{vobj}_{vins}"] = residual(J, lo, hi, y)
            rows.append(rec)
    out[name] = rows
json.dump(out, open("supervisory_mode/evidence/authority_offline.json", "w"), indent=1)

def q(a, p): return float(np.percentile(a, p)) if len(a) else float("nan")
for name, rows in out.items():
    ok = [r for r in rows if r["ik"] and r["ground"]]
    rt = np.array([r["reserve_track"] for r in ok]); ri = np.array([r["reserve_ins"] for r in ok])
    sm = np.array([r["sigma_min_lin"] for r in ok]); nd = sum(1 for r in ok if r["damper_active"])
    print(f"--- {name}: IK+ground feasible {len(ok)}/{len(rows)}; damper active in {nd}")
    print(f"   reserve along object motion [m/s]  min {rt.min():.3f}  p10 {q(rt,10):.3f}  median {q(rt,50):.3f}  max {rt.max():.3f}")
    print(f"   reserve along insertion     [m/s]  min {ri.min():.3f}  p10 {q(ri,10):.3f}  median {q(ri,50):.3f}  max {ri.max():.3f}")
    for vobj in (0.08, 0.16, 0.24):
        for vins in (0.14, 0.38):
            bad = sum(1 for r in ok if r[f"res_{vobj}_{vins}"] > 1e-4)
            print(f"   demand v_obj={vobj:.2f} v_ins={vins:.2f}: residual>0 in {bad}/{len(ok)} feasible grasps")
    rn = np.array([r["reserve_track_nodamper"] for r in ok])
    print(f"   reserve along motion WITHOUT damper: min {rn.min():.3f} median {q(rn,50):.3f}  (damper lowers median by {q(rn,50)-q(rt,50):.3f})")
    for tag in [(1,0.0),(1,337.5),(-1,337.5),(1,45.0)]:
        m=[r for r in rows if r["sign"]==tag[0] and abs(r["phi"]-tag[1])<1e-6]
        if m:
            r=m[0]
            if r["ik"]:
                print(f"   grasp sign={tag[0]:+d} phi={tag[1]:5.1f}: track={r['reserve_track']:.3f} ins={r['reserve_ins']:.3f} sigma_min={r['sigma_min_lin']:.3f} res(0.08,0.38)={r['res_0.08_0.38']:.3f} damper={r['damper_active']}")
            else:
                print(f"   grasp sign={tag[0]:+d} phi={tag[1]:5.1f}: robot-infeasible ({r.get('why','ik')})")
    if len(ok) > 2:
        print(f"   corr(reserve_track, sigma_min) = {np.corrcoef(rt, sm)[0,1]:+.2f}")
