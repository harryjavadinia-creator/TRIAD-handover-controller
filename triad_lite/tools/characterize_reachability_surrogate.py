#!/usr/bin/env python3
"""False-negative characterization of the wrist (rho, z) reachability SDF.

Ground truth by construction: tool poses produced by forward kinematics of joint
configurations drawn uniformly inside URDF limits (independent seed) ARE reachable.
The surrogate must classify them inside (sdf >= -tolerance). Reports the
false-negative rate against the pruning tolerance, and the fraction of an outer
shell of random unreachable points it correctly rejects.
usage: characterize_reachability_surrogate.py URDF HEADER [--samples N]
"""
import argparse, math, re, sys
import numpy as np
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from build_gen3_wrist_reachability import load_chain, rot_axis_batch

def load_header(path):
    s = open(path).read()
    g = lambda k: float(re.search(rf"{k} = ([-0-9.e+]+)", s).group(1))
    nr, nz = int(g("kNRho")), int(g("kNZ"))
    vals = np.array([float(v.rstrip("f")) for v in re.search(r"kSdf\[\d+\] = \{(.*)\};", s, re.S).group(1).split(",")])
    return dict(rho0=g("kRho0"), z0=g("kZ0"), step=g("kStep"), nr=nr, nz=nz,
                off=np.array([g("kWristInToolX"), g("kWristInToolY"), g("kWristInToolZ")]), sdf=vals.reshape(nz, nr))

def query(m, rho, z):
    # bilinear, clamped; outside the grid -> distance to grid box as negative bound
    fr = (rho - m["rho0"]) / m["step"]; fz = (z - m["z0"]) / m["step"]
    i0 = np.clip(np.floor(fr).astype(int), 0, m["nr"] - 2); k0 = np.clip(np.floor(fz).astype(int), 0, m["nz"] - 2)
    a = np.clip(fr - i0, 0, 1); b = np.clip(fz - k0, 0, 1)
    S = m["sdf"]
    v = (1 - a) * (1 - b) * S[k0, i0] + a * (1 - b) * S[k0, i0 + 1] + (1 - a) * b * S[k0 + 1, i0] + a * b * S[k0 + 1, i0 + 1]
    outr = np.maximum(0, np.maximum(m["rho0"] - rho, rho - (m["rho0"] + (m["nr"] - 1) * m["step"])))
    outz = np.maximum(0, np.maximum(m["z0"] - z, z - (m["z0"] + (m["nz"] - 1) * m["step"])))
    return np.where((outr > 0) | (outz > 0), -np.hypot(outr, outz) + np.minimum(v, 0), v)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("urdf"); ap.add_argument("header")
    ap.add_argument("--samples", type=int, default=400_000); a = ap.parse_args()
    m = load_header(a.header); chain = load_chain(a.urdf)
    act = [j for j in chain if j["type"] in ("revolute", "continuous")]
    rng = np.random.default_rng(7)
    n = a.samples; q = np.zeros((n, 7))
    for i, j in enumerate(act):
        lo, hi = (j["lo"], j["hi"]) if j["lo"] is not None else (-math.pi, math.pi)
        q[:, i] = rng.uniform(lo, hi, n)
    M = np.broadcast_to(np.eye(4), (n, 4, 4)).copy(); k = 0
    for j in chain:
        M = M @ j["T"]
        if j["type"] in ("revolute", "continuous"):
            Mr = np.broadcast_to(np.eye(4), (n, 4, 4)).copy(); Mr[:, :3, :3] = rot_axis_batch(j["axis"], q[:, k]); M = M @ Mr; k += 1
    pw = (M[:, :3, :3] @ m["off"]) + M[:, :3, 3]      # wrist point from the TOOL pose (as the controller will do)
    v = query(m, np.hypot(pw[:, 0], pw[:, 1]), pw[:, 2])
    print(f"reachable samples={n} sdf min={v.min():.4f} p0.1={np.percentile(v,0.1):.4f} median={np.median(v):.4f}")
    for tol in (0.0, 0.005, 0.01, 0.02):
        print(f"  false negatives at prune threshold sdf < -{tol:.3f}: {(v < -tol).mean()*100:.4f}%")
    # unreachable probe: points beyond max reach
    u = rng.uniform(0, 1, (n, 3)); dirs = rng.normal(size=(n, 3)); dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    far = dirs * (0.95 + 0.5 * u[:, :1]) + np.array([0, 0, 0.2])
    vf = query(m, np.hypot(far[:, 0], far[:, 1]), far[:, 2])
    print(f"far probe (|p - base| in [0.95,1.45] m around z=0.2): rejected at -0.01: {(vf < -0.01).mean()*100:.2f}%")

if __name__ == "__main__":
    main()
