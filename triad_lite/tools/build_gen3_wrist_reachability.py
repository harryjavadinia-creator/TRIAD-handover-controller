#!/usr/bin/env python3
"""Build the Gen3 wrist-point reachability SDF used by the TRIAD-lite funnel.

Wrist point = gen3_joint_7 origin. It lies on the joint-7 axis and is rigidly
fixed in the tool frame (gen3_robotiq_85_base_link), so its world position is an
exact function of the requested tool pose. It depends on q1..q6 only, and q1 is a
continuous rotation about the world z axis through the base origin, so the
reachable set is a solid of revolution described by a 2-D region in
(rho = sqrt(x^2+y^2), z). Sampling q2..q6 inside URDF limits gives an occupancy
grid; its signed distance (positive inside, metres) is a NECESSARY-condition
reachability field (orientation feasibility of the wrist is ignored).

Outputs a C++ header with the grid, the tool-frame wrist offset and provenance.
usage: build_gen3_wrist_reachability.py URDF OUT_HEADER [--samples N] [--step M] [--seed S]
"""
import argparse, hashlib, math, xml.etree.ElementTree as ET
import numpy as np
from scipy import ndimage

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))

def T(R, p):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p; return M

def load_chain(urdf, tool="gen3_robotiq_85_base_link"):
    root = ET.parse(urdf).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin"); ax = j.find("axis"); lim = j.find("limit")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        r = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        joints[j.find("child").get("link")] = dict(
            name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), T=T(rpy(*r), xyz),
            axis=np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None,
            lo=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
            hi=float(lim.get("upper")) if lim is not None and lim.get("upper") else None)
    chain, link = [], tool
    while link in joints:
        chain.append(joints[link]); link = joints[link]["parent"]
    return chain[::-1]

def rot_axis_batch(axis, angles):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    s = np.sin(angles)[:, None, None]; c = np.cos(angles)[:, None, None]
    return np.eye(3)[None] + s * K[None] + (1 - c) * (K @ K)[None]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf"); ap.add_argument("out")
    ap.add_argument("--samples", type=int, default=4_000_000)
    ap.add_argument("--step", type=float, default=0.01)
    ap.add_argument("--seed", type=int, default=20260916)
    a = ap.parse_args()
    chain = load_chain(a.urdf)
    act = [j for j in chain if j["type"] in ("revolute", "continuous")]
    assert [j["name"] for j in act] == [f"gen3_joint_{i}" for i in range(1, 8)], [j["name"] for j in act]

    # Tool-frame position of the joint-7 origin: product of fixed transforms after joint 7.
    k7 = chain.index(act[6])
    M_after = np.eye(4)
    for j in chain[k7 + 1:]:
        assert j["type"] == "fixed"
        M_after = M_after @ j["T"]
    j7_in_tool = (np.linalg.inv(M_after) @ np.array([0, 0, 0, 1.0]))[:3]  # joint-7 frame origin in tool frame (rotation of q7 irrelevant: on axis)

    rng = np.random.default_rng(a.seed)
    n = a.samples
    q = np.zeros((n, 7))
    for i, j in enumerate(act):
        if i == 0: continue
        lo, hi = (j["lo"], j["hi"]) if j["lo"] is not None else (-math.pi, math.pi)
        q[:, i] = rng.uniform(lo, hi, n)
    # Batched FK to the joint-7 origin (q7 does not move it).
    M = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
    k = 0
    for j in chain[:k7 + 1]:
        M = M @ j["T"]
        if j["type"] in ("revolute", "continuous"):
            if j is act[6]:
                break
            R = rot_axis_batch(j["axis"], q[:, k])
            Mr = np.broadcast_to(np.eye(4), (n, 4, 4)).copy(); Mr[:, :3, :3] = R
            M = M @ Mr
            k += 1
    pw = M[:, :3, 3]
    rho = np.hypot(pw[:, 0], pw[:, 1]); z = pw[:, 2]

    step = a.step
    rho0, z0 = 0.0, math.floor((z.min() - 5 * step) / step) * step
    nr = int(math.ceil((rho.max() + 5 * step - rho0) / step)) + 1
    nz = int(math.ceil((z.max() + 5 * step - z0) / step)) + 1
    occ = np.zeros((nr, nz), dtype=bool)
    occ[np.clip(((rho - rho0) / step).round().astype(int), 0, nr - 1),
        np.clip(((z - z0) / step).round().astype(int), 0, nz - 1)] = True
    # Close sampling pin-holes (1 cell) so interior gaps are not reported as unreachable.
    occ = ndimage.binary_closing(occ, structure=np.ones((3, 3)), iterations=1) | occ
    inside = ndimage.distance_transform_edt(occ) * step
    outside = ndimage.distance_transform_edt(~occ) * step
    sdf = np.where(occ, inside, -outside).astype(np.float32)

    # Sampling-convergence statistic: occupied area with half the samples.
    half = np.zeros_like(occ)
    m = n // 2
    half[np.clip(((rho[:m] - rho0) / step).round().astype(int), 0, nr - 1),
         np.clip(((z[:m] - z0) / step).round().astype(int), 0, nz - 1)] = True
    half = ndimage.binary_closing(half, structure=np.ones((3, 3)), iterations=1) | half
    urdf_sha = hashlib.sha256(open(a.urdf, "rb").read()).hexdigest()
    vals = ",".join(f"{v:.4f}f" for v in sdf.T.reshape(-1))  # row-major in z, then rho
    with open(a.out, "w") as f:
        f.write(f"""#pragma once
// GENERATED by research/triad_lite/tools/build_gen3_wrist_reachability.py - do not edit.
// URDF sha256 {urdf_sha}
// samples {n} seed {a.seed} step {step} m; occupied cells {int(occ.sum())} (half samples: {int(half.sum())})
// Wrist point = gen3_joint_7 origin, in tool frame gen3_robotiq_85_base_link.
// SDF (m) over (rho, z): positive inside the sampled reachable region, negative outside.

#include <array>

namespace call_handover
{{
namespace gen3_wrist_reachability
{{
constexpr double kRho0 = {rho0!r};
constexpr double kZ0 = {z0!r};
constexpr double kStep = {step!r};
constexpr int kNRho = {nr};
constexpr int kNZ = {nz};
constexpr double kWristInToolX = {j7_in_tool[0]!r};
constexpr double kWristInToolY = {j7_in_tool[1]!r};
constexpr double kWristInToolZ = {j7_in_tool[2]!r};
constexpr int kSamples = {n};
constexpr int kSeed = {a.seed};
constexpr const char * kUrdfSha256 = "{urdf_sha}";
// index = iz * kNRho + irho
static const float kSdf[{nr * nz}] = {{{vals}}};
}} // namespace gen3_wrist_reachability
}} // namespace call_handover
""")
    print(f"j7_in_tool={j7_in_tool} grid={nr}x{nz} occupied={int(occ.sum())} half={int(half.sum())} "
          f"rho=[{rho.min():.3f},{rho.max():.3f}] z=[{z.min():.3f},{z.max():.3f}] sdfmax={sdf.max():.3f}")

if __name__ == "__main__":
    main()
