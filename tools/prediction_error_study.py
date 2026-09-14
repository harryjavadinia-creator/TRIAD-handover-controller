#!/usr/bin/env python3
"""Final TRIAD Phase 2A: constant-twist prediction error versus horizon.

Replicates, tick by tick, the controller's object perception and motion
estimator (src/HandoverInterceptionController.cpp):

  recordObjectPerceptionTruthSample / selectDelayedObjectMeasurement
  applyObjectPerceptionEstimate / updateObjectMotionEstimate
  ReceiverV2 currentObjectPredictionV2 (rest-threshold zeroing)
  propagatePoseConstantTwist

and the independent scripted giver (src/IndependentGiverModel.h), then measures
e_p(h) = |p_O(t_k+h) - p^_O(t_k+h | I_k)| and e_R(h) over decision times t_k.

  validate LOG [LOG ...]     replica vs in-situ V2 logs: estimate speed at every
                             [V2Motion] line and the prediction error of every
                             [V2SnapshotAudit] (a logged e_p(h) sample)
  grid OUT.json              envelope over motion / delay / noise / sampling
                             conditions (offline; conditions the simulator
                             cannot produce are labelled OFFLINE_ONLY)
  report IN.json             markdown tables
"""

import json
import lzma
import math
import re
import sys

import numpy as np

DT = 0.001
TAU_FILTER = 0.10          # movingObject.velocityFilterTimeConstant
V_BOUND = 0.30             # movingObject.maximumLinearSpeed
W_BOUND = 1.50             # movingObject.maximumAngularSpeed
REST_V = 0.004             # presentationMaximumLinearSpeed
REST_W = 0.08              # presentationMaximumAngularSpeed


# ---------------------------------------------------------------- rotations
def rotvec_to_mat(r):
    th = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(r) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def mat_to_rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-12:
        return np.zeros(3)
    if abs(math.pi - th) < 1e-6:
        w, v = np.linalg.eigh((R + R.T) / 2)
        axis = v[:, np.argmax(w)]
        return axis * th
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th))
    return axis * th


def quat_to_mat(q):  # (w, x, y, z)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def angle_between(Ra, Rb):
    c = max(-1.0, min(1.0, (np.trace(Ra.T @ Rb) - 1) / 2))
    return math.acos(c)


# ---------------------------------------------------------------- giver truth
def quintic_step(x):
    x = min(1.0, max(0.0, x))
    return x * x * x * (10 + x * (-15 + 6 * x))


def quintic_stop_integral(x):
    x = min(1.0, max(0.0, x))
    x2 = x * x
    x4 = x2 * x2
    return x - 2.5 * x4 + 3.0 * x4 * x - x4 * x2


class GiverTruth:
    """IndependentGiverModel.h, verbatim semantics."""

    def __init__(self, p0, R0, v, w, travel=0.40, stop=0.85):
        self.p0, self.R0 = np.asarray(p0, float), np.asarray(R0, float)
        self.v, self.w = np.asarray(v, float), np.asarray(w, float)
        speed = np.linalg.norm(self.v)
        stop = max(0.0, stop)
        if speed <= 1e-12:
            self.cruise, self.stop = 0.0, 0.0
        else:
            if 0.5 * speed * stop > travel:
                stop = 2 * travel / speed
            self.cruise = max(0.0, (travel - 0.5 * speed * stop) / speed)
            self.stop = stop
        self.speed = speed
        self.rest_time = self.cruise + self.stop

    def eff(self, t):
        t = max(0.0, t)
        if self.speed <= 1e-12:
            return 0.0, 0.0
        if t <= self.cruise:
            return t, 1.0
        if self.stop > 1e-12 and t < self.cruise + self.stop:
            u = (t - self.cruise) / self.stop
            return self.cruise + self.stop * quintic_stop_integral(u), 1 - quintic_step(u)
        return self.cruise + 0.5 * self.stop, 0.0

    def pose(self, t):
        e, _ = self.eff(t)
        return self.p0 + e * self.v, rotvec_to_mat(self.w * e) @ self.R0 if np.linalg.norm(self.w) > 1e-9 and e > 0 else self.R0

    def max_accel(self):
        # peak |d/dt scale| * speed = 1.875 * speed / stop for the quintic stop
        return 1.875 * self.speed / self.stop if self.stop > 1e-12 else float("inf")


class ArcTruth:
    """OFFLINE_ONLY: constant speed along a horizontal heading that starts turning
    at yaw rate `turn` after `turn_start` seconds (direction change), no stop."""

    def __init__(self, p0, v, turn, turn_start, duration=6.0):
        self.p0 = np.asarray(p0, float)
        self.v = np.asarray(v, float)
        self.turn, self.turn_start = turn, turn_start
        self.rest_time = duration
        self.R0 = np.eye(3)

    def pose(self, t):
        t = max(0.0, t)
        t1 = min(t, self.turn_start)
        p = self.p0 + self.v * t1
        if t > self.turn_start and abs(self.turn) > 1e-12:
            s = t - self.turn_start
            vx, vy, vz = self.v
            a = self.turn
            # rotate the horizontal velocity by angle a*tau, integrate analytically
            dx = (vx * math.sin(a * s) + vy * (math.cos(a * s) - 1)) / a
            dy = (vy * math.sin(a * s) - vx * (math.cos(a * s) - 1)) / a
            p = p + np.array([dx, dy, vz * s])
        elif t > self.turn_start:
            p = p + self.v * (t - self.turn_start)
        return p, self.R0

    def max_accel(self):
        return abs(self.turn) * np.linalg.norm(self.v[:2])


class SmoothStartGiver(GiverTruth):
    """OFFLINE_ONLY: the scripted giver with a quintic start of duration `ramp`
    (the simulator's giver starts at full speed)."""

    def __init__(self, *a, ramp=0.5, **k):
        super().__init__(*a, **k)
        self.ramp = ramp

    def eff(self, t):
        t = max(0.0, t)
        if t < self.ramp:
            u = t / self.ramp
            # integral of quinticStep = ramp * (u^4*(2.5 - 3u + u^2)) ... computed numerically-free:
            integ = u ** 4 * (2.5 + u * (-3.0 + u))
            return self.ramp * integ, quintic_step(u)
        e, s = GiverTruth.eff(self, t - self.ramp)
        return 0.5 * self.ramp + e, s

    def max_accel(self):
        return max(1.875 * self.speed / self.ramp, GiverTruth.max_accel(self))


# ---------------------------------------------------------------- estimator replica
def simulate(truth, t_end, delay=0.0, compensate=True, noise_p=0.0, noise_r=0.0,
             frame_period=None, tau=TAU_FILTER, seed=0, pre_roll=0.0):
    """Returns per-tick arrays (controller elapsed time since giver start):
    t, W_T_O_ position, rotation, v_hat, w_hat (estimator, before rest zeroing).
    frame_period: None = a fresh measurement every control tick (simulator);
    otherwise sample-and-hold at that period (OFFLINE_ONLY)."""
    rng = np.random.default_rng(seed)
    n = int(round((t_end + pre_roll) / DT)) + 1
    t = np.empty(n)
    P = np.empty((n, 3))
    Rs = np.empty((n, 3, 3))
    V = np.empty((n, 3))
    W = np.empty((n, 3))
    buf_t, buf_p, buf_R = [], [], []
    held = None
    held_until = -1e9
    v_hat = np.zeros(3)
    w_hat = np.zeros(3)
    have_prev = False
    prev_p = prev_R = None
    prev_time = None
    for k in range(n):
        tk = -pre_roll + k * DT
        # truth sample (perception source)
        tp, tR = truth.pose(tk) if tk >= 0 else truth.pose(0.0)
        if noise_p > 0 or noise_r > 0 or frame_period is not None:
            if frame_period is None or tk >= held_until - 1e-12:
                mp = tp + (rng.normal(0, noise_p, 3) if noise_p > 0 else 0)
                mR = rotvec_to_mat(rng.normal(0, noise_r, 3)) @ tR if noise_r > 0 else tR
                held = (mp, mR)
                if frame_period is not None:
                    held_until = tk + frame_period
            mp, mR = held
        else:
            mp, mR = tp, tR
        buf_t.append(tk)
        buf_p.append(mp)
        buf_R.append(mR)
        # delayed measurement (exact tick-aligned delays are used)
        if delay > 0:
            target = tk - delay
            j = int(round((target - buf_t[0]) / DT))
            if j <= 0:
                meas_t, meas_p, meas_R = buf_t[0], buf_p[0], buf_R[0]
            else:
                meas_t, meas_p, meas_R = target, buf_p[j], buf_R[j]
            age = tk - meas_t
        else:
            meas_t, meas_p, meas_R = tk, mp, mR
            age = 0.0
        # updateObjectMotionEstimate (estimation active from giver start: t >= 0)
        if tk >= -1e-12:
            if not have_prev:
                have_prev = True
                prev_p, prev_R, prev_time = meas_p, meas_R, meas_t
            else:
                sdt = meas_t - prev_time
                if sdt > 1e-9:
                    raw_v = (meas_p - prev_p) / sdt
                    raw_w = mat_to_rotvec(meas_R @ prev_R.T) / sdt
                    if np.linalg.norm(raw_v) <= V_BOUND and np.linalg.norm(raw_w) <= W_BOUND:
                        a = min(1.0, max(0.0, sdt / (tau + sdt)))
                        v_hat = (1 - a) * v_hat + a * raw_v
                        w_hat = (1 - a) * w_hat + a * raw_w
                    prev_p, prev_R, prev_time = meas_p, meas_R, meas_t
        # applyObjectPerceptionEstimate
        if delay > 0 and compensate:
            Pk = meas_p + age * v_hat
            wn = np.linalg.norm(w_hat)
            Rk = rotvec_to_mat(w_hat * age) @ meas_R if wn > 1e-9 and age > 0 else meas_R
        else:
            Pk, Rk = meas_p, meas_R
        t[k], P[k], Rs[k], V[k], W[k] = tk, Pk, Rk, v_hat, w_hat
        if len(buf_t) > int((delay + 0.01) / DT) + 5:
            buf_t.pop(0)
            buf_p.pop(0)
            buf_R.pop(0)
    return t, P, Rs, V, W


def v2_record_velocity(v, w):
    if np.linalg.norm(v) <= REST_V and np.linalg.norm(w) <= REST_W:
        return np.zeros(3), np.zeros(3)
    return v, w


def errors_over(truth, sim, decision_step=0.01, horizons=None, t_from=0.0, t_to=None):
    t, P, Rs, V, W = sim
    if horizons is None:
        horizons = np.round(np.arange(0.0, 8.0001, 0.05), 3)
    t_to = t[-1] if t_to is None else t_to
    idx = [k for k in range(len(t)) if t[k] >= t_from - 1e-9 and t[k] <= t_to + 1e-9
           and abs((t[k] / decision_step) - round(t[k] / decision_step)) < 1e-6]
    ep = np.empty((len(idx), len(horizons)))
    er = np.empty_like(ep)
    tk = np.empty(len(idx))
    for i, k in enumerate(idx):
        v, w = v2_record_velocity(V[k], W[k])
        wn = np.linalg.norm(w)
        for j, h in enumerate(horizons):
            p_true, R_true = truth.pose(t[k] + h)
            p_hat = P[k] + h * v
            R_hat = rotvec_to_mat(w * h) @ Rs[k] if wn > 1e-9 and h > 1e-12 else Rs[k]
            ep[i, j] = np.linalg.norm(p_true - p_hat)
            er[i, j] = angle_between(R_true, R_hat)
        tk[i] = t[k]
    return tk, horizons, ep, er


# ---------------------------------------------------------------- validate
KV = re.compile(r"(\w+)=(\[[^\]]*\]|\S+)")


def kv(line):
    i = line.find("] [")
    return dict(KV.findall(line[i:] if i >= 0 else line))


def vec(s):
    return np.array([float(x) for x in s.strip("[]").split(",")])


def read(path):
    op = lzma.open if path.endswith(".xz") else open
    with op(path, "rt", errors="replace") as f:
        return f.read().splitlines()


def validate(logs):
    rows = []
    for path in logs:
        lines = read(path)
        script = next(kv(l) for l in lines if "[GiverTruthScript]" in l)
        lat = next(kv(l) for l in lines if "[PerceptionLatency]" in l)
        start = float(script["startTime"])
        truth = GiverTruth(vec(script["p0"]), quat_to_mat(vec(script["q0"])), vec(script["v"]), vec(script["w"]),
                           float(script["travelDistance"]), float(script["stopDuration"]))
        delay = float(lat["configuredDelay"].rstrip("s")) if lat.get("mode", "IDEAL") != "IDEAL" else 0.0
        comp = lat.get("compensate") == "true"
        motion = [kv(l) for l in lines if "[V2Motion]" in l]
        audits = [kv(l) for l in lines if "[V2SnapshotAudit]" in l]
        t_end = max(float(m["t"]) for m in motion) - start + 0.01
        sim = simulate(truth, t_end, delay=delay, compensate=comp)
        tt, P, Rs, V, W = sim

        def at(tabs):
            return int(round((tabs - start) / DT))

        dv = []
        for m in motion:
            k = at(float(m["t"]))
            if 0 <= k < len(tt):
                dv.append(abs(np.linalg.norm(V[k]) - float(m["objectEstimateSpeed"])))
        de, ne = [], []
        for a in audits:
            now = float(a["t"])
            sub = now - float(a["snapshotAge"].rstrip("s"))
            ks, kn = at(sub), at(now)
            if not (0 <= ks < len(tt) and 0 <= kn < len(tt)):
                continue
            v, w = v2_record_velocity(V[ks], W[ks])
            pred = P[ks] + (tt[kn] - tt[ks]) * v
            rep = np.linalg.norm(P[kn] - pred)
            de.append(abs(rep - float(a["snapshotPredictionError"].rstrip("m"))))
            ne.append(float(a["snapshotPredictionError"].rstrip("m")))
        rows.append(dict(log=path, delay=delay, compensate=comp, speed=float(np.linalg.norm(truth.v)),
                         stop=truth.stop, motionLines=len(dv), maxSpeedDiff=max(dv) if dv else None,
                         audits=len(de), maxAuditErrDiff=max(de) if de else None,
                         maxLoggedAuditErr=max(ne) if ne else None))
    return rows


# ---------------------------------------------------------------- grid
EPS_P = (0.005, 0.010, 0.015, 0.030)
EPS_R = (0.05, 0.12)


def valid_horizon(h, emax, eps):
    ok = emax <= eps + 1e-12
    if not ok[0]:
        return 0.0
    bad = np.where(~ok)[0]
    return float(h[-1]) if len(bad) == 0 else float(h[bad[0] - 1])


VALID_AFTER = 0.70   # movingObject.minimumObservationTime (and >= 350 samples): V2 record.valid


def summarize(name, label, truth, sim, kind):
    # Decision times only where the controller's prediction record is valid.
    tk, h, ep, er = errors_over(truth, sim, t_from=VALID_AFTER)
    moving = np.array([np.linalg.norm(truth.pose(x + 1e-3)[0] - truth.pose(x)[0]) / 1e-3 > REST_V for x in tk])
    out = dict(name=name, label=label, kind=kind, horizons=h.tolist())
    # estimator velocity error while the truth moves (record velocity after rest zeroing)
    t_all, _, _, V, W = sim
    verr = []
    for k in range(len(t_all)):
        if t_all[k] < VALID_AFTER:
            continue
        vt = (truth.pose(t_all[k] + 1e-4)[0] - truth.pose(t_all[k] - 1e-4)[0]) / 2e-4
        v, _ = v2_record_velocity(V[k], W[k])
        if np.linalg.norm(vt) > REST_V:
            verr.append(np.linalg.norm(v - vt))
    if verr:
        out["velocityError"] = dict(max=float(np.max(verr)), p95=float(np.percentile(verr, 95)), median=float(np.median(verr)))
    # horizons that stay inside the constant-velocity segment (time until the truth's
    # acceleration begins >= h): isolates estimator/latency error from model error
    onset = getattr(truth, "cruise", None)
    if onset is not None and isinstance(truth, SmoothStartGiver):
        onset = truth.cruise + truth.ramp
    if isinstance(truth, ArcTruth):
        onset = truth.turn_start
    if onset is not None:
        cv = np.zeros_like(ep, dtype=bool)
        for i, x in enumerate(tk):
            cv[i] = (x >= (truth.ramp if isinstance(truth, SmoothStartGiver) else 0.0)) & (x + h <= onset + 1e-9)
        E = np.where(cv, ep, np.nan)
        with np.errstate(all="ignore"):
            out["constantVelocityHorizons"] = dict(ep_max=np.nan_to_num(np.nanmax(E, 0), nan=-1).tolist())
        # worst error as a function of time-to-onset: e_p at h for decision times with onset - t_k in bins
        rel = onset - tk
        out["byTimeToOnset"] = {}
        for hh in (0.5, 1.0, 1.8, 3.0):
            j = list(h).index(hh)
            bins = {}
            for lo in np.arange(-1.0, 5.0, 0.5):
                m = (rel >= lo) & (rel < lo + 0.5)
                if m.any():
                    bins[f"{lo:+.1f}"] = float(ep[m, j].max())
            out["byTimeToOnset"][str(hh)] = bins
    for part, mask in (("all", np.ones_like(moving, bool)), ("moving", moving), ("rest", ~moving)):
        if mask.sum() == 0:
            continue
        E, ER = ep[mask], er[mask]
        out[part] = dict(
            n=int(mask.sum()),
            ep_max=E.max(0).tolist(), ep_p95=np.percentile(E, 95, axis=0).tolist(), ep_p50=np.median(E, 0).tolist(),
            er_max=ER.max(0).tolist(),
            hvalid_p={f"{e:.3f}": valid_horizon(h, E.max(0), e) for e in EPS_P},
            hvalid_p95={f"{e:.3f}": valid_horizon(h, np.percentile(E, 95, axis=0), e) for e in EPS_P},
            hvalid_r={f"{e:.2f}": valid_horizon(h, ER.max(0), e) for e in EPS_R})
    try:
        out["amax"] = float(truth.max_accel())
    except Exception:
        pass
    return out


def grid(out_path):
    p0 = np.array([0.92, 0.0, 0.55])
    R0 = quat_to_mat([0.70710548251123628, 0, 0.70710807985947366, 0])
    d = np.array([-1.0, 0.0, 0.0])
    res = []

    def giver(speed, stop=0.85, w=(0, 0, 0), travel=0.40):
        return GiverTruth(p0, R0, d * speed, np.array(w, float), travel, stop)

    # simulator-producible conditions
    for s in (0.04, 0.08, 0.12, 0.16, 0.24):
        g = giver(s)
        res.append(summarize(f"speed{s:.2f}_stop0.85", f"speed {s:.2f} m/s, stop 0.85 s", g, simulate(g, g.rest_time + 1.5), "SIMULATOR"))
    for st in (0.425, 1.7, 3.0):
        g = giver(0.08, st)
        res.append(summarize(f"speed0.08_stop{st}", f"speed 0.08 m/s, stop {st} s", g, simulate(g, g.rest_time + 1.5), "SIMULATOR"))
    g = giver(0.08, 0.85, (0, 0, 0.30))
    res.append(summarize("speed0.08_w0.30", "speed 0.08 m/s, yaw 0.30 rad/s, stop 0.85 s", g, simulate(g, g.rest_time + 1.5), "SIMULATOR"))
    for dl, comp in ((0.10, True), (0.22, True), (0.22, False)):
        g = giver(0.08)
        res.append(summarize(f"delay{dl}_{'comp' if comp else 'uncomp'}", f"speed 0.08, delay {dl} s, {'compensated' if comp else 'uncompensated'}",
                             g, simulate(g, g.rest_time + 1.5, delay=dl, compensate=comp), "SIMULATOR"))
    # offline-only conditions
    g = giver(0.08)
    for npos, nrot in ((0.0002, 0.0), (0.001, 0.0), (0.002, 0.01)):
        res.append(summarize(f"noise{npos}_{nrot}_tick", f"speed 0.08, per-tick noise sigma_p={npos*1000:.1f} mm sigma_R={nrot} rad (1 kHz)",
                             g, simulate(g, g.rest_time + 1.5, noise_p=npos, noise_r=nrot, seed=1), "OFFLINE_ONLY"))
    for fp in (1 / 30.0, 1 / 100.0):
        res.append(summarize(f"frames{int(round(1/fp))}Hz", f"speed 0.08, noise-free sample-and-hold at {int(round(1/fp))} Hz",
                             g, simulate(g, g.rest_time + 1.5, frame_period=fp), "OFFLINE_ONLY"))
    sg = SmoothStartGiver(p0, R0, d * 0.08, np.zeros(3), 0.40, 0.85, ramp=0.5)
    sg.rest_time = sg.cruise + sg.stop + sg.ramp
    res.append(summarize("smoothstart0.5", "speed 0.08, quintic start 0.5 s, stop 0.85 s", sg, simulate(sg, sg.cruise + sg.stop + sg.ramp + 1.5), "OFFLINE_ONLY"))
    for turn in (0.25, 0.5):
        at = ArcTruth(p0, d * 0.08, turn, 1.0, duration=6.0)
        res.append(summarize(f"turn{turn}", f"speed 0.08, heading turn {turn} rad/s after 1 s (no stop)", at, simulate(at, 6.0), "OFFLINE_ONLY"))
    json.dump(res, open(out_path, "w"))
    return res


def report(path):
    res = json.load(open(path))
    print("### Worst case over valid decision times while the object moves\n")
    print("| condition | source | peak accel (m/s²) | decision times | estimate speed error max / p95 (mm/s) | h_valid: max e_p ≤ 5 / 10 / 15 / 30 mm (s) | h_valid: p95 e_p ≤ 15 mm (s) | max e_p at h = 0 / 0.5 / 1.0 / 1.8 / 3.0 s (mm) | h_valid e_R ≤ 0.05 / 0.12 rad (s) |")
    print("|---|---|---:|---:|---|---|---:|---|---|")
    for r in res:
        m = r.get("moving")
        if not m:
            continue
        h = r["horizons"]
        pick = lambda arr, x: arr[h.index(x)] * 1000
        hv = m["hvalid_p"]
        ve = r.get("velocityError", {})
        print(f"| {r['label']} | {r['kind']} | {r.get('amax', float('nan')):.3f} | {m['n']} | "
              f"{ve.get('max', float('nan'))*1000:.1f} / {ve.get('p95', float('nan'))*1000:.1f} | "
              f"{hv['0.005']:.2f} / {hv['0.010']:.2f} / {hv['0.015']:.2f} / {hv['0.030']:.2f} | {m['hvalid_p95']['0.015']:.2f} | "
              f"{pick(m['ep_max'],0.0):.1f} / {pick(m['ep_max'],0.5):.1f} / {pick(m['ep_max'],1.0):.1f} / {pick(m['ep_max'],1.8):.1f} / {pick(m['ep_max'],3.0):.1f} | "
              f"{m['hvalid_r']['0.05']:.2f} / {m['hvalid_r']['0.12']:.2f} |")
    print("\n### Error when the whole horizon stays inside the constant-velocity segment (estimator + latency error only)\n")
    print("| condition | max e_p at h = 0 / 0.5 / 1.0 / 1.8 / 3.0 s (mm; '—' = no such decision time) |")
    print("|---|---|")
    for r in res:
        c = r.get("constantVelocityHorizons")
        if not c:
            continue
        h = r["horizons"]
        cells = []
        for x in (0.0, 0.5, 1.0, 1.8, 3.0):
            v = c["ep_max"][h.index(x)]
            cells.append("—" if v < 0 else f"{v*1000:.2f}")
        print(f"| {r['label']} | {' / '.join(cells)} |")
    print("\n### Worst e_p (mm) by time from the decision to the onset of the unmodelled acceleration (bins of 0.5 s; negative = decision after onset)\n")
    for hh in ("1.0", "1.8"):
        print(f"\nh = {hh} s\n")
        keys = [f"{x:+.1f}" for x in np.arange(-1.0, 5.0, 0.5)]
        print("| condition | " + " | ".join(keys) + " |")
        print("|---|" + "---:|" * len(keys))
        for r in res:
            b = r.get("byTimeToOnset", {}).get(hh)
            if not b:
                continue
            print(f"| {r['label']} | " + " | ".join(f"{b[k]*1000:.0f}" if k in b else "" for k in keys) + " |")


def adoptions(logs):
    """For every provisional adoption and every terminal commitment in V2 logs: the
    horizon h = tau - t used by the decision and the realised prediction error
    |p_O(tau) - predictedPresentation| against the scripted giver truth."""
    rows = []
    for path in logs:
        lines = read(path)
        script = next(kv(l) for l in lines if "[GiverTruthScript]" in l)
        start = float(script["startTime"])
        truth = GiverTruth(vec(script["p0"]), quat_to_mat(vec(script["q0"])), vec(script["v"]), vec(script["w"]),
                           float(script["travelDistance"]), float(script["stopDuration"]))
        done = any("[Completed] full plan-once" in l for l in lines)
        for l in lines:
            if "[V2ProvisionalAdopt]" not in l:
                continue
            d = kv(l)
            t, tau = float(d["t"]), float(d["tau"])
            pred = vec(d["predictedPresentation"])
            p_tau, _ = truth.pose(tau - start)
            moving = np.linalg.norm(truth.pose(t - start + 1e-3)[0] - truth.pose(t - start)[0]) / 1e-3 > REST_V
            rows.append(dict(log="/".join(path.split("/")[-3:-1]), planId=int(d["planId"]), kind=d["kind"], t=t, tau=tau,
                             horizon=tau - t, lead=float(d["eventLead"].rstrip("s")), objectMovingAtDecision=bool(moving),
                             timeToGiverRest=start + truth.rest_time - t,
                             realisedError=float(np.linalg.norm(p_tau - pred)), completed=done))
    return rows


def main(argv):
    if len(argv) >= 3 and argv[1] == "adoptions":
        for r in adoptions(argv[2:]):
            print(json.dumps(r))
        return 0
    if len(argv) >= 3 and argv[1] == "validate":
        rows = validate(argv[2:])
        for r in rows:
            print(json.dumps(r))
        return 0
    if len(argv) == 3 and argv[1] == "grid":
        grid(argv[2])
        return 0
    if len(argv) == 3 and argv[1] == "report":
        report(argv[2])
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
