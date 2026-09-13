#!/usr/bin/env python3
"""Machine checks that the simulated giver truth is independent of the receiver.

Three independent checks:

  static   The source region GIVER_INDEPENDENT_BEGIN..END in
           updateSimulatedObjectMotion() and src/IndependentGiverModel.h may not
           reference any receiver-plan, interception-time, grasp, route, commit
           or provisional state. The model header takes only a script and time.

  replay   Every [GiverTruthSample] in a run log is recomputed in Python from the
           logged [GiverTruthScript] alone (same closed-form law) and must match
           to 1e-9 m / 1e-9 in quaternion components. The truth is therefore a
           function of scenario data and elapsed time only.

  cross    Two logs of the same scenario with different receiver behaviour (for
           example V1 and V2, or different selected plans) must have identical
           truth samples at identical elapsed times, up to the first physical
           attachment in either run.

Usage:
  check_giver_truth_independence.py --static
  check_giver_truth_independence.py --replay RUN.log
  check_giver_truth_independence.py --cross RUN_A.log RUN_B.log
"""

import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "src" / "HandoverInterceptionController.cpp"
HDR = ROOT / "src" / "IndependentGiverModel.h"

FORBIDDEN_TOKENS = [
    r"committed", r"Committed", r"provisional", r"Provisional", r"plan\b",
    r"Plan\b", r"InterceptionPlan", r"candidate", r"Candidate", r"route",
    r"Route", r"grasp", r"Grasp", r"tau\b", r"presentationTime",
    r"selected", r"Selected", r"planning", r"receiver(?!PlanInputs)",
]

FLOAT = r"[-+]?(?:\d+\.?\d*(?:[eE][-+]?\d+)?|nan|inf)"
SCRIPT_RE = re.compile(
    r"\[GiverTruthScript\].*?startTime=(" + FLOAT + r").*?p0=\[(" + FLOAT + r"),(" + FLOAT
    + r"),(" + FLOAT + r")\] q0=\[(" + FLOAT + r"),(" + FLOAT + r"),(" + FLOAT + r"),("
    + FLOAT + r")\] v=\[(" + FLOAT + r"),(" + FLOAT + r"),(" + FLOAT + r")\] w=\[("
    + FLOAT + r"),(" + FLOAT + r"),(" + FLOAT + r")\] travelDistance=(" + FLOAT
    + r") stopDuration=(" + FLOAT + r")")
SAMPLE_RE = re.compile(
    r"\[GiverTruthSample\] elapsed=(" + FLOAT + r") p=\[(" + FLOAT + r"),(" + FLOAT
    + r"),(" + FLOAT + r")\] q=\[(" + FLOAT + r"),(" + FLOAT + r"),(" + FLOAT + r"),("
    + FLOAT + r")\]")


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)


def static_check() -> bool:
    ok = True
    src = CPP.read_text()
    m = re.search(r"GIVER_INDEPENDENT_BEGIN(.*?)GIVER_INDEPENDENT_END", src, re.S)
    if not m:
        print("static: FAIL independent giver region markers not found")
        return False
    region = strip_comments(m.group(1))
    # Log format strings are data, not state reads; drop string literals.
    region = re.sub(r'"(?:[^"\\]|\\.)*"', '""', region)
    header = re.sub(r'"(?:[^"\\]|\\.)*"', '""', strip_comments(HDR.read_text()))
    for name, body in (("updateSimulatedObjectMotion independent region", region),
                       ("IndependentGiverModel.h", header)):
        hits = sorted({t for t in FORBIDDEN_TOKENS if re.search(t, body)})
        if hits:
            ok = False
            print(f"static: FAIL {name} references receiver state tokens {hits}")
        else:
            print(f"static: PASS {name} has no receiver-plan dependency")
    if "independentGiverStateAt(independentGiverScript_, elapsed)" not in region:
        ok = False
        print("static: FAIL truth is not computed from the script and elapsed time")
    else:
        print("static: PASS truth = independentGiverStateAt(script, elapsed)")
    return ok


def quintic_step(x):
    x = min(1.0, max(0.0, x))
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x))


def quintic_stop_integral(x):
    x = min(1.0, max(0.0, x))
    x2 = x * x
    x4 = x2 * x2
    return x - 2.5 * x4 + 3.0 * x4 * x - x4 * x2


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def model(script, t):
    p0, q0, v, w, dist, stop = script
    speed = math.sqrt(sum(c * c for c in v))
    t = max(0.0, t)
    if speed <= 1e-12:
        eff = 0.0
    else:
        d = max(0.0, dist)
        st = max(0.0, stop)
        if 0.5 * speed * st > d:
            st = 2.0 * d / speed
        cruise = max(0.0, (d - 0.5 * speed * st) / speed)
        if t <= cruise:
            eff = t
        elif st > 1e-12 and t < cruise + st:
            eff = cruise + st * quintic_stop_integral((t - cruise) / st)
        else:
            eff = cruise + 0.5 * st
    p = tuple(p0[i] + eff * v[i] for i in range(3))
    q = q0
    om = math.sqrt(sum(c * c for c in w))
    if om > 1e-9 and eff > 0.0:
        ang = om * eff
        axis = tuple(c / om for c in w)
        dq = (math.cos(ang / 2),) + tuple(math.sin(ang / 2) * c for c in axis)
        q = qmul(dq, q0)
    return p, q


def load(log):
    script, samples, attach = None, [], None
    for line in open(log, errors="replace"):
        if script is None:
            m = SCRIPT_RE.search(line)
            if m:
                g = [float(x) for x in m.groups()]
                script = ((g[1], g[2], g[3]), (g[4], g[5], g[6], g[7]),
                          (g[8], g[9], g[10]), (g[11], g[12], g[13]), g[14], g[15])
        m = SAMPLE_RE.search(line)
        if m:
            g = [float(x) for x in m.groups()]
            samples.append((g[0], (g[1], g[2], g[3]), (g[4], g[5], g[6], g[7])))
        if attach is None and "[ForceTransferComplete] robot support established" in line:
            # attachObjectToMouth() precedes this marker; from here the object
            # is carried by the robot, which is physics, not a plan decision.
            attach = samples[-1][0] if samples else 0.0
    return script, samples, attach


def replay_check(log) -> bool:
    script, samples, _ = load(log)
    if script is None or not samples:
        print(f"replay: FAIL {log}: no [GiverTruthScript] or no samples")
        return False
    worst_p = worst_q = 0.0
    for t, p, q in samples:
        mp, mq = model(script, t)
        worst_p = max(worst_p, max(abs(p[i] - mp[i]) for i in range(3)))
        sign = 1.0 if sum(q[i] * mq[i] for i in range(4)) >= 0 else -1.0
        worst_q = max(worst_q, max(abs(q[i] - sign * mq[i]) for i in range(4)))
    ok = worst_p <= 1e-9 and worst_q <= 1e-9
    print(f"replay: {'PASS' if ok else 'FAIL'} {Path(log).name}: {len(samples)} samples, "
          f"max |dp|={worst_p:.3e} m, max |dq|={worst_q:.3e}; truth recomputed from script only")
    return ok


def cross_check(log_a, log_b) -> bool:
    sa, xa, att_a = load(log_a)
    sb, xb, att_b = load(log_b)
    if not xa or not xb:
        print("cross: FAIL missing truth samples")
        return False
    horizon = min(att_a if att_a is not None else math.inf,
                  att_b if att_b is not None else math.inf)
    index_b = {round(t, 6): (p, q) for t, p, q in xb}
    matched, worst = 0, 0.0
    for t, p, q in xa:
        if t > horizon:
            break
        key = round(t, 6)
        if key in index_b:
            pb, _ = index_b[key]
            worst = max(worst, max(abs(p[i] - pb[i]) for i in range(3)))
            matched += 1
    script_equal = sa == sb
    ok = matched > 0 and worst <= 1e-9 and script_equal
    print(f"cross: {'PASS' if ok else 'FAIL'} {Path(log_a).name} vs {Path(log_b).name}: "
          f"{matched} common elapsed samples before attachment, max |dp|={worst:.3e} m, "
          f"identical script={script_equal}")
    return ok


def main(argv) -> int:
    if len(argv) >= 2 and argv[1] == "--static":
        return 0 if static_check() else 1
    if len(argv) == 3 and argv[1] == "--replay":
        return 0 if replay_check(argv[2]) else 1
    if len(argv) == 4 and argv[1] == "--cross":
        return 0 if cross_check(argv[2], argv[3]) else 1
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
