#!/usr/bin/env python3
"""Reproduce the reported quantities of a hardware run from its archived joint record.

A joint record is the CSV that `mc_bin_utils convert --format csv --entries t Executor_Main qIn qOut
kinova_qIn kinova_qOut` writes from the driver's binary log (semicolon-separated, 1 kHz), optionally
xz-compressed. two_robot/tools/extract_hardware_joint_record.sh produces it. For each robot present the
script prints the state-transition timeline, the maximum joint travel from the first sample (measured
joints, unwrapped, in degrees), the maximum |commanded - measured| tracking error (rad, wrap-aware) and,
when a driver text log is given, the giver coordinator phases. --json prints the same as JSON;
--expect <file> compares against a JSON of expected values and exits non-zero on a mismatch (CI guard).

  python3 two_robot/tools/hardware_run_summary.py <record.csv[.xz]> [--log <driver.log[.xz]>] [--json]
                                                  [--expect <expected.json>]
"""
import argparse
import csv
import json
import lzma
import math
import re
import sys

TWO_PI = 2.0 * math.pi


def open_text(path):
    if path.endswith(".xz"):
        return lzma.open(path, "rt", encoding="utf-8", errors="replace")
    return open(path, encoding="utf-8", errors="replace")


def wrap(d):
    """Signed difference reduced to (-pi, pi]."""
    return (d + math.pi) % TWO_PI - math.pi


def summarize(record_path, log_path=None):
    with open_text(record_path) as handle:
        rows = list(csv.DictReader(handle, delimiter=";"))
    if not rows:
        raise SystemExit("empty record: %s" % record_path)
    header = list(rows[0].keys())
    out = {"record": record_path, "samples": len(rows), "t_end_s": float(rows[-1]["t"]), "robots": {}}

    timeline = []
    prev = None
    for r in rows:
        s = r["Executor_Main"].replace("HandoverInterceptionController_", "")
        if s != prev:
            timeline.append({"t_s": round(float(r["t"]), 3), "state": s})
            prev = s
    out["timeline"] = timeline

    for robot, q_in, q_out in (("Robot A (gen3_2f85)", "qIn", "qOut"), ("Robot B (kinova)", "kinova_qIn", "kinova_qOut")):
        cin = [h for h in header if re.fullmatch(q_in + r"_\d+", h)][:7]
        cout = [h for h in header if re.fullmatch(q_out + r"_\d+", h)][:7]
        if len(cin) < 7:
            continue
        first = [float(rows[0][c]) for c in cin]
        unwrapped = list(first)
        last_meas = list(first)
        travel = [0.0] * 7
        tracking = 0.0
        for r in rows:
            for i, c in enumerate(cin):
                q = float(r[c])
                unwrapped[i] += wrap(q - last_meas[i])
                last_meas[i] = q
                travel[i] = max(travel[i], abs(unwrapped[i] - first[i]))
            if len(cout) == 7:
                for i in range(7):
                    tracking = max(tracking, abs(wrap(float(r[cout[i]]) - float(r[cin[i]]))))
        out["robots"][robot] = {
            "first_q_rad": [round(x, 4) for x in first],
            "max_travel_per_joint_deg": [round(math.degrees(x), 1) for x in travel],
            "max_travel_deg": round(math.degrees(max(travel)), 1),
            "max_travel_joint": 1 + travel.index(max(travel)),
            "max_tracking_error_rad": round(tracking, 4) if len(cout) == 7 else None,
        }

    if log_path:
        phases = []
        pat = re.compile(r"\[DualGiver (PREPOSITION START|START-GATE|READY|START|TERMINAL|HOLD|FAILURE|DISABLED)\]")
        with open_text(log_path) as handle:
            for line in handle:
                m = pat.search(line)
                if m:
                    phases.append(m.group(1))
        out["giver_phases"] = phases
    return out


def print_text(s):
    print("record: %s  (%d samples, %.3f s)" % (s["record"], s["samples"], s["t_end_s"]))
    print("state timeline:")
    for e in s["timeline"]:
        print("  t=%8.3f s  %s" % (e["t_s"], e["state"]))
    for robot, v in s["robots"].items():
        print("%s:" % robot)
        print("  max joint travel from first sample: %.1f deg (joint %d); per joint %s" % (v["max_travel_deg"], v["max_travel_joint"], v["max_travel_per_joint_deg"]))
        if v["max_tracking_error_rad"] is not None:
            print("  max |commanded - measured|: %.4f rad" % v["max_tracking_error_rad"])
    if "giver_phases" in s:
        print("giver phases (driver log): %s" % " -> ".join(s["giver_phases"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("record")
    ap.add_argument("--log")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--expect")
    a = ap.parse_args()
    s = summarize(a.record, a.log)
    if a.json:
        print(json.dumps(s, indent=1))
    else:
        print_text(s)
    if a.expect:
        with open(a.expect, encoding="utf-8") as handle:
            exp = json.load(handle)
        bad = []
        if "timeline" in exp and [e["state"] for e in s["timeline"]] != exp["timeline"]:
            bad.append("timeline")
        for robot, ev in exp.get("robots", {}).items():
            got = s["robots"].get(robot)
            if got is None:
                bad.append(robot + " missing")
                continue
            for key in ("max_travel_deg", "max_tracking_error_rad"):
                if key in ev and abs((got[key] or 0) - ev[key]) > 0.05 * max(1e-9, abs(ev[key])) + 1e-3:
                    bad.append("%s %s: got %s expected %s" % (robot, key, got[key], ev[key]))
        if "giver_phases" in exp and s.get("giver_phases") != exp["giver_phases"]:
            bad.append("giver_phases")
        if bad:
            print("EXPECTATION MISMATCH: " + "; ".join(bad))
            sys.exit(1)
        print("expectations met: %s" % a.expect)


if __name__ == "__main__":
    main()
