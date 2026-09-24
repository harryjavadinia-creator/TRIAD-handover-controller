#!/usr/bin/env python3
"""Turn the binary log of a two-robot run into the 20 ms timeline CSV.

usage: extract_timeline.py <mc-control-*.bin> [out.csv]

Columns: t, Executor_Main (Robot A state), dual_giver_phase (1 Prepositioning … 6 Holding),
dual_giver_speed, dual_giver_tracking_error, object_x/y/z (the object pose as carried, i.e. the
call_object robot's floating base), planned_object_x/y/z (the object pose Robot A observes, plans and
retreats with), and object_agreement (their distance). Needs mc_bin_utils on PATH.
"""
import csv
import math
import os
import subprocess
import sys
import tempfile

ENTRIES = ["t", "Executor_Main", "dual_giver_phase", "dual_giver_speed", "dual_giver_tracking_error",
           "call_object_FloatingBase_position", "handover_object_x", "handover_object_y", "handover_object_z"]


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr); return 2
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(src)), "timeline_20ms.csv")
    with tempfile.TemporaryDirectory() as tmp:
        base = os.path.join(tmp, "full")
        subprocess.run(["mc_bin_utils", "convert", "--in", src, "--out", base, "--format", "csv", "--entries"] + ENTRIES,
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rows = list(csv.DictReader(open(base + ".csv"), delimiter=";"))
    worst = 0.0
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "Executor_Main", "dual_giver_phase", "dual_giver_speed", "dual_giver_tracking_error",
                    "object_x", "object_y", "object_z", "planned_object_x", "planned_object_y", "planned_object_z", "object_agreement"])
        for i, r in enumerate(rows):
            ox, oy, oz = (float(r["call_object_FloatingBase_position_" + k]) for k in "xyz")
            px, py, pz = (float(r["handover_object_" + k]) for k in "xyz")
            d = math.dist((ox, oy, oz), (px, py, pz))
            if i > 5: worst = max(worst, d)
            if i % 20 == 0:
                w.writerow([f"{i * 0.001:.3f}", r["Executor_Main"].replace("HandoverInterceptionController_", ""), r["dual_giver_phase"],
                            f"{float(r['dual_giver_speed']):.7f}", f"{float(r['dual_giver_tracking_error']):.7f}",
                            f"{ox:.5f}", f"{oy:.5f}", f"{oz:.5f}", f"{px:.5f}", f"{py:.5f}", f"{pz:.5f}", f"{d:.5f}"])
    print(f"{out}: {len(rows)} samples at 1 kHz, written every 20 ms; max |object - planned object| after t=5 ms: {1e3 * worst:.2f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
