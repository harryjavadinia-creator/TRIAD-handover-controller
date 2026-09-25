#!/usr/bin/env bash
# Extract the joint record of a hardware run from the driver's binary log: time, FSM state, measured and
# commanded joints of Robot A (qIn/qOut) and, when present, Robot B (kinova_qIn/kinova_qOut), at the
# controller rate, semicolon-separated, xz-compressed. Rows more than HOLD_SECONDS (default 10) after the
# controller entered its terminal state are dropped (a run that was left in the fail-safe hold for minutes
# carries nothing after that point).
#   two_robot/tools/extract_hardware_joint_record.sh <run.bin> <out.csv.xz> [HOLD_SECONDS]
set -euo pipefail
BIN="$1"; OUT="$2"; HOLD="${3:-10}"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
mc_bin_utils convert --in "$BIN" --out "$TMP/rec" --format csv --entries t Executor_Main qIn qOut kinova_qIn kinova_qOut > /dev/null 2>&1
python3 - "$TMP/rec.csv" "$HOLD" <<'PY' | xz -9 > "$OUT"
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1]), delimiter=";")); hold = float(sys.argv[2])
cut = None
for r in rows:
    if r["Executor_Main"].endswith(("_Failure", "_Completed")):
        cut = float(r["t"]) + hold; break
w = csv.DictWriter(sys.stdout, fieldnames=list(rows[0].keys()), delimiter=";"); w.writeheader()
for r in rows:
    if cut is not None and float(r["t"]) > cut: break
    w.writerow(r)
PY
echo "wrote $OUT ($(xzcat "$OUT" | wc -l) rows)"
