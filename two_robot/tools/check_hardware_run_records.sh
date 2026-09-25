#!/usr/bin/env bash
# CI guard: every archived joint record of the 25 September 2026 hardware runs reproduces the quantities
# quoted in the record and the paper (state timeline, maximum joint travel, maximum tracking error, giver
# phases), as listed in joints/expected_quantities.json.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
E="$ROOT/two_robot/evidence/hardware_runs_2026-09-25"
EXP="$E/joints/expected_quantities.json"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
fail=0; n=0
for rec in "$E"/joints/*.joints.csv.xz; do
  b="$(basename "$rec" .joints.csv.xz)"
  python3 - "$EXP" "$b" "$TMP/$b.json" <<'PY'
import json, sys
exp = json.load(open(sys.argv[1]))[sys.argv[2]]
json.dump(exp, open(sys.argv[3], "w"))
PY
  args=("$rec" --expect "$TMP/$b.json")
  [[ -f "$E/$b.mc_kortex.log.xz" ]] && args+=(--log "$E/$b.mc_kortex.log.xz")
  if python3 "$ROOT/two_robot/tools/hardware_run_summary.py" "${args[@]}" > "$TMP/$b.out" 2>&1; then
    echo "  PASS  $b"; n=$((n+1))
  else
    echo "  FAIL  $b"; tail -3 "$TMP/$b.out"; fail=1
  fi
done
[[ $fail -eq 0 ]] && echo "HARDWARE RUN RECORDS: PASS ($n records)" || { echo "HARDWARE RUN RECORDS: FAIL"; exit 1; }
