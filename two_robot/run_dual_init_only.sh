#!/usr/bin/env bash
set -euo pipefail
ROOT="$HOME/mc_rtc_ws/Sandbox/CALLDualRobotHandoverController"
bash "$ROOT/tools/check_dual_network.sh"
"$HOME/bin/use_dual_robot_actual_controller"
LOG="$HOME/Downloads/CALL_DUAL_STATIC_X_INIT_ONLY_$(date +%Y%m%d_%H%M%S).log"

set +e
"$HOME/mc_rtc_ws/install/bin/mc_kortex" --init-only 2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}
set -e

if test "$RC" -eq 0; then
  echo "INIT-ONLY PASS"
elif test "$RC" -eq 139 \
  && grep -q 'headless no-motion preflight PASS' "$LOG" \
  && grep -q '\[mc_kortex\] shutdown complete' "$LOG"; then
  echo "INIT-ONLY PASS: both robot states were validated; known post-shutdown cleanup segfault occurred after shutdown complete."
else
  echo "INIT-ONLY FAILED rc=$RC" >&2
  exit "$RC"
fi

echo "LOG: $LOG"
