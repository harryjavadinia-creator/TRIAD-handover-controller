#!/usr/bin/env bash
# No-motion hardware preflight with the configuration written by prepare_hardware_config.sh:
# check the network, then mc_kortex --init-only (connect, read state, initialise mc_rtc, disconnect).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MC_RTC_INSTALL="${MC_RTC_INSTALL:-$HOME/mc_rtc_ws/install}"
KORTEX="${MC_RTC_INSTALL}/bin/mc_kortex"; [[ -x "$KORTEX" ]] || KORTEX="$(command -v mc_kortex)"
bash "${SCRIPT_DIR}/check_dual_network.sh"
LOGDIR="${TRIAD_HARDWARE_LOG_DIR:-$HOME/TRIAD_hardware_logs}"; mkdir -p "$LOGDIR"
LOG="$LOGDIR/init_only_$(date +%Y%m%d_%H%M%S).log"
set +e
"$KORTEX" --init-only 2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}
set -e
if test "$RC" -eq 0; then
  echo "INIT-ONLY PASS"
elif test "$RC" -eq 139 && grep -q 'headless no-motion preflight PASS' "$LOG" && grep -q '\[mc_kortex\] shutdown complete' "$LOG"; then
  echo "INIT-ONLY PASS: both robot states were validated; the known post-shutdown cleanup segfault occurred after shutdown complete."
else
  echo "INIT-ONLY FAILED rc=$RC" >&2
  exit "$RC"
fi
echo "LOG: $LOG"
