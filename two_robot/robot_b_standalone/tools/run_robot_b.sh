#!/usr/bin/env bash
set -euo pipefail

BIN="$HOME/mc_rtc_ws/install/bin/mc_kortex"
test -x "$BIN" || { echo "ERROR: missing executable $BIN" >&2; exit 1; }

if pgrep -x mc_kortex >/dev/null 2>&1 || pgrep -x mc_rtc_ticker >/dev/null 2>&1; then
  echo "ERROR: another mc_kortex/mc_rtc_ticker process is running" >&2
  exit 1
fi
if test -e /tmp/call_robot_b_start; then
  echo "ERROR: stale /tmp/call_robot_b_start exists; remove it before launch" >&2
  exit 1
fi

mkdir -p "$HOME/Downloads"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$HOME/Downloads/CALL_ROBOT_B_$STAMP.log"
echo "Logging to $LOG"
"$BIN" 2>&1 | tee "$LOG"
