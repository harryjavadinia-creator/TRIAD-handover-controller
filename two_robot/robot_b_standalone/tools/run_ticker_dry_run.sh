#!/usr/bin/env bash
set -euo pipefail
mkdir -p "$HOME/Downloads"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$HOME/Downloads/CALL_ROBOT_B_TICKER_$STAMP.log"
echo "Logging to $LOG"
mc_rtc_ticker 2>&1 | tee "$LOG"
