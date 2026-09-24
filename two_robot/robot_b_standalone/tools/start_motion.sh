#!/usr/bin/env bash
set -euo pipefail
TRIGGER="/tmp/call_robot_b_start"
if ! pgrep -x mc_kortex >/dev/null 2>&1 && ! pgrep -x mc_rtc_ticker >/dev/null 2>&1; then
  echo "ERROR: no mc_kortex or mc_rtc_ticker process is running" >&2
  exit 1
fi
rm -f "$TRIGGER"
printf 'START\n' > "$TRIGGER"
echo "Robot-B scenario-motion trigger created: $TRIGGER"
echo "This trigger is valid only after [RobotB READY]."
