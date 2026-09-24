#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -f /tmp/call_robot_b_start
python3 "$ROOT/tools/set_motion_enabled.py" false
pkill -INT -x mc_kortex 2>/dev/null || true
pkill -INT -x mc_rtc_ticker 2>/dev/null || true
echo "Trigger removed, future motion disabled, controller process interrupt requested."
