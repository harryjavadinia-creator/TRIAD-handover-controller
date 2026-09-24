#!/usr/bin/env bash
set -euo pipefail
pkill -INT -x mc_kortex 2>/dev/null || true
pkill -INT -x mc_rtc_ticker 2>/dev/null || true
python3 "$HOME/mc_rtc_ws/Sandbox/CALLDualRobotHandoverController/tools/set_motion_enabled.py" false
rm -f /tmp/call_dual_start /tmp/call_robot_b_start
echo "Dual controller stopped and physical scenario motion disabled."
