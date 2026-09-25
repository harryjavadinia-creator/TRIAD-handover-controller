#!/usr/bin/env bash
# Stop the driver / ticker and switch Robot B's motion off in the controller override.
# After the fail-safe hold mc_kortex ignores SIGINT and SIGTERM (observed 25 September 2026), so the stop
# escalates to SIGKILL after a bounded wait; the robot holds its pose when the session drops.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
stop_one() {
  local name="$1"
  pgrep -x "$name" > /dev/null || return 0
  pkill -INT -x "$name" 2>/dev/null || true
  for _ in $(seq 1 10); do pgrep -x "$name" > /dev/null || return 0; sleep 1; done
  pkill -TERM -x "$name" 2>/dev/null || true
  for _ in $(seq 1 5); do pgrep -x "$name" > /dev/null || return 0; sleep 1; done
  echo "$name ignored SIGINT/SIGTERM; sending SIGKILL"
  pkill -KILL -x "$name" 2>/dev/null || true
}
stop_one mc_kortex
stop_one mc_rtc_ticker
python3 "${SCRIPT_DIR}/tools/set_override_key.py" dualHandover.motionEnabled false || true
echo "Driver stopped and physical scenario motion disabled."
