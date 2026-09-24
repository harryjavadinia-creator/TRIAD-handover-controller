#!/usr/bin/env bash
# Stop the driver / ticker and switch Robot B's motion off in the controller override.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkill -INT -x mc_kortex 2>/dev/null || true
pkill -INT -x mc_rtc_ticker 2>/dev/null || true
python3 "${SCRIPT_DIR}/tools/set_override_key.py" dualHandover.motionEnabled false || true
echo "Driver stopped and physical scenario motion disabled."
