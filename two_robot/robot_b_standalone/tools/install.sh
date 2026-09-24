#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL="${HOME}/mc_rtc_ws/install"

test -d "$INSTALL" || {
  echo "ERROR: missing $INSTALL. Install/build mc_rtc and mc_kortex first." >&2
  exit 1
}

if pgrep -x mc_kortex >/dev/null 2>&1 || pgrep -x mc_rtc_ticker >/dev/null 2>&1; then
  echo "ERROR: stop mc_kortex and mc_rtc_ticker before installation." >&2
  exit 1
fi

cmake -S "$ROOT" -B "$ROOT/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$INSTALL"
cmake --build "$ROOT/build" -j"$(nproc)"
cmake --install "$ROOT/build"

RUNTIME="$INSTALL/lib/mc_controller/etc/CALLRobotBFaceToFaceMover.yaml"
test -f "$RUNTIME"
grep -q '^  motionEnabled: false' "$RUNTIME" || {
  echo "ERROR: installed YAML is not in safe disabled state" >&2
  exit 1
}

echo
printf 'INSTALLED SAFELY\n  controller: %s\n  YAML: %s\n' \
  "$INSTALL/lib/mc_controller/CALLRobotBFaceToFaceMover_controller.so" \
  "$RUNTIME"
echo "Physical trajectory remains disabled until:"
echo "  python3 tools/set_motion_enabled.py true"
