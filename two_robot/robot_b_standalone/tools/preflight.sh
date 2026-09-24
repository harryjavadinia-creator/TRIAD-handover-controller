#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
YAML="$HOME/mc_rtc_ws/install/lib/mc_controller/etc/CALLRobotBFaceToFaceMover.yaml"
echo "===== ROBOT B FIXED-START PREFLIGHT ====="
for p in "$ROOT" "$YAML" "$HOME/mc_rtc_ws/install/bin/mc_kortex"; do test -e "$p" || { echo "MISSING: $p"; exit 1; }; done
python3 - "$YAML" <<'PY'
from pathlib import Path
import sys,yaml,math
p=Path(sys.argv[1]); d=yaml.safe_load(p.read_text())
tr=d['trajectory']; start=d['startPose']['translation']; va=tr['robotAVelocity']; vb=[-va[0],-va[1],va[2]]
eff=tr['constantVelocityDuration']+0.5*tr['decelerationDuration']
final=[start[i]+vb[i]*eff for i in range(3)]
print('scenario:',tr['scenario'])
print('predefined Robot-B start:',start)
print('mirrored velocity:',vb)
print('final:',final)
print('motionEnabled:',d['safety']['motionEnabled'])
print('preposition:',d['preposition'])
print('terminal:',d['terminal'])
PY
CFG="$HOME/.config/mc_rtc/mc_rtc.yaml"
echo; grep -nE 'MainRobot|Enabled|Timestep|ip:' "$CFG" || true
echo; pgrep -af '(^|/)(mc_kortex|mc_rtc_ticker)( |$)' || echo "No controller process running"
test ! -e /tmp/call_robot_b_start || { echo "ERROR: stale trigger exists"; exit 1; }
echo "PASS: preflight sends no robot command"
