#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
YAML="$ROOT/etc/CALLRobotBFaceToFaceMover.in.yaml"
fail() { echo "AUDIT FAIL: $*" >&2; exit 1; }

bash -n "$ROOT"/tools/*.sh
python3 - "$ROOT" <<'PY'
from pathlib import Path
import ast, math, sys, yaml
root=Path(sys.argv[1])
main=yaml.safe_load((root/'etc/CALLRobotBFaceToFaceMover.in.yaml').read_text())
assert main['safety']['motionEnabled'] is False
assert main['trajectory']['faceToFaceYaw180'] is True
assert main['startPose']['configured'] is True
assert len(main['startPose']['translation']) == 3
assert main['states'] == {}
assert main['transitions'] == [['CALLRobotBFaceToFaceMover_Run','OK','CALLRobotBFaceToFaceMover_Run','Strict']]
for script in ('select_scenario.py','set_motion_enabled.py','set_face_to_face_anchor.py'):
    ast.parse((root/'tools'/script).read_text(), filename=script)
for p in sorted((root/'etc/presets').glob('*.yaml')):
    cfg=yaml.safe_load(p.read_text())
    assert cfg['robotBStartPose']['configured'] is True
    assert len(cfg['robotBStartPose']['translation']) == 3
    va=cfg['robotAVelocity']; vb=[-va[0],-va[1],va[2]]
    speed=math.sqrt(sum(x*x for x in vb))
    effective=cfg['constantVelocityDuration']+0.5*cfg['decelerationDuration']
    assert speed <= main['safety']['maximumLinearSpeed']+1e-12
    assert speed*effective <= cfg['maximumTravel']+1e-12
source='\n'.join(p.read_text() for p in (root/'src').rglob('*') if p.is_file())
for forbidden in ('teach_start_pose','TEACHING HOLD','POSE SAMPLE','commandGripper','boost::asio','udp://'):
    assert forbidden not in source, forbidden
for required in ('Phase::Prepositioning','Phase::PrepositionSettling','[RobotB PREPOSITION START]','[RobotB READY]','commandToolPoseWithWorldMotion','[RobotB TERMINAL]','terminal gate passed'):
    assert required in source, required
print('Fixed-start/preposition/feedforward/terminal checks: PASS')
PY

grep -q '^  motionEnabled: false' "$YAML" || fail "motion must be disabled"
grep -q '^  configured: true' "$YAML" || fail "predefined start missing"
grep -q '^states: {}' "$YAML" || fail "compiled state lookup missing"
find "$ROOT" -type d -name __pycache__ -o -name '*.pyc' | grep -q . && fail "Python caches present"
echo "AUDIT PASS: predefined scenario start -> READY -> validated trajectory -> terminal HOLD"
