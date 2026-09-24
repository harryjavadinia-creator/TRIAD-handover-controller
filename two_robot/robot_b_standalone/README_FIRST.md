# CALL Robot B — Fixed predefined scenario starts

This controller does exactly two motions:

1. From the current tool position, move smoothly to the selected scenario's predefined Robot-B start position and wait at `READY`.
2. After one manual trigger, execute the validated mirrored Robot-A object trajectory, decelerate, pass the measured terminal gate, and hold.

There is no teaching routine, no Robot-A communication, no candidate selection, no retry, and no use of the arbitrary current pose as the scenario origin.

The tool orientation present when the controller starts is preserved during preposition and scenario execution.

## Mapping

The delivered presets use the physically validated Canonical-YZ Robot-B start `[0.4567, 0.0010, 0.4337]` as the anchor. Other starts are generated from Robot-A initial positions using the fixed 180-degree relation:

`delta_B = [-delta_A.x, -delta_A.y, +delta_A.z]`.

To change the one known canonical Robot-B anchor numerically:

```bash
python3 tools/set_face_to_face_anchor.py X Y Z
```

This is ordinary configuration, not pose teaching. Then select a scenario again.

## Run

```bash
python3 tools/select_scenario.py canonical_yz
python3 tools/set_motion_enabled.py true
rm -f /tmp/call_robot_b_start
bash tools/preflight.sh
bash tools/run_robot_b.sh
```

Wait for `[RobotB READY]`, then in a second terminal:

```bash
bash tools/start_motion.sh
```

Wait for `[RobotB HOLD]`, then:

```bash
bash tools/stop_and_disable.sh
```
