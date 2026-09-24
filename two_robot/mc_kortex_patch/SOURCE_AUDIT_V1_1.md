# V1.1 source contract

- Robot B loaded from installed `Kinova` RobotModule as instance `kinova`.
- Robot-B frames: `base_link`, `tool_frame`.
- Robot-B physical gripper channel: disabled and not created.
- Robot-B coordinator contains no gripper, release, attachment, retry, or return-home logic.
- `mc_kortex` maps Robot A `gen3_joint_1..7` and Robot B `joint_1..7` independently.
- Per-robot map initialization remains mandatory.
- Robot A receiver source/methodology is preserved.
- Motion defaults to disabled.
