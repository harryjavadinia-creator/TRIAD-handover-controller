# Real robot

**No result in this repository has been validated end-to-end on physical
hardware.** This document separates verified simulation behavior from the
configuration, calibration, and safety work still required before any physical
handover is attempted.

## 1. Simulation-validated

The TRIAD selector and the four canonical scenario results described in
[`simulation.md`](simulation.md) are simulation only, with
`allowPhysicalExecution: false`. The documented perception-latency experiments
also use simulation.

## 2. Hardware support present in code but not end-to-end validated

- Robot naming convention: `toolFrame: gen3_robotiq_85_base_link`,
  `robot: gen3_2f85` (Kinova Gen3 + Robotiq 2F-85).
- `gripper.physicalBridge` configuration block: calibrated feedback endpoints
  (`openPercent: 0.87`, `closePercent: 35.0`, `maxPercent: 35.0`), gated by
  `enabled`/`commandEnabled`/`requireFeedback`.
- `hardwareGripperCommissioning`: a staged, non-contact smoke-test mode. When
  enabled, the arm freezes at the measured startup posture, only the physical
  Robotiq gripper is exercised through the feedback-gated bridge, and the FSM
  never enters observation, planning, reach, capture, transfer or retreat.
- An optional `Kortex.init_posture` startup posture (disabled by default,
  `on_startup: false`).
- A `physicalBridge.source` selector (`virtual_sensor | synthetic |
  force_sensor | disabled`) for switching between simulated and physical
  force/contact sources.

None of these elements has been exercised end-to-end on physical hardware as a
TRIAD handover validation campaign.

## 3. Unverified real-robot procedure

There is currently no end-to-end tested procedure for reproducing a TRIAD
handover on physical hardware. The required software/configuration pieces exist,
but their presence is not evidence of hardware readiness.

## 4. Missing calibration and safety information

The following are not established by this repository and must be defined and
verified locally before any physical attempt:

- emergency-stop procedure;
- safety-zone / workspace-boundary definition;
- network setup and robot IP/credentials (the example config in
  `configs/mc_rtc.yaml.example` contains placeholders only);
- mouth/tool calibration procedure;
- object-frame calibration procedure;
- operational changes required when moving `physicalBridge.source` from
  `virtual_sensor` to `force_sensor`;
- physical workspace/reachability assumptions beyond the simulated scenarios.

Real credentials must never be committed to the repository.

## Safety warning

Any physical attempt must begin with the non-contact
`hardwareGripperCommissioning` smoke-test path, with the arm frozen and the
gripper as the only active component. Do not enable `commandEnabled` or full
FSM execution on hardware without independently establishing the calibration,
workspace, network, and emergency-stop procedures above.

## Implementation status

The frozen scientific source was exercised entirely in simulation, with
`allowPhysicalExecution: false`, the physical gripper bridge disabled, and the
force-transfer source set to the virtual sensor.

`mc_kortex` is a hardware driver and `mc_rtc_ticker` is a simulation harness.
The presence of either in the toolchain is **not** evidence of physical robot
execution.

Accordingly, the hardware material in this repository is a prerequisite
checklist and staged commissioning path, not a validated end-to-end real-robot
handover procedure.

## 4. Two-robot setup (Robot A receiver, Robot B giver) — what exists and what was reached

The July 2026 laboratory setup is integrated on this branch under [`two_robot/`](../two_robot/README.md):
a second Kinova Gen3 (`kinova`, 192.168.1.11) presents the object to Robot A (`gen3_2f85`, 192.168.1.10)
along a fixed world-frame trajectory, coordinated inside the controller (`dualHandover`), or from a second
laptop by the standalone `robot_b_standalone/` mover.

- **Receiver switches for hardware** are collected in
  [`two_robot/HandoverInterceptionController.hardware_receiver.yaml`](../two_robot/HandoverInterceptionController.hardware_receiver.yaml):
  `decisionCost.allowPhysicalExecution: true`, `gripper.physicalBridge` enabled with command and feedback,
  and the gripper closure calibration measured on the real gripper on 16 July 2026 (`closePercent`/`maxPercent`
  50.37, `openPercent` 0.87). The repository default keeps the 15 July value (35.0); the 16 July value is the
  later calibration and the one the last hardware runs used. `transfer.source` stayed `virtual_sensor` in
  every hardware run: no physical force path has been validated.
- **Reached on hardware (July 2026):** Robot A gripper commissioning (tag `v6.4.2`, 15 July); Robot B alone
  executing its presentation with the integrated coordinator (17 July 22:30, phases Prepositioning → Holding);
  the combined run reaching `ExecuteCommittedReach` before failing (18 July 00:51). **No hardware run reached
  `CaptureTransfer`.** Inventory: [`two_robot/evidence/JULY_2026_HARDWARE_LOG_INVENTORY.md`](../two_robot/evidence/JULY_2026_HARDWARE_LOG_INVENTORY.md).
- **Verified in simulation with this code (24 September 2026):** the full two-arm handover completes
  ([`two_robot/results/sim_2026-09-24/TIMELINE.md`](../two_robot/results/sim_2026-09-24/TIMELINE.md)).
- **Procedure:** the seven steps in [`two_robot/README.md`](../two_robot/README.md), motion disabled until
  each step passes.

The statement at the top of this page still holds: no TRIAD handover has been validated end-to-end on
physical hardware.
