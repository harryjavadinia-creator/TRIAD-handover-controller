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
