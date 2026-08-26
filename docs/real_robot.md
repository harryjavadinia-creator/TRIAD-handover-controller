# Real robot

**No result in this repository has been validated end-to-end on physical
hardware.** This document separates what is actually verified from what
still needs to be configured, calibrated, and checked before any physical
handover is attempted.

## 1. Simulation-validated

The full TRIAD selector and all four reported scenario results (see
`docs/simulation.md`) are simulation only, with `allowPhysicalExecution:
false`. A separate, earlier latency-compensation study (perception-latency
forward-prediction capability) also ran in simulation only.

## 2. Hardware support present in code but not end-to-end validated

- Robot naming convention: `toolFrame: gen3_robotiq_85_base_link`,
  `robot: gen3_2f85` (Kinova Gen3 + Robotiq 2F-85).
- `gripper.physicalBridge` configuration block: calibrated feedback
  endpoints (`openPercent: 0.87`, `closePercent: 35.0`, `maxPercent: 35.0`),
  gated by `enabled`/`commandEnabled`/`requireFeedback`.
- `hardwareGripperCommissioning`: a staged, non-contact smoke-test mode —
  when enabled, the arm freezes at the measured startup posture, only the
  physical Robotiq gripper is exercised through the feedback-gated bridge,
  and the FSM never enters observation, planning, reach, capture, transfer
  or retreat. This is the recommended first step before any physical
  handover attempt.
- An optional `Kortex.init_posture` startup posture (disabled by default,
  `on_startup: false`).
- A `physicalBridge.source` selector (`virtual_sensor | synthetic |
  force_sensor | disabled`) for switching between simulated and physical
  force/contact sources.

None of the above has been exercised end-to-end on physical hardware as
part of the work in this repository.

## 3. Unverified real-robot reproduction procedure

There is currently no end-to-end, tested procedure to reproduce a handover
on physical hardware. The pieces above exist in configuration but have not
been assembled and run together on a robot. Do not treat their presence in
the configuration file as evidence of hardware readiness.

## 4. Missing calibration/safety information (explicit gaps)

The following are **not documented anywhere in this repository** and must
be established locally before any physical attempt:

- Emergency-stop procedure.
- Safety-zone / workspace-boundary definition.
- Network setup and robot IP/credentials (the example config in
  `configs/mc_rtc.yaml.example` uses placeholders only — see that file for
  the required `Kortex` block structure; real values must never be
  committed to this repository).
- Mouth/tool calibration procedure.
- Object-frame calibration procedure.
- What changes operationally when moving `physicalBridge.source` from
  `virtual_sensor` to `force_sensor` (perception input, contact source).
- Physical workspace/reachability assumptions beyond the simulated
  scenarios in `docs/simulation.md`.

## Safety warning

Any physical attempt must begin with the non-contact `hardwareGripperCommissioning`
smoke-test path (item 2 above), with the arm frozen and the gripper the only
active component, before any full handover attempt is considered. Do not
enable `commandEnabled` or full FSM execution on hardware without first
independently establishing all of the missing information in section 4.
