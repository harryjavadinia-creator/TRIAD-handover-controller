# Real robot

**Physical dual-robot operation is documented by real-world video, but no complete TRIAD
handover has been validated end-to-end on physical hardware as a synchronized controller
validation campaign.** This document separates direct hardware evidence, verified simulation
behavior, and the calibration/safety work required for reproducible physical execution.

## 1. Simulation-validated

The TRIAD selector and the four canonical scenario results described in
[`simulation.md`](simulation.md) are simulation only, with
`allowPhysicalExecution: false`. The documented perception-latency experiments
also use simulation.

## 2. Hardware support present in code but not end-to-end validated

- Robot naming convention: `toolFrame: gen3_robotiq_85_base_link`,
  `robot: gen3_2f85` (Kinova Gen3 + Robotiq 2F-85).
- `gripper.physicalBridge` configuration block: calibrated feedback endpoints
  (`openPercent: 0.87`; `closePercent`/`maxPercent` 35.0 by repository default, 50.37 in the 16 July hardware calibration), gated by
  `enabled`/`commandEnabled`/`requireFeedback`.
- `hardwareGripperCommissioning`: a staged, non-contact smoke-test mode. When
  enabled, the arm freezes at the measured startup posture, only the physical
  Robotiq gripper is exercised through the feedback-gated bridge, and the FSM
  never enters observation, planning, reach, capture, transfer or retreat.
- An optional `Kortex.init_posture` startup posture (disabled by default,
  `on_startup: false`). On the laboratory Gen3 firmware the robot rejects the
  driver's waypoint (`Time optimal splines are not supported`) and the driver
  then crashes at control-loop start (25 September 2026), so it must stay off;
  `two_robot/tools/kortex_home.cpp` homes the arm through the robot's own
  `Home` action instead.
- A `transfer.source` selector (`virtual_sensor | synthetic |
  force_sensor | disabled`) for switching between simulated and physical
  force/contact sources.

These elements are not documented here as a complete synchronized end-to-end TRIAD hardware
validation campaign. Separate July 2026 evidence does document the two physical robots operating
together; see Section 5.

## 3. Real-robot reproduction status

There is currently no end-to-end tested procedure in this repository for reproducing a complete
TRIAD handover on physical hardware from a fresh checkout. What is reproducible from a fresh checkout is
the single-robot pre-contact sequence: on 25 September 2026 the physical Robot A ran the four reported
scenarios against the virtual object with `two_robot/run_single_robot_scenario.sh` (Section 6). The required software/configuration
pieces exist, and the July laboratory evidence shows physical dual-robot operation, but those are
different claims from a reproducible end-to-end controller validation.

## 4. Missing calibration and safety information

The following are not established by this repository and must be defined and
verified locally before a new physical attempt:

- emergency-stop procedure;
- safety-zone / workspace-boundary definition;
- network setup and robot IP/credentials (`two_robot/mc_rtc.two_kortex.yaml`
  carries placeholders only);
- an object-pose source (perception) for a handover from a human hand; the
  repository provides none; without one the single-robot hardware cases are
  the gripper smoke test and the four scenarios against the virtual object
  (Section 6), which exercise observation, planning, reach and closure but no
  contact;
- mouth/tool calibration procedure;
- object-frame calibration procedure;
- operational changes required when moving `transfer.source` from
  `virtual_sensor` to `force_sensor`;
- physical workspace/reachability assumptions beyond the simulated scenarios.

Real credentials must never be committed to the repository.

### 4.1 Safety warning

Any new physical attempt must begin with the non-contact
`hardwareGripperCommissioning` smoke-test path, with the arm frozen and the
gripper as the only active component. Do not enable `commandEnabled` or full
FSM execution on hardware without independently establishing the calibration,
workspace, network, and emergency-stop procedures above.

### 4.2 Implementation status

The frozen scientific-source simulation campaign was exercised with
`allowPhysicalExecution: false`, the physical gripper bridge disabled, and the
force-transfer source set to the virtual sensor. That statement describes the
simulation validation campaign; it does **not** mean that the two-robot laboratory
setup was never operated physically.

`mc_kortex` is a hardware driver and `mc_rtc_ticker` is a simulation harness.
The presence of either in the toolchain alone is **not** evidence of physical robot
execution. The separate phone-video evidence in `two_robot/media/` is direct visual
evidence of such physical execution.

Accordingly, distinguish three evidence classes: the reproducible simulation campaign, the
July mc_rtc hardware logs, and the real-world dual-robot video footage.

## 5. Two-robot setup (Robot A receiver, Robot B giver) — what exists and what was reached

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
  the inventoried hardware runs: no physical force path has been validated.
- **Controller-log evidence (July 2026):** Robot A gripper commissioning (15 July, sandbox controller logs); Robot B alone
  executing its presentation with the integrated coordinator (17 July 22:30, phases Prepositioning → StartSettling → Ready → Executing → TerminalSettling → Holding);
  and, on 18 July, nine combined runs in which Robot A entered `CaptureTransfer` on the physical arms with
  the gripper bridge enabled and closed the gripper on the object. Reach-and-close was the objective of those
  sessions (the object was taped to Robot B's tool; no transfer or retreat by design); each run then stopped
  in the fail-safe hold at the closure check against the virtual object model, as expected without a
  physical contact signal
  ([`two_robot/evidence/hardware_runs_2026-07-18/`](../two_robot/evidence/hardware_runs_2026-07-18/README.md)).
  In the first four of those runs Robot B was driven by a fixed-joint override in the Kortex driver, in the
  last five by the giver coordinator's references. Inventory of the binary logs:
  [`two_robot/evidence/JULY_2026_HARDWARE_LOG_INVENTORY.md`](../two_robot/evidence/JULY_2026_HARDWARE_LOG_INVENTORY.md).
- **Direct visual hardware evidence:**
  [`two_robot/media/dual_robot_physical_01.mp4`](../two_robot/media/dual_robot_physical_01.mp4) and
  [`two_robot/media/dual_robot_physical_02.mp4`](../two_robot/media/dual_robot_physical_02.mp4) show both
  physical Kinova arms operating together in the laboratory handover setup. In the second clip Robot B
  supports/presents the bottle while Robot A's Robotiq gripper approaches and closes on the bottle neck.
  This video evidence is separate from the state logs and does not by itself assign a specific FSM state.
  Source hashes and the evidence boundary are recorded in
  [`two_robot/evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md`](../two_robot/evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md).
- **Verified in simulation with this code (24 September 2026):** the full two-arm handover completes
  ([`two_robot/results/sim_2026-09-24/TIMELINE.md`](../two_robot/results/sim_2026-09-24/TIMELINE.md)).
- **Procedure:** README §4 and [`two_robot/README.md`](../two_robot/README.md): `prepare_hardware_config.sh`
  writes the mc_rtc profile and the controller override from the repository files, then network check,
  no-motion preflight, gripper smoke test, Robot B alone, the handover; motion disabled until each step passes.

## 6. Robot A alone: the four scenarios on the physical arm (25 September 2026)

With the published sources, `two_robot/prepare_hardware_config.sh --single` (Robot B removed, giver
disabled, the virtual object of the single-robot scenarios) and the receiver hardware overlay, the physical
Robot A ran `longitudinal` (three times), `near-ground`, `lateral-low` and `diagonal`
([`two_robot/evidence/hardware_runs_2026-09-25/`](../two_robot/evidence/hardware_runs_2026-09-25/README.md)):

- every run whose search committed went Initial → ObserveObject → SolveInterception →
  ExecuteCommittedReach → PresentationHold → MovePregrasp → CaptureTransfer on the physical arm and closed
  the physical gripper at the planned capture pose: six of six. Reach targets from [0.163, 0.195, 0.166] to
  [0.475, 0.096, 0.556] m, minimum clearance 54 to 82 mm, measured joints within 0.02 rad of the commands;
- every run then ended in the fail-safe hold of `CaptureTransfer` at the closure check against the virtual
  object model, exactly as the nine two-robot runs of 18 July did: with nothing between the fingers there is
  no contact or force signal, and `Retreat` is not reachable;
- one run lost its commit to a slow search (4.3 s against 2.7 to 3.2 s otherwise; a viewer was running on
  the laptop) and held without moving; one start with `Kortex.init_posture.on_startup: true` was rejected by
  the robot firmware and crashed the driver without motion (both logged in the record);
- the arm was homed between runs through the robot's own `Home` action (`two_robot/tools/kortex_home.cpp`);
  after the fail-safe hold the driver ignores SIGINT and SIGTERM and has to be killed, which the robot
  tolerates (it holds its pose; the next `Home` action recovers it).

This establishes the pre-contact sequence of the receiver on the physical arm for all four scenarios. It
establishes no contact, no load transfer, no retreat and no perception; the force path stayed on the
virtual sensor.

The evidence therefore supports physical two-robot operation and interaction, and the single-robot
pre-contact sequence on Robot A. What remains
unestablished is a synchronized, reproducible end-to-end TRIAD hardware validation in which the complete
FSM execution is tied to the physical run by controller logs and the required force/safety instrumentation.
