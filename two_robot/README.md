# TRIAD with two robots — Robot A receives, Robot B gives

This folder makes TRIAD run a **robot-to-robot handover**: Robot A (Kinova Gen3 +
Robotiq 2F-85, the TRIAD receiver, unchanged) receives the CALL object from Robot B (a second Kinova Gen3 whose
tool has no role in the handover) which presents it along a fixed world-frame trajectory. It is the July 2026 two-robot
setup of the CALL laboratory (Robot A at 192.168.1.10, Robot B at 192.168.1.11).

What is in it:

| path | what |
|---|---|
| `../src/DualGiverCoordinator.{h,cpp}` | Robot B as an integrated giver inside the TRIAD controller: Prepositioning → StartSettling → Ready → Executing → TerminalSettling → Holding, with the object coupled to Robot B's tool until Robot A acquires it |
| `../src/states/HandoverInterceptionController_RobotBScenarioPreview.*` | Robot B alone executes the scenario while Robot A holds (the first thing to run on hardware) |
| `../src/states/HandoverInterceptionController_StaticXTouch.*` | a deliberately simple static two-arm rendezvous with no planning, for commissioning |
| `HandoverInterceptionController.two_robot.yaml` | the configuration overlay: second robot, giver scenario `pure_x`, object coupling, safety limits (values of the July sessions) |
| `run_two_robot_sim.sh` | runs the two-arm handover in mc_rtc's ticker straight from a build tree, without installing |
| `display_two_robot.rviz` | RViz display file with Robot A, Robot B and the object |
| `mc_rtc.two_kortex.yaml` | the global mc_rtc profile for two physical Kortex arms (credentials are placeholders) |
| `mc_kortex_patch/` | the three mc_kortex source files that ran the July sessions: per-robot joint maps for two arms, exception-safe shutdown, and a gated fixed-joint override for Robot B that is off unless `CALL_PHYSICAL_ROBOT_B_FIXED_JOINTS=1` (`SOURCE_AUDIT.md`) |
| `prepare_hardware_config.sh`, `check_dual_network.sh`, `run_dual_init_only.sh`, `disable_and_stop.sh`, `tools/set_override_key.py` | the hardware procedure: write the mc_rtc profile and override from the repository files, check the network, no-motion preflight, switch one override key, stop (escalating to SIGKILL, which the driver needs after the fail-safe hold) |
| `run_single_robot_scenario.sh`, `tools/kortex_home.cpp`, `tools/build_kortex_home.sh` | Robot A alone on hardware: home the arm through the robot's own `Home` action (Kortex API), then run one of the four scenarios against the virtual object and summarise the log; the record of 25 September 2026 is `evidence/hardware_runs_2026-09-25/` |
| `robot_b_standalone/` | `CALLRobotBFaceToFaceMover`: the alternative where Robot B runs from a second laptop with no communication with Robot A (fixed start, one trigger file `/tmp/call_robot_b_start`, one trajectory in Robot B's base frame); its `tools/install.sh` installs into that laptop's mc_rtc, which is the one place where an install is used |
| `results/sim_2026-09-24/` | the recorded runs: log, override and 20 ms timeline of `pure_x`, and the logs of `diagonal_xz` and `static_nominal` |
| `tools/extract_timeline.py` | turns a run's binary log into the 20 ms timeline (states, giver phase, object pose as carried and as planned with) |
| `evidence/` | inventory of the 134 July 2026 mc_rtc logs (31 GB, kept on the lab laptop), the 3.3 MB hardware `StaticXTouch` log of 17 July 22:27, the driver logs of the nine 18 July runs that reached capture (`hardware_runs_2026-07-18/`), Robot B's standalone validation log, and the physical-video evidence note |
| `media/` | five screen recordings of the two-arm simulation from 18–19 July 2026 (`.webm`) plus two real-world physical dual-robot videos (`dual_robot_physical_01.mp4`, `dual_robot_physical_02.mp4`) |

## What has been verified

- **Simulation**: full two-arm handover completed. Robot B prepositions, settles at the start, waits Ready,
  executes the presentation in sync with Robot A's object observation, settles at the terminal gate and holds.
  Robot A observes the carried object (it does not simulate its own: `movingObject.simulateMotion: false` in
  the overlay), evaluates its complete grasp × route bank once at Robot B's fixed endpoint and time
  (`SynchronizedPresentationSolve` in the log), commits, and goes ExecuteCommittedReach → PresentationHold →
  MovePregrasp → CaptureTransfer → Retreat → Completed; Robot B releases the object at acquisition and Robot A
  carries it away. See `results/sim_2026-09-24/TIMELINE.md`. The single-robot scenarios complete on the same build.
- **Hardware, July 2026 — controller logs** (`evidence/JULY_2026_HARDWARE_LOG_INVENTORY.md`,
  `evidence/hardware_runs_2026-07-18/`): on 17 July Robot B executed its presentation alone on the physical
  arm (Prepositioning → StartSettling → Ready → Executing → TerminalSettling → Holding, log of 22:30). On
  18 July nine runs on the two physical arms went Initial → ObserveObject → SolveInterception →
  ExecuteCommittedReach → PresentationHold → MovePregrasp → CaptureTransfer with the physical gripper bridge
  enabled and closed the gripper on the object. That was the objective of those sessions: the object was
  taped to Robot B's tool, so no transfer and no retreat were attempted by design. After the closure each run
  ended in the fail-safe hold inside `CaptureTransfer`, at the closure check against the virtual object model
  (the controller has no physical contact or force signal), as expected in that setup. In the first four of those runs Robot B was driven by a
  fixed-joint override in the Kortex driver; in the last five by the giver coordinator's references, which
  is this repository's configuration. Robot A alone had its physical gripper commissioned on 15–16 July.
- **Hardware, 25 September 2026 — Robot A alone, the four scenarios** (`evidence/hardware_runs_2026-09-25/`):
  with the published sources and `prepare_hardware_config.sh --single`, the physical Robot A ran
  `longitudinal` (three times), `near-ground`, `lateral-low` and `diagonal` against the virtual object.
  Every run whose search committed went Initial → ObserveObject → SolveInterception →
  ExecuteCommittedReach → PresentationHold → MovePregrasp → CaptureTransfer on the physical arm and closed the
  physical gripper at the planned capture pose (reach targets from [0.163, 0.195, 0.166] to
  [0.475, 0.096, 0.556] m, clearance 54–82 mm, joints tracking the commands within 0.02 rad), then ended in
  the fail-safe hold at the closure check, as there was nothing between the fingers. One run lost its
  commit to a slow search (4.3 s) and held without moving; one start with the driver's own start-posture
  option was rejected by the robot and crashed the driver without motion.
- **Hardware, real-world video evidence**: `media/dual_robot_physical_01.mp4` and
  `media/dual_robot_physical_02.mp4` directly show both physical Kinova arms operating together in the lab
  handover setup. In the second clip Robot B supports/presents the bottle while Robot A's Robotiq gripper
  approaches and closes on the bottle neck. See
  `evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md` for source hashes and the evidence boundary.

## Hardware evidence and remaining limits

- Real-world video of the two physical robots **does exist** and is included in `media/`.
- The physical videos establish dual-robot operation and interaction, but they are not synchronized controller
  logs; the driver logs of the same day place the gripper closure in `CaptureTransfer`, the videos are not
  used on their own to assign a state.
- The July runs used the same coordinator and states, then inside a sandbox controller; their integration
  into this controller is verified end-to-end in simulation (`results/sim_2026-09-24/`).
- Robot B's tool has no role in the handover: it carries the object in simulation and, on hardware, the
  object was held by Robot B's tool physically.

## Run it in simulation

```bash
# build this branch (see ../docs/quickstart.md), then:
TRIAD_BUILD_DIR=$PWD/build \
MAIN_ROBOT_MODULE_PATH=$PWD/gen3_2f85_module \
MC_RTC_INSTALL=$HOME/mc_rtc_ws/install \
bash two_robot/run_two_robot_sim.sh 80
```

`TRIAD_GIVER_SCENARIO` selects Robot B's presentation (default `pure_x`); the runner moves the object's start
pose to the scenario's start. All three complete in simulation with no discontinuity in the object pose:

| scenario | Robot B presents | Robot A |
|---|---|---|
| `pure_x` | straight toward Robot A along −x, 0.08 m/s, from [0.92, 0, 0.55] to [0.55, 0, 0.55] m | commits to the endpoint, captures at 14.3 s, completes at 18.6 s (`results/sim_2026-09-24/two_robot_sim.log`) |
| `diagonal_xz` | forward and upward, from [0.90, 0, 0.30] to [0.62, 0, 0.58] m | captures at 14.6 s, completes at 18.8 s (`results/sim_2026-09-24/diagonal_xz.log.xz`) |
| `static_nominal` | holds the object still at [0.55, 0, 0.55] m | plans at the current pose, captures at 12.6 s, completes at 16.9 s (`results/sim_2026-09-24/static_nominal.log.xz`) |

```bash
TRIAD_GIVER_SCENARIO=diagonal_xz bash two_robot/run_two_robot_sim.sh 80
TRIAD_SYNC_RATIO=0.5 bash two_robot/run_two_robot_sim.sh 80     # half speed, for watching in a viewer
TRIAD_INIT_STATE=HandoverInterceptionController_RobotBScenarioPreview bash two_robot/run_two_robot_sim.sh 30   # Robot B alone, Robot A holds
```

The Kinova robot module (`Kinova`) must be installed in `MC_RTC_INSTALL`; it comes from
[mc_kinova](https://github.com/mathieu-celerier/mc_kinova) (the laboratory copy has the same layout and
registers `Kinova`, `KinovaDefault`, `KinovaCallib`, `KinovaBota`, `KinovaBotaDS4`). Build and install it
following its own instructions; it needs `xacro` and `kortex_description`. Robot A's `gen3_2f85` module is
the one built in `../docs/quickstart.md` §3. The script
writes a scratch mc_rtc profile, points `ControllerModulePaths` and `StatesLibraries` at the build tree,
appends the overlay to the controller configuration and prints the giver milestones and the receiver
states. Expect `RESULT: COMPLETED` after about 20 s of simulated time. mc_rtc's ticker segfaults on exit
after `--run-for`; that happens after Completed and is not part of the handover.

To watch it, open RViz with `display_two_robot.rviz` from this folder before starting the script (it shows
Robot A, Robot B and the object; `../docs/quickstart.md` §7 has the commands and a screenshot,
`media/rviz_two_robot.png`).
The July simulation recordings and the two physical laboratory clips are in `media/`.

## Run it on the two physical arms

The controller is driven by `mc_kortex` (mc_rtc's Kortex interface) instead of the ticker. Prerequisites:
mc_kortex rebuilt with the three files in `mc_kortex_patch/` (Robot A `gen3_joint_1..7` and Robot B
`joint_1..7` mapped independently), the `Kinova` module for Robot B, and the receiver's gripper values in
`HandoverInterceptionController.hardware_receiver.yaml` (July 2026 calibration: `openPercent` 0.87,
`closePercent`/`maxPercent` 50.37; measure your own). Motion stays disabled until each step passes.

```bash
# with the four variables of the README set (PATH, MAIN_ROBOT_MODULE_PATH, TRIAD_BUILD_DIR, MC_RTC_INSTALL)
bash two_robot/prepare_hardware_config.sh      # writes ~/.config/mc_rtc/mc_rtc.yaml + the controller override
                                               # from mc_rtc.two_kortex.yaml, the two-robot overlay and the
                                               # receiver hardware overlay; backs up existing files.
#   -> edit ~/.config/mc_rtc/mc_rtc.yaml: Kortex username/password, IPs (A 192.168.1.10, B 192.168.1.11)
bash two_robot/check_dual_network.sh           # laptop 192.168.1.12, both arms reachable, distinct MACs (needs nc, sudo;
                                               #  IFACE=<nic> LAPTOP_IP=… ROBOT_A_IP=… ROBOT_B_IP=… override the defaults)
bash two_robot/run_dual_init_only.sh           # mc_kortex --init-only: connect, read, initialise, disconnect

# Robot A's gripper alone, arm frozen (a switch of the Initial state)
python3 two_robot/tools/set_override_key.py configs.HandoverInterceptionController_Initial.hardwareGripperCommissioning.enabled true
mc_kortex
python3 two_robot/tools/set_override_key.py configs.HandoverInterceptionController_Initial.hardwareGripperCommissioning.enabled false

# Robot B alone presents, Robot A holds (what ran on 17 July)
python3 two_robot/tools/set_override_key.py init HandoverInterceptionController_RobotBScenarioPreview
python3 two_robot/tools/set_override_key.py dualHandover.motionEnabled true
mc_kortex

# the handover (disable_and_stop.sh switches Robot B's motion off after every run)
python3 two_robot/tools/set_override_key.py init HandoverInterceptionController_Initial
python3 two_robot/tools/set_override_key.py dualHandover.motionEnabled true
mc_kortex

bash two_robot/disable_and_stop.sh             # after every run: stops the driver, motion off
```

`set_override_key.py` edits one key of `~/.config/mc_rtc/controllers/HandoverInterceptionController.yaml`
in place. Hardware logs go to `~/TRIAD_hardware_logs` (`TRIAD_HARDWARE_LOG_DIR`). What has and has
not been reached on hardware is stated in `../docs/real_robot.md`.

## Robot A alone: the four scenarios on the physical arm

`prepare_hardware_config.sh --single` writes the configuration without Robot B (giver disabled, the
virtual object of the single-robot scenarios). This is the configuration of the gripper smoke test and of
the runs of 25 September 2026 (`evidence/hardware_runs_2026-09-25/README.md`).

```bash
bash two_robot/prepare_hardware_config.sh --single   # then credentials into ~/.config/mc_rtc/mc_rtc.yaml
bash two_robot/tools/build_kortex_home.sh            # once; KORTEX_ROOT_DIR points at mc_kortex's kortex_api/2.6.0
TRIAD_BUILD_DIR=$PWD/build MC_RTC_INSTALL=$HOME/mc_rtc_ws/install \
bash two_robot/run_single_robot_scenario.sh longitudinal     # or near-ground, lateral-low, diagonal
```

The runner sets the scenario's object start and velocity in the override, sends the arm to the robot's
stored `Home` action (`build/kortex_home`, credentials read from `mc_rtc.yaml`), starts `mc_kortex`, waits
for the terminal outcome, stops the driver and prints the state sequence, the search time, the committed
plan, the reach target with its clearance and the outcome. Expect every run to end in the fail-safe hold
at the closure check; with no object there is no contact signal. Two driver behaviours: keep
`Kortex.init_posture.on_startup: false` (the robot firmware rejects the driver's waypoint and the driver
crashes at control-loop start), and after the hold the driver no longer reacts to SIGINT or SIGTERM, so
the runner and `disable_and_stop.sh` escalate to SIGKILL. The plan search runs in real time while the
virtual object approaches: a search above about 4 s loses the commit (`no_final_timing_admissible_time_plan`,
no motion) and the runner retries once; keep the laptop idle, without a viewer, during a run.

## Provenance

- Giver coordinator, states, scripts and Kortex patch: the laboratory's sandbox controller and the 17 July
  package of the second laptop (17–18 July 2026).
- Robot B standalone mover: `~/mc_rtc_ws/Sandbox/CALLRobotBFaceToFaceMover`, validated 17 July 11:38
  (`evidence/CALL_ROBOT_B_20260717_113843.log`, scenario `canonical_yz` in Robot B's base frame: start
  [0.4567, 0.0010, 0.4337], 0.08 m/s, terminal error ≈ 0.2 mm).
- Physical video evidence: `media/dual_robot_physical_01.mp4`, `media/dual_robot_physical_02.mp4`, with
  source-file hashes recorded in `evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md`.
- Integration in the controller: constructor, reset, run and observation-start hooks, eight pass-through
  methods, two states, CMake entries. All are no-ops unless `dualHandover.enabled: true`.
