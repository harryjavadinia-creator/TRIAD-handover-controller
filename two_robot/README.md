# TRIAD with two robots — Robot A receives, Robot B gives

This folder makes TRIAD run a **robot-to-robot handover**: Robot A (Kinova Gen3 +
Robotiq 2F-85, the TRIAD receiver, unchanged) receives the CALL object from Robot B (a second Kinova Gen3,
no gripper role) which presents it along a fixed world-frame trajectory. It is the July 2026 two-robot
setup of the CALL laboratory (Robot A at 192.168.1.10, Robot B at 192.168.1.11).

What is in it:

| path | what |
|---|---|
| `../src/DualGiverCoordinator.{h,cpp}` | Robot B as an integrated giver inside the TRIAD controller: preposition → READY → present → terminal gate → HOLD, with the object rigidly coupled to Robot B's tool until Robot A acquires it |
| `../src/states/HandoverInterceptionController_RobotBScenarioPreview.*` | Robot B alone executes the scenario while Robot A holds (the first thing to run on hardware) |
| `../src/states/HandoverInterceptionController_StaticXTouch.*` | a deliberately simple static two-arm rendezvous with no planning, for commissioning |
| `HandoverInterceptionController.two_robot.yaml` | the configuration overlay: second robot, giver scenario `pure_x`, object coupling, safety limits (values of the July sessions) |
| `run_two_robot_sim.sh` | runs the two-arm handover in mc_rtc's ticker straight from a build tree, without installing |
| `display_two_robot.rviz` | RViz display file with Robot A, Robot B and the object |
| `mc_rtc.two_kortex.yaml` | the global mc_rtc profile for two physical Kortex arms (credentials are placeholders) |
| `mc_kortex_patch/` | the three mc_kortex source files that map Robot A (`gen3_joint_1..7`) and Robot B (`joint_1..7`) independently, with its source audit |
| `check_dual_network.sh`, `run_dual_init_only.sh`, `disable_and_stop.sh` | the hardware procedure scripts of July 2026 |
| `robot_b_standalone/` | `CALLRobotBFaceToFaceMover`: the alternative where Robot B runs from a second laptop with no communication with Robot A (fixed start, one trigger, one trajectory) |
| `results/sim_2026-09-24/` | the verification run: log, override, timeline |
| `evidence/` | inventory of the 134 July 2026 mc_rtc logs (31 GB, kept on the lab laptop), one 3.3 MB hardware log, Robot B's validation log, and the physical-video evidence note |
| `media/` | five screen recordings of the two-arm simulation from 18–19 July 2026 (`.webm`) plus two real-world physical dual-robot videos (`dual_robot_physical_01.mp4`, `dual_robot_physical_02.mp4`) |

## What has been verified

- **Simulation, 2026-09-24, this port**: full two-arm handover completed. Robot B prepositions, settles at the
  start, waits Ready, executes the presentation in sync with Robot A's object observation, settles at the
  terminal gate and holds; Robot A goes Initial → ObserveObject → SolveInterception → ExecuteCommittedReach →
  PresentationHold → MovePregrasp → CaptureTransfer → Retreat → Completed and Robot B releases the object
  at acquisition. See `results/sim_2026-09-24/TIMELINE.md`. The single-robot scenario on the same build
  still completes, and so does a run with the second robot loaded but the giver disabled.
- **Hardware, July 2026 — controller logs** (see `evidence/JULY_2026_HARDWARE_LOG_INVENTORY.md`): on 17 July Robot B executed
  its presentation alone on the physical arm (phases prepositioning → HOLD, log 22:30:31). The inventoried
  combined run of 18 July 00:51 reached Robot A's committed reach and then failed; the inventoried mc_rtc
  state logs do not record `CaptureTransfer`. Robot A alone had its physical gripper commissioned on
  15–16 July (`v6.4.2`).
- **Hardware, real-world video evidence**: `media/dual_robot_physical_01.mp4` and
  `media/dual_robot_physical_02.mp4` directly show both physical Kinova arms operating together in the lab
  handover setup. In the second clip Robot B supports/presents the bottle while Robot A's Robotiq gripper
  approaches and closes around the bottle neck. See
  `evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md` for source hashes and the evidence boundary.

## Hardware evidence and remaining limits

- Real-world video of the two physical robots **does exist** and is included in `media/`.
- The physical videos establish dual-robot operation and interaction, but they are not synchronized controller
  logs; therefore they are not used on their own to assign a particular FSM state such as `CaptureTransfer`.
- The July hardware runs predate the integration in this repository. The current integrated giver layer is
  verified end-to-end in simulation; the physical clips document the July laboratory implementation/setup.
- Robot B has no gripper role: it carries the object rigidly in simulation and, on hardware, the object
  was held by Robot B's tool physically.

## Run it in simulation

```bash
# build this branch (see ../docs/quickstart.md), then:
TRIAD_BUILD_DIR=/path/to/build \
MAIN_ROBOT_MODULE_PATH=/path/to/kinova_gen3_2f85_mcdesc \
MC_RTC_INSTALL=$HOME/mc_rtc_ws/install \
bash two_robot/run_two_robot_sim.sh 80
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

Follow the July procedure, in this order, with motion disabled until each step passes:

1. Apply `mc_kortex_patch/` to your mc_kortex checkout and rebuild it (per-robot joint maps).
2. Copy `mc_rtc.two_kortex.yaml` to `~/.config/mc_rtc/mc_rtc.yaml`, fill in the IPs and credentials.
3. `bash two_robot/check_dual_network.sh` — both arms reachable.
4. `bash two_robot/run_dual_init_only.sh` — both robots initialise, no motion; the log must show separate
   states for `gen3_2f85` and `kinova`.
5. Robot B alone: set `init: HandoverInterceptionController_RobotBScenarioPreview` in the overlay, then
   `dualHandover.motionEnabled: true`. This is what worked on 17 July.
6. The combined run: default `init`, `motionEnabled: true`. On 18 July the inventoried logged run reached the committed reach.
7. `bash two_robot/disable_and_stop.sh` after every run.

Robot A's physical gripper bridge stays under the existing `physicalBridge` switches of the TRIAD
configuration; they are off by default.

## Provenance

- Giver coordinator, states, scripts and Kortex patch: `~/mc_rtc_ws/Sandbox/CALLDualRobotHandoverController`
  and `~/Downloads/CALL_DUAL_ROBOT_ACTUAL_HANDOVER_V1_1_PLAIN_TRAJECTORY_B_20260717` (17–18 July 2026).
- Robot B standalone mover: `~/mc_rtc_ws/Sandbox/CALLRobotBFaceToFaceMover`, validated 17 July 11:38
  (`evidence/CALL_ROBOT_B_20260717_113843.log`: start [0.4567, 0.0010, 0.4337], 0.08 m/s, terminal error ≈ 0.2 mm).
- Physical video evidence: `media/dual_robot_physical_01.mp4`, `media/dual_robot_physical_02.mp4`, with
  source-file hashes recorded in `evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md`.
- Integration in the controller: constructor, reset, run and observation-start hooks, eight pass-through
  methods, two states, CMake entries. All are no-ops unless `dualHandover.enabled: true`.
