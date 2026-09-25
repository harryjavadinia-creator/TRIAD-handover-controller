# 25 September 2026 on the physical arms: Robot A alone (four scenarios), then the two-robot handover

Two sessions in one day with the published code: in the morning Robot A alone ran the four reported
scenarios against the virtual object (first part of this record); at 11:11–11:23 both arms ran the
robot-to-robot procedure of README §4, Robot B presenting the object trajectory and Robot A receiving
(second part, [below](#two-robots-1111-1123-robot-b-presents-robot-a-receives)).

# Part 1. Robot A alone: the four scenarios against the virtual object

These are the `mc_kortex` text logs (the driver's stdout, which carries the controller's log) of the
session of 25 September 2026 in which the physical Robot A (Kinova Gen3 + Robotiq 2F-85 at 192.168.1.10)
ran the four reported TRIAD scenarios alone. Robot B was not on the network. The object was the
**virtual** one of the single-robot scenarios (`movingObject.simulateMotion: true`, the default): the
controller observed it, planned, reached and closed the physical gripper at the planned capture pose, with
nothing between the fingers. The objective of the session was **observe, plan, reach and close on the
physical arm with the published code**; no transfer and no retreat were possible without an object.

Configuration: `../../prepare_hardware_config.sh --single` (Robot B removed, giver disabled), the receiver
hardware overlay (`allowPhysicalExecution: true`, physical gripper bridge with command and feedback,
16 July calibration), `transfer.source: virtual_sensor`. The two files as written for the session are
`mc_rtc_single.yaml` (credentials and paths replaced by placeholders) and `controller_override_single.yaml`
(longitudinal values; the runner sets `robots.call_object.init_pos.translation`, `object.translation` and
`movingObject.simulatedLinearVelocity` per scenario). Controller built from main at ba69cec
(`triad-2026-09-24`) plus the hardware tooling added with this record; driver: the mc_kortex build of
18 July 2026 (`../../mc_kortex_patch/`). Home paths in the logs are replaced by `<repository>`,
`<mc_rtc-install>`, `<log-directory>` and `<home>`.

## Runs

| time | run | started from | Robot A states | search | committed plan | reach target [m], clearance | ended with | log |
|---|---|---|---|---|---|---|---|---|
| 09:58 | no-motion preflight `mc_kortex --init-only` | Home | (none: connect, read state, initialise, disconnect) | | | | `headless no-motion preflight PASS`; gripper feedback 0.87 %, force sensor read, then the known post-shutdown segfault | `init_only_20260925_095858` |
| 10:00 | gripper smoke test (`hardwareGripperCommissioning`, arm frozen) | Home | Initial only | | | | `PASS target=5.00% finalMeasured=0.87%`, gripper 0.87 → 5.65 → 0.87 %, arm drift 0.00000 rad | `gripper_smoke_20260925_100007` |
| 10:01 | `longitudinal` | Home | Initial → ObserveObject → SolveInterception → ExecuteCommittedReach → PresentationHold → MovePregrasp → CaptureTransfer → Failure | 3.03 s, 10 feasible hypotheses | `axisP_side_337deg` / `direct`, lead 5.5 s, presentation 18.25 s | [0.355, 0.108, 0.486], 82 mm | `[Acquire] hard closure geometry violation` → fail-safe hold | `single_robot_longitudinal_20260925_100155` |
| 10:05 | `longitudinal` | Home (set by hand) | same eight states | 2.97 s, 8 | `axisP_side_337deg` / `ring80mm_1of8`, 5.5 s, 11.42 s | [0.355, 0.106, 0.482], 78 mm | closure check → fail-safe hold | `single_robot_longitudinal_20260925_100537` |
| 10:07 | `longitudinal` with `Kortex.init_posture.on_startup: true` | capture pose of 10:05 | Initial (one cycle) | | | | the robot rejected the driver's waypoint (`TRAJECTORY_ERROR_TYPE_INVALID_DURATION: Time optimal splines are not supported`); the driver went on into its control loop and crashed (segfault) within a second; **no motion** (measured joints unchanged in the binary log) | `single_robot_longitudinal_20260925_100726` |
| 10:10 | `longitudinal` | Home (Kortex `Home` action) | Initial → ObserveObject → SolveInterception → Failure | 4.30 s | none: `no_final_timing_admissible_time_plan` | | the search was slower than in the other runs (a ROS viewer was running on the laptop) and no plan kept the 1.6 s commit lead; **no motion** beyond Initial's readiness posture | `single_robot_longitudinal_20260925_101050` |
| 10:12 | `longitudinal` | readiness posture (left by 10:10) | same eight states | 2.97 s, 9 | `axisP_side_337deg` / `ring80mm_1of8`, 5.5 s, 6.75 s | [0.355, 0.106, 0.482], 80 mm | closure check → fail-safe hold | `single_robot_longitudinal_20260925_101212` |
| 10:14 | `near-ground` | Home (Kortex `Home` action) | same eight states | 3.22 s, 14 | `axisP_side_68deg` / `ring80mm_2of8`, 6.4 s, 12.32 s | [0.163, 0.195, 0.166], 63 mm (ground clearance 73 mm) | closure check → fail-safe hold; afterwards the driver flooded `Full ROS message publishing queue` (249 092 lines, collapsed in the record) and ignored SIGINT for ten minutes until it was killed | `single_robot_near-ground_20260925_101427` |
| 10:26 | `lateral-low` | Home (Kortex `Home` action) | same eight states | 4.24 s, 14 | `axisN_side_45deg` / `ring80mm_6of8`, 8.0 s, 13.92 s | [0.463, 0.033, 0.243], 54 mm (ground 87 mm) | closure check → fail-safe hold; driver killed after ignoring SIGINT/SIGTERM | `single_robot_lateral-low_20260925_102607` |
| 10:26 | `diagonal` | Home (Kortex `Home` action) | same eight states | 2.73 s, 7 | `axisP_side_337deg` / `ring80mm_1of8`, 5.5 s, 18.25 s | [0.475, 0.096, 0.556], 80 mm | closure check → fail-safe hold; driver killed after ignoring SIGINT/SIGTERM | `single_robot_diagonal_20260925_102656` |

"Search" is `elapsed` of `[GlobalTimePlanSearchSummary]`; "committed plan" is candidate / route, event
lead and presentation time of `[PresentationCommit] COMMITTED`; "reach target" and "clearance" are from
`[PredictiveReach] committed state entered … reach to [...]` and `[PredictiveReachGovernor] completed
committed reach minimumClearance=…`. Home is the Kinova Gen3 stored posture (0, 15, 180, 230, 0, 55, 90
degrees); the driver read it within 0.1 degree at every start.

## What the session established

- With the published sources and the single-robot hardware configuration, the physical arm executes the
  complete pre-contact sequence of every one of the four scenarios: readiness posture, observation of the
  moving object, one committed plan, the certified reach to the standoff, presentation hold, pregrasp and
  the closure of the physical gripper. Six of six runs whose search committed reached the closure. The
  measured joints followed the commanded ones within 0.03 rad throughout (`joints/`, see below).
- Every run ends the same way the nine runs of 18 July 2026 did (`../hardware_runs_2026-07-18/`): in the
  fail-safe hold of `CaptureTransfer` at the closure check against the virtual object model. With nothing
  between the fingers there is no contact or force signal; `Retreat` is not reachable in this setup.
- The planner chose different grasps and routes for the low and lateral cases, so the arm visited four
  distinct parts of the workspace, down to 0.17 m above the base plane.
- The plan search runs in real time while the virtual object approaches, so a slow search loses the
  commit: 2.7–3.2 s committed, 4.24 s committed with the largest lead, 4.30 s did not. This is the same
  sensitivity as the viewer-attached case of the simulation (`../../../docs/troubleshooting.md`). The
  runner retries once on that outcome; keep the laptop idle during a run.
- Two driver behaviours to know: `Kortex.init_posture.on_startup: true` is rejected by the robot firmware
  and crashes the driver (keep it `false`; `../../tools/kortex_home.cpp` homes the arm through the robot's
  own `Home` action instead), and after the fail-safe hold the driver no longer reacts to SIGINT or
  SIGTERM, so `../../disable_and_stop.sh` escalates to SIGKILL. The robot holds its pose when the session
  drops, and the next `Home` action recovers it (it also switches the arm back from low-level servoing).

## What it does not establish

- No contact, no load transfer, no retreat: there was no object. The force path stayed on the virtual
  sensor. The closure check that ends every run is the controller doing what it is specified to do
  without a contact signal, not a validated grasp.
- Not a human-to-robot handover: the object was simulated, there is no perception in this repository.
- Part 1 is not a two-robot run: Robot B was absent. The two-robot run of the same day is Part 2 below;
  the earlier two-robot record is `../hardware_runs_2026-07-18/`.

# Part 2. Two robots, 11:11–11:23: Robot B presents, Robot A receives

Configuration: `../../prepare_hardware_config.sh` (two Kortex arms, the two-robot overlay with giver
scenario `pure_x`, the receiver hardware overlay), as written for the session in `mc_rtc_two_robot.yaml`
(credentials and paths replaced by placeholders) and `controller_override_two_robot.yaml`. Robot A
(`gen3_2f85`, 192.168.1.10) is the receiver, Robot B (`kinova`, 192.168.1.11, base at [1.35, 0, 0] facing
Robot A) the giver driven by the giver coordinator's references, which is this repository's configuration.
There was no object on Robot B's tool: Robot A closed on the planned object pose, 0.156 m from Robot B's
tool tip. The two arms were started by hand from the laptop shell (the driver needs the ROS library path:
`source /opt/ros/jazzy/setup.bash`; an attempt without it died with `libament_index_cpp.so: cannot open`
and is not archived).

| time | run | Robot A states | giver phases | committed plan / reach | ended with | log |
|---|---|---|---|---|---|---|
| 11:11 | no-motion preflight, two arms | (none) | | | `headless no-motion preflight PASS: independently validated 2 physical robot state vector(s)` | `init_only_20260925_111130` |
| 11:15 | Robot B preview, first start | (none) | | | the driver died while opening the Kortex connection to Robot B (`not connected !!!`): the laptop's wired link had dropped (kernel: `NIC Link is Down` at 11:16 with three flaps); **no motion**; both web applications stayed reachable once the cable was seated | `two_robot_preview_20260925_111538` |
| 11:18 | no-motion preflight, two arms, after the cable | (none) | | | PASS again, both arms connected | `init_only_20260925_111833` |
| 11:19 | **Robot B alone presents** (`init: HandoverInterceptionController_RobotBScenarioPreview`, `motionEnabled: true`) | RobotBScenarioPreview (Robot A holds) | PREPOSITION START (0.22 m, 2.7 s) → START-GATE → READY (tracking 0.2 mm) → START → TERMINAL → HOLD (endpoint error 0.2 mm, peak tracking 4.9 mm) | | HOLD; Robot B's measured joints travelled up to 41° (joints 2, 4, 6), Robot A's 0° (binary log) | `two_robot_preview_20260925_111934` |
| 11:22 | **the handover** (`init: HandoverInterceptionController_Initial`, `motionEnabled: true`) | Initial (0 s) → ObserveObject (11.43 s) → SolveInterception (12.68 s) → ExecuteCommittedReach (13.09 s) → PresentationHold (16.83 s) → MovePregrasp (16.93 s) → CaptureTransfer (18.00 s) → Failure (19.87 s) | PREPOSITION START (0.37 m, 4.6 s) → READY → START synchronized with Robot A's observation → TERMINAL → HOLD (endpoint error 0.1 mm, peak tracking 5.0 mm) | `SynchronizedPresentationSolve`: one fixed event at Robot B's endpoint [0.55, 0, 0.55], one commit: `axisP_side_337deg` / `ring140mm_1of8`, presentation 16.83 s; reach to [0.463, 0.108, 0.486], clearance 80 mm | `[Acquire] hard closure geometry violation` → fail-safe hold, gripper at 41 %; Robot B's measured joints travelled up to 52° (joint 4), Robot A's up to 106° on joint 5 and a full revolution plus 52° on joint 3 (see the note under `joints/`), tracking error ≤ 0.019 rad (A) and ≤ 0.004 rad (B) | `two_robot_handover_20260925_112234` |

What this establishes beyond the nine runs of 18 July: the same sequence, from the same repository state, with
Robot B driven by the giver coordinator and every state tied to the binary log of the physical run
(measured and commanded joints of both arms in `joints/two_robot_handover_20260925_112234.joints.csv.xz`).
It ends where the July runs ended and for the same reason: no contact or force signal at closure, so no
transfer and no retreat. After the hold the driver again had to be killed (`../../disable_and_stop.sh`).

## Joint records (`joints/`) and how the quoted quantities regenerate

`joints/<run>.joints.csv.xz` is the joint record of each run whose numbers are quoted above and in the
paper: time, FSM state, measured (`qIn`) and commanded (`qOut`) joints of Robot A and, in the two-robot
runs, of Robot B (`kinova_qIn`, `kinova_qOut`), at the 1 kHz controller rate, semicolon-separated, cut
10 s after the terminal state. `../../tools/extract_hardware_joint_record.sh <run.bin> <out.csv.xz>`
produces it from a binary log;
`python3 two_robot/tools/hardware_run_summary.py joints/<run>.joints.csv.xz [--log <run>.mc_kortex.log.xz]`
prints the state timeline, the maximum joint travel from the first sample (measured joints, unwrapped),
the maximum |commanded − measured| tracking error and the giver phases. `joints/expected_quantities.json`
holds the values the record and the paper quote; `../../tools/check_hardware_run_records.sh` (run by CI)
recomputes every record against it.

| run | Robot A max travel (joint) | Robot A max tracking | Robot B max travel (joint) | Robot B max tracking |
|---|---|---|---|---|
| longitudinal 10:01 | 423° (3), 113° (5) | 0.020 rad | | |
| longitudinal 10:05 | 113° (5) | 0.021 rad | | |
| longitudinal 10:12 | 91° (5) | 0.021 rad | | |
| near-ground | 163° (5) | 0.021 rad | | |
| lateral-low | 121° (6) | 0.028 rad | | |
| diagonal | 418° (3), 93° (5) | 0.020 rad | | |
| Robot B preview | 0° | 0.000 rad | 41° (4) | 0.006 rad |
| the handover | 412° (3), 106° (5) | 0.019 rad | 52° (4) | 0.004 rad |

**Joint 3 note.** Robot A's third joint is continuous. When the driver's initial reading placed it at
−π (the Home posture is 180°, which the driver reports as +π or −π depending on the last fraction of a
degree), the readiness move of `Initial` drove it the long way round to its target at 2.74 rad, a rotation
of 337°, before the reach added another 86°; when the reading was +π, the same move was 21° the short way.
That is why three runs show a full revolution plus 52–63° on joint 3 while the others show 63–85°. The
reach, the capture pose and the closure were the same in both cases; the commanded and measured joint
agree within 0.03 rad throughout, so the revolution was executed, not a logging artefact.

## Binary logs (kept on the laboratory laptop, `<log-directory>/`)

| file | size |
|---|---|
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-00-08.bin` | 13 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-01-56.bin` | 72 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-05-38.bin` | 56 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-07-27.bin` | 0 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-10-51.bin` | 44 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-12-13.bin` | 46 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-14-28.bin` | 2046 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-26-08.bin` | 114 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-10-26-57.bin` | 122 MB |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-11-19-35.bin` | 149 MB (Robot B preview) |
| `TRIAD_hardware-HandoverInterceptionController-2026-09-25-11-22-36.bin` | 182 MB (the handover) |

The 2 GB near-ground log is the ten minutes of hold before the driver was killed. `mc_bin_utils convert
--in <file> --out <name> --format csv --entries t Executor_Main qIn qOut` gives the state timeline and
the measured and commanded joints (semicolon-separated).
