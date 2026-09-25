# Robot A alone on the physical arm, 25 September 2026: the four scenarios against the virtual object

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
  measured joints followed the commanded ones within 0.02 rad throughout (binary logs).
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
- Not a two-robot run: Robot B was absent; the two-robot record is `../hardware_runs_2026-07-18/`.

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

The 2 GB near-ground log is the ten minutes of hold before the driver was killed. `mc_bin_utils convert
--in <file> --out <name> --format csv --entries t Executor_Main qIn qOut` gives the state timeline and
the measured and commanded joints (semicolon-separated).
