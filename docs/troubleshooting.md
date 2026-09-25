# Troubleshooting

## CMake configure fails or hangs with a ROS 2 / rclcpp error

mc_rtc's exported CMake configuration optionally probes for ROS 2
(`find_package(rclcpp QUIET)`) whenever the `ROS_VERSION` environment
variable is unset or set to `2`. If a ROS 2 distribution is on your `PATH`
(for example because your shell profile sources `/opt/ros/<distro>/setup.bash`),
CMake can pick it up as a search prefix even without `AMENT_PREFIX_PATH` set,
and the probe can fail deep inside `rosidl_typesupport_c` if the relevant ROS 2
message packages are not actually built in the workspace. This is unrelated to
TRIAD; disable the probe explicitly:

```bash
cmake -S . -B build -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON ...
```

It is also useful to unset `AMENT_PREFIX_PATH`, `COLCON_PREFIX_PATH` and
`ROS_PACKAGE_PATH` for the configure step if the shell sets them:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
  CMAKE_PREFIX_PATH=/path/to/your/mc_rtc/install \
  cmake -S . -B build -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON ...
```

## `colcon build --packages-select ...` fails

Without a `package.xml`, colcon identifies this package by its **clone
directory name**, not by its CMake `project()` name. Consequently,
`--packages-select HandoverInterceptionController` will select nothing.

A colcon workspace that also contains ROS 2 packages can additionally fail with
an error such as:

```
Check that the following packages have been built:
- eigen-fmt
```

`eigen-fmt` is bundled inside mc_rtc's third-party CMake find-module rather than
being a standalone colcon package with an install marker. This is a
workspace/dependency-graph issue in colcon package discovery, not a TRIAD
controller defect. Building with plain CMake, as described in [Quick start](quickstart.md),
is the supported path.

## `global_event_prediction_drift` with a viewer attached

The search runs on a background worker in real time. A viewer connected to the
controller (RViz, mc-rtc-magnum) lengthens it: for `lateral-low`, 3.9 s in the
reference log against 5.0 s with RViz attached (`workerWall` in
`evidence/reference_runs/lateral-low.log.xz` and
`lateral-low_viewer_attached.log.xz`). The simulated object reaches its 0.40 m
travel cap about 4 s after the search epoch, so the longer search fails the
pre-commit consistency check. Run the simulation slower than real time while
watching: `TRIAD_SYNC_RATIO=0.5` (both runners). The run then completes, but
the committed plan can differ from the full-speed reference because timing
admission is evaluated at the simulated time the search returns.

## Install always goes into the mc_rtc installation

The supported way to run TRIAD is from the build tree (`TRIAD_BUILD_DIR`, see
[Quick start](quickstart.md)); installation is not needed and not recommended.
`cmake --install build` ignores `-DCMAKE_INSTALL_PREFIX` for this project.
mc_rtc's exported CMake macros compute the controller install location
(`MC_RTC_LIBDIR`/`MC_CONTROLLER_RUNTIME_INSTALL_PREFIX`) from the mc_rtc
installation because that is where its plugin loader looks.

An mc_rtc controller plugin therefore cannot be treated as an ordinary
standalone library installed into an unrelated prefix unless the corresponding
mc_rtc runtime is also configured to use that installation.

## Stale installed files masking a source change (only if you ever ran `cmake --install build`)

Because installation targets the shared mc_rtc installation, an older build's
`.so` files or FSM state data can remain if a later build stops partway through.
If stale installed code is suspected:

```bash
bash tools/clean_stale_fsm_install.sh --mc-rtc-prefix /path/to/your/mc_rtc/install
```

This removes stale FSM state-data copies only, from the supplied mc_rtc prefix
(or the `MC_RTC_PREFIX` environment variable). Then rebuild and reinstall from
a clean `build/` directory.

To check which `gen3_2f85` robot-module directory is selected, inspect
`MAIN_ROBOT_MODULE_PATH` and the active mc_rtc configuration. See
[Robot module](robot_module.md).

## `mc_rtc_ticker` exit status after a terminal outcome

`mc_rtc_ticker` does not necessarily exit on its own after a scenario reaches a
terminal outcome (`[Completed]` or an FSM failure state). Both
`scripts/run_scenario.sh` and `scripts/reproduce_latency_matrix.sh` terminate it
(`SIGTERM`, then `SIGKILL` after a short grace period if necessary) once the
terminal outcome is logged. This intentional termination can produce a non-zero
process exit status, commonly `137`; that status alone is not a scientific
pass/fail criterion.

The wrappers record `TICKER_STOP_REASON` to distinguish the cases:

- `WRAPPER_TERMINATED` / `WRAPPER_TERMINATED_AFTER_TIMEOUT` — the wrapper ended
  the process after the run outcome;
- `SPONTANEOUS_EXIT_AFTER_TERMINAL_OUTCOME` /
  `SPONTANEOUS_EXIT_BEFORE_TERMINAL_OUTCOME` — the process had already exited
  before the wrapper attempted to stop it.

A spontaneous-exit case has been observed in captured runs, but its cause has
not been isolated. When a terminal outcome is already present, the captured log
can still be evaluated independently by `tools/check_global_time_plan_log.py`
or `tools/check_latency_log.py`.

To diagnose such a case, compare the wrapper invocation with a direct
`mc_rtc_ticker` run using the installed default configuration and no temporary
`HOME`/scenario override. This separates controller/ticker behavior from the
override mechanism.

The wrappers therefore report process termination and runtime verification as
separate quantities: `TICKER_STOP_REASON`, `TICKER_EXIT_STATUS`, and the
applicable checker result are recorded independently.

## `gen3_2f85` robot-module dependency

TRIAD depends on an external Kinova Gen3 + Robotiq 2F-85 mc_rtc robot-module
description directory (URDF, meshes, convex hull data), referenced through
`MAIN_ROBOT_MODULE_PATH`.

The verified model provenance is a camera-subtree-trimmed derivative of
`kortex_description/robots/gen3_2f85.urdf` from
`Kinovarobotics/ros2_kortex` `0.2.6`, combined with
`robotiq_description` `0.0.1` meshes. See [Robot module](robot_module.md) for
the pinned upstream hash, transformation rules, mesh manifest, and equivalence
evidence.

For a fresh checkout, reconstruct the module deterministically from the pinned
upstream packages with `scripts/setup_gen3_2f85_module.py` before running a
simulation or hardware-preflight procedure. The reconstruction steps are given
in [Quick start](quickstart.md) and [Robot module](robot_module.md).

## `mc_kortex` does not stop after the fail-safe hold

After a hardware run ends in `Failure` (the fail-safe hold), the driver no longer reacts to SIGINT or
SIGTERM; in one run it also flooded `Full ROS message publishing queue` until it was killed (25 September
2026, [`two_robot/evidence/hardware_runs_2026-09-25/`](../two_robot/evidence/hardware_runs_2026-09-25/README.md)).
`two_robot/disable_and_stop.sh` and `two_robot/run_single_robot_scenario.sh` escalate to SIGKILL after a
bounded wait. The robot holds its pose when the session drops; the next `Home` action
(`two_robot/tools/kortex_home.cpp`) switches it back from low-level servoing and moves it.

## `Kortex.init_posture.on_startup: true` is rejected and the driver crashes

The robot firmware answers the driver's start-posture waypoint with
`TRAJECTORY_ERROR_TYPE_INVALID_DURATION: Time optimal splines are not supported`; the driver logs
`Error found in trajectory to initial position`, goes on into its control loop and segfaults within a second
(no motion). Keep the option `false` and home the arm with `two_robot/tools/kortex_home.cpp` (the robot's
own stored `Home` action) before a run.

## `no_final_timing_admissible_time_plan` on hardware

The plan search runs in real time while the virtual object approaches. On the laboratory laptop it takes
2.7 to 3.2 s when idle; at 4.3 s (a viewer running) no plan kept the 1.6 s commit lead and the run ended in
`Failure` from `SolveInterception` without moving. Same cause as the viewer-attached case above. Close the
viewer and rerun; `run_single_robot_scenario.sh` retries once by itself.

## `mc_kortex: error while loading shared libraries: libament_index_cpp.so`

The driver links mc_rtc's ROS plugin. Start it from a shell where the ROS environment is sourced
(`source /opt/ros/<distro>/setup.bash`) and extend `LD_LIBRARY_PATH` with the build tree and the mc_rtc
install rather than replacing it. A driver that dies this way has not touched the robots.
