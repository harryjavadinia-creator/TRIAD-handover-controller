# Troubleshooting

## CMake configure fails or hangs with a ROS 2 / rclcpp error

mc_rtc's exported CMake configuration optionally probes for ROS 2
(`find_package(rclcpp QUIET)`) whenever the `ROS_VERSION` environment
variable is unset or set to `2`. If a ROS 2 distribution happens to be on
your `PATH` (for example because your shell profile sources
`/opt/ros/<distro>/setup.bash`), CMake can pick it up as a search prefix
even without `AMENT_PREFIX_PATH` set, and the probe can fail hard deep
inside `rosidl_typesupport_c` if the relevant ROS 2 message packages are not
actually built in your workspace. This is unrelated to this controller;
disable the probe explicitly:

```bash
cmake -S . -B build -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON ...
```

It is also worth unsetting `AMENT_PREFIX_PATH`, `COLCON_PREFIX_PATH` and
`ROS_PACKAGE_PATH` for the configure step if your shell sets them, to avoid
unrelated packages leaking into the search path:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
  CMAKE_PREFIX_PATH=/path/to/your/mc_rtc/install \
  cmake -S . -B build -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON ...
```

## `colcon build --packages-select ...` fails

Without a `package.xml`, colcon identifies this package by its **clone
directory name**, not by its CMake `project()` name — `--packages-select
HandoverInterceptionController` will select nothing. Even with the correct
name, colcon build has been observed to fail in a workspace that also
contains ROS 2 packages, with an error such as:

```
Check that the following packages have been built:
- eigen-fmt
```

This happens because `eigen-fmt` is bundled inside mc_rtc's own 3rd-party
CMake find-module, not a real colcon package with an install marker — this
is a workspace/dependency-graph issue in colcon's package discovery, not a
defect in this repository. Building with plain CMake (see `README.md`) is
the verified path; colcon is not currently supported for this package.

## Install always goes into your mc_rtc installation

`cmake --install build` ignores `-DCMAKE_INSTALL_PREFIX` for this project.
mc_rtc's exported CMake macros compute the controller's install location
(`MC_RTC_LIBDIR`/`MC_CONTROLLER_RUNTIME_INSTALL_PREFIX`) directly from where
mc_rtc itself is installed, because that is where mc_rtc's plugin loader
looks. This is expected, not a bug: there is no meaningful way to install an
mc_rtc controller plugin into an isolated prefix without also running an
isolated mc_rtc installation.

## Stale installed files masking a source change

Because installation always targets your shared mc_rtc installation, an old
build's `.so` files or FSM state data can persist there if a later build
fails partway through. If you suspect you are running stale installed code:

```bash
bash tools/clean_stale_fsm_install.sh --mc-rtc-prefix /path/to/your/mc_rtc/install
```

removes stale FSM state-data copies only, from the given mc_rtc install
prefix (or the `MC_RTC_PREFIX` environment variable — there is no
private-workspace default). Then rebuild and reinstall from a clean
`build/` directory.

To check which `gen3_2f85` robot-module directory is currently selected,
inspect `MAIN_ROBOT_MODULE_PATH` and your generated
`$HOME/.config/mc_rtc/mc_rtc.yaml` directly — see `docs/robot_module.md`.

## `mc_rtc_ticker` exit status after a terminal outcome

`mc_rtc_ticker` does not exit on its own after a scenario reaches a
terminal outcome (`[Completed]` or an FSM failure state); both
`scripts/run_scenario.sh` and `scripts/reproduce_latency_matrix.sh`
terminate it themselves (`SIGTERM`, then `SIGKILL` after a short grace
period if it has not exited) once that outcome is logged, so no manual
intervention is required. This intentional termination is why
`TICKER_EXIT_STATUS` is routinely non-zero (typically `137`, i.e.
`SIGKILL`) — that by itself is expected behavior, not a fault.

Both scripts record `TICKER_STOP_REASON` to distinguish this intentional
termination from a separate, unresolved issue:

- `WRAPPER_TERMINATED` / `WRAPPER_TERMINATED_AFTER_TIMEOUT` — the wrapper
  itself ended the process, as designed.
- `SPONTANEOUS_EXIT_AFTER_TERMINAL_OUTCOME` /
  `SPONTANEOUS_EXIT_BEFORE_TERMINAL_OUTCOME` — the process had already
  exited on its own before the wrapper attempted to stop it. This case has
  been observed on the development machine; its cause has not been
  isolated. In every case observed so far, the captured log up to that
  point is complete and, when a terminal outcome was already logged,
  passes `tools/check_global_time_plan_log.py` /
  `tools/check_latency_log.py`.

**What is not established**: whether a `SPONTANEOUS_EXIT_*` stop reason is
specific to the isolated-`HOME` scenario-override mechanism these scripts
use, or whether it also occurs with a plain, non-redirected `mc_rtc_ticker`
invocation. A direct comparison has not been run.

*Proposed check (not yet performed)*: run `mc_rtc_ticker` directly (no `-f`,
no `HOME` redirection, no wrapper script) against the installed default
configuration, to a natural terminal outcome, and observe whether it exits
on its own. This would isolate whether `SPONTANEOUS_EXIT_*` is a property
of `mc_rtc_ticker`/the controller itself versus something specific to the
override mechanism.

Regardless of root cause, both scripts treat process termination and
scientific runtime verification as independent results: they report
`TICKER_STOP_REASON`, `TICKER_EXIT_STATUS` and the applicable checker
result(s) separately, print a visible note when `TICKER_STOP_REASON` is a
`SPONTANEOUS_EXIT_*` value, and never let `TICKER_EXIT_STATUS` by itself
stand in for pass/fail.

## `gen3_2f85` robot-module dependency

This repository depends on an external Kinova Gen3 + Robotiq 2F-85 mc_rtc
robot-module description directory (URDF, convex hulls, RSDF), referenced
as `MAIN_ROBOT_MODULE_PATH` in `configs/mc_rtc.yaml.example` and
`scripts/run_scenario.sh`.

On the development machine this existed as a plain, locally-flattened and
locally-trimmed directory with no version-control history of its own — not
redistributed by this repository. Its provenance has since been traced and
verified: it is a locally-flattened, camera-subtree-trimmed derivative of
`kortex_description/robots/gen3_2f85.urdf` from `Kinovarobotics/ros2_kortex`
`0.2.6`, combined with `robotiq_description` `0.0.1` meshes. See
`docs/robot_module.md` for the full provenance chain, the pinned upstream
hash, and the equivalence evidence (identical 19-link/18-joint model,
identical joint limits/inertials/geometry, identical Dataset B diagonal
selector result).

For a public checkout, do not attempt to recover or reuse that original
local directory — instead reconstruct the module deterministically from the
pinned upstream packages with `scripts/setup_gen3_2f85_module.py`, as
documented in `docs/robot_module.md` and in `README.md`'s quick-start
sequence, before running any simulation or hardware scenario.
