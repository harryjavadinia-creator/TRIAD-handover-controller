# Build and watch a simulation

This guide runs the TRIAD controller in mc_rtc's open-loop simulation. It
requires Linux and local robot-description dependencies. The
[result figures](results.md) can be viewed on GitHub without installing
anything.

## 1. Prepare the environment

The recorded development environment is Ubuntu 24.04 LTS, GCC 13.3,
Python 3.12, Eigen 3.4, and mc_rtc 2.14.0 (RBDyn 1.9.3, SpaceVecAlg 1.2.9,
Tasks 1.8.3, TVM 0.9.3; recorded CMake 4.3.1).

Install mc_rtc and its dependencies using the
[official installation guide](https://jrl.cnrs.fr/mc_rtc/tutorials/introduction/installation-guide.html).
Keep the actual installation prefix available. This repository does not bundle
mc_rtc or a ready-to-run system image.

Set the prefix to your installation:

```bash
export TRIAD_MC_RTC_PREFIX=/path/to/your/mc_rtc/install
export PATH="$TRIAD_MC_RTC_PREFIX/bin:$PATH"
command -v mc_rtc_ticker
```

Use a fresh terminal without a sourced ROS overlay for the controller build.
The standalone viewer below avoids requiring ROS support in TRIAD.

## 2. Clone TRIAD

```bash
git clone https://github.com/harryjavadinia-creator/TRIAD-handover-controller.git
cd TRIAD-handover-controller
```

## 3. Prepare the robot model

The tagged upstream source trees contain the required URDF and meshes;
building a ROS workspace is unnecessary for this reconstruction. From the
TRIAD repository root, download the matching descriptions into a sibling
directory:

```bash
export TRIAD_DEPENDENCIES="$PWD/../TRIAD-dependencies"
mkdir -p "$TRIAD_DEPENDENCIES"

git clone --depth 1 --branch 0.2.6 \
  https://github.com/Kinovarobotics/ros2_kortex.git \
  "$TRIAD_DEPENDENCIES/ros2_kortex"
git clone --depth 1 --branch 0.0.1 \
  https://github.com/PickNikRobotics/ros2_robotiq_gripper.git \
  "$TRIAD_DEPENDENCIES/ros2_robotiq_gripper"

export TRIAD_KORTEX_SOURCE="$TRIAD_DEPENDENCIES/ros2_kortex/kortex_description"
export TRIAD_ROBOTIQ_SOURCE="$TRIAD_DEPENDENCIES/ros2_robotiq_gripper/robotiq_description"

python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf "$TRIAD_KORTEX_SOURCE/robots/gen3_2f85.urdf" \
  --kortex-share "$TRIAD_KORTEX_SOURCE" \
  --robotiq-share "$TRIAD_ROBOTIQ_SOURCE" \
  --output "$PWD/gen3_2f85_module"

export MAIN_ROBOT_MODULE_PATH="$PWD/gen3_2f85_module"
```

The setup script checks the upstream URDF and all referenced mesh contents
before generating the module. A content mismatch means the inputs differ
from the documented robot model; use the matching packages.

The two tags above passed the supplied URDF and all 26 unique mesh checks
during robot-model validation. Keep the dependency directories in place:
the generated module refers to their local meshes. Further model details
are in [Robot module](robot_module.md).

## 4. Build (do not install)

From the repository root:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
  CMAKE_PREFIX_PATH="$TRIAD_MC_RTC_PREFIX" \
  cmake -S . -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON

cmake --build build -j"$(nproc)"
```

`-G Ninja` needs `ninja-build`; drop it to use make. The build produces `build/src/HandoverInterceptionController_controller.so`
and the state libraries under `build/src/states/`. Every run script in this
repository can load the controller straight from that build tree, so nothing
has to be installed. Do not run `cmake --install build`: mc_rtc's CMake macros
install controllers into the mc_rtc installation itself, whatever prefix you
pass, and overwrite any controller already there (see
[Troubleshooting](troubleshooting.md)).

## 5. Run a scenario and check the outcome

In the terminal where `MAIN_ROBOT_MODULE_PATH` and `PATH` are set:

```bash
export TRIAD_BUILD_DIR="$PWD/build"
export MC_RTC_INSTALL="$TRIAD_MC_RTC_PREFIX"
scripts/run_scenario.sh longitudinal
```

The script writes a scratch mc_rtc profile under a temporary `HOME`, points
mc_rtc at the build tree, runs `mc_rtc_ticker`, saves the scenario input and
log under `results/`, checks the outcome and stops the ticker after a terminal
state. The last lines must read:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

The log itself (`results/<run>/longitudinal.log`) shows the state sequence
`Initial → ObserveObject → SolveInterception → ExecuteCommittedReach →
PresentationHold → MovePregrasp → CaptureTransfer → Retreat → Completed`, the
size of the plan set (`completePlans`, `timingAdmissiblePlans`) and the
committed plan (`candidate=… route=… globalJ=…`). The
committed plans of the reference runs are listed in
[Experiments](experiments.md#reference-run-winners) so you can compare your run
with the recorded one. The other scenarios are
`near-ground`, `lateral-low` and `diagonal`; all four complete from a fresh
clone built as above. `TRIAD_RECEIVER_MODE=receding scripts/run_scenario.sh <scenario>`
runs the receding mode (its log is checked by `tools/check_v2_run_log.py`);
`TRIAD_MAX_WAIT` bounds the wall-clock wait (default 180 s).

`mc_rtc_ticker` may print a segmentation fault when it exits after the
wrapper stops it. That happens after the terminal state has been reached and
checked; the three result lines above are the outcome.

## 6. Run the two-robot handover

Robot B (a second Kinova Gen3, module name `Kinova`) presents the object and
Robot A receives it. Robot B's module comes from
[mc_kinova](https://github.com/mathieu-celerier/mc_kinova): build and install it
into your mc_rtc installation following its instructions (it needs `xacro` and
`kortex_description`).
Then:

```bash
bash two_robot/run_two_robot_sim.sh 80
```

The script prints Robot B's phases (Prepositioning → StartSettling → Ready →
Executing → TerminalSettling → Holding), Robot A's states, and ends with
`RESULT: COMPLETED` after about 19 s of simulated time. Its log and timeline
land under `two_robot/results/`. `TRIAD_GIVER_SCENARIO=diagonal_xz` or
`TRIAD_GIVER_SCENARIO=static_nominal` selects another presentation by Robot B.
Details, the hardware procedure and the laboratory videos are in
[`two_robot/`](../two_robot/README.md).

## 7. Watch it

Both runners start mc_rtc's GUI server on TCP ports 4242/4343 (the single-robot
runner listens on all interfaces, the two-robot runner on 127.0.0.1), so an
mc_rtc viewer attached to the local controller shows the robots, the object
and the **Handover → Methodology** markers while a run is in progress. Start
the viewer first: the wrappers stop the ticker as soon as a run terminates.

**Run at half speed while watching.** The finite search runs on a background
worker in real time, and a viewer attached to the controller lengthens it: for
`lateral-low`, 3.9 s in the reference log against 5.0 s with RViz attached
(`workerWall` in `evidence/reference_runs/lateral-low.log.xz` and
`lateral-low_viewer_attached.log.xz`). The simulated object reaches its 0.40 m
travel cap about 4 s after the search epoch, so the longer search fails the
pre-commit consistency check with `global_event_prediction_drift`. Both runners
therefore accept `TRIAD_SYNC_RATIO`, the simulated-to-real time ratio of the
ticker:

```bash
TRIAD_SYNC_RATIO=0.5 scripts/run_scenario.sh lateral-low
TRIAD_SYNC_RATIO=0.5 bash two_robot/run_two_robot_sim.sh 80
```

At 0.5 the simulation runs at half speed and the run completes with the same
three result lines, but the committed plan can differ from the full-speed
reference: timing admission is evaluated at the simulated time the search
returns, and a slower simulation gives the worker more simulated time. Use full
speed without a viewer to reproduce the reference winners.

**RViz** (mc_rtc built with its ROS plugin). In a second terminal, source ROS
and the mc_rtc ROS workspace, then open the display file shipped with this
repository, which has a RobotModel display for Robot A, Robot B and the
object:

```bash
source /opt/ros/jazzy/setup.bash          # your ROS distribution
source /path/to/mc_rtc_ros_ws/install/setup.bash
rviz2 -d two_robot/display_two_robot.rviz
```

Then start the run in the first terminal. This is what the two-robot run
looks like in it (Robot A left, Robot B right holding the object):

![RViz view of the two-robot handover](../two_robot/media/rviz_two_robot.png)

The same file works for the single-robot scenarios; the Robot B display then
simply reports that its topic is absent. The `display.rviz` shipped in
mc_rtc_ros's `mc_rtc_ticker` package shows Robot A only.

**mc-rtc-magnum** (no ROS needed): install the standalone
[mc_rtc-magnum viewer](https://github.com/mc-rtc/mc_rtc-magnum) and start
`mc-rtc-magnum` with the local controller connection. The viewer needs the
same robot mesh files as the controller.

The upstream [controller/viewer guide](https://jrl.cnrs.fr/mc_rtc/tutorials/introduction/running-a-controller.html)
covers both viewers.

## What to look for

The intended completed sequence is observation, planning, committed reach,
presentation, pregrasp approach, closure and transfer, then retreat. During
planning the finite search runs on a background worker while the controller
continues its normal cycle.

Open **Handover → Methodology** in the GUI:

| GUI label | Meaning |
| --- | --- |
| `OBJECT O` | Current object pose |
| `PREDICTED OBJECT O(t+h)` | Predicted object pose |
| `COMMITTED OBJECT AT CONTACT` | Selected presentation pose |
| `ACTUAL MOUTH M` | Actual gripper mouth pose |
| `TRANSIT STANDOFF` / `CAPTURE PREGRASP` | Approach targets |
| `CERTIFIED RETREAT` | Retreat target from the model checks |

These are controller outputs. “Certified” in the retreat label refers to the
implemented sampled model checks.

## The four scenarios

Logs and the exact scenario override are saved under `results/`.
A rejected plan or execution failure is reported as a failed run.
`TICKER_STOP_REASON=WRAPPER_TERMINATED` describes the wrapper stopping
the ticker; its process exit code alone does not determine scientific success.

| Scenario | Object motion |
| --- | --- |
| `longitudinal` | Toward the robot along the longitudinal axis |
| `lateral-low` | Lateral motion near the ground |
| `near-ground` | Lateral motion from the opposite side near the ground |
| `diagonal` | Combined longitudinal and upward motion |

The [simulation reference](simulation.md) lists exact inputs and historical
winners. Wall time, live timing admission, and rendering depend on the machine;
an identical winner or completion is not guaranteed across different timing
conditions. Physical execution is disabled by the default tracked
configuration.
