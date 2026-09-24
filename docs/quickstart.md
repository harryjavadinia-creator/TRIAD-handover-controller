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

The build produces `build/src/HandoverInterceptionController_controller.so`
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
[simulation reference](simulation.md) lists the reference winners so you can
compare your run with the recorded one. The other scenarios are
`near-ground`, `lateral-low` and `diagonal`; all four complete from a fresh
clone built as above.

`mc_rtc_ticker` may print a segmentation fault when it exits after the
wrapper stops it. That happens after the terminal state has been reached and
checked; the three result lines above are the outcome.

## 6. Run the two-robot handover

Robot B (a second Kinova Gen3, module name `Kinova`) presents the object and
Robot A receives it. Robot B's module comes from
[mc_kinova](https://github.com/mathieu-celerier/mc_kinova) (the laboratory
copy has the same layout; build and install it into your mc_rtc installation
following its own instructions, it needs `xacro` and `kortex_description`).
Then:

```bash
bash two_robot/run_two_robot_sim.sh 80
```

The script prints Robot B's phases (Prepositioning → StartSettling → Ready →
Executing → TerminalSettling → Holding), Robot A's states, and ends with
`RESULT: COMPLETED` after about 20 s of simulated time. Its log and timeline
land under `two_robot/results/`. Details, the hardware procedure and the
laboratory videos are in [`two_robot/`](../two_robot/README.md).

## 7. Watch it

Both runners start mc_rtc's GUI server (TCP 4242 / 4343 on localhost), so any
mc_rtc viewer attached to the local controller shows the robots, the object
and the **Handover → Methodology** markers while a run is in progress. Start
the viewer first: the wrappers stop the ticker as soon as a run terminates.

- **RViz** (if your mc_rtc was built with its ROS plugin): source ROS and
  the mc_rtc ROS workspace in a second terminal, then

  ```bash
  rviz2 -d "$(ros2 pkg prefix mc_rtc_ticker)/share/mc_rtc_ticker/launch/display.rviz"
  ```

  The controller publishes `/control/gen3_2f85/robot_description`,
  `/control/call_object/robot_description` and, in the two-robot run,
  `/control/kinova/robot_description`; the shipped display file shows the
  first one, add a RobotModel display for the others.
- **mc-rtc-magnum** (no ROS needed): install the standalone
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
