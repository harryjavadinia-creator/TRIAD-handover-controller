# Build and watch a simulation

This guide runs the published asynchronous TRIAD controller in mc_rtc's
open-loop simulation. It requires Linux and local robot-description
dependencies. The [result figures](results.md) can be viewed on GitHub without
installing anything.

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
git clone --branch publication/supervisor-release https://github.com/harryjavadinia-creator/TRIAD-handover-controller.git
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
from the reported robot model; use the matching packages.

The two tags above passed the supplied URDF and all 26 unique mesh checks
during publication preparation. Keep the dependency directories in place:
the generated module refers to their local meshes. Further model details
are in [Robot module](robot_module.md).

## 4. Build and install

From the repository root:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
  CMAKE_PREFIX_PATH="$TRIAD_MC_RTC_PREFIX" \
  cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON

cmake --build build -j"$(nproc)"
cmake --install build
```

Installation uses the controller and FSM destinations of the selected mc_rtc
installation. You need write access to those destinations. For configuration,
linking, or loading errors, see [Troubleshooting](troubleshooting.md).

## 5. Open the viewer, then run a scenario

Install the standalone [mc_rtc-magnum viewer](https://github.com/mc-rtc/mc_rtc-magnum)
following its upstream instructions. In a second terminal, start:

```bash
mc-rtc-magnum
```

Use the local controller connection. TRIAD's GUI server uses TCP ports 4242
and 4343. The viewer and controller need access to the same robot mesh files.
The upstream [controller/viewer guide](https://jrl.cnrs.fr/mc_rtc/tutorials/introduction/running-a-controller.html)
also documents RViz for environments built with ROS support.

In the first terminal, where `MAIN_ROBOT_MODULE_PATH` is set, run:

```bash
scripts/run_scenario.sh longitudinal
```

The script saves the scenario input and log, checks the outcome, and stops
the ticker after a terminal state. Open the viewer first because the wrapper
closes the completed run automatically.

The viewer instructions follow the upstream interface and the GUI fields in
the published source. They are not a record of a new desktop/viewer test of
this checkout; see [Validation scope](release_validation.md).

## What to look for

The intended completed sequence is observation, planning, committed reach,
presentation, pregrasp approach, closure and transfer, then retreat. During
planning the finite search runs on a worker while the controller continues.

Open **Handover → Methodology** in the GUI:

| GUI label | Meaning |
| --- | --- |
| `OBJECT O` | Current object pose |
| `PREDICTED OBJECT O(t+h)` | Predicted object pose |
| `COMMITTED OBJECT AT CONTACT` | Selected presentation pose |
| `ACTUAL MOUTH M` | Actual gripper mouth pose |
| `TRANSIT STANDOFF` / `CAPTURE PREGRASP` | Approach targets |
| `CERTIFIED RETREAT` | Retreat target from the model checks |

These are existing controller outputs. “Certified” in the retreat label
refers to the implemented sampled model checks.

## Check the outcome

A completed run must report all three:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

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
conditions. Physical execution is disabled by the published defaults.
