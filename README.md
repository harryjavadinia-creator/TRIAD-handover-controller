# TRIAD

**Joint Event-Time, Grasp and Route Selection for Predictive Human-to-Robot Handover**

*A predictive finite complete-plan selection framework for human-to-robot handover in mc_rtc, developed within the CALL research project.*

TRIAD observes a moving object, predicts a bounded set of future presentation events, evaluates complete **event-time–grasp–route** alternatives on copied robot state, rejects alternatives that violate hard feasibility constraints, and commits the minimum-cost timing-admissible finite plan once for the mc_rtc FSM/QP layer to execute.

The planner decides **what and when**: event time, grasp and route. The mc_rtc task/QP layer decides **how** to track the committed plan.

> **Evidence status:** the reported end-to-end results in this repository are simulation results. Hardware-facing support exists, but a validated end-to-end physical-robot handover campaign is not yet reported. See [`docs/real_robot.md`](docs/real_robot.md).

## Scientific core

For moving-object handover, TRIAD solves the bounded discrete problem

\[
(\tau^*,g^*,r^*)=
\arg\min_{(\tau,g,r)\in\mathcal F_h(s_0)}
J_{\mathrm{global}}(\tau,g,r;s_0).
\]

This is **exhaustive minimization over a finite bounded approximation**. It is not continuous-time global optimization, gradient descent, or MPC over event time. Hard reachability, collision, acquisition, retreat and timing checks are applied before soft-cost ranking.

The shortest path for understanding the implementation is:

1. [`docs/mathematics.md`](docs/mathematics.md) — decision variables, feasible set and objective.
2. [`src/FiniteEventPlanSelector.h`](src/FiniteEventPlanSelector.h) — exact finite argmin across event time, grasp and route.
3. [`src/FinitePlanSelector.h`](src/FinitePlanSelector.h) — within-event complete-plan selection and timing admission.
4. [`src/states/HandoverInterceptionController_SolveInterception.cpp`](src/states/HandoverInterceptionController_SolveInterception.cpp) — bounded event generation, complete scan and one-time commit.
5. [`src/HandoverInterceptionController.cpp`](src/HandoverInterceptionController.cpp) — candidate generation, copied-state preview, feasibility, metrics and execution support.

## What is new in the current research package

Beyond the frozen finite-plan Dataset-B baseline, the project now includes a corrected hardware-timing interpretation and exact serial performance characterization:

- timing admissibility is **scenario-specific** rather than governed by one universal planner deadline;
- the historical `3.976 s` value belongs to PURE_X / longitudinal, not CANONICAL_YZ;
- the four derived fail-closed frontiers are approximately 3.900 s, 3.975 s, 5.140 s and 5.735 s for GROUND_NEAR, PURE_X, CANONICAL_YZ and DIAGONAL_XZ respectively;
- winner preservation is a stricter, separate timing property from remaining timing-admissible;
- exact serial implementation optimizations reduce wall time while preserving complete scientific records and collision-oracle results.

See [`docs/timing_frontiers.md`](docs/timing_frontiers.md) and [`docs/performance.md`](docs/performance.md).

## Repository layout

```text
src/                     TRIAD controller and active FSM implementation
etc/                     controller configuration template
call_object_description/ handover-object URDF
configs/                 safe example configuration and robot-model hashes
scripts/                 scenario and historical-experiment reproduction
tools/                   runtime checkers, baseline verifier and regression tests
docs/                    method, experiments, timing, simulation and hardware notes
```

External software and robot-description packages are not copied into the repository; they are referenced or reconstructed from pinned upstream artifacts.

## Requirements

Verified development environment:

| Component | Verified version |
| --- | --- |
| Ubuntu | 24.04 LTS |
| GCC | 13.3.0 |
| CMake | 4.3.1 |
| Eigen | 3.4.0 |
| mc_rtc | 2.14.0 |
| RBDyn | 1.9.3 |
| SpaceVecAlg | 1.2.9 |
| Tasks | 1.8.3 |
| TVM | 0.9.3 |
| Python | 3.12, standard library only for repository checkers/tests |

A working mc_rtc installation and its normal dependency chain are required. Building mc_rtc itself is outside the scope of this repository.

Simulation also requires a Kinova Gen3 + Robotiq 2F-85 mc_rtc robot module. The repository reconstructs the verified module from pinned upstream `kortex_description` 0.2.6 and `robotiq_description` 0.0.1 assets; see [`docs/robot_module.md`](docs/robot_module.md).

## Clone and build

```bash
git clone https://github.com/harryjavadinia-creator/TRIAD-handover-controller.git
cd TRIAD-handover-controller

env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
  CMAKE_PREFIX_PATH=/path/to/your/mc_rtc/install \
  cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON

cmake --build build -j"$(nproc)"
cmake --install build
```

The controller installs into the runtime directories of the mc_rtc installation it was configured against. `-DCMAKE_INSTALL_PREFIX` does not isolate an mc_rtc controller installation; see [`docs/troubleshooting.md`](docs/troubleshooting.md).

## Reconstruct the robot module

```bash
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf /path/to/kortex_description/robots/gen3_2f85.urdf \
  --kortex-share /path/to/share/kortex_description \
  --robotiq-share /path/to/share/robotiq_description \
  --output /path/to/gen3_2f85_module

export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
```

The setup script verifies the pinned upstream URDF and all referenced mesh contents before generating the mc_rtc module. Details and hashes are documented in [`docs/robot_module.md`](docs/robot_module.md).

## Reproduce Dataset B

Available moving-object scenarios:

| Command name | Motion |
| --- | --- |
| `near-ground` | near-ground lateral |
| `longitudinal` | longitudinal / PURE_X |
| `lateral-low` | lateral low-height / CANONICAL_YZ |
| `diagonal` | diagonal forward/upward |

Run one scenario:

```bash
scripts/run_scenario.sh longitudinal
```

A successful reproduction reports:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

The output directory contains the complete runtime log, exact scenario override and independent checker outputs. The scientific checker verifies that the committed alternative is the minimum of the cost-valid, timing-admissible finite set and independently reconstructs the frozen objective from logged terms.

For the full scenario table, expected values and tolerance policy, see [`docs/simulation.md`](docs/simulation.md).

## Dependency-free verification

These checks do not require mc_rtc or the robot module:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/test_setup_gen3_2f85_module.py
python3 tools/test_verify_latency_matrix_cell.py
python3 tools/test_verify_scenario_identity.py
python3 tools/test_scenario_override_yaml.py
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
```

The last command verifies the frozen scientific baseline directly from Git blob content rather than from the current working tree.

For a staged reproduction workflow, see [`docs/reproducibility.md`](docs/reproducibility.md).

## Reported experiment sets

Two historical experiment sets are preserved and must not be mixed:

- **Dataset B — finite global event-time–grasp–route planning.** Four moving-object simulation scenarios from the scientific baseline. Reproduce with `scripts/run_scenario.sh`.
- **Dataset A — perception-latency study.** Five scenarios × three latency conditions at the preserved `dataset-a-baseline` source state. Reproduce with `scripts/reproduce_latency_matrix.sh`.

The complete attribution, limitations and historical evidence classification are in [`docs/experiments.md`](docs/experiments.md).

## Correct timing interpretation

There is no single experimentally established universal planner deadline for all four moving-object scenarios.

The exact counterfactual replay of the controller timing rule gives scenario-specific fail-closed frontiers:

| Scenario | Fail-closed frontier (s) |
| --- | ---: |
| GROUND_NEAR | 3.900000 |
| PURE_X | 3.975000 |
| CANONICAL_YZ | 5.139608 |
| DIAGONAL_XZ | 5.735285 |

The previously quoted `3.976 s` value is the next 1 ms grid point above the exact PURE_X boundary. It must not be presented as a universal hardware timing requirement.

The same analysis also derives **winner-preservation bands**, which are stricter than fail-closed feasibility. See [`docs/timing_frontiers.md`](docs/timing_frontiers.md).

## Active FSM

The compiled release FSM is:

```text
Initial
  → ObserveObject
  → SolveInterception
  → ExecuteCommittedReach
  → PresentationHold
  → MovePregrasp
  → CaptureTransfer
  → Retreat
  → Completed
```

Any failure enters `Failure`. Capture, bilateral grasp confirmation and load transfer are handled continuously by `CaptureTransfer` in the compiled release.

## Documentation

- [`docs/architecture.md`](docs/architecture.md) — pipeline and component responsibilities.
- [`docs/mathematics.md`](docs/mathematics.md) — finite decision problem and objective.
- [`docs/global_time_plan.md`](docs/global_time_plan.md) — detailed cross-event selector notes.
- [`docs/binding_cost.md`](docs/binding_cost.md) — detailed within-event selector notes.
- [`docs/simulation.md`](docs/simulation.md) — reported scenarios and expected outputs.
- [`docs/experiments.md`](docs/experiments.md) — provenance for Dataset A and Dataset B.
- [`docs/timing_frontiers.md`](docs/timing_frontiers.md) — exact scenario-specific timing replay and frontiers.
- [`docs/performance.md`](docs/performance.md) — exact serial acceleration and equivalence evidence.
- [`docs/reproducibility.md`](docs/reproducibility.md) — staged reproducibility workflow and release checklist.
- [`docs/robot_module.md`](docs/robot_module.md) — Gen3 + 2F-85 model reconstruction and verification.
- [`docs/real_robot.md`](docs/real_robot.md) — hardware support and unvalidated gaps.
- [`docs/troubleshooting.md`](docs/troubleshooting.md) — build/runtime caveats.

## Real-robot status

Hardware-facing support exists in the codebase, including staged gripper commissioning and physical force/gripper interfaces, but **there is currently no validated end-to-end real-robot handover result in this repository**. Do not treat configuration support as hardware validation. Read [`docs/real_robot.md`](docs/real_robot.md) before any physical attempt.

## Scientific baseline

Dataset B is anchored to the frozen `scientific-baseline` source state. `SCIENTIFIC_BASELINE.sha256` and the verifier provide a source-integrity record for that curated snapshot. The repository history also preserves `dataset-a-baseline` for the earlier latency study.

## Citation

The paper citation will be added when the associated article metadata is finalized. Until then, cite the repository by title and exact release/tag or commit used for reproduction.
