# TRIAD

**Joint Event-Time, Grasp and Route Selection for Predictive Human-to-Robot Handover**

*A predictive finite complete-plan selection framework for human-to-robot handover in mc_rtc, developed within the CALL research project.*

TRIAD predicts a bounded set of future object-presentation events, evaluates complete **event-time–grasp–route** alternatives on copied robot state, rejects alternatives that violate hard physical feasibility, reapplies timing admission at final selection time, and commits one minimum-cost admissible finite plan for the mc_rtc FSM/QP layer to execute.

The planner decides **what and when**: event time, grasp and route. The mc_rtc task/QP layer decides **how** to track the committed references.

> **Evidence status:** the reported end-to-end results are simulation results. Hardware-facing support exists, but no validated end-to-end physical-robot handover campaign is reported. See [`docs/real_robot.md`](docs/real_robot.md).

## At a glance

For a first review of the project, the important points are:

- **Method:** deterministic exhaustive selection over a bounded finite set of event-time, grasp and route alternatives; no continuous optimization solver is claimed.
- **Decision rule:** hard physical feasibility first, valid finite objective second, final timing admission at selector time, then the minimum `J_global` over the remaining finite set.
- **Reported Dataset B:** four moving-object simulation scenarios, each with a deterministic event/grasp/route winner and independent runtime-log verification.
- **Timing result:** planner timing admissibility is scenario-specific; the historical `3.976 s` value is PURE_X-specific rather than a universal deadline.
- **Current publication source:** the audited exact-serial implementation preserves the tested scientific records/winners while reducing measured planning wall time; the frozen `scientific-baseline` remains the provenance anchor for the original Dataset-B campaign.

A supervisor/reviewer who wants the shortest technical path can read, in order: [`docs/mathematics.md`](docs/mathematics.md), [`docs/simulation.md`](docs/simulation.md), [`docs/timing_frontiers.md`](docs/timing_frontiers.md), and [`docs/release_validation.md`](docs/release_validation.md).

## Scientific formulation

For a frozen decision state `s0`, TRIAD generates a bounded finite plan set `X_h`. Copied-state hard physical checks define `F_h(s0)`; finite objective construction gives the cost-valid subset `F_J(s0)`. After the complete bounded schedule has been evaluated, the selector reapplies timing admission at final selector time `t_sel` and chooses

\[
(\tau^*,g^*,r^*)=
\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
\]

This is **exhaustive minimization over the generated bounded finite approximation**. It is not continuous-space global optimization, gradient descent, or MPC over event time.

The finite approximation used in the reported moving-object campaign contains 14 event-time hypotheses, 32 grasp candidates and 17 transit routes, for an upper pre-pruning combinatorial bound of `14 × 32 × 17 = 7616`.

The controller configuration sets `maximumEventHypotheses: 15` as an upper cap on the bounded lead bank; the reported Dataset-B campaign generated 14 hypotheses within that cap, which is the value logged as `configuredHypotheses` and used in the bound above.

See [`docs/mathematics.md`](docs/mathematics.md) for the full set definitions, objective and final timing gate.

## Implementation map

The shortest path through the code is:

1. [`src/FiniteEventPlanSelector.h`](src/FiniteEventPlanSelector.h) — final cross-event timing admission and finite argmin.
2. [`src/FinitePlanSelector.h`](src/FinitePlanSelector.h) — within-event selection/refinement logic.
3. [`src/states/HandoverInterceptionController_SolveInterception.cpp`](src/states/HandoverInterceptionController_SolveInterception.cpp) — bounded event generation, complete scan and one-time global selection.
4. [`src/HandoverInterceptionController.cpp`](src/HandoverInterceptionController.cpp) — candidate generation, copied-state preview, hard feasibility, metrics and commit support.
5. [`src/states/`](src/states/) — mc_rtc execution FSM.

TRIAD is the public method name. The C++ namespace `call_handover`, controller name `HandoverInterceptionController`, and object identifier `call_object` are retained implementation identifiers from the CALL project lineage.

## Repository layout

```text
src/                     controller and active FSM implementation
etc/                     controller configuration template
call_object_description/ handover-object URDF and model notes
configs/                 simulation template and robot-model hashes
scripts/                 scenario and historical-experiment reproduction
tools/                   selectors/checkers/regression and replay utilities
docs/                    method, provenance, timing, simulation and hardware notes
.github/workflows/        dependency-free CI
```

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
| Python | 3.12 |

A working mc_rtc installation and its normal dependency chain are required. Building mc_rtc itself is outside the scope of this repository.

Simulation also requires a Kinova Gen3 + Robotiq 2F-85 mc_rtc robot module. The verified module is reconstructed from pinned upstream `kortex_description` 0.2.6 and `robotiq_description` 0.0.1 artifacts; see [`docs/robot_module.md`](docs/robot_module.md).

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

The controller installs into the runtime directories of the mc_rtc installation used at configure time. See [`docs/troubleshooting.md`](docs/troubleshooting.md).

## Reconstruct the robot module

```bash
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf /path/to/kortex_description/robots/gen3_2f85.urdf \
  --kortex-share /path/to/share/kortex_description \
  --robotiq-share /path/to/share/robotiq_description \
  --output /path/to/gen3_2f85_module

export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
```

The setup script checks the pinned upstream URDF and every referenced mesh against content hashes before producing the mc_rtc module.

## Reproduce Dataset B

| Command | Dataset-B label | Motion |
| --- | --- | --- |
| `near-ground` | `GROUND_NEAR` | near-ground lateral |
| `longitudinal` | `PURE_X` | longitudinal |
| `lateral-low` | `CANONICAL_YZ` | lateral, low height |
| `diagonal` | `DIAGONAL_XZ` | diagonal forward/upward |

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

The wrapper preserves the log, temporary scenario override and checker outputs under `results/`. See [`docs/simulation.md`](docs/simulation.md) for the reference winner fingerprints and additional metrics.

## Dependency-free verification

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/test_replay_timing_frontier.py
python3 tools/test_setup_gen3_2f85_module.py
python3 tools/test_verify_latency_matrix_cell.py
python3 tools/test_verify_scenario_identity.py
python3 tools/test_scenario_override_yaml.py
python3 tools/check_markdown_links.py

python3 tools/verify_scientific_baseline.py \
  SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline

sha256sum -c docs/source_sync_82e6eaa.sha256
```

These checks are also represented in the repository's GitHub Actions workflow. The full staged reproduction procedure is in [`docs/reproducibility.md`](docs/reproducibility.md).

## Experiment sets

Two historical experiment sets are preserved and reported separately:

- **Dataset B — finite global event-time–grasp–route planning:** four moving-object simulation scenarios anchored to the frozen scientific baseline.
- **Dataset A — perception-latency study:** five scenarios × three latency conditions at the preserved `dataset-a-baseline` source state.

See [`docs/experiments.md`](docs/experiments.md) for provenance, source attribution and evidence limitations.

## Timing interpretation

Final timing admission is scenario-specific. The analytical fail-closed boundaries derived from each scenario's complete-plan records are:

| Scenario | Boundary (s) |
| --- | ---: |
| GROUND_NEAR | 3.900000 |
| PURE_X | 3.975000 |
| CANONICAL_YZ | 5.139608 |
| DIAGONAL_XZ | 5.735285 |

The historical `3.976 s` figure is the next 1-ms PURE_X grid point above its exact boundary; it is not a universal hardware planner deadline.

Winner preservation is a separate and stricter property. See [`docs/timing_frontiers.md`](docs/timing_frontiers.md).

Reproduce the timing gate from a run log with:

```bash
python3 tools/replay_timing_frontier.py <log> --planner-time 3.976
```

## Exact serial implementation

The current publication source includes exact implementation-only serial accelerations synchronized from audited development commit `82e6eaa`.

The synchronized files are pinned by [`docs/source_sync_82e6eaa.sha256`](docs/source_sync_82e6eaa.sha256). The performance study reports **8,168,732 collision-oracle comparisons with zero mismatches** and unchanged complete-plan records, logical planning cycles, selector time and committed winners.

See [`docs/performance.md`](docs/performance.md).

## Active FSM

```text
Initial
  -> ObserveObject
  -> SolveInterception
  -> ExecuteCommittedReach
  -> PresentationHold
  -> MovePregrasp
  -> CaptureTransfer
  -> Retreat
  -> Completed
```

Any rejected/unsafe execution path enters `Failure`.

## Documentation

- [`docs/architecture.md`](docs/architecture.md) — component boundaries and selection/execution pipeline.
- [`docs/mathematics.md`](docs/mathematics.md) — finite sets, objective and final timing-admission formulation.
- [`docs/global_time_plan.md`](docs/global_time_plan.md) — cross-event global selector.
- [`docs/binding_cost.md`](docs/binding_cost.md) — within-event selector.
- [`docs/simulation.md`](docs/simulation.md) — Dataset-B reproduction and reference outputs.
- [`docs/experiments.md`](docs/experiments.md) — Dataset-A/B provenance.
- [`docs/timing_frontiers.md`](docs/timing_frontiers.md) — scenario-specific timing replay and interpretation.
- [`docs/performance.md`](docs/performance.md) — exact serial acceleration and equivalence evidence.
- [`docs/reproducibility.md`](docs/reproducibility.md) — staged reproduction workflow.
- [`docs/release_validation.md`](docs/release_validation.md) — validation of the synchronized publication state.
- [`docs/robot_module.md`](docs/robot_module.md) — Gen3 + 2F-85 model reconstruction.
- [`docs/real_robot.md`](docs/real_robot.md) — hardware support and unvalidated gaps.
- [`docs/troubleshooting.md`](docs/troubleshooting.md) — build/runtime caveats.

## Scientific baseline and publication source

Dataset B remains anchored to the frozen `scientific-baseline` tag. `SCIENTIFIC_BASELINE.sha256` verifies that historical source snapshot directly from Git blobs.

The source shipped on the current publication branch contains the exact implementation-only serial delta synchronized from `82e6eaa`. The two states therefore serve different purposes:

- **scientific-baseline:** provenance anchor for the frozen Dataset-B campaign;
- **current publication source:** tested equivalent implementation with lower serial wall time.

The publication synchronization and four-scenario revalidation are recorded in [`docs/release_validation.md`](docs/release_validation.md).

## Real-robot status

Hardware-facing configuration and staged gripper commissioning support exist, but there is currently **no validated end-to-end physical-robot handover result** in this repository. Configuration support is not evidence of hardware validation. Read [`docs/real_robot.md`](docs/real_robot.md) before any physical attempt.

## Citation

The associated paper citation will be added when article metadata is finalized. Until then, cite the repository title together with the exact release/tag or commit used for reproduction.
