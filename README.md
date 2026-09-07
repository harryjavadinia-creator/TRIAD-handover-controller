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
- **Current publication source:** the frozen asynchronous-planner state. The complete finite search runs on one background worker, off the 1 kHz control callback, with the frozen scientific evaluation unchanged; see [`docs/provenance.md`](docs/provenance.md) for which source state each reported number belongs to.
- **Evidence:** four canonical scenarios, a 62-scenario predeclared held-out envelope, 66 predeclared perturbations, a corrected perception-latency ablation, repeated-run determinism and fault-injection safety evidence — all published in reduced form under [`evidence/`](evidence/).

A supervisor/reviewer who wants the shortest technical path can read, in order: [`docs/mathematics.md`](docs/mathematics.md), [`docs/provenance.md`](docs/provenance.md), [`evidence/README.md`](evidence/README.md), [`docs/simulation.md`](docs/simulation.md) and [`docs/timing_frontiers.md`](docs/timing_frontiers.md).

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

The seven objective weights are frozen controller-specific engineering preference values. They are not literature-derived, are not claimed optimal, and **no weight-space sensitivity result is reported in this repository**.

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

sha256sum -c docs/source_sync_f56add3.sha256

cd evidence && sha256sum -c MANIFEST.sha256 && cd ..
python3 tools/check_evidence_manifest.py
```

These checks are also represented in the repository's GitHub Actions workflow. The full staged reproduction procedure is in [`docs/reproducibility.md`](docs/reproducibility.md).

## Experiment sets

Results in this repository come from several campaigns on several source states. **Every reported number belongs to exactly one of them**, and [`docs/provenance.md`](docs/provenance.md) is the canonical mapping.

| campaign | what it establishes | source state |
| --- | --- | --- |
| **Dataset A** — perception-latency matrix | historical latency study, 5 scenarios × 3 conditions | `dataset-a-baseline` |
| **Dataset B** — finite event-time/grasp/route planning | the four deterministic winners | `scientific-baseline` |
| **Exact-serial study** | serial wall-time reduction with audited equivalence | `csi-2026-release` |
| **Held-out generalization** — 62 predeclared scenarios | behaviour outside the development scenarios | asynchronous planner |
| **Local robustness** — 66 predeclared perturbations | behaviour under bounded state perturbation | asynchronous planner |
| **Asynchronous planner** — control-loop timing, corrected latency, determinism, fault injection | runtime behaviour of the frozen state published here | **this branch** |

See [`docs/experiments.md`](docs/experiments.md) for source attribution and evidence limitations, and [`evidence/`](evidence/) for the published primary records.

### Current evidence, in one place

- **62 predeclared held-out scenarios** (generated from a fixed seed and hashed before execution): 36 completed, 2 committed then failed in execution, 3 with no physically feasible plan, 18 with no timing-admissible plan, 3 rejected by the commit-freshness gate. Of the 38 scenarios the planner committed to, **36 completed**. Completion rises with object height across the tested bands.
- **66 predeclared perturbations** about six pre-registered anchors: 40 completed, 14 safely rejected, 12 execution failures after commitment — eight of which belong to a single anchor whose unperturbed case already sits on the 8 mm dynamic clearance reserve. That anchor is reported, not removed.
- **Perception latency**: with the corrected configuration read, the measured measurement age tracks the configured delay exactly; uncompensated state-estimation error follows `e = v·tau`; forward compensation removes that bias to within 2.4 mm at 0.60 s.
- **Control-loop timing**: with the search on a background worker, at most one control cycle per run exceeds 1 ms during planning and none exceeds 2 ms, against 424–1362 cycles above 1 ms beforehand. This is an empirical tail measurement, not a hard real-time guarantee.
- **Determinism**: repeated runs of the same scenario produce one frozen plan-set hash. Where a repeated run selects a different winner, the frozen plan set is identical and the difference tracks the result-receipt instant crossing a timing-admission boundary — the designed semantics.
- **Fail-closed behaviour**: exercised by fault injection, not argued. Every rejection path leaves the robot stationary.

### Limitations

- The candidate bank — 14 event times, 32 grasps, 17 routes — is a **frozen engineering discretisation**. No resolution or convergence study has been performed.
- Exactness applies to the minimisation **over that bank**. There is no continuous-space optimality and no completeness guarantee, so an outcome of "no feasible plan" or "no timing-admissible plan" never proves that no physical solution exists.
- The 18 timing rejections in the held-out campaign **remain unresolved** with respect to the true physical feasible domain.
- There is **no arbitrary environment perception, no self-collision checking and no human-body geometry**. The obstacle set is a ground plane plus three capsules derived from the object's own pose, and the mc_rtc QP carries no collision constraint.
- Collision certification is **sampled** (25 interpolated poses per commanded segment), not continuous.
- Prediction is **deterministic**; no uncertainty is represented.
- The plan is **committed once**; there is no replanning, retiming or reselection.
- The load-transfer source is a **virtual sensor** by default, so transfer results are evidence about the admittance policy, not physical load sharing.
- One object geometry at one orientation; no object-independence or orientation generalization is claimed.
- **No physical-robot and no human-subject validation.**

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

## Asynchronous planner

The source published on this branch runs the complete finite search on **one background worker**, off the 1 kHz mc_rtc control callback. The control thread performs a single atomic load per cycle and never waits: no mutex, condition variable, future or join is reachable from the callback.

The synchronized implementation files are pinned by [`docs/source_sync_f56add3.sha256`](docs/source_sync_f56add3.sha256).

**No scientific parameter changed.** The preview integration step, the event, grasp and route banks, hard feasibility, the objective and its weights, the timing thresholds, the tie rules and the one-commit / no-replanning semantics are identical to the previously published state. The only configuration addition is `routeWorkUnitsPerCycle`, a work-unit scheduling budget.

Frozen plan sets are byte-identical between the control-thread and worker builds in three of the four canonical scenarios. The fourth is a **strict superset**: all 229 control-thread records are byte-identical at 17 significant digits, and 54 further records are enumerated because pinning the admission instant to the search epoch removes a wall-clock-dependent prune that had skipped one event hypothesis before its geometry ran.

The defensible statement is *"the same frozen scientific evaluation, with the wall-clock-dependent premature enumeration truncation removed and the admission instant moved earlier."* It is **not** a claim that the two modes produce the same plan set, and the additional records were never rejected on scientific grounds — they were never enumerated. The control-thread prune was itself logically sound.

The earlier exact-serial study remains valid for its own source state; see [`docs/performance.md`](docs/performance.md) and [`docs/provenance.md`](docs/provenance.md).

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

- [`docs/provenance.md`](docs/provenance.md) — **which source state every reported number belongs to.**
- [`evidence/README.md`](evidence/README.md) — the published primary records and how to check them.
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

The source shipped on this branch is the frozen asynchronous-planner state, pinned by [`docs/source_sync_f56add3.sha256`](docs/source_sync_f56add3.sha256). The states serve different purposes:

- **`scientific-baseline`:** provenance anchor for the frozen Dataset-B campaign;
- **`csi-2026-release`:** the exact-serial publication release, and the state against which [`docs/source_sync_82e6eaa.sha256`](docs/source_sync_82e6eaa.sha256) verifies;
- **this branch:** the frozen asynchronous-planner state, with the evidence published under [`evidence/`](evidence/).

`docs/source_sync_82e6eaa.sha256` is retained as the historical record of the exact-serial state. It verifies against the `csi-2026-release` tag, **not** against this branch, and is no longer part of the automated checks.

The publication synchronization and four-scenario revalidation of the earlier state are recorded in [`docs/release_validation.md`](docs/release_validation.md).

## Real-robot status

Hardware-facing configuration and staged gripper commissioning support exist, but there is currently **no validated end-to-end physical-robot handover result** in this repository. Configuration support is not evidence of hardware validation. Read [`docs/real_robot.md`](docs/real_robot.md) before any physical attempt.

## Citation

The associated paper citation will be added when article metadata is finalized. Until then, cite the repository title together with the exact release/tag or commit used for reproduction.
