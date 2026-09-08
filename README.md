# TRIAD

**Joint Event-Time, Grasp and Route Selection for Predictive Human-to-Robot Handover**

*A finite complete-plan selection framework for predictive human-to-robot handover in mc_rtc, developed within the CALL research project.*

TRIAD predicts a bounded set of future object-presentation events, evaluates
complete **event-time–grasp–route** alternatives, rejects candidates that violate
its modeled hard feasibility checks, reapplies timing admission at selector time,
and commits one minimum-cost admissible finite plan for execution.

TRIAD does more than choose the handover event: it selects the **event time,
grasp and transit route**, then generates and safety-governs the committed
task-space motion references along that route. The downstream mc_rtc task/QP
layer realizes those per-cycle references at the robot/joint level; it does not
choose the event, grasp, route or high-level objective.

> **Validation scope:** all reported end-to-end handover results are simulation
> results. Hardware-facing support exists, but no validated end-to-end physical
> Kinova human-handover campaign is claimed. See
> [`docs/real_robot.md`](docs/real_robot.md).

## At a glance

- **Decision variable:** `ξ = (τ, g, r)` — event time, grasp, route.
- **Globality:** exhaustive minimization over the generated bounded finite bank;
  no continuous-space global optimality is claimed.
- **Reported finite bank:** 14 event-time hypotheses, 32 grasps and 17 routes,
  giving an upper pre-pruning product of `14 × 32 × 17 = 7616`.
- **Prediction:** deterministic; delayed measurements are forward propagated and
  the object is given a prescribed stop model. No uncertainty distribution is
  represented.
- **Collision model:** sampled proxy checks only; no arbitrary environment
  perception, no human-body geometry, no self-collision checker and no mc_rtc
  collision constraint are claimed.
- **Commit semantics:** one-shot commitment; no post-commit global replanning,
  retiming or reselection.
- **Current publication source:** frozen asynchronous scientific state
  `f56add3`, pinned by [`docs/source_sync_f56add3.sha256`](docs/source_sync_f56add3.sha256).
- **Evidence:** historical Dataset A/B and exact-serial studies plus reduced
  primary records for the 62-scenario held-out campaign, 66 perturbations,
  corrected latency ablation and asynchronous planner evidence.

For a short technical path, read:

1. [`docs/mathematics.md`](docs/mathematics.md)
2. [`docs/architecture.md`](docs/architecture.md)
3. [`docs/provenance.md`](docs/provenance.md)
4. [`evidence/README.md`](evidence/README.md)
5. [`docs/corrections_of_record.md`](docs/corrections_of_record.md)

## Scientific formulation

For a frozen decision state `s0`, TRIAD generates a bounded finite plan set
`X_h`. Copied-state/model-relative hard checks define `F_h(s0)`; finite
objective construction gives the cost-valid subset `F_J(s0)`. After the bounded
schedule has been evaluated, the selector reapplies timing admission at final
selection time `t_sel` and chooses

$$
(\tau^{\ast},g^{\ast},r^{\ast})=
\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
$$

This is **exact finite selection over the generated bounded approximation**. It
is not gradient descent, continuous-space global optimization or MPC over event
time.

The controller configuration allows up to 15 event hypotheses; the reported
moving-object campaign generated 14. The reported grasp bank is 16
circumferential samples under two wrist/gripper conventions (32 total), and each
grasp receives 17 route generators (1 direct, 8 at 0.08 m and 8 at 0.14 m).

The seven objective weights are frozen engineering preference values. They are
not literature-derived, are not claimed optimal and **no weight-sensitivity
result is reported**.

## Implementation map

1. `src/FiniteEventPlanSelector.h` — final cross-event timing admission and
   finite argmin.
2. `src/FinitePlanSelector.h` — within-event finite selection/refinement.
3. `src/states/HandoverInterceptionController_SolveInterception.cpp` — bounded
   event generation, complete scan and one-time selection.
4. `src/HandoverInterceptionController.cpp` — candidate generation,
   copied-state preview, feasibility metrics and commit support.
5. `src/states/` — mc_rtc execution FSM.

TRIAD is the public method name. `call_handover`,
`HandoverInterceptionController` and `call_object` are retained implementation
identifiers from the CALL project lineage.

## Repository layout

```text
src/                     controller and active FSM implementation
etc/                     controller configuration template
call_object_description/ handover-object URDF/model notes
configs/                 simulation template and robot-model hashes
scripts/                 scenario and historical-experiment reproduction
tools/                   selectors, checkers, regression and replay utilities
docs/                    method, provenance, validation and hardware notes
evidence/                compact primary evidence and integrity manifest
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

A working mc_rtc installation and its normal dependency chain are required.
Simulation also requires a Kinova Gen3 + Robotiq 2F-85 mc_rtc robot module; see
[`docs/robot_module.md`](docs/robot_module.md).

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

See [`docs/reproducibility.md`](docs/reproducibility.md) and
[`docs/troubleshooting.md`](docs/troubleshooting.md).

## Reconstruct the robot module

```bash
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf /path/to/kortex_description/robots/gen3_2f85.urdf \
  --kortex-share /path/to/share/kortex_description \
  --robotiq-share /path/to/share/robotiq_description \
  --output /path/to/gen3_2f85_module

export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
```

The setup tool validates pinned upstream model/mesh contents before generating
the module.

## Reproduce historical Dataset B

| Command | Dataset-B label | Motion |
| --- | --- | --- |
| `near-ground` | `GROUND_NEAR` | near-ground lateral |
| `longitudinal` | `PURE_X` | longitudinal |
| `lateral-low` | `CANONICAL_YZ` | lateral, low height |
| `diagonal` | `DIAGONAL_XZ` | diagonal forward/upward |

Example:

```bash
scripts/run_scenario.sh longitudinal
```

The wrapper records its log, temporary scenario override and checker outputs
under `results/`. Historical expected winner fingerprints and their source state
are documented in [`docs/simulation.md`](docs/simulation.md) and
[`docs/provenance.md`](docs/provenance.md).

Do not interpret the historical exact-serial four-scenario rerun as a new
current-asynchronous-head runtime campaign; see
[`docs/release_validation.md`](docs/release_validation.md).

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

These checks are represented in the GitHub Actions workflow. They establish
source/checker contracts, frozen-source/evidence integrity and documentation
consistency; they are not physical validation and do not by themselves relabel
historical runtime experiments as current-head reruns.

## Experiment/source-state map

[`docs/provenance.md`](docs/provenance.md) is canonical.

| campaign | role | source/run state |
| --- | --- | --- |
| Dataset A | historical 5×3 latency matrix | `e2e194d` / `dataset-a-baseline` |
| Dataset B | four finite event/grasp/route winners | `c07368c` / `scientific-baseline` |
| Exact-serial study | serial wall-time/oracle/revalidation | `82e6eaa` → `a006912` / `csi-2026-release` |
| Held-out generalization | 62 predeclared scenarios | `90549ca`, ideal sensing |
| Local robustness | 66 predeclared perturbations | `90549ca`, ideal sensing |
| Corrected latency | corrected nondefault perception-delay sweep | `f56add3` |
| Async timing/determinism/safety records | archived asynchronous-development evidence | record-specific provenance; compatible with frozen source where documented |

Generalization and robustness used ideal sensing in all 128 records, so the later
nonzero-delay configuration-read correction does not affect those outcomes.

## Current evidence

### Held-out generalization

62 predeclared scenarios:

- 36 completed;
- 2 committed then failed during execution;
- 3 had no physically feasible **TRIAD-generated** plan;
- 18 had no timing-admissible TRIAD plan at result receipt;
- 3 were rejected by commit freshness.

Committed: **38/62**. Completed given commitment: **36/38 = 94.7%**.

These are direct empirical rates from the tested finite method. They are **not
feasible-space coverage estimates**. The 18 timing rejections remain unresolved
with respect to the true physical feasible domain.

### Robustness

Across all **66 predeclared perturbations**, the primary result is:

- 40 completed;
- 14 safely rejected before commitment;
- 12 failed during execution after commitment.

The H002 family contains 11 perturbations: **3 complete and 8 fail**. An
H002-excluded analysis exists only as a **secondary, post-hoc diagnostic**; it
does not replace the full 66-case result.

### Corrected latency

For the corrected nonzero-delay sweep, measurement age follows the configured
delay and the straight-motion uncompensated position bias follows `e = vτ`.
At 0.30 s, four compensated repeats completed while four uncompensated repeats
committed and then failed.

At 0.60 s:

- compensated: freshness rejection before commitment;
- uncompensated: **no finite planning**, because observation classification is
  `AMBIGUOUS` (`0.0241 m < 0.0250 m` displacement threshold while estimated
  speed is `0.0760 m/s > 0.0100 m/s` static threshold).

Both modes fail after commitment at 0.40 and 0.50 s; this is not generalized to
all delays `>= 0.40 s`.

### Asynchronous control-loop evidence

Published clean-machine **in-planning `ControllerRun`** after-profiles are:

| scenario | median ms | p90 ms | p99 ms | max ms | >1 ms | >2 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| lateral-low | 0.043 | 0.063 | 0.134 | 1.135 | 1 | 0 |
| near-ground | 0.040 | 0.072 | 0.122 | 1.174 | 1 | 0 |
| longitudinal | 0.067 | 0.084 | 0.122 | 1.090 | 1 | 0 |
| diagonal | 0.018 | 0.040 | 0.049 | 1.206 | 1 | 0 |

The `<2 ms` observation applies **only** to these in-planning `ControllerRun`
samples. Whole-run controller/global maxima are larger. This is an empirical
measurement on one machine, not WCET, a hard-real-time guarantee or a formal
schedulability result. See [`docs/performance.md`](docs/performance.md).

### Near-ground 229 → 283 relation

Three canonical scenarios retain identical frozen-plan-set hashes. Near-ground
is different: after removing only the nonsemantic `sourceIndex` field, all
**229 prior scientific payloads are retained and 54 are added**. Full raw
records and full set hashes are **not** identical.

The added payloads come from a hypothesis not enumerated in the historical
control-thread run after an elapsed-time prune. Do not describe the raw record
sets as byte-identical or claim that the admission/result-receipt instant always
moves earlier across separate runs.

## Asynchronous worker lifecycle and copied-state qualification

Ordinary planning-result polling is nonblocking and uses atomic state. However,
FSM teardown can reach worker cancellation and `join()` from the controller call
path. No bounded join latency or WCET is established.

Most candidate kinematics use the frozen copied state, but an absolute
"copied-state pure" claim is not supported: residual live fingertip-frame reads
determine gripper aperture, and live model-limit accessors remain. The static
guard does not cover those paths; stable hashes do not prove full purity or race
freedom. These residual issues remain documented and unfixed in the frozen
scientific implementation.

See [`docs/architecture.md`](docs/architecture.md) and
[`docs/corrections_of_record.md`](docs/corrections_of_record.md).

## Limitations

- finite engineering discretization: 14 event times, 32 grasps, 17 routes;
- no bank-resolution/convergence study;
- exactness only over the generated finite bank;
- numerical/local differential IK;
- sampled collision checking: 25 evaluated poses per commanded segment, not
  continuous collision detection or swept-volume proof;
- ground/object-derived proxy obstacle model only; no arbitrary environment,
  self-collision or human-body geometry;
- deterministic prediction; no uncertainty distribution;
- one-shot commit; no post-commit global replanning;
- virtual transfer source by default;
- one object geometry/orientation in the reported campaigns;
- no end-to-end physical-robot or human-subject validation.

## Timing interpretation

Final timing admission is scenario-specific. Historical analytical fail-closed
boundaries derived from complete-plan records are:

| Scenario | Boundary (s) |
| --- | ---: |
| GROUND_NEAR | 3.900000 |
| PURE_X | 3.975000 |
| CANONICAL_YZ | 5.139608 |
| DIAGONAL_XZ | 5.735285 |

The historical `3.976 s` value is the next 1-ms PURE_X grid point above its
exact boundary. It is not a universal planner/hardware deadline. Winner
preservation is a separate and stricter property; see
[`docs/timing_frontiers.md`](docs/timing_frontiers.md).

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

Rejected/unsafe execution paths enter `Failure`.

## Documentation

- [`docs/provenance.md`](docs/provenance.md) — canonical source/run-state map.
- [`docs/corrections_of_record.md`](docs/corrections_of_record.md) — publication
  corrections while frozen bytes remain preserved.
- [`evidence/README.md`](evidence/README.md) — reduced primary evidence.
- [`docs/architecture.md`](docs/architecture.md) — planner/QP and async boundaries.
- [`docs/mathematics.md`](docs/mathematics.md) — finite sets/objective/timing gate.
- [`docs/performance.md`](docs/performance.md) — historical serial and async
  timing evidence, with provenance distinctions.
- [`docs/release_validation.md`](docs/release_validation.md) — historical vs
  current validation scope.
- [`docs/reproducibility.md`](docs/reproducibility.md) — staged reproduction.
- [`docs/real_robot.md`](docs/real_robot.md) — unvalidated hardware status.

## Citation

Paper citation metadata will be added when finalized. Until then, cite the
repository title together with the exact release/tag or commit used for
reproduction.
