# TRIAD

**Finite complete-plan selection for predictive human-to-robot handover**

TRIAD is the receiver-side controller of a Kinova Gen3 arm with a Robotiq
2F-85 gripper, implemented in C++ on mc_rtc. It predicts where the object
will be, enumerates a bounded finite set of complete plans over event time,
grasp and transit route, screens each plan through a modelled feasibility
chain that reaches through acquisition and retreat, commits to one plan, and
executes it. The same controller runs in three modes: the finite-plan mode
(default), a receding mode that re-plans while the object moves, and a
supervisory mode that adds a sampled interception solver and a control-aware
grasp supervisor. A second Kinova Gen3 can act as the giver, so a complete
robot-to-robot handover runs in simulation and on the two physical arms of the
laboratory.

This repository contains the controller and its configuration, the four
reference scenarios and the perception-latency sweep with their logs, the
comparison of the supervisory-mode selectors, the two-robot setup with its
hardware procedure and laboratory videos, verification tools that check every
reported number against source and data, and the paper.

## Start here

| To… | Read |
| --- | --- |
| Clone, build and run the simulations yourself | [Clone and run](#clone-and-run), then [Quick start](docs/quickstart.md) |
| Understand the equations and decision rule | [Mathematical formulation](docs/mathematics.md) |
| Follow the algorithm and execution stages | [Architecture and pseudocode](docs/architecture.md) |
| Inspect the simulation results | [Results](docs/results.md) |
| Position the contribution in the literature | [Related work](docs/related_work.md) |
| Read the paper | [`paper/triad_system_paper.pdf`](paper/triad_system_paper.pdf) (source `paper/triad_system_paper.tex`) |
| Run it with two robots, or on the two physical arms | [`two_robot/`](two_robot/README.md), [Real robot](docs/real_robot.md) |
| Check data, source versions, and validation | [Reproducibility](docs/reproducibility.md) |

## Method

The decision is a complete plan $\xi=(\tau,g,r)$: **when** to receive the
object, **which grasp** to use, and **which transit route** to follow.

Each axis is enumerated, then screened by copied-state hard feasibility checks
before any ranking. A final prediction-consistency check precedes a one-time
commitment; TRIAD then governs task-space references that the mc_rtc task/QP
layer realises at the joint level.

**Size of the search.** For the moving-object bank, 14 × 32 × 17 = 7616 bounds
the generated set before pruning. Candidates are screened by hard feasibility
before any ranking, and the timing axis is resolved by admission rather than by
trading against the objective, so the number of plans actually ranked is much
smaller (198 to 432 complete plans and 10 to 227 timing-admissible ones in the
four reference scenarios).

**Origins of the decision rule.** The timing rule is earliest-feasible
rendezvous (Croft, Fenton & Benhabib, *IEEE T-SMC* 1998; Hujić et al.,
*T-Mech* 1998), cited in the source header itself. The grasp stage is a
cheap-ranking → bounded-exact-IK → first-success funnel in the manner of
Akinola et al. (2021). What TRIAD adds is the complete instantiation: the
feasibility screens, the copied-state discipline, the tie conventions, the
timing admission and the evidence trail.

The seven objective weights are fixed engineering preferences;
**no weight-sensitivity result is reported**, and off-line replay found that a
two-level lexicographic rule (completion → clearance → effort) captures the
useful part. The [full formulation](docs/mathematics.md) covers prediction,
local IK, objective terms, timing, ties, and commitment.

## Modes

| mode | configuration | what it does |
| --- | --- | --- |
| finite-plan (default) | `receiverArchitecture: v1_frozen_prereach` | one frozen planning state, the full bank, one commitment, execution through acquisition and retreat |
| receding | `receiverArchitecture: v2_receding` | re-plans against a robot-independent giver while the object moves |
| supervisory | `v2_receding` + `supervisorMode: control_aware` | sampled earliest-feasible interception over a 530-hypothesis grasp family, with a directional QP-authority supervisor |

Acquisition closes on the object once it is at rest (`requireObjectStopped`,
4 mm/s) in every mode; the predictive selectors reach the grasp pose before the
giver stops (1.5–3 mm, up to 1.6 s early) and then wait for that gate.

## Measured performance

**Finite-plan mode.** The four reference scenarios (longitudinal, near-ground,
lateral-low, diagonal) complete with recorded event time, grasp, route and
objective values; the perception-latency sweep shows completion up to 0.30 s
of latency. See [Results](docs/results.md) and [Experiments](docs/experiments.md).

**Supervisory mode.** Four selectors were compared over four scenarios × three
repeats, identical plant, identical grasp pool, one evaluator applied to every
run:

| selector | description | completions |
| --- | --- | --- |
| B0 | reactive tracker | 5/12 |
| B1 | plain predictive interception | 11/12 |
| B2 | predictive with a capability tie-break | 10/12 |
| FULL | predictive with the authority supervisor as a hard filter | 6/12 |

The plain predictive selector completes the most scenarios. Used as a hard
filter the authority supervisor makes selection start-state dependent and
produces adopt/abort cycling, so B1 is the configuration to build on. A first
campaign had reported FULL 12/12 and B1 9/12; that came from a
predictive-rollout defect on replans from a moving arm, which inflated the
path-demand signal the supervisor consumes. Both campaigns are included,
[`supervisory_mode/evidence/phaseF_sim_prefix`](supervisory_mode/evidence/phaseF_sim_prefix)
and [`supervisory_mode/evidence/phaseF_sim`](supervisory_mode/evidence/phaseF_sim),
so the sensitivity is auditable. Supporting analyses:
[grasp family and funnel](supervisory_mode/PHASE_B_GRASP_FAMILY_AND_FUNNEL.md),
[interception solver](supervisory_mode/PHASE_C_INTERCEPTION_SOLVER.md),
[authority demands](supervisory_mode/PHASE_D_AUTHORITY_DEMANDS.md),
[receding execution](supervisory_mode/PHASE_E_RECEDING_EXECUTION.md),
[supervisor audit](supervisory_mode/TRIAD_CONTROL_AWARE_SUPERVISOR_AUDIT.md).

**Two robots.** With a second Kinova Gen3 as the giver, the full handover
completes in simulation (Robot B presents, Robot A observes, plans, reaches,
captures and retreats). Phone videos in [`two_robot/media/`](two_robot/media/)
show both physical arms in the laboratory setup, Robot B holding the bottle
while Robot A's gripper approaches and closes on its neck. The July mc_rtc logs
are inventoried separately and do not record `CaptureTransfer`; see
[`two_robot/`](two_robot/README.md) and the
[video evidence note](two_robot/evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md).

## Evidence

The principal simulation evidence consists of:

- four canonical moving-object scenarios with reference event-time, grasp,
  route, and objective values;
- the corrected perception-latency ablation;
- source, model, and evidence-integrity checks.

See [Results](docs/results.md), [Simulation](docs/simulation.md),
[Experiments](docs/experiments.md), and the [evidence index](evidence/README.md).
Historical implementation-performance measurements are documented separately in
[Performance](docs/performance.md).

## Clone and run

Everything below is executed from a fresh clone; the numbers in `evidence/`
and `results/` are what these commands produce, not something to read instead
of running them. You need Linux, an mc_rtc installation
(`TRIAD_MC_RTC_PREFIX`) and, for the two-robot run, the `Kinova` robot module
from mc_kinova. Step-by-step detail: [Quick start](docs/quickstart.md).

```bash
# 1. TRIAD and the two pinned robot-description packages
git clone https://github.com/harryjavadinia-creator/TRIAD-handover-controller.git
cd TRIAD-handover-controller
export TRIAD_DEPENDENCIES="$PWD/../TRIAD-dependencies"
git clone --depth 1 --branch 0.2.6 https://github.com/Kinovarobotics/ros2_kortex.git "$TRIAD_DEPENDENCIES/ros2_kortex"
git clone --depth 1 --branch 0.0.1 https://github.com/PickNikRobotics/ros2_robotiq_gripper.git "$TRIAD_DEPENDENCIES/ros2_robotiq_gripper"

# 2. the Gen3 + 2F-85 robot module (URDF and 26 meshes are hash-checked)
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf "$TRIAD_DEPENDENCIES/ros2_kortex/kortex_description/robots/gen3_2f85.urdf" \
  --kortex-share  "$TRIAD_DEPENDENCIES/ros2_kortex/kortex_description" \
  --robotiq-share "$TRIAD_DEPENDENCIES/ros2_robotiq_gripper/robotiq_description" \
  --output "$PWD/gen3_2f85_module"

# 3. build (nothing is installed)
export TRIAD_MC_RTC_PREFIX=/path/to/your/mc_rtc/install
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH CMAKE_PREFIX_PATH="$TRIAD_MC_RTC_PREFIX" \
  cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON
cmake --build build -j"$(nproc)"

# 4. run from the build tree
export PATH="$TRIAD_MC_RTC_PREFIX/bin:$PATH"
export MAIN_ROBOT_MODULE_PATH="$PWD/gen3_2f85_module"
export TRIAD_BUILD_DIR="$PWD/build"
export MC_RTC_INSTALL="$TRIAD_MC_RTC_PREFIX"
scripts/run_scenario.sh longitudinal          # one human-to-robot scenario
bash two_robot/run_two_robot_sim.sh 80        # Robot B gives, Robot A receives

# to watch in RViz: open two_robot/display_two_robot.rviz first, then run at half speed
TRIAD_SYNC_RATIO=0.5 scripts/run_scenario.sh lateral-low
TRIAD_SYNC_RATIO=0.5 TRIAD_GIVER_SCENARIO=diagonal_xz bash two_robot/run_two_robot_sim.sh 80
```

What you must see:

| run | success marker |
| --- | --- |
| `scripts/run_scenario.sh <scenario>` | `HANDOVER_COMPLETED=true`, `RUNTIME_CHECKER_RESULT=PASS`, `SCENARIO_IDENTITY_RESULT=PASS`, log under `results/` with the state sequence up to `Completed` and the committed `candidate=… route=… globalJ=…` |
| `two_robot/run_two_robot_sim.sh` | Robot B phases Prepositioning → … → Holding, Robot A states up to `Completed`, `RESULT: COMPLETED`, log and timeline under `two_robot/results/` |

The single-robot scenarios are `longitudinal`, `near-ground`, `lateral-low` and
`diagonal`; the two-robot presentations are `pure_x` (default), `diagonal_xz` and
`static_nominal`. [Quick start §7](docs/quickstart.md#7-watch-it) explains the
viewer, the reason for the half speed while watching, and what you should see. The
[simulation guide](docs/simulation.md) lists the reference winners for each
scenario so that a run can be compared with the recorded one.

To check the included evidence records without a simulator:

```bash
python3 tools/check_evidence_manifest.py
```

To run TRIAD on the physical arms (Robot A alone, or the two-robot setup),
follow [Real robot](docs/real_robot.md) and
[`two_robot/README.md`](two_robot/README.md); the laboratory videos of the two
physical arms are in [`two_robot/media/`](two_robot/media/).

## Scope

The reference scenarios and the selector comparison are simulation results.
The physical work is the two-robot setup and its videos; there is no validated
end-to-end physical human-to-robot handover campaign in this repository, and
physical execution is disabled in the tracked configuration.

The method uses deterministic prediction, a finite engineering discretization,
local numerical IK, and sampled object/ground proxy checks. No bank-resolution
convergence study, continuous collision proof, arbitrary-clutter perception,
human-body model, comprehensive self-collision checking, or post-commit global
replanning is included.

The finite search runs on a background worker. Ordinary result polling is
nonblocking, while shutdown/reset paths may cancel and join the worker; no
WCET or formal schedulability guarantee is given. Residual live
fingertip-frame reads affect aperture checks, so complete copied-state purity
or formal race freedom is not established. See
[Architecture](docs/architecture.md) and [Real robot](docs/real_robot.md).

## Repository contents

| Directory | Contents |
| --- | --- |
| `src/` | The controller: finite-plan selectors and execution states, the receding receiver (`ReceiverV2.cpp`), the robot-independent giver model, the supervisory-mode interception solver and grasp supervisor, the two-robot giver coordinator |
| `etc/`, `configs/` | Controller and simulation configuration |
| `call_object_description/` | Handover object model |
| `scripts/`, `tools/` | Reproduction, verification, and figure generation |
| `docs/` | Method, setup, results, provenance, and technical notes |
| `evidence/` | Evidence records and integrity manifests |
| `supervisory_mode/` | Supervisory mode: phase reports, the two headers the results rest on, tooling, and both selector campaigns |
| `two_robot/` | Robot-to-robot handover: giver configuration, hardware procedure, receiver hardware overlay, log inventory, videos |
| `paper/` | The paper and the script that generates its figures from the repository data |

TRIAD is the method name; `HandoverInterceptionController`, `call_handover`,
and `call_object` are implementation identifiers used by the build and logs.

## Citation

Cite the repository title together with the commit or tag used for the reported
results.
