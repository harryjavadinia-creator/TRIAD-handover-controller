# TRIAD

**An auditable implementation of event-time, grasp and route selection for
human-to-robot handover**

TRIAD is a research controller for a Kinova Gen3 arm with a Robotiq 2F-85
gripper, implemented in C++ on mc_rtc. It enumerates a bounded finite set of
(event-time, grasp, route) plans, screens them by copied-state feasibility
checks, commits once, and executes through acquisition and retreat.

**What this repository is for.** It is a complete, reproducible instantiation
with its decision structure written down, its prior art attributed, and its
performance measured against matched baselines — including where those
measurements are unfavourable. It is not a claim of a new decision method; the
structure is prior art, and we say whose below.

**Validation scope: simulation.** The repository contains the controller,
mathematical formulation, canonical simulation scenarios, verification tools, and
source/evidence provenance. There is no validated end-to-end physical
human-to-robot handover campaign.

**The TRIAD-lite mode was measured against baselines, and lost.** *TRIAD-lite* adds
a predictive interception solver and a directional QP-authority filter to the
receiver; compared against matched baselines under an identical plant, the full
variant completed 6/12 scenarios against 11/12 for a plain predictive baseline.
That comparison is reported in full under [`triad_lite/`](triad_lite/) and
summarised in [Measured performance](#measured-performance). The TRIAD-lite code
is part of `src/` (see [ABOUT.md](ABOUT.md)); the default configuration runs the
finite-plan controller, and the comparison is included because it is the part of
the work most useful to anyone building on it.

**Two robots.** The July 2026 laboratory setup — Robot A receiving from a second Kinova Gen3 that presents the
object — is integrated here and verified in simulation (a full robot-to-robot handover completes).
Real-world phone videos now included under [`two_robot/media/`](two_robot/media/) directly document both
physical Kinova arms operating together in the laboratory handover setup; one clip shows Robot B
supporting/presenting the bottle while Robot A's Robotiq gripper approaches and closes around the bottle
neck. The July mc_rtc logs are inventoried separately and, in that log inventory, do not record
`CaptureTransfer`. See [`two_robot/`](two_robot/README.md) and the
[physical-video evidence note](two_robot/evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md).

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

**How large the search actually is.** For the moving-object bank,
14 × 32 × 17 = 7616 bounds the generated set before pruning. That figure is an
upper bound on generation, not a count of ranked alternatives, and two
mechanisms reduce it:

That figure bounds generation, and the number of alternatives actually ranked is
smaller: candidates are screened by hard feasibility before any ranking, and the
timing axis is resolved by admission rather than by trading against the objective.
The TRIAD-lite measurements in [Measured performance](#measured-performance)
quantify how far this goes in that variant.

**Attribution.** This decision structure is not new and we do not present it as
new. The timing rule is earliest-feasible rendezvous (Croft, Fenton & Benhabib,
*IEEE T-SMC* 1998; Hujić et al., *T-Mech* 1998), cited in the source header
itself. The grasp stage is a cheap-ranking → bounded-exact-IK → first-success
funnel in the manner of Akinola et al. (2021). TRIAD's contribution is the
instantiation: the feasibility screens, the copied-state discipline, the tie
conventions, and the evidence trail — not the decision rule.

The seven objective weights are fixed engineering preferences;
**no weight-sensitivity result is reported**, and off-line replay found that a
two-level lexicographic rule (completion → clearance → effort) captures the
useful part. The [full formulation](docs/mathematics.md) covers prediction,
local IK, objective terms, timing, ties, and commitment.

## Measured performance

**Scope of this section.** These results are from **TRIAD-lite**, the mode that
adds a predictive interception solver and a directional QP-authority filter.
They are *not* a measurement of the default finite-plan controller. The
TRIAD-lite sources live in `src/` (`ControlAwareGraspSupervisor.h`,
`PredictiveInterception.h`, `ReceivingGraspFamily.h`, `Gen3WristReachabilityMap.h`)
and are selected with `receiverArchitecture: v2_receding` plus
`supervisorMode: control_aware`; the two headers the claims rest on are also kept
as [`triad_lite/src/`](triad_lite/src/), with the phase reports and both campaigns,
so every figure here can be checked against source and data.

Four selectors compared over four scenarios x three repeats, identical plant,
identical grasp pool, one evaluator applied to every run:

| selector | description | completions |
| --- | --- | --- |
| B0 | reactive baseline | 5/12 |
| **B1** | **plain predictive baseline** | **11/12** |
| B2 | capability tie-break | 10/12 |
| **FULL** | **joint selection with the authority filter** | **6/12** |

**The proposed variant is the second-worst of the four.** Used as a hard filter it
makes selection start-state dependent and produces adopt/abort cycling.

A first campaign reported FULL 12/12, B1 9/12. That result came from a
predictive-rollout defect on replans from a moving arm, which inflated exactly the
path-demand signal the authority filter consumes. Both campaigns are included —
[`triad_lite/evidence/phaseF_sim_prefix`](triad_lite/evidence/phaseF_sim_prefix)
and [`triad_lite/evidence/phaseF_sim`](triad_lite/evidence/phaseF_sim) — so the
sensitivity is auditable.

**Acquisition requires a stationary object (TRIAD-lite).** `requireObjectStopped`
defaults to `true`. The predictive variants meet the grasp pose before the giver
stops (1.5-3 mm, up to 1.6 s early), but acquisition itself is gated on the object
having stopped; the variant does not close on a moving object.

**Timing reduces to earliest-feasible (TRIAD-lite).** The τ axis is an ascending
first-feasible scan and the selector is `selectEarliestInterception`: admissible →
within a tie band of the earliest → tie-breaks → final order by τ. Below
`restLinearSpeed = 0.004` m/s the τ axis emits a single event
(`triad_lite/src/PredictiveInterception.h`), so with
`requireObjectStopped = true` the operative event-time axis is of size 1. Grasp
and route break ties among plans already near the earliest feasible one; they do
not trade against it.

Supporting analyses: [Phase B](triad_lite/PHASE_B_GRASP_FAMILY_AND_FUNNEL.md),
[C](triad_lite/PHASE_C_INTERCEPTION_SOLVER.md),
[D](triad_lite/PHASE_D_AUTHORITY_DEMANDS.md),
[E](triad_lite/PHASE_E_RECEDING_EXECUTION.md),
[supervisor audit](triad_lite/TRIAD_CONTROL_AWARE_SUPERVISOR_AUDIT.md).

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
```

What you must see:

| run | success marker |
| --- | --- |
| `scripts/run_scenario.sh <scenario>` | `HANDOVER_COMPLETED=true`, `RUNTIME_CHECKER_RESULT=PASS`, `SCENARIO_IDENTITY_RESULT=PASS`, log under `results/` with the state sequence up to `Completed` and the committed `candidate=… route=… globalJ=…` |
| `two_robot/run_two_robot_sim.sh` | Robot B phases Prepositioning → … → Holding, Robot A states up to `Completed`, `RESULT: COMPLETED`, log and timeline under `two_robot/results/` |

The other scenarios are `near-ground`, `lateral-low` and `diagonal`. To watch a
run, open an mc_rtc viewer (RViz or mc-rtc-magnum) before starting it;
[Quick start §7](docs/quickstart.md#7-watch-it) gives the commands. The
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

## Scope and limitations

**No control-performance gap is established.** For TRIAD-lite the measured
comparison does not show the joint selection outperforming a plain predictive
baseline, and the authority filter's benefit is unsupported. For the default
finite-plan controller, no equivalent baseline comparison has been run at all — its
scenarios are reference runs, not a controlled comparison against an alternative
method. Read the contribution as a formulation, an implementation and an evidence
trail, not as a demonstrated control improvement. Anyone building on the
predictive variant should treat B1 (plain predictive) as the baseline to beat,
because on this evidence it is.

The method uses deterministic prediction, a finite engineering discretization,
local numerical IK, and sampled object/ground proxy checks. No bank-resolution
convergence study, continuous collision proof, arbitrary-clutter perception,
human-body model, comprehensive self-collision checking, or post-commit global
replanning is established.

The finite search is executed by a background worker. Ordinary result polling
is nonblocking, while shutdown/reset paths may cancel and join the worker; no
WCET or formal schedulability guarantee is claimed. Residual live
fingertip-frame reads affect aperture checks, so complete copied-state purity or
formal race freedom is not established. See [Architecture](docs/architecture.md)
and [Hardware status](docs/real_robot.md).

## Repository contents

| Directory | Contents |
| --- | --- |
| `src/` | V1 controller, finite selectors and execution states; the V2 receding receiver (`ReceiverV2.cpp`), the independent giver model and the TRIAD-lite supervisor and interception solver |
| `etc/`, `configs/` | Controller and simulation configuration |
| `call_object_description/` | Handover object model |
| `scripts/`, `tools/` | Reproduction, verification, and figure generation |
| `docs/` | Method, setup, results, provenance, and technical notes |
| `evidence/` | Evidence records and integrity manifests |
| `triad_lite/` | TRIAD-lite variant: phase reports, the two headers the claims rest on, tooling, and the pre/post-fix campaigns |

TRIAD is the method name; `HandoverInterceptionController`, `call_handover`,
and `call_object` are implementation identifiers used by the build and logs.

## Citation

Cite the repository title together with the commit or tag used for the reported
results.
