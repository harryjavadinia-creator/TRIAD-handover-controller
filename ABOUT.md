# About TRIAD

TRIAD is the receiver-side controller of a Kinova Gen3 arm with a Robotiq
2F-85 gripper, implemented in C++ on mc_rtc. It predicts where the object
will be, enumerates a bounded finite set of complete plans over event time,
grasp and transit route, screens each plan through a modelled feasibility
chain that reaches through acquisition and retreat, commits to one plan, and
executes it. The same controller runs in three modes: the finite-plan mode
(default), a receding mode that re-plans while the object moves, and a
supervisory mode that adds a sampled interception solver and a control-aware
grasp supervisor. A second Kinova Gen3 can act as the giver: the complete
robot-to-robot handover runs in simulation, and the two physical arms are shown
operating together on video (no complete synchronized hardware handover is logged).
Robot A alone has run the four scenarios on the physical arm against the virtual
object, up to the closure of the gripper, and the two arms have run this
repository's two-robot procedure together up to the same point (25 September 2026).

This repository contains the controller and its configuration, the four
reference scenarios and the perception-latency sweep with their logs, the
comparison of the supervisory-mode selectors, the two-robot setup with its
hardware procedure and laboratory videos, verification tools that check every
reported number against source and data, and the paper.

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
lexicographic rule (completion → clearance → effort) captures the
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
giver stops (1.5–3.1 mm, up to 1.6 s early) and then wait for that gate.

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
[grasp family and funnel](supervisory_mode/grasp_family_and_funnel.md),
[interception solver](supervisory_mode/interception_solver.md),
[authority demands](supervisory_mode/authority_demands.md),
[receding execution](supervisory_mode/receding_execution.md),
[supervisor design review](supervisory_mode/supervisor_design_review.md).

**Two robots.** With a second Kinova Gen3 as the giver, the full handover
completes in simulation (Robot B presents, Robot A observes, plans, reaches,
captures and retreats). Phone videos in [`two_robot/media/`](two_robot/media/)
show both physical arms in the laboratory setup, Robot B holding the bottle
while Robot A's gripper approaches and closes on its neck. The July mc_rtc logs
record nine runs that entered `CaptureTransfer` on the physical arms and closed the gripper on the
object, the objective of those sessions (the object was taped to Robot B's tool, no transfer or retreat by
design); see
[`two_robot/`](two_robot/README.md) and the
[video evidence note](two_robot/evidence/PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md).
On 25 September 2026 Robot A alone ran the four reported scenarios on the physical
arm against the virtual object, through observation, planning, the certified reach
and the closure of the physical gripper in every scenario, each run ending in the
fail-safe hold at the closure check because there was no object. The same day the
two arms ran the two-robot procedure of this repository: Robot B presenting under
the giver coordinator, Robot A observing, committing, reaching and closing in step,
with both arms' joints in the binary log
([record](two_robot/evidence/hardware_runs_2026-09-25/README.md)).

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

## Scope

The reference scenarios and the selector comparison are simulation results.
The physical work is the two-robot setup with its videos and the runs of
25 September 2026, single-robot and two-robot (pre-contact sequence only, no
object); there is no validated
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
| `supervisory_mode/` | Supervisory mode: phase reports, copies of `PredictiveInterception.h` and `ControlAwareGraspSupervisor.h` as measured (the other two supervisory-mode headers, `ReceivingGraspFamily.h` and `Gen3WristReachabilityMap.h`, are in `src/`), tooling, and both selector campaigns |
| `two_robot/` | Robot-to-robot handover: giver configuration, hardware procedure, receiver hardware overlay, log inventory, videos |
| `paper/` | The paper and the script that generates its figures from the repository data |

TRIAD is the method name; `HandoverInterceptionController`, `call_handover`,
and `call_object` are implementation identifiers used by the build and logs.

## Paper and hardware

- `paper/triad_system_paper.tex` / `.pdf`: the paper that matches this repository; `paper/make_figures.py`
  regenerates every figure from the records in `evidence/`, `supervisory_mode/` and `two_robot/`.
- `two_robot/`: the robot-to-robot setup, the receiver's hardware overlay, the hardware procedure, the July
  log inventory, the laboratory videos and the single-robot hardware record of 25 September 2026 with its
  runner and homing tool. `docs/real_robot.md` states what has and has not been reached on hardware.
