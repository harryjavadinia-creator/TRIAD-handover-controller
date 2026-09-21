# TRIAD

**Joint Event-Time, Grasp and Route Selection for Predictive Human-to-Robot Handover**

TRIAD is a research controller for a Kinova Gen3 arm with a Robotiq 2F-85
gripper. It predicts future object-presentation events, compares complete
event–grasp–route plans, and executes one admissible plan through acquisition
and retreat. It is implemented in C++ using mc_rtc.

**Validation scope: simulation.** The repository contains the controller,
mathematical formulation, canonical simulation scenarios, verification tools, and
source/evidence provenance. There is no validated end-to-end physical
human-to-robot handover campaign.

> **Read [`triad_lite/TRIAD_PREDICTIVE_INTERCEPTION_FINAL_REPORT.md`](triad_lite/TRIAD_PREDICTIVE_INTERCEPTION_FINAL_REPORT.md)
> before using the method.** A controlled comparison against matched baselines
> finds that the full joint selection **does not outperform a plain predictive
> baseline** — 6/12 against 11/12 scenario completions. The measured benefit of
> the authority filter is **not supported**. Details in
> [Measured performance](#measured-performance) below.

## Start here

| To… | Read |
| --- | --- |
| Build the controller and watch a simulation | [Quick start](docs/quickstart.md) |
| Understand the equations and decision rule | [Mathematical formulation](docs/mathematics.md) |
| Follow the algorithm and execution stages | [Architecture and pseudocode](docs/architecture.md) |
| Inspect the simulation results | [Results](docs/results.md) |
| Position the contribution in the literature | [Related work](docs/related_work.md) |
| Check data, source versions, and validation | [Reproducibility](docs/reproducibility.md) |

## Method

The decision is a complete plan $\xi=(\tau,g,r)$: **when** to receive the
object, **which grasp** to use, and **which transit route** to follow.

For the reported moving-object bank, **14 × 32 × 17 = 7616** is the upper
number of combinations before pruning. Each retained alternative is previewed
through the modeled handover phases and ranked only after hard feasibility
checks pass. At result receipt, the controller reapplies timing admission:

```math
\xi^*\in
\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
```

Selection is exhaustive over this generated finite set, with a documented
numerical tie convention. A final prediction-consistency check precedes the
one-time commitment. TRIAD then generates and governs task-space references;
the mc_rtc task/QP layer realizes those references at the joint level.

The seven objective weights are fixed engineering preferences;
**no weight-sensitivity result is reported**. The [full formulation](docs/mathematics.md)
covers prediction, local IK, objective terms, timing, ties, and commitment.

## Measured performance

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

An earlier campaign reported FULL 12/12, B1 9/12. That result came from a
predictive-rollout defect on replans from a moving arm, which inflated exactly the
path-demand signal the authority filter consumes. Both campaigns are included —
[`triad_lite/evidence/phaseF_sim_prefix`](triad_lite/evidence/phaseF_sim_prefix)
and [`triad_lite/evidence/phaseF_sim`](triad_lite/evidence/phaseF_sim) — so the
sensitivity is auditable.

**Acquisition requires a stationary object.** `requireObjectStopped` defaults to
`true` (`src/HandoverInterceptionController.h`). The predictive variants meet the
grasp pose before the giver stops (1.5-3 mm, up to 1.6 s early), but acquisition
itself is gated on the object having stopped. The stack does not close on a
moving object.

**Selection reduces to earliest-feasible.** Across the reported bank the argmin of
the global objective coincides with the earliest timing-admissible candidate, so
the event-time term is effectively imposed by the feasibility frontier rather than
traded off against grasp and route.

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

```bash
git clone https://github.com/harryjavadinia-creator/TRIAD-handover-controller.git
cd TRIAD-handover-controller
```

Follow [Quick start](docs/quickstart.md) to prepare mc_rtc, reconstruct the
robot module, build TRIAD, and open the viewer. Once configured, run:

```bash
scripts/run_scenario.sh longitudinal
```

The other scenarios are `near-ground`, `lateral-low`, and `diagonal`.
The [simulation guide](docs/simulation.md) gives their inputs, completion
markers, and reference outputs.

To verify the included evidence without installing a simulator:

```bash
python3 tools/check_evidence_manifest.py
```

## Scope and limitations

**No control-performance gap is established.** The measured comparison above does
not show that the joint event-time/grasp/route selection outperforms a plain
predictive baseline, and the authority filter's benefit is not supported. The
contribution should be read as a formulation and an implementation, not as a
demonstrated control improvement.

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
| `src/` | Controller, finite selectors, and execution states |
| `etc/`, `configs/` | Controller and simulation configuration |
| `call_object_description/` | Handover object model |
| `scripts/`, `tools/` | Reproduction, verification, and figure generation |
| `docs/` | Method, setup, results, provenance, and technical notes |
| `evidence/` | Evidence records and integrity manifests |
| `triad_lite/` | Controlled baseline comparison: phase reports, tooling, and the pre/post-fix campaigns |

TRIAD is the method name; `HandoverInterceptionController`, `call_handover`,
and `call_object` are implementation identifiers used by the build and logs.

## Citation

Cite the repository title together with the commit or tag used for the reported
results.
