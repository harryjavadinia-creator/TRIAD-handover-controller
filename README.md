# TRIAD

**Joint Event-Time, Grasp and Route Selection for Predictive Human-to-Robot Handover**

TRIAD is a research controller for a Kinova Gen3 arm with a Robotiq 2F-85
gripper. It predicts future object-presentation events, compares complete
event–grasp–route plans, and executes one admissible plan through acquisition
and retreat. It is implemented in C++ using mc_rtc.

**Validation scope: simulation.** This release publishes the asynchronous
controller, mathematical method, canonical scenario commands, latency evidence,
and asynchronous control-loop evidence. There is no validated end-to-end
physical human-to-robot handover campaign.

## Start here

| To… | Read |
| --- | --- |
| Build the controller and watch a simulation | [Quick start](docs/quickstart.md) |
| Understand the equations and decision rule | [Mathematical formulation](docs/mathematics.md) |
| Follow the algorithm and execution stages | [Architecture and pseudocode](docs/architecture.md) |
| See the current release evidence | [Results](docs/results.md) |
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

## Current release evidence

This release keeps the evidence that is directly used for the present
controller/method discussion:

- the four canonical moving-object simulation scenarios and their historical
  reference winners;
- the corrected perception-latency ablation;
- asynchronous planning/control-loop profiles and frozen plan-set checks;
- planner-core integrity checks.

See [Results](docs/results.md), [Simulation](docs/simulation.md),
[Performance](docs/performance.md), and the [evidence index](evidence/README.md).

## Clone and run

```bash
git clone --branch publication/supervisor-release https://github.com/harryjavadinia-creator/TRIAD-handover-controller.git
cd TRIAD-handover-controller
```

Follow [Quick start](docs/quickstart.md) to prepare mc_rtc, reconstruct the
robot module, build TRIAD, and open the viewer. Once configured, run:

```bash
scripts/run_scenario.sh longitudinal
```

The other scenarios are `near-ground`, `lateral-low`, and `diagonal`.
The [simulation guide](docs/simulation.md) gives their inputs, completion
markers, and historical reference outputs.

To verify the published evidence without installing a simulator:

```bash
python3 tools/check_evidence_manifest.py
```

## Scope and limitations

The method uses deterministic prediction, a finite engineering discretization,
local numerical IK, and sampled object/ground proxy checks. No bank-resolution
convergence study, continuous collision proof, arbitrary-clutter perception,
human-body model, comprehensive self-collision checking, or post-commit global
replanning is established.

Ordinary worker-result polling is nonblocking; FSM teardown can reach worker
cancellation and `join()`. Residual live fingertip-frame reads affect aperture
checks. The measured timing is not WCET, and full copied-state purity or race
freedom is not established. See [Architecture](docs/architecture.md),
[Performance](docs/performance.md), and [Hardware status](docs/real_robot.md).

## Repository contents

| Directory | Contents |
| --- | --- |
| `src/` | Controller, finite selectors, and execution states |
| `etc/`, `configs/` | Controller and simulation configuration |
| `call_object_description/` | Handover object model |
| `scripts/`, `tools/` | Reproduction, verification, and figure generation |
| `docs/` | Method, setup, results, and technical appendices |
| `evidence/` | Current release evidence and integrity checks |

TRIAD is the method name; `HandoverInterceptionController`, `call_handover`,
and `call_object` are implementation identifiers used by the build and logs.

## Citation

Cite the repository title and the release or commit used. Publication citation
metadata will be added when a paper is finalized.
