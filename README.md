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

**A later variant was measured against baselines, and lost.** *TRIAD-lite* — a
separate line of work that adds a predictive interception solver and a
directional QP-authority filter — was compared against matched baselines under an
identical plant. The full variant completed 6/12 scenarios against 11/12 for a
plain predictive baseline. That comparison is reported in full under
[`triad_lite/`](triad_lite/) and summarised in
[Measured performance](#measured-performance). **It concerns TRIAD-lite, not the
controller in `src/` on this branch**, and is included because it is the part of
the work most useful to anyone building on it.

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
**no weight-sensitivity result is reported**, and a later analysis found a
two-level lexicographic rule (completion → clearance → effort) captures the
useful part. The [full formulation](docs/mathematics.md) covers prediction,
local IK, objective terms, timing, ties, and commitment.

## Measured performance

**Scope of this section.** These results are from **TRIAD-lite**, a later variant
developed on `research/triad-control-aware-supervisor` (starting HEAD `04e9efc`),
which adds a predictive interception solver and a directional QP-authority filter.
They are *not* a measurement of the controller in `src/` on this branch. The two
headers the claims below rest on are included as
[`triad_lite/src/`](triad_lite/src/), together with the phase reports and both
campaigns, so every figure here can be checked against source and data.

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

**No control-performance gap is established.** For TRIAD-lite the measured
comparison does not show the joint selection outperforming a plain predictive
baseline, and the authority filter's benefit is unsupported. For the controller in
`src/` on this branch, no equivalent baseline comparison has been run at all — its
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
| `src/` | Controller, finite selectors, and execution states |
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
