# Reduced evidence package

This directory publishes the immutable primary records needed to check every
headline claim made in the README and in [`../docs/experiments.md`](../docs/experiments.md).
It is a curated subset of a larger archive; every file here is byte-identical
to its archived original and its origin is recorded in
[`MANIFEST.sha256`](MANIFEST.sha256).

Source-state provenance for every campaign below is given in
[`../docs/provenance.md`](../docs/provenance.md). Nothing in this directory is
a physical-robot result.

One file is a labelled exception to byte-identity:
`robustness/generate_perturbations.py` is a published variant of the campaign
script in which two hard-coded archive paths were replaced by command-line
arguments, so that it runs anywhere. Its anchor rule and every perturbation
magnitude are unchanged, and it reproduces `perturbations.csv` byte for byte:

```bash
cd evidence/robustness
python3 generate_perturbations.py ../generalization/held_out_scenarios.csv /tmp/p.csv
cmp /tmp/p.csv perturbations.csv
```

The sha256 of the original campaign script is recorded in its docstring and in
`ORIGINS.txt`.

## Verify the package

```bash
cd evidence && sha256sum -c MANIFEST.sha256
python3 ../tools/check_evidence_manifest.py
```

## `generalization/` — 62 predeclared held-out scenarios  (source state 4)

The scenario set was generated from a fixed seed and **hashed before any
scenario was executed**; `held_out_scenarios.csv.sha256` is that
pre-registration record. The four development scenarios are excluded by
construction. All runs used ideal sensing.

| outcome | n | share |
| --- | ---: | ---: |
| A — plan found and completed | 36 | 58.1 % |
| B — committed, then execution failed | 2 | 3.2 % |
| C — no physically feasible plan | 3 | 4.8 % |
| D — no timing-admissible plan | 18 | 29.0 % |
| E — commit freshness rejected | 3 | 4.8 % |
| **total** | **62** | |

Committed **38/62**; of those, **36/38 = 94.7 %** completed.

Completion by object height band:

| band z [m] | completed | n | share |
| --- | ---: | ---: | ---: |
| 0.18-0.25 | 4 | 11 | 36 % |
| 0.30-0.40 | 25 | 42 | 60 % |
| 0.45-0.55 | 7 | 9 | 78 % |

Among the completions the planner selected 9 distinct event hypotheses, 11 distinct grasps, 12 distinct routes and produced 36 distinct frozen plan sets.

### How these outcomes must be read

Outcomes C, D and E are **fail-closed rejections in which the robot never
moves**; they are not execution failures. Only B is a failure after
commitment.

**The 18 category-D outcomes remain unresolved with respect to the true
physical feasible domain.** The independent offline analysis in
[`../feasibility_reference/`](../evidence/feasibility_reference/) excludes gross
kinematic unreachability and a gross joint-velocity temporal bound as
explanations, but it is deliberately permissive: it admits any wrist
orientation, models no gripper proxy hierarchy, no grasp corridor and no
closure sweep, and consequently marks all 62 scenarios feasible. It therefore
**discriminates nothing**, and the coverage figures derived from it are lower
bounds only.

> **Supersession notice.** An earlier internal report on this campaign stated
> that in these cases the arm could not reach those events in time. That
> interpretation was withdrawn by the independent feasibility audit
> ([`../feasibility_reference/TIMING_REJECTION_AUDIT.md`](feasibility_reference/TIMING_REJECTION_AUDIT.md))
> and must not be repeated. What is established is only that all 18 produced
> complete plans — between 3 and 356 of them — none of which satisfied the
> timing-admission rule at the instant the completed search was received.

Similarly, the monotone height trend is an **empirical association**. The
attribution of that trend to the interaction between a fixed finite grasp and
route bank and a shrinking admissible approach cone is an attribution, not a
proof: nothing enumerates which specific grasps become inadmissible.

## `robustness/` — 66 predeclared local perturbations  (source state 4)

Six anchors were selected by a rule fixed **before any perturbation ran**,
consulting only scenario geometry and never an outcome
(`generalization/ROBUSTNESS_ANCHOR_RULE.md`). The perturbation set was hashed
before execution.

| outcome | all 66 | excluding anchor H002 |
| --- | ---: | ---: |
| completed | 40/66  (60.6 %) | 37/55  (67.3 %) |
| safely rejected | 14/66  (21.2 %) | 14/55  (25.5 %) |
| execution failure after commitment | 12/66  (18.2 %) | 4/55  (7.3 %) |
| completed given commitment | 40/52  (76.9 %) | 37/41  (90.2 %) |

**The left-hand column is the result.** Anchor H002 was chosen by the
pre-registered rule and is not removed from it. Its own unperturbed case
already fails at the dynamic clearance reserve — roughly 0.0079 m against the
0.0080 m minimum — and no tested perturbation moves it off that boundary, so
eight of the twelve execution failures belong to that single anchor. The
right-hand column isolates its influence and is diagnostic context only.

The perturbation magnitudes (±20 and ±40 mm in position, ±0.02 m/s in speed,
±10° in direction) are **experimental values, not system specifications**.
All twelve execution failures entered the fail-safe hold; no run exhibited any
fallback behaviour.

## `latency/` — corrected perception-latency ablation  (source state 5)

`sweep_rows.json` is the corrected sweep. The pre-correction sweep is
**invalid** — a configuration-mirror defect pinned every configured delay to
0.220 s — and is deliberately **not** published here; it must never be plotted
as evidence.

The unit ratio between the uncompensated estimation error and `v·tau` is an
identity of the straight constant-velocity simulated trajectory, not an
empirical fit.

## `async/` — control-loop timing and plan-set equivalence  (source state 5)

For each of the four canonical scenarios: the frozen plan set at 17
significant digits, its hash, the pre-thread hash, the worker timing record,
the timing-frontier replay and a clean-machine control-loop profile.

Three of the four scenarios reproduce the control-thread plan-set hash
exactly. `near-ground` is a strict superset: all 229 control-thread records
are byte-identical and 54 further records are enumerated, because pinning the
admission instant to the search epoch removes a wall-clock-dependent prune
that had skipped one event hypothesis before its geometry ran.

`perf_analysis_control_thread_reference.txt` is the one surviving raw
pre-thread control-loop profile. The complete four-scenario before/after
comparison is reported in [`../docs/performance.md`](../docs/performance.md).

`near-ground/normalized/` carries the per-candidate streams from which the
full feasibility funnel of that run can be recomputed exactly.

## `feasibility_reference/` — independent offline analysis

A deliberately permissive upper-bound analysis built from the same robot
model, object geometry and ground model but **not** from TRIAD's banks. Read
`README_METHOD.md` before using any number from it: it discriminates nothing,
and its own limitations are the reason the category-D outcomes remain
unresolved.

## `safety/` — planner purity guard

The repository's static purity guard output and its mutation-test record: nine
deliberate reintroductions of live-state reads, nine caught.

The guard's claim must be stated in its narrow form: *the copied-state planner
reads no live robot pose or configuration after the snapshot; it does still
read three model-constant quantities — joint position limits, joint velocity
limits and the open-gripper mouth half-gap — through accessors the guard does
not cover.* See [`../docs/architecture.md`](../docs/architecture.md).
