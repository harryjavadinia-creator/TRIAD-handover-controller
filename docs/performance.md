# Exact serial performance study

This study reduced planner wall time without changing TRIAD's scientific decision problem. The optimized implementation is the source currently shipped on the publication branch; the frozen Dataset-B tag remains the provenance anchor for the original four-scenario scientific campaign.

## Fixed scientific contract

The serial optimization campaign held fixed:

- event, grasp and route banks;
- complete-plan enumeration;
- hard feasibility and timing-admission rules;
- objective terms and weights;
- strict/tie semantics;
- collision proxy and swept-pose sampling;
- `planningStepsPerCycle = 96`;
- logical planning cycle count and controller `now`.

No multicore planner, asynchronous architecture, timing-policy change or methodology change was introduced.

## Optimization A: hierarchy distance overhead

Profiling identified repeated construction of segment-invariant quantities in the clearance hierarchy. The optimized path precomputes those invariants once per query and uses an internal helper for the repeated point/segment evaluations.

The brute-force oracle path remains separate. Geometry, thresholds, traversal order, sampled poses and comparison semantics are unchanged.

## Optimization B: IK preview bookkeeping

The second optimization removes repeated bookkeeping around unchanged IK arithmetic:

- construct an invariant `rbd::Jacobian` once rather than once per preview call;
- prepare constant velocity-limit vectors once;
- replace repeated joint-name copies/prefix tests with a precomputed mask.

No IK equation, iteration rule, seed, convergence condition, tolerance or final-configuration arithmetic is changed.

## Equivalence evidence

Across the four moving-object scenarios, the optimized implementation reproduced:

- byte-identical complete-plan/timing-admissibility record streams;
- identical final selector `now`;
- identical logical planning cycle counts;
- identical event/grasp/route winners;
- identical objective values;
- identical timing-admissible sets.

The independent collision oracle compared **8,168,732** evaluations with zero mismatches across the full report contract:

| Scenario | Oracle comparisons | Mismatches |
| --- | ---: | ---: |
| CANONICAL_YZ | 2,977,590 | 0 |
| GROUND_NEAR | 2,003,452 | 0 |
| PURE_X | 1,926,145 | 0 |
| DIAGONAL_XZ | 1,261,545 | 0 |
| **Total** | **8,168,732** | **0** |

The oracle can be rerun for a scenario with:

```bash
tools/check_collision_hierarchy_oracle.sh longitudinal
```

Oracle mode intentionally evaluates both collision paths and must not be used for performance measurement.

## Correctly scoped wall time

An early exploratory timing window included work outside the `SolveInterception` planning state. The results below supersede that measurement and use a state-scoped planner wall interval.

| Scenario | Baseline wall (s) | Final wall (s) | Paired speedup |
| --- | ---: | ---: | ---: |
| CANONICAL_YZ | 4.3488 | 3.9724 | ~1.100× |
| GROUND_NEAR | 3.3151 | 3.0717 | ~1.085× |
| PURE_X | 3.3930 | 3.1615 | ~1.075× |
| DIAGONAL_XZ | 2.7966 | 2.6199 | ~1.068× |

CANONICAL_YZ used 10 interleaved baseline/final pairs: all 20 runs passed the runtime and scenario-identity gates, all 10 pairs favored the optimized implementation, and all runs retained 879 logical planning cycles.

## Synchronous control-cycle behavior

For CANONICAL_YZ:

- cycles above 100 ms changed from `[5,6,7,6,5,7,9,5,5,6]` to `[3,1,2,1,2,1,2,1,1,1]`;
- maximum observed cycle duration changed from about 137.3 ms to 125.3 ms;
- cycles above 50 ms remained 41 in every run of both arms.

The planner is still synchronous. These changes reduce blocking; they do not make the planner asynchronous or non-blocking.

## Relation to timing frontiers

Timing frontiers are scenario-specific. `3.976 s` is the historical PURE_X 1-ms grid point immediately above its exact fail-closed boundary, not a universal planner threshold.

The measured final implementation lies inside every scenario's own fail-closed boundary, while its measured wall time lies outside every frozen simulated winner-preservation band. Timing feasibility and winner preservation are therefore separate results.

See [`timing_frontiers.md`](timing_frontiers.md).

## Publication source state

The exact optimized implementation was synchronized from development commit `82e6eaa`. The three imported implementation files and their SHA-256 digests are recorded in [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256).

A clean configure/build and all four Dataset-B reproduction runs were revalidated after synchronization. See [`release_validation.md`](release_validation.md).

---

## Control-loop cost of planning (asynchronous planner)

The study above measures the *serial* wall time of the search. This section
measures something different: what the search costs the **1 kHz control
callback** while it runs. Both are reported because they are not the same
quantity.

On the source state published on this branch, the complete finite search runs on
one background worker. Matched before/after measurement on the four canonical
scenarios, of the in-planning `ControllerRun` statistic:

| scenario | median before → after [ms] | p90 before → after [ms] | max before → after [ms] | cycles > 1 ms before → after |
| --- | --- | --- | --- | ---: |
| lateral-low | 0.642 → 0.020 | 3.357 → 0.073 | 6.168 → 1.172 | 1362 → 1 |
| near-ground | 0.542 → 0.019 | 3.180 → 0.044 | 12.201 → 0.992 | 928 → 0 |
| longitudinal | 0.597 → 0.042 | 3.365 → 0.079 | 4.917 → 1.014 | 756 → 1 |
| diagonal | 0.631 → 0.048 | 3.175 → 0.082 | 5.016 → 1.088 | 424 → 1 |

After the change at most one cycle per run exceeds 1 ms, and it is the
result-receipt cycle — the selector plus the commit — not planning. No cycle
exceeds 2 ms. `GlobalRun` above 50 ms and above 100 ms is zero in all four
scenarios; the largest `GlobalRun` outliers are 99 per cent or more logging.

**Provenance and its limits.** The "after" column is independently reproduced by
the clean-machine profiles published as
`evidence/async/<scenario>/perf_analysis_clean_machine.txt`. Of the "before"
state, one raw profile survives in the archive and is published as
`evidence/async/perf_analysis_control_thread_reference.txt` (in-planning
`ControllerRun` p90 3.409 ms, 1374 cycles above 1 ms); the complete
four-scenario before/after table above is the measurement preserved in the
development record of the change.

**What must not be claimed.** This is an empirical tail measurement on four
scenarios on one machine. It is **not** a hard real-time guarantee, **not** a
worst-case execution-time bound and **not** a formal schedulability proof. Wall
time remains machine-dependent and is excluded from the deterministic
reproducibility contract.

## Scientific equivalence of the asynchronous planner

Frozen plan sets, at 17 significant digits:

| scenario | control-thread records | worker records | relation |
| --- | ---: | ---: | --- |
| lateral-low | 432 | 432 | identical hash |
| longitudinal | 198 | 198 | identical hash |
| diagonal | 233 | 233 | identical hash |
| near-ground | 229 | 283 | strict superset — all 229 byte-identical |

The near-ground difference has a single, identified cause. In control-thread
mode a per-hypothesis prune skipped an event hypothesis **before its geometry
ran** once the elapsed wall time exceeded `L − 1.6 s`; the gate removes whole
hypotheses, that is all 32 × 17 of their combinations. On the worker the
admission instant is pinned to the search epoch, so the condition reduces to
`lead ≥ 1.6 s` while the minimum configured lead is 1.8 s — it can never fire.

The defensible statement is:

> the same frozen scientific evaluation, with the wall-clock-dependent premature
> enumeration truncation removed and the admission instant moved earlier.

Do **not** write "scientifically equivalent" without that qualification, do
**not** claim the two modes produce the same plan set, and do **not** say the
additional records were previously rejected on scientific grounds — they were
never enumerated. Conversely, the control-thread prune was itself logically
sound: time only advances, so a hypothesis already below the safe commit lead
could not have been committed later in that same run either.

The published records are under [`../evidence/async/`](../evidence/async/).
