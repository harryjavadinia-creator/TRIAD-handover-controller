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
