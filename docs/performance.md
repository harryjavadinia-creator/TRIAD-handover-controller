# Exact serial performance study

This document summarizes the implementation-only serial acceleration work performed after the frozen Dataset-B scientific baseline. The purpose was to reduce planner wall time **without changing the scientific decision problem**.

The optimization study is separate from the Dataset-B scientific baseline: the baseline remains the provenance anchor for the reported finite event-time–grasp–route experiment set.

## Constraints of the optimization study

The following were held fixed:

- event-hypothesis bank
- grasp bank and route bank
- complete-plan enumeration
- hard feasibility tests
- timing-admission rule
- objective terms and weights
- strict/tie semantics
- collision proxy count and swept-pose count
- `planningStepsPerCycle = 96`
- logical planning cycle count and controller `now`

No multicore planning, asynchronous planning, timing-policy change, or scientific-methodology change was introduced.

## Optimization A — hierarchy distance overhead

Profiling of CANONICAL_YZ found approximately 165.8 million calls to `pointSegmentDistance` in the hierarchy path. Disassembly showed the hot calls were reached through the PLT and repeatedly reconstructed segment invariants for only three segments per query.

The exact optimization prepares the invariant segment quantities once per query and reuses an internal-linkage helper. The independent brute-force/oracle path remains on the original member function.

This changes where invariant expressions are computed, not the collision geometry, thresholds, hierarchy traversal, sample set, or comparison semantics.

## Optimization B — IK preview bookkeeping

The second exact optimization removes repeated bookkeeping around the unchanged IK arithmetic:

- constructs the invariant `rbd::Jacobian` once instead of once per preview call;
- prepares constant velocity-limit vectors once;
- replaces repeated joint-name string copies/prefix checks with a precomputed mask.

No numerical IK equation, iteration rule, convergence condition, seed, tolerance, or final configuration arithmetic is changed.

## Exact-equivalence gates

Across the four moving-object scenarios, the optimized implementation reproduced the complete scientific output of the accepted serial implementation:

- byte-identical complete-plan/admissibility record streams;
- identical final-selection `now`;
- identical logical planning cycle counts;
- identical event/grasp/route winners;
- identical objective values;
- identical timing-admissible sets.

An independent collision oracle compared **8,168,732** evaluations across the four scenarios with **zero mismatches** in:

- safe/unsafe result;
- minimum clearance;
- ground clearance;
- limiting sample;
- limiting obstacle.

Per-scenario oracle counts were:

| Scenario | Oracle comparisons | Mismatches |
| --- | ---: | ---: |
| CANONICAL_YZ | 2,977,590 | 0 |
| GROUND_NEAR | 2,003,452 | 0 |
| PURE_X | 1,926,145 | 0 |
| DIAGONAL_XZ | 1,261,545 | 0 |
| **Total** | **8,168,732** | **0** |

## Correctly scoped wall-time measurement

An earlier exploratory performance summary used a defective wall-time window that extended beyond the `SolveInterception` planning state. Those numbers are obsolete.

The corrected measurement scopes planner wall time to the `SolveInterception` FSM state and therefore preserves the exact logical planning-cycle count for each scenario.

Measured state-scoped wall times were:

| Scenario | Baseline wall (s) | Final wall (s) | Paired speedup |
| --- | ---: | ---: | ---: |
| CANONICAL_YZ | 4.3488 | 3.9724 | ~1.10x |
| GROUND_NEAR | 3.3151 | 3.0717 | ~1.085x |
| PURE_X | 3.3930 | 3.1615 | ~1.075x |
| DIAGONAL_XZ | 2.7966 | 2.6199 | ~1.068x |

For CANONICAL_YZ, the final confirmation used 10 interleaved baseline/final pairs. All 20 runs passed the runtime and scenario-identity gates, all 10 pairs favored the optimized implementation, and the logical planning count remained 879 cycles in every run.

## Control-cycle stall reduction

The serial optimizations also reduced the heavy synchronous control-cycle tail. In the CANONICAL_YZ confirmation:

- cycles above 100 ms dropped from `[5,6,7,6,5,7,9,5,5,6]` to `[3,1,2,1,2,1,2,1,1,1]`;
- maximum observed cycle duration dropped from approximately 137.3 ms to 125.3 ms;
- cycles above 50 ms remained 41 in every run of both arms, which is consistent with unchanged logical planning work.

The controller remains synchronous; these optimizations reduce blocking but do not turn the architecture into a non-blocking or asynchronous planner.

## Relation to timing frontiers

Timing frontiers are scenario-specific; `3.976 s` is the historical PURE_X grid point and is not a universal threshold. See [`timing_frontiers.md`](timing_frontiers.md).

The measured final implementation remains below every scenario's own fail-closed frontier, but measured wall time is outside every frozen simulated winner-preservation band. Timing feasibility and winner preservation are therefore reported separately.

## Reproducibility status of this study

The optimization campaign preserved independent evidence including patches, instrumentation diffs, oracle outputs, timing traces, interleaved benchmark harnesses, record digests and SHA-256 manifests in the project audit archive.

The frozen Dataset-B baseline remains the primary repository-level reproducibility anchor. Before a publication release promotes the exact-optimization lineage as the default source state, its exact source commits and the corresponding clean-clone build/run checks should be synchronized into the publication repository as one controlled release update.
