# Exact serial performance study

> **Historical source state.** This section documents the exact-serial optimization campaign associated with development state `82e6eaa`, published as `a006912` / tag `csi-2026-release`. It is retained as historical evidence and must not be read as a runtime-validation claim for the later asynchronous branch.

This study reduced planner wall time without changing TRIAD's scientific decision problem.

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

No multicore planner, asynchronous architecture, timing-policy change or methodology change was introduced in this historical campaign.

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

Across the four moving-object scenarios, the optimized historical implementation reproduced:

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

## Correctly scoped historical wall time

An early exploratory timing window included work outside the `SolveInterception` planning state. The results below supersede that measurement and use a state-scoped planner wall interval.

| Scenario | Baseline wall (s) | Final wall (s) | Paired speedup |
| --- | ---: | ---: | ---: |
| CANONICAL_YZ | 4.3488 | 3.9724 | ~1.100× |
| GROUND_NEAR | 3.3151 | 3.0717 | ~1.085× |
| PURE_X | 3.3930 | 3.1615 | ~1.075× |
| DIAGONAL_XZ | 2.7966 | 2.6199 | ~1.068× |

The corresponding raw-derived reductions are approximately **6.3–8.7%** across these four scenarios.

CANONICAL_YZ used 10 interleaved baseline/final pairs: all 20 runs passed the runtime and scenario-identity gates, all 10 pairs favored the optimized implementation, and all runs retained 879 logical planning cycles.

## Synchronous control-cycle behavior

For CANONICAL_YZ in the historical exact-serial campaign:

- cycles above 100 ms changed from `[5,6,7,6,5,7,9,5,5,6]` to `[3,1,2,1,2,1,2,1,1,1]`;
- maximum observed cycle duration changed from about 137.3 ms to 125.3 ms;
- cycles above 50 ms remained 41 in every run of both arms.

The planner was still synchronous in this state. These changes reduced blocking; they did not make the planner asynchronous or non-blocking.

## Relation to timing frontiers

Timing frontiers are scenario-specific. `3.976 s` is the historical PURE_X 1-ms grid point immediately above its exact fail-closed boundary, not a universal planner threshold.

The measured final implementation lies inside every scenario's own fail-closed boundary, while its measured wall time lies outside every frozen simulated winner-preservation band. Timing feasibility and winner preservation are therefore separate results.

See [`timing_frontiers.md`](timing_frontiers.md).

## Historical publication source state

The exact optimized implementation was synchronized from development commit `82e6eaa` and published as `a006912` / tag `csi-2026-release`. The imported implementation files and their SHA-256 digests are recorded in [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256).

A clean configure/build and all four Dataset-B reproduction runs were revalidated for that historical source state. See [`release_validation.md`](release_validation.md).

---

# Asynchronous planner: control-loop evidence

This section reports a different metric: the cost visible in the **nominal 1 kHz control callback while planning is active**. It does not replace the historical serial wall-time study above.

## Published clean-machine after profiles

The reduced evidence package contains the following in-planning `ControllerRun` measurements:

| scenario | planning cycles | median [ms] | p90 [ms] | p99 [ms] | max [ms] | >1 ms | >2 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| lateral-low | 3850 | 0.043 | 0.063 | 0.134 | 1.135 | 1 | 0 |
| near-ground | 3023 | 0.040 | 0.072 | 0.122 | 1.174 | 1 | 0 |
| longitudinal | 2792 | 0.067 | 0.084 | 0.122 | 1.090 | 1 | 0 |
| diagonal | 2124 | 0.018 | 0.040 | 0.049 | 1.206 | 1 | 0 |

These values are read from `evidence/async/<scenario>/perf_analysis_clean_machine.txt`. They are the public primary basis for the **after** profile.

For the full run, not just the in-planning sample, the same files include larger controller/global-loop outliers. For example, near-ground records a whole-run `ControllerRun` maximum of 5.563 ms and the four `GlobalRun` maxima are around 40.4–40.8 ms, dominated by logging in the largest examples. Therefore the statement **"no cycle exceeds 2 ms" applies only to the in-planning `ControllerRun` sample shown above**, not to every controller/global cycle in the run.

## Before profile provenance

One raw pre-thread control-loop profile survives and is published as `evidence/async/perf_analysis_control_thread_reference.txt`; it reports in-planning `ControllerRun` median 0.677 ms, p90 3.409 ms, p99 4.007 ms and 1374 cycles above 1 ms for that captured reference run.

A complete four-scenario before/after table also survives as historical summary evidence from the development record. Because the four clean-machine after files above do **not** reproduce the different after-values in that summary table, the repository does not claim that one reproduces the other. When quoting current public after-values, use the clean-machine table above and identify the before data as historical summary evidence unless a raw trace is available.

## What may be claimed

The asynchronous evidence supports a narrow empirical statement: in each of the four published clean-machine planning samples, at most one in-planning `ControllerRun` cycle exceeded 1 ms and none exceeded 2 ms.

This is **not** a hard real-time guarantee, **not** a WCET bound, **not** a formal schedulability proof, and not a claim that every controller/global cycle remains below 2 ms. Wall-time distributions are machine-dependent and excluded from the deterministic reproducibility contract.

## Frozen plan-set relation

At 17-significant-digit scientific payload precision:

| scenario | control-thread records | worker records | supported relation |
| --- | ---: | ---: | --- |
| lateral-low | 432 | 432 | identical hash |
| longitudinal | 198 | 198 | identical hash |
| diagonal | 233 | 233 | identical hash |
| near-ground | 229 | 283 | all 229 prior scientific payloads retained after excluding `sourceIndex`; 54 payloads added |

For near-ground, **full raw records and full set hashes are not identical**. A direct normalized comparison that removes only the nonsemantic `sourceIndex` field retains all 229 prior scientific payloads and adds 54 worker payloads.

The additional payloads correspond to a hypothesis that the historical control-thread run did not enumerate after an elapsed-time prune. The asynchronous worker pins the planning-time enumeration reference to the frozen search epoch and performs current timing admission at result receipt. This explains the supported set relation; it does not justify an unconditional claim that the admission/result-receipt instant always moves earlier across separate runs.

The defensible statement is:

> the asynchronous implementation preserves the frozen scientific evaluation rules while removing one wall-clock-dependent enumeration truncation; in near-ground, the normalized scientific payload set is a strict superset, not a byte-identical raw set.

See [`corrections_of_record.md`](corrections_of_record.md) and the published records under [`../evidence/async/`](../evidence/async/).
