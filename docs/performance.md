# Historical performance notes

This page documents implementation-level performance evidence from an earlier
exact-serial TRIAD source state. These measurements characterize software
optimization; they do not define the TRIAD decision problem and do not validate
the later background-planning implementation.

> **Historical source state.** The study below belongs to development state
> `82e6eaa`, published as `a006912` / tag `csi-2026-release`.

## What was held fixed

The serial optimization campaign preserved the TRIAD scientific decision problem:

- the event, grasp and route banks;
- complete-plan enumeration;
- hard feasibility and timing-admission rules;
- objective terms and weights;
- selector/tie semantics;
- collision proxies and sampled swept-pose checks;
- logical planning cycle count and controller-time semantics.

The study therefore targeted software overhead without intentionally changing
the event–grasp–route decision itself.

## Implementation optimizations

Two implementation changes were evaluated.

### Clearance-hierarchy overhead

Repeated segment-invariant quantities in the clearance hierarchy were
precomputed once per query and reused during repeated point/segment evaluations.

The independent brute-force oracle path remained separate. Geometry,
thresholds, traversal order, sampled poses and comparison semantics were
unchanged.

### IK preview bookkeeping

Repeated bookkeeping around unchanged IK arithmetic was reduced by:

- constructing an invariant `rbd::Jacobian` once rather than once per preview call;
- preparing constant velocity-limit vectors once;
- replacing repeated joint-name copies/prefix tests with a precomputed mask.

No IK equation, seed, iteration rule, convergence condition, tolerance or
final-configuration arithmetic was intentionally changed.

## Scientific-equivalence checks

Across the four canonical moving-object scenarios, the optimized historical
implementation reproduced the same scientific outputs used by that campaign,
including:

- complete-plan and timing-admissibility record streams;
- final selector controller time;
- logical planning cycle counts;
- selected event/grasp/route winners;
- objective values;
- timing-admissible sets.

These checks support the interpretation that the measured speedup came from
implementation-level optimization rather than a changed TRIAD decision rule.

## Collision-oracle evidence

The independent collision oracle compared **8,168,732** evaluations with
**zero mismatches**:

| Scenario | Oracle comparisons | Mismatches |
| --- | ---: | ---: |
| CANONICAL_YZ | 2,977,590 | 0 |
| GROUND_NEAR | 2,003,452 | 0 |
| PURE_X | 1,926,145 | 0 |
| DIAGONAL_XZ | 1,261,545 | 0 |
| **Total** | **8,168,732** | **0** |

A scenario-specific oracle check can be run with:

```bash
tools/check_collision_hierarchy_oracle.sh longitudinal
```

Oracle mode intentionally evaluates both collision paths and should not itself
be used for performance measurement.

## Historical planner wall time

The state-scoped planner wall-time comparison was:

| Scenario | Baseline wall (s) | Final wall (s) | Paired speedup |
| --- | ---: | ---: | ---: |
| CANONICAL_YZ | 4.3488 | 3.9724 | ~1.100× |
| GROUND_NEAR | 3.3151 | 3.0717 | ~1.085× |
| PURE_X | 3.3930 | 3.1615 | ~1.075× |
| DIAGONAL_XZ | 2.7966 | 2.6199 | ~1.068× |

The corresponding wall-time reductions are approximately **6.3–8.7%** across
these four scenarios.

For CANONICAL_YZ, 10 interleaved baseline/final pairs were used. All 20 runs
passed the runtime and scenario-identity gates, all 10 pairs favored the
optimized implementation, and all runs retained 879 logical planning cycles.

These measurements are machine- and implementation-state dependent. They are
not a WCET bound, hard-real-time guarantee, formal schedulability result or
physical-handover performance claim.

## Source and reproducibility

The optimized implementation was synchronized from development commit
`82e6eaa` and published as `a006912` / tag `csi-2026-release`. Imported
implementation files and SHA-256 digests are recorded in
[`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256).

A clean configure/build and the four Dataset-B reproduction runs were
revalidated for that source state. See
[`release_validation.md`](release_validation.md).
