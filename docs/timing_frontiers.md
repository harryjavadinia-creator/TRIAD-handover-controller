# Scenario-specific timing admissibility

This note documents the corrected interpretation of TRIAD's timing-admission analysis.

## Why this exists

An earlier hardware-timing audit reported a value near **3.976 s** and that value was subsequently used too broadly as though it were a universal planner deadline. It is not.

The value belongs to the **PURE_X / longitudinal** scenario only. Each scenario has its own timing-admissibility frontier because event leads and predicted presentation durations differ across complete plans.

## Controller rule

For each complete plan record, the selector evaluates

```text
remaining = eventPresentationTime - now

eventWindowAdmissible =
    remaining + 1e-12 >= minimumSafeCommitLead

timingAdmissible =
    eventWindowAdmissible
    && predictedPresentationDuration + minimumReachEntryLead
           <= remaining + 1e-12
```

For a counterfactual physical-planning duration `T_p`, use

```text
now(T_p) = planStart + T_p
remaining(T_p) = eventLead - T_p
```

which gives the plan-specific timing breakpoint

```text
T_p,max(p) = eventLead_p + 1e-12
             - max(minimumSafeCommitLead,
                   predictedPresentationDuration_p + minimumReachEntryLead)
```

The replay must respect the selector's non-strict comparisons and the `1e-12` epsilon.

## Validated PURE_X derivation

The historical PURE_X replay is reproduced exactly from 198 complete-plan records:

- logged planning value: `T_p = 1.386 s`
- timing-admissible plans at that point: 158
- `T_p = 3.808 s`: one timing-admissible plan remains
- exact fail-closed boundary: `3.975000 s`
- the historical `3.976 s` value was the next 1 ms grid point at which zero plans remained

The `3.976 s` value is therefore specific to PURE_X and is not a universal hardware planner deadline.

## Scenario-specific frontiers

The same exact replay was applied independently to the four moving-object Dataset-B scenarios.

| Scenario | Record count | Winner-preservation band (s) | Fail-closed frontier (s) |
| --- | ---: | ---: | ---: |
| CANONICAL_YZ / lateral-low | 432 | `[0.000000, 1.975000)` | **5.139608** |
| GROUND_NEAR / near-ground | 283 | `[1.018546, 1.467141)` | **3.900000** |
| PURE_X / longitudinal | 198 | `[1.293474, 1.675000)` | **3.975000** |
| DIAGONAL_XZ / diagonal | 233 | `[0.000000, 1.632358)` | **5.735285** |

Two different concepts must be kept separate:

1. **Winner-preservation band** — the range of physical planner durations for which the same plan selected in the frozen simulation remains selected.
2. **Fail-closed frontier** — the planner duration beyond which no timing-admissible complete plan remains.

A scenario can change selected plan long before it becomes timing-infeasible.

## Measured serial wall time versus fail-closed frontier

A later exact serial-optimization study measured the following state-scoped planner wall durations. These measurements refer to the exact-optimization development lineage and are reported here as timing-analysis evidence; they do not redefine the Dataset-B scientific baseline.

| Scenario | Baseline wall (s) | Final wall (s) | Fail-closed frontier (s) | Final margin (s) |
| --- | ---: | ---: | ---: | ---: |
| CANONICAL_YZ | 4.3488 | 3.9724 | 5.139608 | +1.1672 |
| GROUND_NEAR | 3.3151 | 3.0717 | 3.900000 | +0.8283 |
| PURE_X | 3.3930 | 3.1615 | 3.975000 | +0.8135 |
| DIAGONAL_XZ | 2.7966 | 2.6199 | 5.735285 | +3.1154 |

All four measured final times remain inside their own fail-closed frontiers.

The most timing-critical pair is GROUND_NEAR / PURE_X: GROUND_NEAR has the lowest absolute fail-closed frontier, while PURE_X has the smallest relative final margin.

CANONICAL_YZ was **not** failing timing admission at approximately 4.35--4.40 s; its own fail-closed frontier is about 5.14 s.

## Important hardware-transfer interpretation

The no-sync simulation advances controller logical time according to control cycles. Physical hardware does not freeze while the planner consumes wall time.

Therefore the counterfactual replay asks a hardware-facing question:

> If the physical world continues to advance while complete-plan evaluation consumes real wall-clock time, which complete plans remain timing-admissible and which plan would be selected?

The exact serial implementation can remain below every scenario's fail-closed frontier while still lying outside the frozen simulated winner-preservation band. Thus **timing feasibility** and **winner preservation** are different hardware-transfer properties.

## Interpretation

Three statements are not supported by this analysis: that `3.976 s` is the
hardware planner deadline, that CANONICAL_YZ at about 4.4 s fails that frontier,
and that the four scenarios share a single frontier. Each is contradicted by the
per-scenario table above.

Preferred wording:

> Timing admissibility is scenario-specific. Each interception scenario has its own fail-closed frontier, derived from its own complete-plan records under the controller's exact admission rule. The previously cited 3.976 s value is the PURE_X frontier, not a universal planner deadline. Serial acceleration increases timing margin in all four moving-object scenarios, while preserving the frozen simulated winner is a stricter and separate timing requirement.
