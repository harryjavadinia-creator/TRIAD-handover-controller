# Scenario-specific timing admissibility

TRIAD's final cross-event selector reapplies timing admission after the full bounded schedule has been inspected. The resulting hardware-facing timing boundary depends on the complete-plan records of the scenario; there is no single universal planner deadline for all four scenarios.

## Exact selector rule

For each cost-valid complete plan:

```text
remaining = eventPresentationTime - now

eventWindowAdmissible =
    remaining + 1e-12 >= minimumSafeCommitLead

timingAdmissible =
    eventWindowAdmissible
    && predictedPresentationDuration + minimumReachEntryLead
           <= remaining + 1e-12
```

For a counterfactual physical planner duration `T_p`, the common search epoch gives

```text
now(T_p) = planStart + T_p
remaining(T_p) = eventLead - T_p
```

and the plan-specific admissible boundary is

```text
T_p,max(p) = eventLead_p + 1e-12
             - max(minimumSafeCommitLead,
                   predictedPresentationDuration_p + minimumReachEntryLead)
```

Because the implementation comparisons are non-strict, a plan can remain admissible at its exact analytical boundary; durations greater than the largest record boundary leave the scenario with no timing-admissible plan.

## Repository replay

After reproducing a Dataset-B run:

```bash
python3 tools/replay_timing_frontier.py <log>
```

The tool:

1. joins `[GlobalPlanCost]` and `[GlobalPlanTimingAdmissibility]` records;
2. verifies one common search epoch and one final selector time;
3. independently replays every logged timing-admissibility flag;
4. derives each plan's analytical boundary;
5. reports the scenario fail-closed boundary;
6. optionally evaluates requested counterfactual `T_p` values.

Example:

```bash
python3 tools/replay_timing_frontier.py longitudinal.log \
  --planner-time 3.808 \
  --planner-time 3.976
```

The dependency-free replay logic is regression-tested by `tools/test_replay_timing_frontier.py`.

## Historical PURE_X correction

The historical PURE_X analysis used 198 complete-plan records. The exact rule reproduced:

- logged logical planner duration: 1.386 s;
- 158 timing-admissible plans at that logged duration;
- 1 admissible plan at `T_p = 3.808 s`;
- analytical boundary: 3.975000 s (to displayed precision);
- 0 admissible plans at `T_p = 3.976 s`.

Thus `3.976 s` is a PURE_X-specific 1-ms grid point immediately above the boundary, not a universal hardware requirement.

## Scenario-specific fail-closed boundaries

| Scenario | Complete records | Boundary (s) |
| --- | ---: | ---: |
| GROUND_NEAR | 283 | **3.900000** |
| PURE_X | 198 | **3.975000** |
| CANONICAL_YZ | 432 | **5.139608** |
| DIAGONAL_XZ | 233 | **5.735285** |

These values are the scenario-specific supremum of planner duration for which at least one timing-admissible plan remains under the exact selector rule.

## Winner preservation is a different property

The frozen simulated winner is preserved only over a narrower planner-time region:

| Scenario | Reported winner-preservation band (s) |
| --- | ---: |
| CANONICAL_YZ | `[0.000000, 1.975000)` |
| GROUND_NEAR | `[1.018546, 1.467141)` |
| PURE_X | `[1.293474, 1.675000)` |
| DIAGONAL_XZ | `[0.000000, 1.632358)` |

The key distinction is structural:

- **fail-closed boundary**: whether any timing-admissible complete plan remains;
- **winner preservation**: whether the same plan selected in the frozen no-sync simulation remains the minimum after timing admission changes.

A scenario can change winner well before its admissible set becomes empty. For GROUND_NEAR and PURE_X, the preserved-winner region does not begin at zero; a sufficiently faster counterfactual planner can make additional, lower-cost plans admissible.

At an exact breakpoint, the selector's non-strict inequalities and deterministic tie semantics govern the endpoint. The tabulated bands report the transition intervals from the preserved analysis; record-level equality questions should be checked with the replay and selector logs rather than rounded values alone.

## Measured serial wall time

The correctly scoped exact-serial performance campaign measured:

| Scenario | Baseline wall (s) | Final wall (s) | Boundary (s) | Final margin (s) |
| --- | ---: | ---: | ---: | ---: |
| CANONICAL_YZ | 4.3488 | 3.9724 | 5.139608 | +1.1672 |
| GROUND_NEAR | 3.3151 | 3.0717 | 3.900000 | +0.8283 |
| PURE_X | 3.3930 | 3.1615 | 3.975000 | +0.8135 |
| DIAGONAL_XZ | 2.7966 | 2.6199 | 5.735285 | +3.1154 |

All four final wall-time medians lie inside their own fail-closed boundary. The measured wall durations nevertheless lie outside all four frozen winner-preservation bands.

CANONICAL_YZ at approximately 4.35–4.40 s was therefore not timing-infeasible under its own scenario boundary.

## Hardware-transfer interpretation

No-sync simulation advances logical controller time by control cycles. A physical object continues to move while real CPU computation consumes wall time.

The counterfactual replay therefore asks:

> Given the exact complete-plan records, which alternatives would still pass the controller's timing gate if real planner wall time elapsed before the final selection?

This is a hardware-facing analysis of the selector rule. It is not itself a physical-robot experiment.

## Recommended interpretation

Timing admissibility is scenario-specific. The historical `3.976 s` value is specific to PURE_X, while each scenario has its own fail-closed boundary. Serial acceleration increases timing margin in all four scenarios. Preservation of the frozen simulated winner is stricter and must be reported separately from timing feasibility.
