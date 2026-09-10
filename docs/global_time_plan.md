# Final event–grasp–route selection

## What this stage does

For a moving presentation, TRIAD evaluates a bounded set of future handover events. For each event time `tau`, it considers the corresponding grasp and route alternatives, rejects complete plans that fail the modeled hard checks, and computes the objective for the surviving plans.

After all configured event times have been evaluated, the surviving complete plans are compared together. The controller then reapplies the current timing gate and selects one final admissible plan:

```math
(\tau^{\ast},g^{\ast},r^{\ast})=
\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
```

In plain terms:

```text
for every future event time:
    evaluate grasp and route alternatives
    reject hard-infeasible complete plans
    compute their objective

pool all surviving plans
    -> apply current timing admission
    -> choose the minimum-cost admissible (tau, grasp, route)
    -> commit once
```

The set notation is defined precisely in [`mathematics.md`](mathematics.md). Final timing admission is selection-time dependent and is separate from copied-state physical feasibility.

## Why the comparison is called global

Here, `global` means **across the complete finite event–grasp–route bank**. It does not mean continuous-space global optimization.

The within-event selector can compare grasp/route alternatives associated with one event-time hypothesis. The final event selector compares the retained complete plans across the whole configured event schedule before commitment.

The result is therefore one complete decision:

```math
\xi^*=(\tau^*,g^*,r^*).
```

## Global objective

For one complete plan,

```math
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{(\tau-t_0)-T_{\mathrm{reach}}}{T_{\mathrm{ref}}}.
```

`J_motion` scores the motion associated with the complete plan. The additional common-epoch time contribution allows plans belonging to different future event times to be compared from the same search epoch `t0`.

Because `J_motion` already contains its normalized execution-time term, the combined time contribution represents the controller's preference over both robot execution and when the selected future handover event occurs. It is a total-time preference across complete plans, with the exact decomposition retained for reproducibility.

## Seven-term motion objective

The binding motion objective uses the seven terms `(T,E,L,C,Q,K,V)`:

| Term | Weight |
| --- | ---: |
| T | 0.4210526 |
| E | 0.1052632 |
| L | 0.1052632 |
| C | 0.1578947 |
| Q | 0.0842105 |
| K | 0.0736842 |
| V | 0.0526316 |

`R` (orientation) is computed and logged diagnostically but has binding weight `0.0`.

These weights are frozen controller-specific engineering preference values. They are not literature-derived, are not claimed optimal, and **no weight-space sensitivity result is reported in this repository**.

The exact term definitions and normalization are given in [`mathematics.md`](mathematics.md).

## Selector policy

The reported configuration is:

```yaml
decisionCost:
  selectionMode: binding_cost
  eventSelectionMode: global_time_plan
  allowPhysicalExecution: false
```

In `global_time_plan` mode, TRIAD evaluates the fixed bounded event set, reapplies current timing admission after the complete schedule has been inspected, and selects the finite minimum over event time, grasp and route.

The controller also retains an alternative policy, `first_admissible_center_out`, which accepts the first timing-admissible event in center-out order and performs binding route selection within that event. This alternative policy is not the reported global result.

Invalid policy combinations fail closed rather than silently falling back.

## Relation to the two selector classes

The implementation uses two finite selector classes:

- `FinitePlanSelector` handles selection among complete grasp/route alternatives associated with one event-time hypothesis;
- `FiniteEventPlanSelector` performs the final comparison across event-time hypotheses after current timing admission is reapplied.

These are two implementation layers of the same TRIAD decision process, not two different scientific planners.

## Runtime evidence

A valid global run contains diagnostic records such as:

- `[GlobalTimePlanSearchConfiguration]` for the frozen event schedule;
- `[GlobalPlanCost]` for complete alternatives;
- `[GlobalPlanTimingAdmissibility]` for the final timing gate;
- one `[GlobalTimePlanSelection]`;
- one matching `[GlobalTimePlanCommitProof] committed=true`;
- `[Completed]` after capture, transfer and retreat.

A run can be checked with:

```bash
python3 tools/check_global_time_plan_log.py /path/to/run.log
```

The checker verifies schedule completeness, exclusion of invalid cost records, pooled-candidate reconciliation, selection/commit agreement, objective reconstruction and—when timing diagnostics are present—the exact finite minimum over the cost-valid and final-timing-admissible set.

## Dependency-free checks

```bash
bash tools/run_binding_cost_checks.sh
```

The suite covers the finite selectors, timing admission, deterministic tie handling, source integration and runtime-log fixtures.

## Reproduce a canonical scenario

```bash
scripts/run_scenario.sh diagonal
python3 tools/check_global_time_plan_log.py \
  results/<timestamp>_diagonal/diagonal.log
```

See [`simulation.md`](simulation.md) for the canonical scenario definitions and expected scientific outputs.
