# Global event-time–grasp–route selector

## What it solves

For a moving presentation, TRIAD freezes one bounded event schedule and the corresponding predicted object poses from one motion-estimate snapshot at a common search epoch `t0`. Every event is evaluated with the same grasp and route banks on copied robot state.

After the entire configured event schedule has been inspected, the controller reapplies the final timing gate using the current selector time and commits one plan:

\[
(\tau^*,g^*,r^*)=
\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
\]

The set notation is defined precisely in [`mathematics.md`](mathematics.md). In particular, final timing admission is selection-time dependent and is separate from the copied-state physical feasibility set.

## Global objective

For one complete plan,

\[
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{(\tau-t_0)-T_{\mathrm{reach}}}{T_{\mathrm{ref}}}.
\]

Because `J_motion` already contains its normalized execution-time term, the combined time contribution represents predicted time from the common search epoch to completion.

The binding seven-term motion objective is `(T,E,L,C,Q,K,V)`:

| Term | Weight |
| --- | ---: |
| T | 0.4210526 |
| E | 0.1052632 |
| L | 0.1052632 |
| C | 0.1578947 |
| Q | 0.0842105 |
| K | 0.0736842 |
| V | 0.0526316 |

`R` (orientation) is computed and logged but has binding weight `0.0`.

These weights are controller-specific engineering preferences supported by a finite-set weight-space sensitivity analysis. They are not claimed to be literature-derived or globally optimal weights.

## Selector policy

The reported configuration is:

```yaml
decisionCost:
  selectionMode: binding_cost
  eventSelectionMode: global_time_plan
  allowPhysicalExecution: false
```

The event policies retained by the controller are:

- `global_time_plan`: evaluate the fixed bounded event set, reapply final timing admission, then select the finite minimum over event time, grasp and route. This is the reported Dataset-B policy.
- `first_admissible_center_out`: retain the first timing-admissible event in center-out order, with binding route selection within that event. It is an alternative policy and is not the Dataset-B global result.

Invalid policy combinations fail closed rather than silently falling back.

## `planningStepsPerCycle`

Copied-state preview work is processed in batches controlled by `planningStepsPerCycle`. In the reported configuration this is `96`.

In no-sync simulation, the number of planning batches also determines how many controller cycles elapse before the final selector runs. That logical controller time is therefore distinct from the external wall-clock time consumed by the CPU. The distinction is documented in [`simulation.md`](simulation.md) and [`timing_frontiers.md`](timing_frontiers.md).

## Runtime evidence

A valid global run contains:

- `[GlobalTimePlanSearchConfiguration]` describing the frozen event schedule;
- `[GlobalPlanCost]` records for complete alternatives;
- `[GlobalPlanTimingAdmissibility]` records capturing the exact final timing gate used by the selector;
- one `[GlobalTimePlanSelection]`;
- one matching `[GlobalTimePlanCommitProof] committed=true`;
- `[Completed]` after capture, transfer and retreat.

Validate a run with:

```bash
python3 tools/check_global_time_plan_log.py /path/to/run.log
```

The checker verifies schedule completeness, proper exclusion of invalid cost records, pooled-candidate reconciliation, selection/commit agreement, the frozen objective reconstruction, and—when timing records are present—the exact finite minimum over the cost-valid and final-timing-admissible set.

## Replay timing admission

A completed run can also be replayed counterfactually for an external planner duration:

```bash
python3 tools/replay_timing_frontier.py /path/to/run.log \
  --planner-time 3.808 \
  --planner-time 3.976
```

The replay derives each plan's analytical timing breakpoint from the exact logged quantities and validates the logged admissibility flags before reporting the scenario fail-closed boundary and admissible-plan counts.

## Dependency-free checks

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/test_replay_timing_frontier.py
```

These cover both selectors, timing admission, deterministic tie handling, source integration, runtime-proof fixtures and timing-frontier replay logic.

## Reproduce a scenario

```bash
scripts/run_scenario.sh diagonal
python3 tools/check_global_time_plan_log.py \
  results/<timestamp>_diagonal/diagonal.log
```

See [`simulation.md`](simulation.md) for scenario definitions and expected scientific outputs.
