# Plan selection and binding-cost mode

TRIAD selects a complete handover plan

\[
\xi=(\tau,g,r)
\]

where `tau` is the future handover event time, `g` is the grasp, and `r` is the transit route.

The selection logic is implemented in two finite layers because TRIAD first evaluates grasp/route alternatives for one event-time hypothesis and then performs a final comparison across all event times. These are implementation layers of the same TRIAD decision process, not separate planning methods.

## Selection flow

```text
Generate complete (event time, grasp, route) plans
        -> hard-feasibility checks
        -> construct objective for surviving plans
        -> apply current timing admission
        -> choose the finite minimum-cost admissible plan
        -> commit once
```

The complete moving-object result is finalized by the cross-event selector described in [`global_time_plan.md`](global_time_plan.md).

## Within one event

For one event hypothesis `tau`, `FinitePlanSelector` receives already-certified complete grasp/route plans and performs finite minimization over the cost-valid, timing-admissible subset for that event.

The selector does not generate plans and does not soften feasibility. Its inputs are complete plans produced by the copied-state evaluation pipeline.

When timing is enforced, a plan is commit-admissible only when the event window and its candidate-specific reach-entry lead are both safe.

If no plan is currently timing-admissible, the fastest valid plan may still be returned with `commitAdmissible=false` solely to drive the existing event-time refinement path. Such a result is **not allowed to commit**. This is an implementation detail of refinement, not an alternative commitment rule.

## Why there are two selector classes

The repository contains both `FinitePlanSelector.h` and `FiniteEventPlanSelector.h` because the selection problem is evaluated at two levels:

- `FinitePlanSelector` compares complete grasp/route alternatives for one event-time hypothesis;
- `FiniteEventPlanSelector` performs the final comparison across event-time hypotheses after the full schedule has been evaluated.

The moving `global_time_plan` policy does **not** commit the first within-event winner. Instead it evaluates every configured event, captures complete cost-valid plan records, and defers commitment.

After the complete schedule is evaluated, `FiniteEventPlanSelector`:

1. reapplies timing admission using the final selector `now`;
2. forms the final cost-valid/timing-admissible set; and
3. chooses the cross-event minimum `J_global`.

The final committed plan is therefore selected from the admissible complete `(tau, g, r)` alternatives available at final selection time.

## What `binding_cost` means

The reported selector mode is explicit:

```yaml
decisionCost:
  selectionMode: binding_cost
```

In `binding_cost` mode, the objective value is **binding**: among the admissible finite plans, the minimum-cost plan is the one selected.

The alternative mode remains available in the implementation:

- `binding_cost`: choose the admissible finite minimum;
- `protected_heuristic`: retain the heuristic selector, with the cost remaining diagnostic rather than choosing the committed plan.

There is no silent fallback from `binding_cost` to the heuristic.

Within-event binding selection refuses commitment when the complete-plan cost set is incomplete or non-finite, no plan is currently timing-admissible, the selection proof is inconsistent with the reported minimum, or physical binding execution is not explicitly enabled.

## Relation to the global objective

The within-event selector operates on plans associated with one event hypothesis. The cross-event selector then compares the retained complete plans across event times using `J_global` after current timing admission is reapplied.

This distinction is useful for implementation and verification, but scientifically the decision remains one finite selection problem over complete event-grasp-route plans.

The objective itself, including all seven motion terms and the common-epoch time contribution, is defined in [Mathematics](mathematics.md).

## Runtime proof and diagnostics

Within-event evidence includes:

- `[PlanSelectionConfiguration] mode=binding_cost ...`;
- `[CompletePlanCost]` records;
- `[BindingCostSelection]`;
- `[BindingCostCommitProof]` when a within-event binding result is actually committed.

The global moving-object campaign additionally emits the cross-event evidence described in [`global_time_plan.md`](global_time_plan.md).

These records are used to verify that the selected result is consistent with the finite cost set and timing rule rather than relying only on the final controller outcome.

## Dependency-free checks

```bash
bash tools/run_binding_cost_checks.sh
```

The suite covers ordinary minimum-cost selection, invalid-cost failure, timing-constrained selection, non-committable refinement results, deterministic ties, source-level integration and runtime-log fixtures.

## Default safety state

The tracked configuration defaults to:

- `selectionMode: binding_cost`;
- `eventSelectionMode: global_time_plan`;
- `physicalBridge.enabled: false`;
- `allowPhysicalExecution: false`.

These defaults reproduce the simulation-oriented release configuration and do not constitute physical-robot validation.
