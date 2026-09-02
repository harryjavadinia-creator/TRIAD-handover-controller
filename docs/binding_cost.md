# Within-event binding-cost selector

`FinitePlanSelector` is the within-event selection layer used while TRIAD evaluates one event-time hypothesis. The reported moving-object result is completed by the cross-event selector described in [`global_time_plan.md`](global_time_plan.md).

## Within one event

For one event hypothesis `tau`, the selector receives already-certified complete grasp/route plans and performs finite minimization over the cost-valid, timing-admissible subset for that event.

The selector does not generate plans and does not soften feasibility. Its inputs are complete plans produced by the copied-state evaluation pipeline.

When timing is enforced, a plan is commit-admissible only when the event window and its candidate-specific reach-entry lead are both safe. If no plan is currently admissible, the fastest valid plan may be returned with `commitAdmissible=false` solely to drive the existing event-time refinement; that result is not allowed to commit.

## Selector modes

The reported mode is explicit:

```yaml
decisionCost:
  selectionMode: binding_cost
```

- `binding_cost`: choose the admissible finite minimum.
- `protected_heuristic`: retained alternative selector; the cost is diagnostic in that mode and does not choose the committed plan.

There is no silent fallback from `binding_cost` to the heuristic.

Within-event binding selection refuses commitment when the complete-plan cost set is incomplete/non-finite, no plan is currently timing-admissible, the selection proof is inconsistent with the reported minimum, or physical binding execution is not explicitly enabled.

## Relation to the global selector

The moving `global_time_plan` policy does **not** commit the first within-event winner. Instead it evaluates every configured event, captures complete cost-valid plan records, and defers commitment.

After the complete schedule is evaluated, `FiniteEventPlanSelector`:

1. reapplies timing admission using the final selector `now`;
2. forms the final cost-valid/timing-admissible set; and
3. chooses the cross-event minimum `J_global`.

This two-layer structure is why the repository contains both `FinitePlanSelector.h` and `FiniteEventPlanSelector.h`.

## Runtime proof

Within-event evidence includes:

- `[PlanSelectionConfiguration] mode=binding_cost ...`;
- `[CompletePlanCost]` records;
- `[BindingCostSelection]`;
- `[BindingCostCommitProof]` when a within-event binding result is actually committed.

The global moving-object campaign additionally emits the cross-event evidence described in [`global_time_plan.md`](global_time_plan.md).

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

These defaults reproduce the simulation evidence and do not constitute physical-robot validation.
