# Within-event binding-cost selector

This document describes the `binding_cost` selection mode, the layer that
TRIAD's finite complete-plan selector uses to rank and commit a plan for a
single event-time hypothesis. For the full cross-event selector (the exact
argmin over the entire bounded event-time schedule used to produce the
reported results), see [`global_time_plan.md`](global_time_plan.md).

## Outcome

For one event hypothesis, the controller solves

\[
P^*=\arg\min_{P\in\mathcal F(\tau)}J(P),
\]

where `P` is a complete grasp-route-approach-contact-acquisition-retreat plan
and `F(tau)` contains only complete plans that pass the copied-state hard
feasibility checks and the candidate-specific final timing-admission gate.

This is exhaustive finite-set minimization, not continuous trajectory
optimization; `J` does not optimize the event time itself at this layer (the
event time is selected by the cross-event search described in
[`global_time_plan.md`](global_time_plan.md)).

## Selector modes

The mode is explicit under `decisionCost.selectionMode`:

```yaml
decisionCost:
  selectionMode: binding_cost
```

- `binding_cost`: select and commit the admissible minimum-`J` complete plan.
  This is the mode used to produce the reported results.
- `protected_heuristic`: an alternative, non-cost-based selector retained for
  comparison. The cost is computed and logged as diagnostic only in this mode
  and does not affect the committed plan.

There is no silent fallback from `binding_cost` to the heuristic selector.
Binding mode refuses commitment when:

- the complete-plan set is empty;
- one or more complete feasible plans has an invalid or non-finite cost;
- no plan is currently timing-admissible;
- the selected cost differs from the logged minimum by more than the
  configured numerical tolerance;
- the physical gripper bridge is enabled while
  `decisionCost.allowPhysicalExecution` is false.

## What reaches the robot

The binding selector writes its winner into `planningBestCandidate_`. The
commit path then verifies the selection proof before copying that plan's
grasp, route, postures and timing into the immutable interception plan. The
mc_rtc QP tracks the committed plan; it is not the high-level cost solver.

Successful runtime evidence contains:

- `[PlanSelectionConfiguration] mode=binding_cost ...`;
- `[CompletePlanCost]` for every complete route;
- `[BindingCostSelection] ... selectedJ=... minimumAdmissibleJ=...`;
- exactly one `[BindingCostCommitProof] committed=true ...` line.

Validate a produced log with:

```bash
python3 tools/check_binding_cost_log.py /path/to/run.log
```

## Dependency-free checks

These checks run without mc_rtc or a build:

```bash
tools/run_binding_cost_checks.sh
```

They test:

- ordinary minimum-cost selection;
- fail-closed handling of an incomplete/non-finite cost set;
- timing-constrained selection when the unconstrained minimum is too slow;
- prevention of commitment while event-time refinement is required;
- deterministic numerical tie-breaking;
- source-level integration of selection, commit proof and preview
  corrections;
- acceptance/rejection behaviour of the runtime log checker.

## Configuration status

The default configuration uses simulation-safe settings:

- `selectionMode: binding_cost`;
- `physicalBridge.enabled: false`;
- `allowPhysicalExecution: false`.

See [`global_time_plan.md`](global_time_plan.md) for the frozen seven-term
objective weights and the cross-event search that together produced the
reported results.
