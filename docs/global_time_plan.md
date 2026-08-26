# Global time-grasp-route selector

## What this solves

For a moving presentation, the controller freezes one bounded set of event
times **and their predicted object poses** from one motion-estimate snapshot
at a common search epoch `t0`. For every event it evaluates the same complete
grasp and route bank on copied robot state. It commits exactly once, after
the complete finite set has been evaluated, by solving

\[
(\tau^*,P^*)=\arg\min_{\tau_k,\;P\in\mathcal F(\tau_k)}
J_{\mathrm{global}}(\tau_k,P).
\]

`F(tau)` contains only complete plans that pass the hard reachability,
collision, corridor, acquisition, retreat and final timing-admission checks.
The objective is

\[
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{(\tau_k-t_0)-T_{\mathrm{reach}}}{T_{\mathrm{ref}}}.
\]

Because `J_motion` already contains `w_T * T_execution / T_ref`, its
resulting time contribution is

\[
w_T\frac{(\tau_k-t_0)-T_{\mathrm{reach}}+T_{\mathrm{execution}}}
{T_{\mathrm{ref}}},
\]

which is predicted time from the common search epoch to completion.

This is exhaustive **discrete finite-set minimization** over a bounded
approximation of the decision space. It is not a claim of continuous-time
optimality, gradient-based optimization, or MPC over the event time.

`J_motion` is the frozen seven-term binding preference objective
(T, E, L, C, Q, K, V) with weights

| Term | Weight |
| --- | --- |
| T (time efficiency) | 0.4210526 |
| E (joint-speed effort) | 0.1052632 |
| L (route length) | 0.1052632 |
| C (clearance reserve) | 0.1578947 |
| Q (joint-limit reserve) | 0.0842105 |
| K (conditioning reserve) | 0.0736842 |
| V (velocity reserve) | 0.0526316 |

An eighth term, R (orientation), is computed and logged as a diagnostic only;
its weight is fixed at 0.0 and it never enters the binding sum. These weights
are controller-specific engineering values from a finite-set weight-space
sensitivity analysis; they are not claimed to be literature-derived or
optimal.

## Configuration

The default configuration is:

```yaml
decisionCost:
  selectionMode: binding_cost
  eventSelectionMode: global_time_plan
  allowPhysicalExecution: false

gripper:
  physicalBridge:
    enabled: false
    commandEnabled: false
    requireFeedback: false
```

Available event policies are:

- `global_time_plan`: evaluate the complete fixed event set, reapply final
  timing admission, then minimize over time, grasp and route. This is the
  policy used to produce the reported results.
- `first_admissible_center_out`: select the first timing-admissible
  candidate in center-out order instead of the global argmin, while
  preserving binding route selection within that single event.

Global mode is compatible only with `selectionMode: binding_cost`; every
invalid combination fails closed without falling back to the heuristic.

`planningStepsPerCycle` batches copied-state preview calculations so the
exhaustive scan does not exceed the candidate events within one control
cycle in simulation. It does not change preview integration, candidate
generation, constraints, costs or robot motion.

## Runtime verification requirements

A valid run must contain:

- `[GlobalTimePlanSearchConfiguration]` with a fixed epoch and finite set;
- one `[GlobalPlanCost]` for every complete time-grasp-route alternative;
- one `[GlobalTimePlanSelection]` with a complete schedule and exact minimum;
- one `[GlobalTimePlanCommitProof] committed=true` for the same alternative;
- `[Completed]` after grasp, transfer and retreat.

Validate the log with:

```bash
python3 tools/check_global_time_plan_log.py /path/to/run.log
```

The checker rejects incomplete schedules, invalid cost rows, expired sets,
selection/commit mismatches, non-minimum commitment, incomplete handovers,
and reconstructs the reported cost from the frozen seven-term weight vector
independently of the controller's own weights.

## Dependency-free checks

```bash
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
bash tools/run_binding_cost_checks.sh
```

These cover the within-event selector, the cross-event selector, final
timing readmission, invalid-cost fail-closed behavior, deterministic ties,
source integration and synthetic runtime-proof acceptance/rejection.

## Reproducing a scenario

```bash
scripts/run_scenario.sh diagonal
python3 tools/check_global_time_plan_log.py results/<timestamp>_diagonal/diagonal.log
```

See [`simulation.md`](simulation.md) for the full scenario table and expected
output ranges.
