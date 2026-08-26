# Mathematical formulation

[global_time_plan.md](global_time_plan.md) / [binding_cost.md](binding_cost.md) additionally include
exact internal configuration field names, to help reproduction; this
document states the model itself.

## Decision object

A complete plan is

```
xi = (tau, g, r)
```

where `tau` is a future event time, `g` a grasp orientation, and `r` a
transit route.

## Finite approximation

The continuous decision space is approximated by a bounded finite product

```
X_h = T_h x G_h x R_h
```

- `T_h`: 14 event-lead hypotheses from a bounded center-out schedule.
- `G_h`: 32 grasp-orientation candidates — 16 angular samples around the
  object handle x 2 handle-axis orientations. The two handle-axis
  orientations are alternative gripper-frame conventions around the same
  single receiver grasp point; they are not two physical ends of the
  object, and not an attempt to grasp the giver-occupied side. They can
  still produce materially different wrist/approach poses at the same
  point, so they are not geometric duplication.
- `R_h`: 17 routes (one direct route plus 16 ring routes).

The upper combinatorial bound before feasibility pruning is
`14 x 32 x 17 = 7616`; this is not the number of plans actually scored.

## Hard feasibility

```
F_h(s0) = { xi in X_h : all hard checks pass }
```

evaluated from one frozen decision state `s0` (robot and object state at the
search epoch `t0`), so every hypothesis is compared from the same snapshot.
Hard checks include IK/reachability, collision, joint position and velocity
limits, acquisition/open-gripper geometry, terminal/capture settling, timing
admissibility, and complete receiver-action and retreat feasibility. Invalid
candidates are excluded from `F_h(s0)` before any cost is compared — the
cost never substitutes for a hard-feasibility check.

## Seven-term objective

```
J_motion = wT*T + wE*E + wL*L + wC*C + wQ*Q + wK*K + wV*V
```

| Term | Meaning | Weight |
| --- | --- | --- |
| T | time efficiency (`t_complete / 8`) | 0.4210526 |
| E | cumulative squared joint-speed effort proxy — not physical energy | 0.1052632 |
| L | geometric route efficiency (`path_length / 0.50`) — not a measure of human predictability | 0.1052632 |
| C | clearance reserve (soft barrier) | 0.1578947 |
| Q | joint-limit reserve (soft barrier) | 0.0842105 |
| K | metric-scaled kinematic conditioning reserve — not exactly Yoshikawa's manipulability index | 0.0736842 |
| V | joint-velocity-utilization reserve, `[clamp[0,1](u)]^4` — not terminal velocity | 0.0526316 |

An eighth term, R (orientation), is computed and logged for every candidate
but excluded from the binding sum (weight fixed at 0.0); it is diagnostic
only.

`C`, `Q` and `K` are each a three-branch soft barrier: 0 in the comfortable
region, a `-ln(...)` soft-log penalty in an intermediate region, and a large
defensive value (`1e6`) below a threshold that hard feasibility has already
rejected — the defensive branch is insurance, not the mechanism that
enforces hard feasibility, which happens earlier in `F_h(s0)`.

## Global schedule-wait contribution

Comparing candidates across different event times requires accounting for
the wait until each candidate's event:

```
J_global = J_motion + wT * scheduleWait / T_ref      (T_ref = 8 s)
```

`J_global` is never reduced to only `t_complete / T_ref` when comparing
hypotheses — see [global_time_plan.md](global_time_plan.md) for the full derivation.

## Exact finite argmin

```
xi*_h = argmin_{xi in F_h(s0)} J_global(xi; s0)
```

This is the exact minimum over the bounded finite approximation. It is not a
claim of continuous-space global optimality, and it is not solved by
gradient-based optimization, MPC, or an unrestricted search.

## Architecture: WHAT/WHEN vs HOW

```
Observe -> Predict -> Generate(tau,g,r) -> Preview -> Hard feasibility
  -> Soft ranking -> Exact finite argmin -> Commit -> mc_rtc execution
```

The finite planner (left of "Commit") decides *what* and *when*: event time,
grasp, route. mc_rtc's FSM and QP layer (right of "Commit") decides *how*:
joint-level tracking of the committed plan. The QP is never asked to choose
the event time or solve the argmin; it executes the selected references.
