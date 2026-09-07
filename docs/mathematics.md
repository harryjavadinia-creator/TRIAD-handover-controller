# Mathematical formulation

This page states the decision problem. [`global_time_plan.md`](global_time_plan.md)
and [`binding_cost.md`](binding_cost.md) connect the notation to the exact
controller fields and selector behavior.

## Decision object

A complete plan is

\[
\xi=(\tau,g,r),
\]

where `tau` is a future presentation event, `g` is a receiver grasp
orientation, and `r` is a transit route.

## Bounded finite approximation

The continuous decision space is approximated by the finite product

\[
\mathcal X_h=\mathcal T_h\times\mathcal G_h\times\mathcal R_h.
\]

For the reported moving-object campaign:

- `T_h`: 14 bounded event-lead hypotheses in a deterministic center-out
  schedule;
- `G_h`: 32 grasp candidates = 16 angular samples × 2 handle-axis
  orientations;
- `R_h`: 17 routes = 1 direct route + 16 ring routes.

The upper combinatorial bound before feasibility pruning is

\[
14\times32\times17=7616.
\]

This is an upper bound on generated combinations, not the number of plans
that survive complete evaluation.

The two handle-axis orientations are alternative gripper-frame conventions
around the same receiver grasp point. They are not two physical ends of the
handover object.

## Object estimation and prediction

The object estimate is a latency-compensated measurement. When a perception
delay is configured, the measurement used is the buffer value at `t - tau`,
interpolated between the bracketing samples and clamped at the buffer ends, and
the estimate is then propagated forward by the measured age of that sample using
the filtered twist:

\[
\hat p(t)=p_{\mathrm{meas}}(t-\tau)+\mathrm{age}\cdot\hat v(t),
\qquad
\hat R(t)=\mathrm{Exp}\!\left(\mathrm{age}\cdot\hat\omega(t)\right)R_{\mathrm{meas}}(t-\tau).
\]

The twist estimate is a first-order low-pass filter of finite differences of the
measurement stream, gated against implausible raw values.

Prediction to a candidate event uses the same constant-twist law composed with a
**prescribed** C²-continuous quintic terminal deceleration of fixed duration
`D`, ending exactly at the hypothesised presentation instant, after which the
object is modelled as stationary. Because the integral of the quintic smoothstep
complement is exactly one half, the predicted presentation pose at lead `h` is

\[
\Pi(h)=\mathrm{Prop}\!\left(W\_T\_O(t_0),\; h-\tfrac12\min(h,D),\; \hat v,\hat\omega\right).
\]

The prediction model is **deterministic**. There is no covariance, no filter
bank, no learned model and no representation of uncertainty anywhere in the
pipeline, and the deceleration profile is **prescribed identically for every
hypothesis** rather than inferred from the partner's motion. Uncertainty is
handled discretely instead — by a bounded-twist sanity gate on the raw estimate,
by the commit-time prediction-freshness bound, and by the fail-closed runtime
guards.

## Hard physical feasibility

Let `s0` denote the frozen robot/object decision state at the common search
epoch. Define

\[
\mathcal F_h(s_0)
=\{\xi\in\mathcal X_h:\text{all copied-state hard physical checks pass}\}.
\]

The hard checks include reachability/IK, collision and ground clearance,
joint position/velocity limits, corridor and acquisition geometry, terminal
capture conditions, and complete receiver-action/retreat feasibility.

The objective never replaces these checks.

## Cost-valid set

A hard-feasible plan is ranked only if every required objective quantity is
finite and valid:

\[
\mathcal F_J(s_0)
=\{\xi\in\mathcal F_h(s_0):J_{\mathrm{global}}(\xi;s_0)
\text{ is finite and valid}\}.
\]

Invalid/non-finite records are excluded from the global pooled set rather than
being assigned a favorable fallback cost.

## Seven-term motion objective

\[
J_{\mathrm{motion}}
=w_TT+w_EE+w_LL+w_CC+w_QQ+w_KK+w_VV.
\]

| Term | Meaning | Weight |
| --- | --- | ---: |
| `T` | time efficiency (`t_complete / 8`) | 0.4210526 |
| `E` | cumulative squared joint-speed effort proxy; not physical energy | 0.1052632 |
| `L` | geometric route efficiency (`path_length / 0.50`) | 0.1052632 |
| `C` | clearance reserve soft barrier | 0.1578947 |
| `Q` | joint-limit reserve soft barrier | 0.0842105 |
| `K` | metric-scaled kinematic-conditioning reserve | 0.0736842 |
| `V` | joint-velocity-utilization reserve, `clamp01(u)^4` | 0.0526316 |

An eighth logged quantity, `R` (orientation), is diagnostic only. Its binding
weight is fixed at zero.

These seven weights are frozen controller-specific engineering preference
values. They are not literature-derived, are not claimed optimal, and **no
weight-space sensitivity result is reported in this repository**. The values are
the ratios `8 : 2 : 2 : 3 : 1.6 : 1.4 : 1` over 19 and sum to exactly 1.

`E` is a cumulative squared joint-speed effort proxy and is **not** physical
energy; `K` is a project-specific condition-index reserve and is **not** exactly
Yoshikawa's manipulability index. A separately logged terminal velocity
utilisation does **not** enter `V`, and does **not** gate the terminal timing
audit — it is computed after that audit returns.

`C`, `Q` and `K` use soft-barrier terms in their preference regions. Hard
feasibility has already rejected physically invalid plans before these terms
are compared.

## Cross-event time contribution

To compare plans belonging to different event times, the same normalized time
weight is extended back to the common search epoch:

\[
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{\mathrm{scheduleWait}}{T_{\mathrm{ref}}},
\qquad T_{\mathrm{ref}}=8\text{ s}.
\]

In the implementation,

\[
\mathrm{scheduleWait}
=(\tau-t_0)-T_{\mathrm{reach}},
\]

so the full time contribution represents predicted search-to-completion time.
No additional independent weight is introduced.

## Final timing-admissible set

The final timing gate is evaluated **after the complete bounded event schedule
has been inspected**. Let `t_sel` be the final selector time (`now` in
`FiniteEventPlanSelector`). For each cost-valid complete plan,

\[
\mathrm{remaining}(\xi,t_{\mathrm{sel}})
=t_{\mathrm{event}}(\xi)-t_{\mathrm{sel}}.
\]

With the implementation epsilon \(\varepsilon=10^{-12}\), the two required
inequalities are

\[
\mathrm{remaining}+\varepsilon
\ge L_{\mathrm{safe}}
\]

and

\[
T_{\mathrm{presentation}}+L_{\mathrm{reach}}
\le \mathrm{remaining}+\varepsilon.
\]

Define

\[
\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})
=\{\xi\in\mathcal F_J(s_0):\text{both inequalities hold}\}.
\]

This distinction matters: physical/geometric feasibility is evaluated from
the frozen decision state, while final timing admissibility depends on the
time at which the completed search is committed.

## Exact finite argmin

The reported moving-object policy selects

\[
\xi_h^*
=\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
\]

The minimum is exhaustive over the bounded generated finite set. It is not a
claim of continuous-space global optimality and is not solved by gradient
descent, MPC over event time, or an unrestricted continuous optimizer.

The bank carries **no completeness guarantee**. An outcome of "no feasible plan"
or "no timing-admissible plan" therefore never proves that no physically
feasible handover existed; it states only that no element of the generated
finite set satisfied the checks.

Every cardinality and spacing in the bank — 14 event instants, 32 grasps, 17
route generators, 25 swept samples per commanded segment — is a frozen
engineering discretisation. None is derived from a convergence argument and no
resolution-sensitivity study has been performed.

Numerical ties within the configured cost tolerance are resolved
deterministically by the selector's fixed secondary ordering.

## WHAT/WHEN vs HOW

```text
finite planner: Observe -> Predict -> Generate -> Preview -> Feasibility
                -> Final timing admission -> finite argmin -> Commit

mc_rtc FSM/QP:  execute the committed references
```

The finite planner decides event time, grasp and route. The mc_rtc QP tracks
the committed plan; it does not solve the high-level argmin.
