# Mathematical formulation

This page states the finite decision problem. [`global_time_plan.md`](global_time_plan.md)
and [`binding_cost.md`](binding_cost.md) connect the notation to the exact
controller fields and selector behavior.

## Decision object

A complete plan is

$$
\xi=(\tau,g,r),
$$

where `tau` is a future presentation event, `g` is a receiver grasp
orientation, and `r` is a transit route.

## Bounded finite approximation

The continuous decision space is approximated by the finite product

$$
\mathcal X_h=\mathcal T_h\times\mathcal G_h\times\mathcal R_h.
$$

For the reported moving-object campaign:

- `T_h`: 14 bounded event-lead hypotheses in a deterministic center-out
  schedule;
- `G_h`: 32 grasp candidates = 16 angular samples × 2 gripper-frame/handle-axis
  conventions;
- `R_h`: 17 routes = 1 direct route + 16 ring routes.

The upper combinatorial bound before feasibility pruning is

$$
14\times32\times17=7616.
$$

This is an upper bound on generated combinations, not the number of plans that
survive complete evaluation.

The two grasp conventions are alternative gripper-frame conventions around the
same receiver point; they are not two physical ends of the handover object.

## Object estimation and prediction

The object estimate is a latency-compensated measurement. When a perception
delay is configured, the selected buffered measurement is propagated forward by
its measured age using the filtered twist:

$$
\hat p(t)=p_{\mathrm{meas}}(t-\tau)+\mathrm{age}\cdot\hat v(t),
\qquad
\hat R(t)=\mathrm{Exp}\!\left(\mathrm{age}\cdot\hat\omega(t)\right)R_{\mathrm{meas}}(t-\tau).
$$

The twist estimate is a first-order low-pass filter of finite differences of the
measurement stream, gated against implausible raw values.

Prediction to a candidate event uses the same constant-twist law composed with a
**prescribed** C²-continuous quintic terminal deceleration of fixed duration
`D`, ending at the hypothesised presentation instant, after which the object is
modelled as stationary. Because the integral of the quintic smoothstep
complement is one half, the predicted presentation pose at lead `h` is

$$
\Pi(h)=\mathrm{Prop}\!\left(\mathrm{W\_T\_O}(t_0),\; h-\tfrac12\min(h,D),\; \hat v,\hat\omega\right).
$$

The prediction model is **deterministic**. There is no covariance, learned
prediction model or probability distribution over future states. The terminal
deceleration is prescribed identically for every hypothesis rather than learned
from the partner.

## Model-relative hard feasibility

Let `s0` denote the common search epoch. In the intended finite formulation,
TRIAD evaluates hard feasibility relative to a frozen decision snapshot and its
modeled environment:

$$
\mathcal F_h(s_0)
=\{\xi\in\mathcal X_h:\text{the modeled hard checks pass}\}.
$$

Most candidate kinematics are evaluated from the copied `MultiBodyConfig` taken
at that epoch. The implementation has two residual live-access qualifications:

- gripper aperture used by corridor checks is derived from live fingertip-frame
  positions;
- joint position/velocity limits are obtained through live model accessors.

Therefore `F_h(s0)` is the mathematical decision abstraction, not a claim that
every implementation read is copied-state pure. The residual reads and the
static guard's coverage are documented in [`architecture.md`](architecture.md)
and [`corrections_of_record.md`](corrections_of_record.md).

The modeled hard checks include reachability/IK, sampled collision and ground
clearance, joint position/velocity limits, corridor and acquisition geometry,
terminal capture conditions, and complete receiver-action/retreat feasibility.
The objective never replaces these checks.

## Cost-valid set

A hard-feasible plan is ranked only if every required objective quantity is
finite and valid:

$$
\mathcal F_J(s_0)
=\{\xi\in\mathcal F_h(s_0):J_{\mathrm{global}}(\xi;s_0)
\text{ is finite and valid}\}.
$$

Invalid/non-finite records are excluded rather than assigned a favorable
fallback cost.

## Seven-term motion objective

$$
J_{\mathrm{motion}}
=w_TT+w_EE+w_LL+w_CC+w_QQ+w_KK+w_VV.
$$

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
weight is zero.

These seven weights are frozen controller-specific engineering preference
values. They are not literature-derived, are not claimed optimal, and **no
weight-space sensitivity result is reported in this repository**. The values are
ratios `8 : 2 : 2 : 3 : 1.6 : 1.4 : 1` over 19 and sum to 1.

`E` is a squared joint-speed effort proxy and is not physical energy; `K` is a
project-specific conditioning reserve and is not exactly Yoshikawa's
manipulability index. A separately logged terminal velocity utilisation does
**not** enter `V` and does **not** gate the terminal timing audit; it is computed
after that audit returns. Its non-diagnostic role is limited to the cost-validity
finiteness contract documented in source/provenance.

`C`, `Q` and `K` are preference terms. Hard feasibility has already rejected
plans that violate the modeled hard constraints before those terms are compared.

## Cross-event time contribution

To compare plans belonging to different event times, the same normalized time
weight is extended to the common search epoch:

$$
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{\mathrm{scheduleWait}}{T_{\mathrm{ref}}},
\qquad T_{\mathrm{ref}}=8\text{ s}.
$$

In the implementation,

$$
\mathrm{scheduleWait}
=(\tau-t_0)-T_{\mathrm{reach}},
$$

so the full time contribution represents predicted search-to-completion time.
No independent eighth binding weight is introduced.

## Final timing-admissible set

The final timing gate is evaluated after the bounded event schedule has been
inspected. Let `t_sel` be the final selector time. For each cost-valid complete
plan,

$$
\mathrm{remaining}(\xi,t_{\mathrm{sel}})
=t_{\mathrm{event}}(\xi)-t_{\mathrm{sel}}.
$$

With implementation epsilon $\varepsilon=10^{-12}$, the required
inequalities are

$$
\mathrm{remaining}+\varepsilon \ge L_{\mathrm{safe}}
$$

and

$$
T_{\mathrm{presentation}}+L_{\mathrm{reach}}
\le \mathrm{remaining}+\varepsilon.
$$

Define

$$
\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})
=\{\xi\in\mathcal F_J(s_0):\text{both inequalities hold}\}.
$$

This distinction matters: modeled geometric/kinematic feasibility is evaluated
against the frozen planning problem (subject to the implementation live-read
qualification above), while final timing admission depends on selector time.

## Exact finite argmin

The reported moving-object policy selects

$$
\xi_h^{\ast}
=\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
$$

The minimum is exhaustive over the bounded generated finite set. It is not a
claim of continuous-space global optimality and is not solved by gradient
descent, MPC over event time or an unrestricted continuous optimizer.

The bank carries **no completeness guarantee**. An outcome of "no feasible
TRIAD plan" or "no timing-admissible TRIAD plan" never proves that no physical
handover exists; it describes the generated finite set under the modeled checks.

Every cardinality/resolution is an engineering discretisation: 14 event
instants, 32 grasps, 17 route generators and **25 evaluated swept poses per
commanded segment** (`sweepSamples=24` with endpoints included). None is derived
from a convergence argument and no resolution-sensitivity study has been
performed. Sampled swept checks are not continuous collision detection or a
swept-volume proof.

Numerical ties within the configured tolerance are resolved deterministically by
fixed secondary ordering.

## WHAT/WHEN vs HOW

```text
finite planner: Observe -> Predict -> Generate -> Preview -> Feasibility
                -> Final timing admission -> finite argmin -> Commit

mc_rtc FSM/QP:  execute the committed references
```

The finite planner decides event time, grasp and route. The mc_rtc QP tracks the
committed plan; it does not solve the high-level argmin.
