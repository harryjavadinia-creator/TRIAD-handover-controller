# Mathematical formulation

This page summarizes the finite decision problem implemented by TRIAD. For field-level implementation details, see [`global_time_plan.md`](global_time_plan.md), [`binding_cost.md`](binding_cost.md), and [`architecture.md`](architecture.md).

## Decision object

A complete plan is

$$
\xi=(\tau,g,r),
$$

where `tau` is a future presentation event, `g` is a receiver grasp orientation, and `r` is a transit route.

## Bounded finite approximation

TRIAD searches a finite product

$$
\mathcal X_h=\mathcal T_h\times\mathcal G_h\times\mathcal R_h.
$$

For the reported moving-object campaign:

- `T_h`: 14 event-time hypotheses;
- `G_h`: 32 grasp candidates = 16 angular samples × 2 gripper-frame conventions;
- `R_h`: 17 routes = 1 direct route + 16 ring routes.

The upper pre-pruning product is

$$
14\times32\times17=7616.
$$

This is the generated finite bank, not the number of plans that survive complete evaluation and not a claim of continuous-space global optimality.

## Object estimation and prediction

A delayed object measurement is propagated forward by its measured age using the filtered linear and angular velocity estimates:

$$
\hat p(t)=p_{\mathrm{meas}}(t-\tau)+\mathrm{age}\cdot\hat v(t),
\qquad
\hat R(t)=\mathrm{Exp}\!\left(\mathrm{age}\cdot\hat\omega(t)\right)R_{\mathrm{meas}}(t-\tau).
$$

For each candidate event, TRIAD then applies a prescribed smooth terminal deceleration. With stop duration `D`, the predicted presentation pose at lead `h` is

$$
\Pi(h)=\mathrm{Prop}\!\left({}^{W}T_{O}(t_0),\; h-\tfrac12\min(h,D),\; \hat v,\hat\omega\right).
$$

The predictor is deterministic: no covariance, learned predictor, or probability distribution over future states is used.

## Hard-feasible set

Let `s0` denote the common search epoch. The finite hard-feasible set is

$$
\mathcal F_h(s_0)
=\{\xi\in\mathcal X_h:\text{the modeled hard checks pass}\}.
$$

The checks include reachability/IK, sampled collision and ground clearance, joint limits, corridor/acquisition geometry, terminal capture conditions, and receiver-action/retreat feasibility.

Most rollout quantities are evaluated from the common copied planning state. The narrow implementation qualifications to that copied-state abstraction are documented in [`architecture.md`](architecture.md) and [`corrections_of_record.md`](corrections_of_record.md).

## Cost-valid set

A hard-feasible plan is ranked only when the required objective quantities are finite and valid:

$$
\mathcal F_J(s_0)
=\{\xi\in\mathcal F_h(s_0):J_{\mathrm{global}}(\xi;s_0)\text{ is finite and valid}\}.
$$

## Seven-term motion objective

$$
J_{\mathrm{motion}}
=w_TT+w_EE+w_LL+w_CC+w_QQ+w_KK+w_VV.
$$

| Term | Meaning | Weight |
| --- | --- | ---: |
| `T` | time efficiency (`t_complete / 8`) | 0.4210526 |
| `E` | squared joint-speed effort proxy | 0.1052632 |
| `L` | geometric route efficiency (`path_length / 0.50`) | 0.1052632 |
| `C` | clearance reserve | 0.1578947 |
| `Q` | joint-limit reserve | 0.0842105 |
| `K` | kinematic-conditioning reserve | 0.0736842 |
| `V` | joint-velocity-utilization reserve | 0.0526316 |

**Hard feasibility comes first; soft ranking comes second.** The reserve terms `C`, `Q`, `K`, and `V` distinguish among plans that already satisfy the corresponding hard requirements; `T`, `E`, and `L` express efficiency preferences.

### Orientation treatment

Orientation remains part of feasibility through the grasp pose, IK, terminal orientation tolerance, and corridor alignment checks. The residual orientation quantity `R` is logged diagnostically but has `w_R = 0`, so orientation-feasible plans are not additionally ranked by residual orientation error.

The seven binding weights are fixed engineering preferences for the reported controller. No claim of weight optimality or weight-sensitivity robustness is made.

## Cross-event time contribution

The within-event objective already contains the execution-time term

$$
T=\frac{T_{\mathrm{exec}}}{T_{\mathrm{ref}}},
\qquad T_{\mathrm{ref}}=8\text{ s}.
$$

For plans belonging to different future event times, TRIAD also accounts for the wait from the common search epoch to the start of the planned presentation motion:

$$
T_{\mathrm{wait}}=h-T_{\mathrm{pres}}.
$$

The global objective is therefore

$$
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{T_{\mathrm{wait}}}{T_{\mathrm{ref}}}.
$$

This extends the same time preference to a common temporal origin; it is not a separate eighth objective.

## Final timing-admissible set

After the complete bounded schedule has been inspected, TRIAD applies a hard timing gate using the actual selector time `t_sel`:

$$
\mathrm{remaining}(\xi,t_{\mathrm{sel}})
=t_{\mathrm{event}}(\xi)-t_{\mathrm{sel}}.
$$

With implementation epsilon $\varepsilon=10^{-12}$, a plan must satisfy

$$
\mathrm{remaining}+\varepsilon \ge L_{\mathrm{safe}}
$$

and

$$
T_{\mathrm{pres}}+L_{\mathrm{reach}}
\le \mathrm{remaining}+\varepsilon.
$$

Thus

$$
\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})
=\{\xi\in\mathcal F_J(s_0):\text{both timing inequalities hold}\}.
$$

The timing gate is an admission constraint, not another cost term: a good plan can still be rejected if it has become too late to commit safely.

## Exact finite argmin

The selected plan is

$$
\xi_h^{\ast}
=\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
$$

The minimum is exhaustive over the generated bounded finite set. The bank carries no completeness guarantee: failure to find a TRIAD plan does not prove that no physical handover exists outside the tested discretization.

The reported discretization uses 14 event times, 32 grasps, 17 route generators, and 25 evaluated swept poses per commanded segment. These are engineering resolutions; no bank-resolution convergence study is claimed.

## Planner/execution boundary

TRIAD selects the event time, grasp, and transit route, constructs the committed task-space reference, and applies its execution-side reference governor and fail-closed safety logic. The downstream mc_rtc task/QP layer realizes the per-cycle task-space, posture, and gripper references at the robot/joint level; it does not choose the event, grasp, route, or high-level TRIAD objective.

```text
TRIAD: Observe -> Predict -> Generate -> Preview -> Feasibility
       -> Final timing admission -> finite argmin -> Commit
       -> task-space reference + runtime governor

mc_rtc task/QP: realize task/posture/gripper references
```
