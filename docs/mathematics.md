# Mathematical formulation

This page summarizes the finite decision problem implemented by TRIAD. For field-level implementation details, see [`global_time_plan.md`](global_time_plan.md), [`binding_cost.md`](binding_cost.md), and [`architecture.md`](architecture.md).

## Decision object

A complete plan is

```math
\xi=(\tau,g,r),
```

where `tau` is a future presentation event, `g` is a receiver grasp orientation, and `r` is a transit route.

## Bounded finite approximation

| Symbol | Meaning |
| --- | --- |
| $t_0$, $s_0$ | Search epoch and copied decision state |
| $h$ | Event lead measured from $t_0$ |
| $t_{\mathrm{event}}=t_0+h$ | Absolute presentation time |
| $t_{\mathrm{sel}}$ | Controller time when the worker result is selected |
| $T_{\mathrm{pres}}$ | Predicted presentation duration used by the timing gate |
| $L_{\mathrm{safe}}$, $L_{\mathrm{reach}}$ | Minimum safe commit and reach-entry leads |
| $ {}^{W}T_O=(R,p)$ | Object pose in the world frame |

TRIAD searches a finite product

```math
\mathcal X_h=\mathcal T_h\times\mathcal G_h\times\mathcal R_h.
```

For the reported moving-object campaign:

- `T_h`: 14 event-time hypotheses;
- `G_h`: 32 grasp candidates = 16 angular samples × 2 gripper-frame conventions;
- `R_h`: 17 routes = 1 direct route + 16 ring routes.

The upper pre-pruning product is

```math
14\times32\times17=7616.
```

This is the generated finite bank, not the number of plans that survive complete evaluation and not a claim of continuous-space global optimality.

Route labels are generators: each grasp produces a direct route and eight
offset directions at each of two radii, 0.08 m and 0.14 m. Their plane is
perpendicular to that grasp's start-to-standoff chord. They are not identical
world-space paths shared by all grasps.

The reported event leads are 1.80, 1.90, 2.35, 2.80, 3.25, 3.70, 4.15,
4.60, 5.05, 5.50, 5.95, 6.40, 6.85, and 8.00 s. The configured cap is 15;
clamping and deduplication produce 14 distinct leads in this bank.

## Object estimation and prediction

Let `a` denote the age of the delayed object measurement. TRIAD propagates that measurement to the current estimation time using the filtered linear and angular velocity estimates:

```math
\hat p(t)=p_{\mathrm{meas}}(t-a)+a\,\hat v(t),
\qquad
\hat R(t)=\mathrm{Exp}\!\left(a\,\hat\omega(t)\right)R_{\mathrm{meas}}(t-a).
```

For each candidate event, TRIAD then applies a prescribed smooth terminal deceleration. With stop duration `D`, the predicted presentation pose at lead `h` is

```math
\Pi(h)=\mathrm{Prop}\!\left({}^{W}T_{O}(t_0),\; h-\tfrac12\min(h,D),\; \hat v,\hat\omega\right).
```

The predictor is deterministic: no covariance, learned predictor, or probability distribution over future states is used.

Here $\hat\omega$ is expressed in world coordinates, and
$\mathrm{Exp}(a\hat\omega)$ abbreviates the matrix exponential of
$a[\hat\omega]_\times$. The stop velocity multiplier, for normalized stop
progress $u\in[0,1]$, is

```math
b(u)=1-10u^3+15u^4-6u^5,\qquad
\int_0^1 b(u)\,du=\tfrac12.
```

This gives the half-stop-duration reduction in effective travel above.

## Local kinematic preview

The preview uses weighted damped least-squares differential IK. For geometric
Jacobian $J$, task twist $v_{\mathrm{task}}$, and diagonal joint mobility $M$:

```math
J^\#=MJ^\top(JMJ^\top+\lambda^2I)^{-1},
```

```math
\dot q=J^\#v_{\mathrm{task}}
 +(I-J^\#J)(\dot q_{\mathrm{posture}}+\dot q_{\mathrm{limit}}).
```

Mobility decreases near joint limits, with a floor of 0.02. The configured
damping is $\lambda=0.040$ and preview step is 0.020 s. Directional joint-speed
clipping and a 0.98 fraction-to-boundary scale precede integration and forward
kinematics. Each segment has a finite iteration budget.

This is local numerical IK. The damped null-space expression is not an exact
orthogonal projector, and failed preview does not prove that no IK solution
exists. The [configuration template](../etc/HandoverInterceptionController.in.yaml)
defines the gains, phase tolerances, and iteration limits.

## Hard-feasible set

Let $s_0$ denote the copied decision state at search epoch $t_0$. The finite hard-feasible set is

```math
\mathcal F_h(s_0)
=\{\xi\in\mathcal X_h:\text{the modeled hard checks pass}\}.
```

The checks include reachability/IK, sampled collision and ground clearance, joint limits, corridor/acquisition geometry, terminal capture conditions, and receiver-action/retreat feasibility.

Most rollout quantities are evaluated from the common copied planning state. The narrow implementation qualifications to that copied-state abstraction are documented in [`architecture.md`](architecture.md) and [`corrections_of_record.md`](corrections_of_record.md).

### Geometric and phase checks

For proxy pair $k$ at sampled pose $j$, the geometric check compares signed
distance $d_{k,j}$ with its phase-specific margin $m_k$. Such a margin check
can be written $d_{k,j}-m_k\ge0$ over the pairs checked in that phase.
Ground height, joint bounds, corridor alignment, and bilateral-contact
conditions are checked separately. Intended contact pairs receive the
phase-specific treatment implemented by the controller.

The gripper model uses spherical proxies against object-derived geometry and
the ground plane. Arm-ground and attached-object retreat checks extend this
model; it is not a comprehensive environment, human-body, or self-collision
model. A commanded swept segment is checked at 25 poses. Closure uses a
separate aperture sweep. No bound on collisions between samples is established.

The complete preview checks reach, approach/insertion, dwell, closure, and
attached retreat. It predicts receiver actions under the implemented model;
it does not establish frictional force closure or physical transfer dynamics.

## Cost-valid set

A hard-feasible plan is ranked only when the required objective quantities are finite and valid:

```math
\mathcal F_J(s_0)
=\{\xi\in\mathcal F_h(s_0):J_{\mathrm{global}}(\xi;s_0)\text{ is finite and valid}\}.
```

## Seven-term motion objective

```math
J_{\mathrm{motion}}
=w_TT+w_EE+w_LL+w_CC+w_QQ+w_KK+w_VV.
```

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

### Objective term definitions

Stored weights are normalized by their sum. Effort is the preview's integrated
squared joint-speed proxy divided by 8; it is not electrical or mechanical
energy. Path length is the recorded transit-route length divided by 0.50 m.

For clearance $d$, hard reference $d_h$, and soft reference $d_s$, the clearance reserve is:

```math
\phi_C(d)=0,\qquad d\ge d_s.
```

```math
\phi_C(d)=-\log\!\left(\max\!\left(10^{-12},\frac{d-d_h}{d_s-d_h}\right)\right),\qquad d_h<d<d_s.
```

```math
\phi_C(d)=10^6,\qquad d\le d_h\ \text{or}\ d\ \text{is nonfinite}.
```

Here $d$ is the minimum non-contact reach/retreat clearance, $d_h=0.020$ m,
and $d_s=0.080$ m.

For a positive reserve $x$ with soft reference $x_s$, the joint and conditioning terms use:

```math
\phi(x;x_s)=0,\qquad x\ge x_s.
```

```math
\phi(x;x_s)=-\log\!\left(\max\!\left(10^{-12},\frac{x}{x_s}\right)\right),\qquad 0<x<x_s.
```

```math
\phi(x;x_s)=10^6,\qquad x\le0\ \text{or}\ x\ \text{is nonfinite}.
```

$Q$ uses minimum normalized joint-limit margin with $x_s=0.20$.
$K$ uses minimum condition index with $x_s=0.10$; the Jacobian's angular
rows are scaled by a 0.20 m characteristic length. This is a condition
index, not determinant-based manipulability. Finally,
$V=\mathrm{clip}(\rho_q,0,1)^4$, where $\rho_q$ is maximum directional
joint-velocity utilization. The separately logged terminal velocity
utilization is not included in $V$.

For the scaled Jacobian $\widetilde J$, the condition index is
$\sigma_{\min}(\widetilde J)/\sigma_{\max}(\widetilde J)$.
The joint margin is the minimum of $1-|s_i|$ along the checked preview,
where $s_i$ is position relative to the centre of its safe joint interval,
normalized by the interval's half-width.

## Cross-event time contribution

The within-event objective already contains the execution-time term

```math
T=\frac{T_{\mathrm{exec}}}{T_{\mathrm{ref}}},
\qquad T_{\mathrm{ref}}=8\text{ s}.
```

For plans belonging to different future event times, TRIAD also accounts for the wait from the common search epoch to the start of the planned presentation motion:

```math
T_{\mathrm{wait}}=h-T_{\mathrm{pres}}.
```

The global objective is therefore

```math
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{T_{\mathrm{wait}}}{T_{\mathrm{ref}}}.
```

This extends the same time preference to a common temporal origin; it is not a separate eighth objective.

## Final timing-admissible set

After the complete bounded schedule has been inspected, TRIAD applies a hard timing gate using the actual selector time `t_sel`:

```math
\mathrm{remaining}(\xi,t_{\mathrm{sel}})
=t_{\mathrm{event}}(\xi)-t_{\mathrm{sel}}.
```

With implementation epsilon $\varepsilon=10^{-12}$, a plan must satisfy

```math
\mathrm{remaining}+\varepsilon \ge L_{\mathrm{safe}}
```

and

```math
T_{\mathrm{pres}}+L_{\mathrm{reach}}
\le \mathrm{remaining}+\varepsilon.
```

Thus

```math
\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})
=\{\xi\in\mathcal F_J(s_0):\text{both timing inequalities hold}\}.
```

The timing gate is an admission constraint, not another cost term: a good plan can still be rejected if it has become too late to commit safely.

## Exact finite argmin

The selected plan is

```math
\xi_h^{\ast}
=\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
```

The minimum is exhaustive over the generated bounded finite set. The bank carries no completeness guarantee: failure to find a TRIAD plan does not prove that no physical handover exists outside the tested discretization.

The implementation first computes the exact numerical minimum $J_{\min}$,
then chooses deterministically among records satisfying
$|J-J_{\min}|\le10^{-9}$. Secondary ordering uses predicted search-to-completion
duration, event time, presentation duration, larger clearance, candidate
name, route name, hypothesis index, and source index. Thus “finite argmin”
includes this numerical tie convention; a selected cost can be up to the
tie tolerance above the numerical minimum.

The reported discretization uses 14 event times, 32 grasps, 17 route generators, and 25 evaluated swept poses per commanded segment. These are engineering resolutions; no bank-resolution convergence study is claimed.

## Planner/execution boundary

Before execution, the selected winner is checked again for timing at
commitment. The live predictor refreshes its future event pose using the
remaining lead. Writing the frozen selected event pose as $(R_f,p_f)$
and the refreshed event pose as $(R_c,p_c)$:

```math
\|p_c-p_f\|_2\le0.015\ {\rm m},\qquad
\|\mathrm{Log}(R_cR_f^\top)\|_2\le0.12\ {\rm rad}.
```

This is a prediction-consistency test, not just a timestamp-age threshold.
Here $\mathrm{Log}$ denotes the rotation-vector logarithm on $SO(3)$.
An expired or stale winner is rejected without choosing a replacement at
commitment. After one-time commitment, execution guards supervise the
selected plan; there is no global reselection, retiming, or replanning.

TRIAD selects the event time, grasp, and transit route, constructs the committed task-space reference, and applies its execution-side reference governor and fail-closed safety logic. The downstream mc_rtc task/QP layer realizes the per-cycle task-space, posture, and gripper references at the robot/joint level; it does not choose the event, grasp, route, or high-level TRIAD objective.

The [architecture diagram and pseudocode](architecture.md) show the worker,
live admission, and execution boundaries. Per cycle the implementation
supplies a target pose, body-frame reference velocity, zero reference
acceleration, posture target, and gripper target. Its mc_rtc configuration
adds kinematics constraints, with no QP collision constraint or contacts.
