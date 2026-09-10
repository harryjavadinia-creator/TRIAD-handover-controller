# TRIAD mathematical formulation

This page summarizes the finite decision problem implemented by TRIAD. It is the mathematical reference for the method; implementation structure is described separately in [`architecture.md`](architecture.md), [`binding_cost.md`](binding_cost.md), and [`global_time_plan.md`](global_time_plan.md).

## Method at a glance

TRIAD follows one ordered decision pipeline:

```text
complete plan ξ = (event time, grasp, route)
        -> finite candidate bank
        -> predict the future object pose
        -> preview the complete robot action
        -> reject plans that fail modeled hard checks
        -> compute the seven-term motion objective
        -> place time on a common search-epoch basis
        -> apply the current timing-admission gate
        -> choose the finite minimum-cost admissible plan
        -> recheck prediction consistency
        -> commit once and execute
```

The important separation is:

**hard feasibility first, soft ranking second, current timing admission before commitment.**

The final timing gate is a constraint, not another objective term.

## 1. Complete decision variable

A complete handover plan is

```math
\xi=(\tau,g,r),
```

where `tau` is the future handover event time, `g` is the receiver grasp orientation, and `r` is the transit route.

Let `t_0` denote the common search epoch and `s_0` the copied decision state frozen at that epoch. If `h` is the event lead measured from `t_0`, then

```math
t_{\mathrm{event}}=t_0+h.
```

## 2. Finite candidate bank

TRIAD searches a bounded finite product

```math
\mathcal X_h=\mathcal T_h\times\mathcal G_h\times\mathcal R_h.
```

For the reported moving-object campaign:

| Component | Meaning | Size |
| --- | --- | ---: |
| $\mathcal T_h$ | future event-time hypotheses | 14 |
| $\mathcal G_h$ | grasp candidates | 32 |
| $\mathcal R_h$ | route generators | 17 |
| **Upper finite bank** | $14\times32\times17$ | **7616** |

The 32 grasps are 16 angular samples under two gripper-frame conventions. The 17 routes are one direct route plus 16 ring routes.

Route labels are generators rather than fixed world-space paths. For each grasp, the controller generates one direct route and eight offset directions at each of two radii, 0.08 m and 0.14 m. The offset plane is perpendicular to that grasp's start-to-standoff chord.

The reported event leads are:

```text
1.80, 1.90, 2.35, 2.80, 3.25, 3.70, 4.15,
4.60, 5.05, 5.50, 5.95, 6.40, 6.85, 8.00 s
```

The configured cap is 15 event hypotheses; clamping and deduplication produce 14 distinct leads in the reported bank.

The value 7616 is therefore the upper pre-pruning product. It is not the number of plans that survive complete evaluation, and it is not a claim of continuous-space global optimality.

## 3. Object estimation and future-event prediction

Let `a` denote the age of a delayed object measurement. TRIAD propagates that measurement to the current estimation time using the filtered linear and angular velocity estimates:

```math
\hat p(t)=p_{\mathrm{meas}}(t-a)+a\,\hat v(t),
```

```math
\hat R(t)=\mathrm{Exp}\!\left(a\,\hat\omega(t)\right)R_{\mathrm{meas}}(t-a).
```

Here `hat omega` is expressed in world coordinates, and `Exp(a hat omega)` abbreviates the matrix exponential of the corresponding skew-symmetric rotation generator.

For each candidate event, TRIAD then applies a prescribed smooth terminal deceleration. With stop duration `D`, the predicted presentation pose at event lead `h` is represented as

```math
\Pi(h)=\mathrm{Prop}\!\left({}^{W}T_O(t_0),\;h-\frac{1}{2}\min(h,D),\;\hat v,\hat\omega\right).
```

The smooth stop multiplier, for normalized stop progress `u` in `[0,1]`, is

```math
b(u)=1-10u^3+15u^4-6u^5,
```

with

```math
\int_0^1 b(u)\,du=\frac{1}{2}.
```

This produces the half-stop-duration reduction in effective constant-twist travel used above.

The predictor is deterministic. No covariance model, learned predictor, or probability distribution over future states is used.

## 4. Local kinematic preview

Each candidate is previewed with weighted damped least-squares differential inverse kinematics. For geometric Jacobian `J`, task twist `v_task`, and diagonal joint-mobility matrix `M`:

```math
J^\#=MJ^\top\left(JMJ^\top+\lambda^2I\right)^{-1},
```

```math
\dot q=J^\#v_{\mathrm{task}}
+\left(I-J^\#J\right)
\left(\dot q_{\mathrm{posture}}+\dot q_{\mathrm{limit}}\right).
```

Mobility decreases near joint limits, with a floor of 0.02. The configured damping is

```math
\lambda=0.040,
```

and the preview integration step is 0.020 s. Directional joint-speed clipping and a 0.98 fraction-to-boundary scale are applied before integration and forward kinematics. Each segment has a finite iteration budget.

This is local numerical IK. The damped null-space expression is not an exact orthogonal projector, and a failed preview does not prove that no IK solution exists elsewhere in configuration space. The [configuration template](../etc/HandoverInterceptionController.in.yaml) contains the gains, tolerances, and iteration limits.

## 5. Hard-feasible complete plans

The finite hard-feasible set is

```math
\mathcal F_h(s_0)
=\left\{\xi\in\mathcal X_h:\mathrm{modeled\ hard\ checks\ pass}\right\}.
```

A complete plan is retained only after the implemented checks for reachability/local IK, sampled collision and ground clearance, joint limits, corridor/acquisition geometry, terminal capture conditions, and receiver-action/attached-object retreat feasibility have passed.

For a checked proxy pair `k` at sampled pose `j`, a geometric margin condition can be written as

```math
d_{k,j}-m_k\ge0,
```

where `d_(k,j)` is the signed distance and `m_k` is the phase-specific required margin. Ground height, joint bounds, corridor alignment, and bilateral-contact conditions are checked separately; intended contact pairs receive the phase-specific treatment implemented by the controller.

The gripper model uses spherical proxies against object-derived geometry and the ground plane. Arm-ground and attached-object retreat checks extend this model. It is **not** a comprehensive environment, human-body, or self-collision model.

A commanded swept segment is evaluated at 25 poses. Closure uses a separate aperture sweep. No continuous collision guarantee between sampled poses is established.

The complete preview covers reach, approach/insertion, dwell, closure, and attached retreat. It predicts receiver actions under the implemented model; it does not establish frictional force closure or physical transfer dynamics.

Most rollout quantities are evaluated from the common copied planning state. The narrow implementation qualifications to that copied-state abstraction are documented in [`architecture.md`](architecture.md) and [`corrections_of_record.md`](corrections_of_record.md).

## 6. Cost-valid set

Hard feasibility does not automatically make a plan rankable. The required objective quantities must also be finite and valid:

```math
\mathcal F_J(s_0)
=\left\{\xi\in\mathcal F_h(s_0):J_{\mathrm{global}}(\xi;s_0)\ \mathrm{is\ finite\ and\ valid}\right\}.
```

Only plans in this set can enter the final cost comparison.

## 7. Seven-term motion objective

For each cost-valid complete plan, TRIAD computes

```math
J_{\mathrm{motion}}
=w_TT+w_EE+w_LL+w_CC+w_QQ+w_KK+w_VV.
```

| Term | Preference represented | Normalization / definition | Weight |
| --- | --- | --- | ---: |
| `T` | predicted execution-time efficiency | $T_{\mathrm{exec}}/8$ | 0.4210526 |
| `E` | integrated squared joint-speed effort proxy | divided by 8 | 0.1052632 |
| `L` | geometric route efficiency | path length / 0.50 m | 0.1052632 |
| `C` | clearance reserve | logarithmic reserve penalty | 0.1578947 |
| `Q` | joint-limit reserve | soft reference 0.20 | 0.0842105 |
| `K` | kinematic-conditioning reserve | soft reference 0.10 | 0.0736842 |
| `V` | joint-velocity utilization | $\mathrm{clip}(\rho_q,0,1)^4$ | 0.0526316 |

Stored weights are normalized by their sum. These seven weights are frozen controller-specific engineering preferences. They are not literature-derived, are not claimed optimal, and no weight-sensitivity robustness result is claimed for this release.

The effort term is an integrated squared joint-speed proxy, not electrical or mechanical energy. The route term uses the recorded transit-route length.

### Orientation treatment

Orientation remains part of hard feasibility through the grasp pose, IK, terminal orientation tolerance, and corridor alignment checks. The residual orientation quantity `R` is computed and logged diagnostically, but its binding weight is

```math
w_R=0.
```

Thus orientation-feasible plans are not additionally ranked by residual orientation error.

### Clearance reserve

Let `d` be the minimum non-contact reach/retreat clearance, with hard reference

```math
d_h=0.020\ \mathrm{m}
```

and soft reference

```math
d_s=0.080\ \mathrm{m}.
```

The implemented clearance reserve penalty is

```math
\phi_C(d)=0,\qquad d\ge d_s,
```

```math
\phi_C(d)=-\log\!\left(\max\!\left(10^{-12},\frac{d-d_h}{d_s-d_h}\right)\right),\qquad d_h<d<d_s,
```

```math
\phi_C(d)=10^6,\qquad d\le d_h\ \mathrm{or}\ d\ \mathrm{is\ nonfinite}.
```

### Joint and conditioning reserves

For a positive reserve `x` with soft reference `x_s`, the joint-margin and conditioning penalties use

```math
\phi(x;x_s)=0,\qquad x\ge x_s,
```

```math
\phi(x;x_s)=-\log\!\left(\max\!\left(10^{-12},\frac{x}{x_s}\right)\right),\qquad 0<x<x_s,
```

```math
\phi(x;x_s)=10^6,\qquad x\le0\ \mathrm{or}\ x\ \mathrm{is\ nonfinite}.
```

`Q` uses the minimum normalized joint-limit margin with soft reference 0.20.

`K` uses the minimum condition index with soft reference 0.10. The angular rows of the geometric Jacobian are scaled by a 0.20 m characteristic length before the condition index is evaluated:

```math
K_{\mathrm{index}}
=\frac{\sigma_{\min}(\widetilde J)}{\sigma_{\max}(\widetilde J)}.
```

This is a condition index, not determinant-based manipulability.

The joint margin is the minimum of `1-|s_i|` along the checked preview, where `s_i` is joint position relative to the center of its safe interval and normalized by that interval's half-width.

Finally,

```math
V=\mathrm{clip}(\rho_q,0,1)^4,
```

where `rho_q` is maximum directional joint-velocity utilization. The separately logged terminal velocity utilization is not included in `V`.

## 8. Cross-event time comparison

This is the main place where the time notation can otherwise look more complicated than the method actually is.

The seven-term motion objective already contains the normalized predicted execution-time term

```math
T=\frac{T_{\mathrm{exec}}}{T_{\mathrm{ref}}},
\qquad T_{\mathrm{ref}}=8\ \mathrm{s}.
```

For plans associated with different future event times, the implementation extends that **same time preference** back to the common search epoch `t_0`.

Let `T_reach` denote the candidate-specific predicted reach duration used by the event selector. The waiting time before reach is

```math
T_{\mathrm{wait}}=h-T_{\mathrm{reach}}.
```

The cross-event objective is therefore

```math
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{T_{\mathrm{wait}}}{T_{\mathrm{ref}}}.
```

Equivalently,

```math
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{h-T_{\mathrm{reach}}}{T_{\mathrm{ref}}}.
```

This does **not** introduce an eighth objective or a second independent time cost. It places the existing time preference on a common search-to-completion basis so plans attached to different event times can be compared consistently.

The implementation field historically named `predictedPresentationTime` is the candidate-specific predicted reach duration in this calculation; this page uses `T_reach` to make that role explicit and to match [`global_time_plan.md`](global_time_plan.md).

## 9. Current timing admission

Cost ranking and timing admission are different operations.

After the complete bounded event schedule has been evaluated, TRIAD reapplies timing using the actual selector time `t_sel`. The time remaining until a candidate event is

```math
\mathrm{remaining}(\xi,t_{\mathrm{sel}})
=t_{\mathrm{event}}(\xi)-t_{\mathrm{sel}}.
```

Let `L_safe` be the minimum safe commit lead and `L_reach` the minimum reach-entry reserve. With implementation epsilon

```math
\varepsilon=10^{-12},
```

a candidate must satisfy both

```math
\mathrm{remaining}+\varepsilon\ge L_{\mathrm{safe}},
```

and

```math
T_{\mathrm{reach}}+L_{\mathrm{reach}}
\le\mathrm{remaining}+\varepsilon.
```

The final timing-admissible set is

```math
\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})
=\left\{\xi\in\mathcal F_J(s_0):\mathrm{both\ timing\ inequalities\ hold}\right\}.
```

A plan may therefore have an excellent objective value and still be rejected if too little time remains to commit it safely. The timing gate is a **hard admission constraint**, not another term in `J`.

## 10. Final finite selection

The reported moving-object decision is

```math
\xi_h^*
=\arg\min_{\xi\in\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})}
J_{\mathrm{global}}(\xi;s_0).
```

This minimum is exhaustive only over the generated bounded finite bank. The bank has no completeness guarantee: failure to find a TRIAD plan does not prove that no physically feasible handover exists outside the tested discretization.

The implementation first computes the numerical minimum `J_min`, then forms a deterministic tie set satisfying

```math
|J-J_{\min}|\le10^{-9}.
```

Secondary ordering uses predicted search-to-completion duration, event time, predicted reach/presentation duration, larger clearance, candidate name, route name, hypothesis index, and source index. A selected cost may therefore lie within the configured tie tolerance of the numerical minimum.

The reported discretization uses 14 event times, 32 grasps, 17 route generators, and 25 evaluated swept poses per commanded segment. These are engineering resolutions; no bank-resolution convergence study is claimed.

## 11. Commit-time prediction consistency

Before execution, the selected winner is checked again for timing and future-pose consistency.

Let `(R_f,p_f)` be the frozen event pose attached to the selected plan, and `(R_c,p_c)` the refreshed event pose predicted at commitment. The implemented acceptance thresholds are

```math
\|p_c-p_f\|_2\le0.015\ \mathrm{m},
```

and

```math
\|\mathrm{Log}(R_cR_f^\top)\|_2\le0.12\ \mathrm{rad}.
```

`Log` denotes the rotation-vector logarithm on `SO(3)`.

This is a prediction-consistency check, not merely a timestamp-age threshold. An expired or prediction-inconsistent winner is rejected without selecting a replacement at commitment.

After one-time commitment, execution guards supervise the selected plan. There is no global reselection, retiming, or replanning after commitment.

## 12. Planner–execution boundary

TRIAD chooses the event time, grasp, and transit route. It also constructs and governs the committed task-space reference used during execution.

The downstream mc_rtc task/QP layer realizes the per-cycle task-space, posture, and gripper references at the robot/joint level. It does **not** choose `tau`, `g`, `r`, or the high-level TRIAD objective.

Per cycle, the implementation supplies a target pose, body-frame reference velocity, zero reference acceleration, posture target, and gripper target. Its mc_rtc configuration adds kinematics constraints, with no QP collision constraint or contacts configured by this controller.

The [architecture diagram and pseudocode](architecture.md) show the background-planning, final-admission, commitment, and execution boundaries.
