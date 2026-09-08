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
\Pi(h)=\mathrm{Prop}\!\left({}^{W}T_{O}(t_0),\; h-\tfrac12\min(h,D),\; \hat v,\hat\omega\right).
$$

Here ${}^{W}T_{O}(t_0)$ denotes the object pose expressed in the world frame at
the common search epoch.

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

### Orientation treatment

Orientation is **not removed from the planning problem**. Each grasp candidate
contains an orientation, the copied-state preview/IK must realize the required
pose, and terminal/corridor feasibility includes orientation/alignment checks.
Thus an orientation-infeasible candidate is rejected before ranking.

A separate residual orientation quantity `R` is nevertheless computed and
logged for diagnostics. In the reported binding selector its soft-ranking weight
is `w_R = 0`: among plans that already satisfy the required orientation
constraints, TRIAD does not add a further preference for making that residual
orientation error smaller. This is a frozen engineering design choice, not a
claim that zero orientation weight is theoretically optimal; no objective-weight
sensitivity study is reported.

The seven **binding** weights are frozen controller-specific engineering
preference values. They are not literature-derived and are not claimed optimal.
The values are ratios `8 : 2 : 2 : 3 : 1.6 : 1.4 : 1` over 19 and sum to 1.

`E` is a squared joint-speed effort proxy and is not physical energy; `K` is a
project-specific conditioning reserve and is not exactly Yoshikawa's
manipulability index. A separately logged terminal velocity utilisation does
**not** enter `V` and does **not** gate the terminal timing audit; it is computed
after that audit returns. Its non-diagnostic role is limited to the cost-validity
finiteness contract documented in source/provenance.

**Hard feasibility comes first; soft ranking comes second.** In particular,
`C`, `Q`, `K` and `V` do not replace collision, joint-limit, conditioning or
velocity checks. They discriminate among already admissible plans by rewarding
larger geometric/kinematic reserves, while `T`, `E` and `L` express efficiency
preferences.

## Cross-event time contribution

The within-event motion objective already contains the complete-plan execution
time term

$$
T=\frac{T_{\mathrm{exec}}}{T_{\mathrm{ref}}},
\qquad T_{\mathrm{ref}}=8\text{ s}.
$$

Plans associated with different future event times also differ in how long the
controller must wait from the common search epoch before the planned
presentation motion begins. Let `h` be the event lead from the search epoch and
let $T_{\mathrm{pres}}$ be the candidate's predicted presentation duration. The
implemented pre-reach wait is

$$
T_{\mathrm{wait}}=h-T_{\mathrm{pres}}.
$$

The cross-event objective therefore extends the **same time preference** to the
common search epoch:

$$
J_{\mathrm{global}}
=J_{\mathrm{motion}}
+w_T\frac{T_{\mathrm{wait}}}{T_{\mathrm{ref}}}.
$$

Because `J_motion` already contains $w_T T_{\mathrm{exec}}/T_{\mathrm{ref}}$,
the total time-dependent contribution is

$$
w_T\frac{T_{\mathrm{wait}}+T_{\mathrm{exec}}}{T_{\mathrm{ref}}},
$$

which ranks alternatives by predicted search-to-completion time using one time
weight. The wait term is therefore **not an independent eighth objective and not
a second time weight**; it puts plans from different event hypotheses onto the
same temporal origin.

## Final timing-admissible set

Cost ranking alone cannot guarantee that a plan is still executable when the
finite search finishes. The selector therefore applies a separate **hard timing
gate** after the complete bounded event schedule has been inspected. Let
`t_sel` be the actual selector time. For each cost-valid complete plan,

$$
\mathrm{remaining}(\xi,t_{\mathrm{sel}})
=t_{\mathrm{event}}(\xi)-t_{\mathrm{sel}}.
$$

With implementation epsilon $\varepsilon=10^{-12}$, two conditions must hold:

$$
\mathrm{remaining}+\varepsilon \ge L_{\mathrm{safe}}
$$

and

$$
T_{\mathrm{pres}}+L_{\mathrm{reach}}
\le \mathrm{remaining}+\varepsilon.
$$

The first requires a minimum safe lead before commitment. The second requires
enough remaining time for that candidate's predicted presentation duration plus
the configured minimum reach-entry lead. These are **admission constraints, not
additional cost terms**.

Define

$$
\mathcal F_{\mathrm{timing}}(s_0,t_{\mathrm{sel}})
=\{\xi\in\mathcal F_J(s_0):\text{both timing inequalities hold}\}.
$$

This separation is deliberate: geometric/kinematic feasibility and cost are
evaluated on the frozen planning problem, while timing admission is checked
against the clock at the moment the completed search is ready to commit. A plan
that was attractive at the search epoch can therefore be rejected if it has
become stale by selector time.

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

## Planner/execution boundary

TRIAD does more than choose a plan label. It selects the event time, grasp and
transit route, constructs the committed task-space reference along that route,
and applies the controller's live execution-side reference governor and
fail-closed safety logic. The downstream mc_rtc task/QP layer realizes the
per-cycle task-space, posture and gripper references at the robot/joint level.
It does not choose the event time, grasp, route or high-level TRIAD objective.

```text
TRIAD: Observe -> Predict -> Generate -> Preview -> Feasibility
       -> Final timing admission -> finite argmin -> Commit
       -> committed task-space reference + runtime governor

mc_rtc task/QP: realize the commanded task/posture/gripper references
```
