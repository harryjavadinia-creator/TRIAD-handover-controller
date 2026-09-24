# TRIAD_PREDICTIVE_INTERCEPTION_AUDIT

Phase A: read-only audit written before the predictive-interception implementation, starting from the control-aware supervisor as first implemented (`TRIAD_CONTROL_AWARE_SUPERVISOR_AUDIT.md`, `TRIAD_CONTROL_AWARE_IMPLEMENTATION.md`). "Phase 2–6" and "TRIAD Phase n" below refer to the receding-mode characterisation studies whose numbers are quoted inline; those studies are not part of this repository.


Evidence tags: `CODE` (this repository), `DEP` (mc_rtc / Tasks / Kortex), `UPSTREAM` (jingxixu/dynamic-grasping @ `5c1e01f`, read in full for the functions cited), `LIT` (primary text read), `REPO DOC`, `DERIVED`, `ARBITRARY` (engineering choice without further provenance).

---

## 1. Closest primary work, as actually implemented

### 1.1 Akinola, Xu et al., dynamic grasping (`UPSTREAM`, `dynamic_grasping_world.py`, `grasp_utils.py`)

**Loop** (`dynamic_grasp`), repeated every cycle while the object moves:

1. **Prediction horizon** from `calculate_prediction_time(distance)`:
   - 2 s if the end-effector–pregrasp distance exceeds `large_prediction_threshold` or is unknown;
   - 1 s in the intermediate band;
   - 0 s below `small_prediction_threshold`.

   **There is no interception-time solve**: the look-ahead is a distance-threshold heuristic.
2. `rank_grasps`: every pregrasp is expressed in the world at the *predicted* object pose, then scored by trilinear interpolation of a **6-D reachability SDF** over (x, y, z, r, p, y). The SDF comes from a precomputed binary IK-reachability grid, converted with `skfmm.distance`, periodic in orientation. An optional learned "motion-aware" quality is added.
3. `select_grasp_with_ik_from_ranked_grasp`: try IK on the ranked list up to `max_check` and take the **first** success.
4. **Incumbent retention**: if the previous grasp's pregrasp and grasp IK still succeed at the new prediction, it is kept without re-ranking (unless `always_try_switching`).
5. **Motion planning** from a future waypoint of the current plan (`predicted_period = 0.25 s`), seeded with the unexecuted previous trajectory (`use_seed_trajectory`); "lazy" replanning skips small changes.
6. **Timed final approach** (`get_grasping_plan_timed_control`):
   - object velocity by finite difference;
   - `approach_duration = back_off / max_eef_speed`, with `max_eef_speed = back_off` and the in-code comment **"should be dependent on the Jacobian"**;
   - closure duration 0.2 × approach;
   - IK waypoints seeded in sequence, rejected on a joint jump > 2 rad.

**What transfers:** the funnel structure (cheap reachability ranking → bounded exact IK → first success), incumbent retention, seeded replanning, and prediction-conditioned grasp poses.

**What does not transfer:** their 6-D SDF data, `max_check`, back-off distances and thresholds. Their horizon rule and constant approach speed are exactly the parts the formulation here replaces.

### 1.2 Croft, Fenton, Benhabib (IEEE TSMC 28(2), 1998) and Hujić, Croft, Zak, Fenton, Mills, Benhabib (IEEE/ASME T-Mech 3(3), 1998) (`LIT`, open PDF of the latter)

**Active prediction, planning and execution (APPE).** The rendezvous point is the *pregrasping* location on the predicted object trajectory; afterwards "a fine-motion tracking strategy would be utilized to grasp".

**Planning** is a nested search:
- the **outer loop** evaluates potential rendezvous states on the predicted trajectory;
- the **inner loop** computes the minimum robot-motion time to each (near-time-optimal point-to-point quintic trajectories under robot velocity/acceleration limits);
- the optimum is **"the earliest feasible rendezvous point"**: the first intersection of the robot travel-time line with the target arrival-time line. The travel-time line is shifted by the planning time `t_calc`.

**Solver.** An iterative intersection search between the workspace entry and exit states, proven monotonically convergent in their reference [16] under their monotone travel-time structure.

**Replanning** with each new prediction:
- keep the aimed rendezvous if the new predicted state lies within tolerance/uncertainty limits;
- otherwise plan a trajectory **patch** with motion continuity at the patch start;
- conservative rule: **"better early than late"**.

**Reported:** 39–44 replanned patches per interception at 0.025–0.05 s cycles.

### 1.3 Other audited work

- **Djeha RO-MAN 2022** (`LIT`): QP tracking of an observed moving grasp frame, with no future-time selection. This is **B0** here (reactive).
- **Yang ICRA 2022** (`LIT`): MPC over a grasp set with learned reachability, then a blocking grasp.
- **Faris et al. 2025/26** (`LIT`): reactive virtual-model tracking with reopen/retry.
- **Finger Flow** (`LIT`): reactive, torque-controlled hand.
- **Menon ICRA 2014**: earliest feasible pickup on a known trajectory.

None of these contradicts the APPE structure. None provides a controller-authority reserve at the rendezvous.

### 1.4 Repository evidence already derived

- **Phase 3** (`REPO DOC`):
  - At rest the time dimension collapses to one event (DERIVED).
  - While moving, events must be **anchored** at the time the decision will be used (DERIVED + OFFLINE).
  - Spacing Δτ = min(ε_p/|v̂|, ε_R/|ω̂|) (DERIVED form; ε_p = 15 mm, ε_R = 0.12 rad inherited).
  - Latency anchor A = 1.0 s (EMPIRICAL).
  - This is the repository's analogue of Hujić's `t_calc` shift and first-intersection rule.
- **Phase 2** (`REPO DOC`): constant-twist prediction is accurate to ≤ 0.23 mm at 3 s while velocity is constant, but valid only 0.10–0.65 s across an unannounced stop. Receding updates are therefore mandatory, not optional.
- **Phase 5** (`REPO DOC`): the route dimension is a reach-duration artefact.

---

## 2. What the existing timed-reach machinery actually does (`CODE`)

- `interceptionMouthPoseAt` (`HandoverInterceptionController.cpp:2245`) blends from `mouthAtReachStart` to the standoff at the object pose **at `standoffTime`** with `naturalReachStep(u)`. The reference therefore arrives at a *fixed* predicted meeting pose.
- In `presentationMode` the object is modelled at rest from `presentationTime` (`conditionalPresentationV2`). `executeProvisionalReachV2` zeroes the reference twist at `standoffTime`.
- **Consequence:** the receding receiver's rendezvous is a meeting with an object *assumed to stop there*. If the object keeps moving, the terminal relative velocity equals the object velocity, and **there is no terminal velocity matching**. Hujić's rendezvous "state" includes the target's motion.
- The certification rollout (`stepPredictiveRouteCandidate`) integrates the **same** reference, with the clearance governor, lead limits, swept collision and joint limits. Its terminal chain (approach, dwell, closure, carried retreat) is certified with the object held at the rendezvous anchor.
- **Downstream acquisition interface.** MovePregrasp freezes its world goal at commit. CaptureTransfer fails if the pose error to a fixed nominal target exceeds 30 mm, and centers only along the closing axis. **The existing acquisition controller cannot accept a moving terminal state.** This limitation is exposed here and is not hidden (§6).

---

## 3. Formulation

### 3.1 State available at decision time t (`CODE`)

I_t = (q, q̇ (model), T̂_O(t), V̂_O(t) (constant-twist record, zeroed below rest thresholds), measurement age, gripper opening, incumbent (g, τ, plan), controller clock).

Object prediction: T̂_O(t+τ) = propagateConstantTwist(T̂_O(t), τ, V̂_O). This is the repository predictor, used unchanged.

### 3.2 Decision variables

**(g, τ)**, with g from the mechanical family (§4) and τ ≥ 0 a *continuous event time* measured from t.

There is no route variable (Phase 5; the direct reference only) and no time bank.

### 3.3 Feasible interception set (implemented definition)

T_G(g, τ) = T̂_O(t+τ) · ᴼT_G,standoff(g). The rendezvous is the pregrasp/standoff, as in Hujić; capture follows.

(g, τ) ∈ 𝓕_I iff all of:

1. **Mechanical**: g ∈ 𝒢_mech (§4).
2. **Exact robot feasibility at the rendezvous**: the controller's preview IK converges from the current frozen state to T_G(g, τ) and to the capture pose (corridor). The terminal closure sweep and carried retreat at the rendezvous anchor succeed (the existing layered checks). No limited joint lies inside the QP security distance.
3. **Timing (Hujić travel-time ≤ arrival time, Phase 3 anchor)**: τ ≥ T_reach(g, τ) + L_calc + L_entry.
   - T_reach = `timingArmScale` × preview reach duration to T_G(g, τ) (the certifier's own motion-time model);
   - L_calc = measured solver latency bound (§5.3);
   - L_entry = `minimumReachEntryLead`.
4. **Dynamic interception**: the timed rollout from the current state tracks the **moving-rendezvous reference** (§3.4) and ends within the reach tolerance of T_G(g, τ) at t+τ, with no clearance, joint-limit or swept-collision violation.
5. **Terminal relative motion**: the reference relative twist at t+τ is zero by construction (§3.4). The rollout's terminal mouth velocity differs from the predicted grasp velocity by at most v_rel,max.
   - v_rel,max = `terminalLinearSpeedTolerance` 0.04 m/s and 0.08 rad/s, the existing MovePregrasp entry gate (`CODE`).

Earliest encounter per grasp: τ*_g = inf{τ : (g, τ) ∈ 𝓕_I}. Meeting pose: T*_meet,g = T_G(g, τ*_g).

### 3.4 Moving-rendezvous reference with boundary conditions (literature form, new code)

Hujić/Croft plan quintic patches between specified initial and final *states*, with continuity at patch start. The existing reference only matches final position.

For the predictive mode, position error decays with cubic Hermite boundary conditions relative to the moving grasp trajectory. With u = (s − t₀)/(τ* − t₀) ∈ [0, 1] and D = τ* − t₀:

p_ref(s) = p_G(s) + h₀₀(u)·e₀ + D·h₁₀(u)·ė₀,  h₀₀ = 2u³ − 3u² + 1,  h₁₀ = u³ − 2u² + u,

where
- e₀ = p_ref(t₀) − p_G(t₀);
- ė₀ = v_ref(t₀) − v_G(t₀) (zero for a first plan);
- R_ref(s) = R_G(s)·Exp(h₀₀(u)·Log(R_G(t₀)ᵀ R_ref(t₀))).

**Properties** (`DERIVED`):
- p_ref(t₀) and v_ref(t₀) match the current reference (patch continuity);
- p_ref(t₀+D) = p_G and v_ref(t₀+D) = v_G (velocity matching at the rendezvous);
- orientation matches in pose, and angular-rate continuity is not enforced (stated approximation).

The rollout and execution use the same function, preserving the Phase 2 from-start parity property.

### 3.5 Solver for τ*_g (numerical method, not a bank)

Hujić's secant-type intersection needs monotone travel time. The predicate here includes IK convergence, collision and joint limits, so it is **not monotone** (Phase 3 observed infeasible gaps). The defensible method is an **ascending first-feasible scan** of the continuous event at derived resolution.

1. **Lower bound** (`DERIVED`, necessary condition): τ_lb = smallest τ ≥ 0 with ‖p_G(t+τ) − p_mouth(t)‖ ≤ v̄·τ.
   - v̄ = `predictiveReachPolicy.farLinearSpeed`, the reference speed cap;
   - closed form (quadratic) for a constant-velocity prediction.
2. **Scan**: τ_j = max(τ_lb, L_calc + L_entry) + j·Δτ, Δτ = max(Δτ_num, min(ε_p/|v̂_O|, ε_R/|ω̂_O|)) (Phase 3; at rest a single event). Stop at the first τ_j satisfying §3.3, or at τ_max.
3. **Cost control**: condition 2 (exact static IK) runs first; 3 uses its reach time; the rollout (4, 5) runs only when 2–3 hold.
4. **Guarantee**: the returned τ*_g is the first feasible sample. Any earlier feasible island narrower than Δτ may be missed; by construction such an island corresponds to a meeting-pose change of less than ε_p, the certification tolerance.
5. **Horizon τ_max**: `DERIVED` from the scripted-giver travel is not available to the receiver (independence rule). It is a declared numerical horizon (default 8 s = old maximum lead) — `ARBITRARY`, logged.

### 3.6 Receding update (Hujić replanning, Akinola incumbent retention)

On every completed worker job, with the newest prediction:
1. **Aimed-state check**: if the incumbent's predicted grasp pose at its τ lies within ε_p / ε_R of the plan's meeting pose, keep the plan; the receding-mode retain rule updates targets in place.
2. **Otherwise**: re-solve τ*_g for the incumbent from the current state. The new plan is a Hermite patch starting at the current reference position and velocity (§3.4). The robot never stops to compute.
3. **Challengers**: an alternative (g′, τ′) replaces the incumbent only if the incumbent is infeasible, or if the challenger stays materially earlier (τ′ < τ_inc − Δτ) for the declared dwell. This uses the existing hysteresis machinery.
4. **Local synchronization**: from t ≥ τ* − T_sync (the reference within the terminal tolerance), control passes to the existing live object-relative tracking law (TerminalTrack: bounded step toward T̂_O(t)·ᴼT_G with object-twist feedforward). The repository has no terminal local-sync law beyond this.
5. **Freeze** only at the irreversible acquisition commit.

### 3.7 Controller-authority demand (replaces the reactive supervisor's y_follow + 0.38·y_insert)

The previous sum mixed two phases that cannot co-occur under the current interface: insertion starts only after the object is at rest. The demand is now the **set of phase-specific task-space twists the downstream controllers actually command**. The existing solver is used unchanged.

| Phase | Configuration | y_req (tool body, world, [ω; v]) | Source |
|---|---|---|---|
| Interception motion | rollout samples q(s_k) at a declared stride | reference twist of §3.4 at s_k | same function the rollout tracks |
| Synchronization | q at the rendezvous (rollout end) | [ω̂_O; v̂_O + ω̂_O × (p_B − p_O)] at t+τ | TerminalTrack feedforward law |
| Insertion (acquisition interface, object at rest) | q at capture | MovePregrasp commanded twist: far-speed cap along −y_M | MovePregrasp speed law (`farLinearSpeed`) |

κ(g, τ) = min over phases/samples of κ*(y_req, q), and 𝓕_C = {(g, τ) ∈ 𝓕_I : κ(g, τ) ≥ κ_min}. **κ never decides interception existence.**

### 3.8 Selection (no weights)

1. Hard: (g, τ) ∈ 𝓕_I, clearance ≥ floor, and (FULL only) κ ≥ κ_min.
2. **Earliest τ*.** This is the literature objective (Croft/Hujić earliest rendezvous).
3. Among candidates with τ* within Δτ of the earliest (numerically indistinguishable encounters):
   - FULL: larger min(κ, κ_sat), then clearance;
   - B1: clearance only;
   - B2: larger generic capability (condition index of the length-scaled Jacobian at the rendezvous — the established manipulability-type measure), then clearance.
4. Deterministic id.

Hysteresis per §3.6.

---

## 4. Mechanical receiving-grasp family (derivation; implemented in Phase B)

Blue handle: cylinder with radius R_H = 11.25 mm and half-length L_H = 68.7 mm, centred at ᴼT_H (z = −86.9 mm), coaxial with the object z axis. The sensor core (R = 17 mm, half-length 18.2 mm) and the grey human handle are coaxial (`CONFIG`).

Robotiq 2F-85 model (`CODE` collision lattice):
- parallel jaw, closing axis x_M;
- fingertip body half-width along the finger lateral direction 8 + 4.5 = 12.5 mm;
- inner pad contact points at ±25.3 mm in the tip frame.

1. **Antipodal parallel-jaw contact on a cylinder.** Contact normals must be opposed along the closing axis, so the closing axis passes through the handle axis and is perpendicular to it: x_M = n(θ), c± = p_H + s ĥ ± R_H n(θ). This leaves 2 continuous DOF (s, θ).
2. **Pad line contact and insertion corridor.** The finger width direction must lie along ĥ, so z_M = σĥ (corridor tolerance 10°). **ψ is not free**: it is fixed at 0 up to the corridor tolerance. σ ∈ {+1, −1}.
3. **Hand symmetry.** (σ = −1, θ) and (σ = +1, θ + π) give the same contact set with the fingers swapped. They are one mechanical grasp but **two robot configurations** (wrist flip). σ is kept as a discrete kinematic branch, not a mechanical DOF.
4. **Axisymmetry.**
   - Force closure holds for every θ with μ > 0 (antipodal, exactly opposed normals).
   - The coaxial sensor core and grey handle are θ-invariant.
   - **Mechanics imposes no restriction on θ**; θ is decided by robot, environment and task.
5. **Axial range** (task constraint made explicit): |s| ≤ s_max = min(L_H − w/2 − m, a_corr) = min(68.7 − 12.5 − 3, 35) mm = **35 mm**. The binding term is the acquisition corridor's axial tolerance (`corridor.axialTolerance`), a downstream task constraint. A centred-grasp task sets s_max = 0 explicitly.
6. **Approach, capture, retreat offsets** (standoff 0.12 m, capture depth 6 mm, retreat 0.18 m) are inherited acquisition-interface parameters (`CONFIG`), not scientific constants.
7. **Resolution** (numerical, `DERIVED` pose-change floor):
   - Δθ ≤ min(ε_p/R_lever, ε_R) = 6.8° (Phase 4);
   - Δs ≤ ε_p = 15 mm.
   - Sampling counts are configurable; a convergence test is required.

**Cheap mechanical/environment filter** (analytic, per hypothesis): the axial range, plus ground margin of the standoff, capture and retreat points and of the approach segment. Mechanical validity is expected to prune only s and ground-facing θ.

---

## 5. Cheap reachability funnel (Phase B design)

### 5.1 Surrogate

The Gen3 joint-7 origin lies on the joint-7 axis and is rigidly fixed in the tool frame. Its world position p_w(g) = T_W,B(g)·ᴮp_j7 is therefore an **exact function of the requested tool pose**.

It is also a function of q₁…q₆ only, and joint 1 is continuous about the world z axis through the base (`DEP` URDF). So the set of reachable p_w is a **solid of revolution**, described exactly by a 2-D region in (ρ = √(x² + y²), z).

The region is sampled offline over q₂…q₆ within URDF limits; its signed distance field in (ρ, z) is a **necessary-condition reachability SDF**. It is the reduced analogue of Akinola's 6-D SDF, with the orientation feasibility of joints 5–7 ignored.

Rank score = SDF value in metres (positive inside). This is `DERIVED` except for the grid step (numerical).

### 5.2 Shortlist

The top K by score, with diversity (at most one per (σ, s, θ-bin of 2Δθ)). K is a numerical setting, characterized by:
- recall of the exact-best candidate against K;
- the surrogate false-negative rate against exact preview IK, measured in characterization mode where pruning is disabled.

### 5.3 Latency

L_calc is measured per solve and bounded by the maximum over the previous N solves. This is the adaptive form suggested by Hujić's note that planning time can enter each travel-time computation. It replaces the EMPIRICAL constant A = 1.0 s with a measured quantity; a fixed floor (configurable) covers the first solve.

---

## 6. Interface limitation (not hidden)

The existing MovePregrasp/CaptureTransfer cannot acquire a moving object. In predictive mode:
- interception, velocity-matched meeting and local synchronization run while the object moves;
- **acquisition commit still requires the object at rest** (`requireObjectStopped` retained at the acquisition interface).

H1 can therefore be tested only on approach and meeting behaviour (time to meeting, relative error at meeting, time to commit after the giver stops, completion). **Moving acquisition itself is out of scope** until a moving-terminal acquisition controller exists.

Baselines B0/B1/B2/FULL share the grasp family, funnel, perception, low-level controller and acquisition interface.

---

## 7. Provenance classification

| Element | Class |
|---|---|
| Rendezvous at the pregrasp on the predicted trajectory; earliest feasible rendezvous; travel time ≤ arrival time with planning-time shift; replanning with continuous patches; fine tracking after rendezvous; "early rather than late" | **Literature** (Croft 1998; Hujić 1998) |
| Reachability-SDF ranking → bounded exact IK → first success; incumbent retention; seeded replanning | **Literature/upstream** (Akinola/Xu) |
| Parallel-jaw antipodal reduction to (s, θ, σ); axisymmetric θ invariance; axial bound | **Derived** from geometry |
| Anchoring, Δτ = min(ε_p/\|v\|, ε_R/\|ω\|), rest collapse | **Derived** (Phase 3) |
| Joint-7 wrist-point (ρ, z) reachability SDF | **Derived** (exact solid of revolution; necessary only) |
| Hermite boundary-condition reference, velocity matching | **Literature form** (boundary-condition polynomial patches), **derived** properties |
| Phase-specific authority demands | **Derived** from the downstream control laws in code |
| Earliest-τ primary ordering; κ/clearance tie-break within Δτ | Earliest: literature. Tie-break order: **ARBITRARY** but transparent. |
| ε_p = 15 mm, ε_R = 0.12 rad, v_rel,max, reach/terminal tolerances, standoff/capture/retreat offsets | **Inherited interface parameters** (`CONFIG`) |
| τ_max = 8 s, K, grid steps, rollout κ stride, dwell | **Numerical / tunable** (logged, characterized) |
| κ_min = 1 | **Definitional** (the declared demand must be realizable) |

---

## 8. Verdict

**A defensible formulation exists: the Croft/Hujić APPE earliest-rendezvous problem, instantiated with this controller's own certification model.** It is not a new method.

It is implementable with three required modifications:
1. **ascending first-feasible scan** instead of secant intersection, because the predicate is non-monotone;
2. **velocity-matched Hermite rendezvous reference** shared by certification and execution;
3. **phase-specific authority demands** consistent with the at-rest acquisition interface.

**PROCEED to Phase B.** Stop conditions carried forward:
- if Phase B shows the reachability surrogate has material false negatives that cannot be bounded, use exact IK on all mechanical hypotheses and report the latency instead;
- if the Hermite reference cannot be introduced without changing receding-mode behaviour when disabled, stop and report.
