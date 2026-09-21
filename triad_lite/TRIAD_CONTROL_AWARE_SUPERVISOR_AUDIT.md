# TRIAD_CONTROL_AWARE_SUPERVISOR_AUDIT

Read-only checkpoint written **before any code change**. Date 2026-09-16.
Subject: the proposal that TRIAD becomes a *control-aware supervisory grasp/entry selector* (not a feedback controller).

Evidence tags: `CODE` (read this session), `DEP` (mc_rtc / Tasks / mc_rbdyn / mc_kortex source), `CONFIG`, `MEASURED` (computed this session), `REPO DOC`, `LIT-FULL` (primary text read in this or the immediately preceding sessions), `BACKGROUND` (standard robotics, not re-read), `INFERENCE`.

---

## 0. What was inspected

- Reports: `/tmp/CALL_ROBOTIC_CONTROL_GAP_DISCOVERY.md`, `/tmp/CALL_FEEDBACK_CONTROL_GAP_AUDIT.md`, `/tmp/TRIAD_CONTROL_DIAGNOSTIC_CHECKPOINT.md`, Astra material (`~/last_5_astra_responses.txt`, `~/astra_contact_literature_recovered.txt`), `research/triad_v2/{V2_EVIDENCE, PHASE2..PHASE6}.md`, `research/triad_control_shot/*`.
- Repositories (separately):
  - **Scientific-audit snapshot** `/home/harry/TRIAD_SCIENTIFIC_AUDIT`, branch `research/triad-continuation-control` @ `0e26083`, remote `github.com/harryjavadinia-creator/TRIAD-handover-controller` (branches `main`, `publication/*`, `private/heldout-robustness-archive-20260910`). Only untracked item: `research/triad_control_shot/`. **This is the only tree containing the V2 receiver.**
  - **Live sandbox** `~/mc_rtc_ws/Sandbox/handover_interception_controller`, checked out `v6.0-c2.1-committed-capture-window-entry` @ `89808a7` with 64 uncommitted paths, remote `CALL-handover-controller`; worktrees for `release/csi-2026`, `v6.4-hardware-staged-bringup`, `v6.5-real-grasp-contact-entry`, `v6.6-binding-cost-dev`, `v6.7.1-cost-validity-fix`. It is an **older controller generation** (no `ReceiverV2`, `Acquire` state instead of `CaptureTransfer`). `CODE`
- Dependencies: `Tasks/src/QPConstr.cpp` (`DamperJointLimitsConstr`), `mc_rtc/src/mc_solver/KinematicsConstraint.cpp`, `mc_rbdyn/Robot.cpp`, `mc_kortex/src/KinovaRobot.cpp`, robot module `kinova_gen3_2f85_mcdesc`. `DEP`
- Literature: Djeha RO-MAN 2022; Finger Flow RAS 2026; **Faris, Tadeja, Forni, arXiv 2511.19543 v2 (Apr 2026)**; CoorGrasp; Tokiwa; Costanzo; Medina; Yan; van Steen; Wang/Dehio/Tanguy/Kheddar; Akinola ICRA 2021 and Yang ICRA 2022 (grasp selection by reachability/manipulability). `LIT-FULL`. Maranci 2026, Koyama 2019 remain inaccessible.

---

## 1. Derived facts that constrain the formulation

### 1.1 What the QP actually enforces (exact) `DEP`

The controller config uses `constraints: [{type: kinematics, damper: [0.1, 0.01, 0.5], velocityPercent: 0.95}]` and `collisions: []`, `contacts: []` `CONFIG`. On the Tasks backend this instantiates `tasks::qp::DamperJointLimitsConstr` (`KinematicsConstraint.cpp:140-170`), whose per-tick bounds on the decision variable q̈ are (`QPConstr.cpp:234-285`):

- velocity: `(0.95·vl − q̇)/Δt ≤ q̈ ≤ (0.95·vu − q̇)/Δt`;
- position damper, for a 1-DoF limited joint with range r, `iDist = 0.1 r`, `sDist = 0.01 r`: when the distance d to a limit is `< iDist`, the next velocity toward that limit is bounded by `ξ (d − sDist)/(iDist − sDist)`, with `ξ = |q̇_entry|·(iDist − sDist)/(d_entry − sDist) + 0.5` fixed at damper activation;
- acceleration and jerk bounds `al/au`, `jl/ju`: passed in, but **±∞** for this robot — `mc_rbdyn` defaults them to ±INFINITY when the module supplies none (`Robot.cpp:96-106, 325-333`), and the Gen3 module provides only a URDF (no acceleration tags) `DEP`.

URDF limits: joints 1, 3, 5, 7 continuous; joint 2 ±2.41, joint 4 ±2.66, joint 6 ±2.23 rad; velocity 1.3963 rad/s (joints 1–4) and 1.2218 rad/s (joints 5–7) `DEP`.

**Consequence 1 — the correct level is velocity, not acceleration.** q̈ is the decision variable, but with unbounded acceleration the one-tick reachable set of q̇ is exactly the velocity box ∩ damper set. Sustained realizable end-effector motion at configuration q is therefore

𝒱_R(q) = { J(q) q̇ : q̇ ∈ 𝔅(q) },  𝔅(q) = [0.95 vl, 0.95 vu] ∩ damper(q).

The acceleration-level set 𝒜_R would be the whole of ℝ⁶ in the column space of J and carries no information for this module. ξ depends on history; using `ξ = offset (0.5)` gives a **history-free inner approximation** (conservative) of the damper bound.

**Consequence 2 — task weights and the posture task are not authority.** They are soft objectives: they shape which q̇ the QP picks, not which q̇ is admissible. They must not enter a feasibility test (they remain an approximation of realized behavior).

**Consequence 3 — no collision constraint is in the QP.** Clearance is enforced by the external geometric filter, so clearance belongs to the geometric/robot feasibility layers, not to 𝒱_R.

**Consequence 4 — Kortex realization.** On hardware the default actuator mode is `POSITION` (`KinovaRobot.cpp:187`); Kortex-internal velocity/acceleration saturation is **not modelled** by mc_rtc and is an unquantified approximation.

### 1.2 Minimal grasp coordinates `CODE`

`buildCandidate(angle, axisSign, …)` (`HandoverInterceptionController.cpp:7398-7446`) defines the grasp frame from the handle axis z_H:
z_M = σ z_H (σ ∈ {+1, −1}); y_M = Rot(z_M, φ)·outward (outward = current mouth − handle centre, projected ⊥ z_H); x_M = y_M × z_M; capture point = p_H + 6 mm·y_M; standoff = capture + 0.12 m·y_M; retreat = capture + 0.18 m·y_M.

So **g = (σ, φ)**. The proposed axial coordinate s is not currently a variable: the capture point is the handle centre and axial offset is tolerated by the insertion corridor (±35 mm, `corridor.axialTolerance`). Moving s by ≤ 35 mm translates the tool frame by ≤ 35 mm; rotating φ re-orients the whole approach. **Dropping s is a deliberate, testable reduction**, not a derived necessity. Resolution for φ: floor Δθ_min = min(ε_p/R, ε_R) = 6.8° (Phase 4, derived from pose tolerances); 32 per sign validated for no loss of earliest admissible time, 53 per sign meets the floor `REPO DOC`.

### 1.3 Timing, routes, commitment `CODE`/`REPO DOC`

- V2 commits only when the object is stopped (`ReceiverV2.cpp:1452-1460`); MovePregrasp's goal is frozen at commit and CaptureTransfer tracks the object only laterally. Prediction validity is 0.10–0.65 s against searched leads of 1.8–8 s (Phase 2).
- Phase 5: "route required" in lateral-low is a reach-duration artefact.
- Phase 2 §2.4: *resumed* preview rollouts disagree with the runtime they certify; from-start previews track runtime within 4.7 mm.
- The FSM has no transition from MovePregrasp/CaptureTransfer back to the receiver: post-commit abort = FAIL.

---

## 2. Falsification attempts

### F1 — "The authority test is just manipulability." **Partly falsified.** `MEASURED`

Offline check (`research/triad_lite/tools/authority_offline.py`, real URDF, exact 𝔅(q) above with ξ = 0.5, IK from the ready posture, 64 φ per sign, required twist = object velocity + insertion along −y_M). Correlation between the directional reserve along the object-motion direction and σ_min(J_lin), across IK-feasible grasps: **+0.72 near-ground, +0.95 longitudinal, +0.31 lateral-low, +0.23 diagonal**. In two geometries an isotropic index would rank grasps almost identically; in the other two it would not. The directional constrained test carries information manipulability does not — but not everywhere.

### F2 — "The test never binds inside the CALL envelope." **Falsified (it binds), with an important qualification.** `MEASURED`

Number of IK+ground-feasible grasps (security distance respected) whose required twist is **not realizable** (residual > 0):

| scenario | feasible | v_obj 0.08, v_ins 0.14 | 0.08, 0.38 | 0.24, 0.14 | 0.24, 0.38 | min reserve along object motion |
|---|---:|---:|---:|---:|---:|---:|
| near-ground | 46 | 18 | 34 | 18 | 46 | **0.042 m/s** (p10 0.090) |
| longitudinal | 24 | 0 | 3 | 0 | 12 | 0.371 |
| lateral-low | 64 | 0 | 5 | 0 | 21 | 0.394 |
| diagonal | 28 | 0 | 0 | 0 | 3 | 0.545 |

Qualification: except near-ground, **following the object alone is never authority-limited**; what binds is following the object *while inserting at the configured far insertion speed (0.38 m/s)*. A downstream controller can often remove that by inserting more slowly — i.e. the consequence of low authority is frequently **timing**, not feasibility. Only in the near-ground/near-limit geometry does low authority threaten tracking itself (damper lowers the median reserve by 0.055 m/s there). This matches Faris et al.'s own limitation ("struggles … when its joints are near their limits").

Offline-check caveats: the Python IK is a plain DLS from one seed, weaker than the controller's barrier-DLS preview — it rejects grasps the controller certifies (e.g. longitudinal `axisP_side_337deg`). Counts are therefore solver-dependent; the *existence* of binding cases is the robust conclusion.

### F3 — "Prior literature makes it trivial." **Largely true as a concept.**

Choosing grasps by robot capability is established: reachability and motion-aware grasp ranking (Akinola ICRA 2021), learned manipulability ranking inside a handover MPC (Yang ICRA 2022) `LIT-FULL`; capability/reachability maps and velocity/force polytopes are textbook-level tools `BACKGROUND`. A supervisor that filters grasps with an exact constrained velocity-polytope test is **standard robotics applied to this stack**. What is not shown anywhere I could read is the *consequence claim*: that on a QP-controlled receiver with a coupled gripper, constraint-aware entry selection improves closed-loop acquisition over reachability-only selection. That is an experimental hypothesis, not a method novelty.

### F4 — "Entry conditioning is known to change outcomes here." **Not supported.**

Repository evidence shows authority-relevant quantities differ across certified grasps (isotropy 0.031–0.117; earliest-completion selection costs ~25% isotropy and 29 mm clearance in lateral-low) — but **no run links them to success or failure**. The recorded campaign cannot, since acquisition is quasi-static, contact is geometric and force is virtual.

### F5 — "τ must stay a searched variable." **Falsified for the present downstream controller.**

The contact event is imposed by the giver's stop; long-horizon τ predictions are invalid while the object moves. τ becomes the first time the measured acquisition-entry conditions hold. Predictive timing would be physically necessary only if acquisition must *start* while the object moves faster than the tracking reserve, or if the object never stops — neither is supported by the current MovePregrasp/CaptureTransfer (goal frozen at commit).

### F6 — "Routes are needed." **Falsified** (Phase 5; ground plane + object are the only obstacles; the external filter and clearance governor handle proximity).

### F7 — "Freeze = commitment = no retry." **Needs correction.** Faris et al. reopen and resume tracking when the object moves during closure. Before admission, reselection with hysteresis is sound. After admission, retry requires FSM transitions out of MovePregrasp/CaptureTransfer, which belong to the acquisition-controller work explicitly out of scope — so the implementation can freeze and log, but cannot yet offer post-admission retry.

---

## 3. The formulation that survives (with modifications)

1. **Decision variable** a = g = (σ, φ), φ on a ring of N per sign (config; default 32). No τ bank, no route bank, no J₇.
2. **Layered feasibility**, reusing the controller's own preview kinematics from the frozen current state (not a new IK):
   - 𝒢_geom: capture reached inside the insertion corridor; bilateral pad contact on the handle without penetration (closure sweep);
   - 𝒢_robot: standoff and capture convergence, joint limits with the preview margin, **no limited joint within the QP security distance sDist**, ground/sensor-core/grey-handle clearance, carried retreat;
   - clearance floor c_floor = 25 mm (Phase 6: runtime reserve 8 mm + measured preview optimism 17 mm).
3. **Authority (velocity level)** at q_capture and q_standoff:
   - demand y_req: tool-frame twist that follows the estimated object motion plus insertion along −y_M at the configured MovePregrasp far speed; optionally a declared disturbance set;
   - residual ε(g) = min_{q̇ ∈ 𝔅(q)} ‖W (J q̇ − y_req)‖, W scales angular rows by the repo's characteristic length (0.20 m);
   - reserve κ*(g) = max{κ ≥ 0 : ε(κ y_req) ≤ tol} (exact by bisection because the feasible κ set is an interval containing 0);
   - admissible iff κ* ≥ κ_min (default 1).
4. **Selection**: lexicographic — admissible → clearance (tie band) → min(κ*, κ_sat) (tie band) → shortest reach. No weights.
5. **Hysteresis**: switch only if the incumbent becomes inadmissible or a challenger stays strictly better beyond the tie bands for a dwell time.
6. **τ as event**: admit when pose error, object-stopped (default on, because the downstream goal is frozen), gripper open, fresh perception, safety, **current-state authority**, and the existing fresh terminal certificate all hold for the stable dwell. Freeze g on admission; reuse the existing single commit into MovePregrasp.
7. **Feature flag**, default off; the V2 bank-search path must remain byte-for-byte behaviorally unchanged.

---

## 4. Answers

**Is this formulation mechanically/control-theoretically sound?**
**Yes, after modification**: the authority set must be velocity-level (the module has unbounded acceleration), must use only hard QP bounds (velocity box ∩ damper, ξ inner-approximated), must exclude soft task weights, and must be evaluated against an explicitly declared demand twist. Without a declared demand, a "reserve" is meaningless.

**Does it map to the actual mc_rtc/Kinova interfaces?**
**Yes in simulation, partially on hardware.** The bounds, Jacobian frame (`gen3_robotiq_85_base_link`) and damper parameters are read from the same model and configuration the QP uses. Kortex-internal saturation (POSITION mode) is not modelled; the IK branch of the evaluated configuration may differ from the configuration the QP actually reaches.

**Is the proposed authority test computable online?**
**Yes.** A 7-variable box-constrained least squares per demand (active-set, finite) plus a bisection; microseconds–milliseconds per candidate in C++. The dominant cost is the existing preview IK/geometry per candidate (already used by V2 static screens), run on the worker thread.

**Does prior literature already make the entire idea scientifically trivial?**
**As a method, yes — it is standard robotics** (capability-aware grasp selection with a constrained velocity-polytope test). It is not trivial as an *engineering replacement* for the old TRIAD, and it is not settled as a *consequence claim*.

**What exact hypothesis would still require experiment?**
> H: On the Kinova Gen3 + Robotiq 2F-85 under the mc_rtc kinematics-constrained QP, among grasps that pass identical geometric and robot feasibility, selecting the grasp whose constrained velocity authority realizes the declared acquisition demand (κ* ≥ 1) yields higher acquisition completion and/or lower acquisition time than reachability-only (nearest-feasible) selection, under matched object presentations — and the difference persists when the downstream insertion speed is tuned competently.

The last clause is essential: if simply slowing insertion removes the difference, the supervisor has no scientific role beyond engineering.

---

## Verdict

**IMPLEMENT WITH MODIFICATIONS**

Modifications relative to the proposal: velocity-level authority from exact hard bounds only; declared demand; s omitted (testable later); object-stopped admission retained by default; freeze on admission with pre-admission reselection only (post-admission retry deferred to the acquisition-controller work); preview-IK configuration dependence stated as an approximation; feature-flagged and off by default; no novelty claim.
