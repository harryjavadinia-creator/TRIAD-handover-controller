# Phase C: predictive interception solver (characterization; execution unchanged)

Formulation: `TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md` §3.3–3.5.
Code: `src/PredictiveInterception.h` (pure math, unit-tested) and `ReceiverV2.cpp` (`solveInterceptionV2`, `rolloutInterceptionV2`, `evaluateControlAwareExactLayersV2`).

The config switch is `controlAware.interception.mode`:
- `disabled` (default): no change;
- `characterize`: solve and log alongside TRIAD-lite;
- `characterize_hold`: solve and log, never adopt.

Tags: `CODE` / `DERIVED` / `LIT` / `NUM` / `EXP`.

## 1. What is solved

**Grasp pose.** T_G(g, τ) = T̂_O(t+τ) · ᴼT_G(g). T̂_O is the repository constant-twist predictor (`predictionPoseAtV2`), and ᴼT_G is object-fixed (Phase B family).

**Membership.** (g, τ) ∈ 𝓕_I when every condition below holds, checked in order of cost:

| # | Condition | Type | Provenance |
|---|---|---|---|
| 1 | every mouth centre of T_G above ground at τ | necessary | `DERIVED` |
| 2 | ‖p_G(τ) − p_M(t)‖ ≤ v̄ (τ − L_calc) + lead, angle ≤ ω̄ (τ − L_calc) + lead (skipped at rest) | necessary | `DERIVED` (rate cap and lead tube of the shared reach policy) |
| 3 | wrist SDF ≥ −20 mm at standoff and capture | necessary | Phase B |
| 4 | exact layers at the event pose: standoff reach, corridor capture, closure, carried retreat, security distance, clearance ≥ 25 mm | controller model | `CODE` (unchanged TRIAD-lite layers; now a shared helper) |
| 5 | τ ≥ armScale · T_reach,static + L_calc + L_entry | timing | `LIT` (Hujić: travel time ≤ arrival time, planning-time shift) |
| 6 | timed rollout from the frozen state tracks the Hermite rendezvous reference with the V2 reach law. Ends within 12 mm / 0.05 rad, clearance ≥ 20 mm. | dynamic | `CODE` law + `LIT` boundary-condition form |
| 7 | terminal relative speed ≤ 0.04 m/s and 0.08 rad/s | terminal | `CODE` (MovePregrasp entry gate) |

**Authority** is logged but is not part of 𝓕_I (§3.7). Phase C evaluates it with the TRIAD-lite demand; Phase D replaces the demand.

**Reference.** p_ref = p_G(t) + h00(u) e0 + D h10(u) ė0, and R_ref = R_G(t) Exp(h00(u) Log(R_G0ᵀ R0)).
- Unit tests show position and velocity continuity at t0, and position and velocity matching at t0 + D.
- The velocity matches its finite difference.
- Angular-rate continuity at t0 is not enforced.

**Events.**
- τ_j = L_calc + L_entry + jΔτ, with Δτ = max(20 ms, min(ε_p/|v_G|, ε_R/|ω|)).
- |v_G| ≤ |v_O| + |ω_O| r_max, where r_max bounds the distance from the object origin to any mouth centre.
- ε_p = 15 mm and ε_R = 0.12 rad (commit-freshness tube, `CODE`).
- At rest there is a single event, and τ_g = its travel time (rest collapse, TRIAD Phase 3).
- The horizon is min(8 s, rollout length).

**Solver: ascending best-first sweep over (g, τ_j).**
- A priority queue orders cursors by τ.
- Each grasp starts at its first surrogate-feasible event, which is a proven lower bound.
- A pop evaluates conditions 1–7. On failure the cursor advances.
- The sweep stops when the popped τ exceeds τ_best + Δτ. Remaining grasps are `dominated`, with their current τ as a lower bound.
- There is no secant step and no time bank.
- Guarantee: the returned τ*_g is the first feasible scan sample of the grasp. A feasible island narrower than Δτ may be missed; by construction it corresponds to less than ε_p of meeting-pose change.

**Timing skip** (`timingSkip`, model-based):
- after a timing failure with deficit δ, events closer than δ/(1 + r) are skipped, with r = armScale · |v_G| / v̄;
- this is exact for a straight-line motion-time model;
- it is checked on data in §3.

**Selection** (logged only in Phase C): earliest τ, then the tie-break within Δτ.
- B1: clearance.
- B2: condition index at the rendezvous, then clearance.
- FULL: min(κ, κ_sat), then clearance.

## 2. Unit tests (`tools/test_predictive_interception.cpp`, all pass)

- Pursuit lower bound, closed form vs brute force: stationary, approaching, faster-escaping and 18 random cases.
- Event schedule: rest collapse, ε/|v| spacing, numerical floor, bound beyond the horizon.
- Analytic moving-target oracle:
  - earliest encounter within one step, with the meeting pose within ε_p of the exact crossing;
  - impossible interception (the target leaves the workspace) returns ∞;
  - a changed target velocity changes the bound and the solution.
- Hermite reference: boundary conditions, post-rendezvous following, finite-difference velocity.
- Selection modes (B1/B2/FULL cascades, earliest dominance, none admissible).
- No chatter: 30 alternating near-equal updates give 0 switches with the existing hysteresis. An infeasible incumbent is replaced (`switch_inadmissible`).
- Shortlist order by (surrogate event, score); at rest it reduces to the Phase B order.

Legacy regression is covered by `test_control_aware_grasp_supervisor` and `test_receiving_grasp_family`, unchanged and passing.

## 3. Characterization (`evidence/phaseC_sim`, `EXP`)

**Setup.** `characterize_hold` mode: the arm stays at rest, so the rollout start state is exact. 4 scenarios × {timing skip on, off}, with uncensored budgets (3000 exact evaluations / 400 rollouts, never binding). Summary: `interception_characterization.txt`.

**Stage timing** (all jobs; moving-prediction jobs separately):

| Stage | Median | p90 | Max |
|---|---|---|---|
| front end (530 hypotheses × events) | 0.4–0.5 ms | ≤ 1.6 ms | 1.7 ms |
| sweep, moving-object jobs (n = 61) | 529 ms | 752 ms | 3574 ms |
| sweep, at-rest jobs (n = 325) | 75 ms | 141 ms | 181 ms |
| exact layers per evaluation | ≈ 1–1.4 ms | | |
| rollout per (g, τ) | ≈ 2–4 ms | | |
| job wall, moving (includes the TRIAD-lite at-rest selection) | 0.605 s | 0.816 s | 3.709 s |

**L_calc.** The initial value of 0.25 s was wrong for moving objects. The default is now **0.80 s**, the p90 moving-job wall. Ten percent of moving jobs exceed it; Phase E must refuse an adoption whose realised latency exceeds L_calc. This is a measured bound on this machine. It is not portable.

**Candidate counts per job** (median): G0 530 → G_mech 420–530 → G_R 420–530 → G_K 40.
- 1–5 grasps pass the exact layers at some τ.
- 1–5 are in 𝓕_I within the tie band; 0–5 are dominated.
- Exact evaluations until the first feasible encounter: median 9–24, p90 223–332, max 388.

**Failure layers**, summed over attempts: closure and capture geometry dominate, then standoff collision / IK and timing. Rollout-only failures are mostly `terminal_relative_motion`. Longitudinal is the exception, where `runtime_clearance_reserve` is the most common rollout failure.

**Moving-object encounters.** Every moving job had at least one feasible encounter except in near-ground (6/7 skip, 4/5 noskip). τ_best median is 2.9 s, range 2.36–3.49 s.

**Timing-skip losslessness** (paired, on exact-scan logs): 2590 events that the skip rule would have skipped were evaluated, and **0** passed the timing condition. Its benefit is small (sweep medians 5–10 ms lower). The skip vs no-skip τ_best medians differ by at most 0.375 s because the runs are different simulations (wall-clock-dependent job timing); that comparison is not paired.

**Tie-break rules** (logged, Phase C κ demand):
- B1 = FULL in 92–95 % of jobs with a feasible encounter;
- B1 ≠ B2 in 6–13 % (near-ground: 0 %).

**Full trace:** `evidence/phaseC_sim/example_trace_gen1.txt` (lateral-low, generation 1, object at 0.08 m/s).
- G0 530 → G_mech 420 → G_R 420 → G_K 40.
- Grasp 388: timing failure at τ = 1.613 s (required 2.275 s); skip to 2.175 s; timing failure (required 2.300 s); feasible at **τ* = 2.363 s**. Rollout end error 2.5 mm / 0.011 rad, relative speed 0.026 m/s.
- The other 39 grasps are dominated (lower bound 2.738 s).
- Selected by B1 = B2 = FULL: 388.

**Execution-coupled run** (`tracked_run`, mode `characterize`, reactive TRIAD-lite execution):
- The tracker failed early in 7/8 runs (clearance reserve), so the solver saw few jobs. That motivated the hold mode.
- Disabled-path smoke runs completed: `legacy_control_aware` and `bank_search` on lateral-low.

## 4. Unsupported / approximate

- **Latency.** The rollout assumes the arm is held for L_calc from the snapshot. That is exact in hold mode, approximate while tracking (Phase E), and violated by the 10 % of jobs whose latency exceeds L_calc.
- **Timing model.** T_reach is the static preview duration × armScale (inherited, `EMP`). The rollout is a stronger motion model but does not replace condition 5.
- **Object held at the event pose.** The exact layers assume the object is at the event pose during capture, closure and retreat. The acquisition interface still requires rest (Audit §6).
- **Prediction.** Constant-twist prediction is used unchanged. Phase 2 measured 0.10–0.65 s validity at 15 mm while the giver decelerates, so τ* ≈ 2.9 s encounters rely on predictions far beyond their measured validity. Every result here is *conditional on the prediction*.
- **Seeding.** IK is not seeded from the previous solution. Every job starts from the frozen state. Seeding would break the from-snapshot determinism the purity checkers enforce, and the earliest-encounter guarantee needs the full ascending scan.
- **Budgets and horizon.** `maximumExactEvaluations` 240 / `maximumRollouts` 24 are defaults, not yet derived: the uncensored maximum was 1360 exact evaluations. They are logged when binding. The 8 s horizon is numerical.
