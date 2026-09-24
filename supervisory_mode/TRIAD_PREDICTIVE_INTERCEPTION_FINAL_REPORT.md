# TRIAD predictive control-aware interception: final report

Tags:
- `CODE`: read in this repo
- `LIT`: literature
- `DERIVED`: derived here
- `EXP`: measured here
- `NUM`: numerical choice
- `CONFIG`: inherited interface value
- `EMP`: inherited empirical calibration

No novelty is claimed. The architecture instantiates the Croft–Hujić earliest-rendezvous problem with this controller's own motion model. It adds the Akinola/Xu reachability funnel and a directional QP-authority filter.

## 0. Documents

Starting point: the control-aware supervisor as first implemented (`TRIAD_CONTROL_AWARE_IMPLEMENTATION.md`). "Phase 2–6" and "TRIAD Phase n" below refer to the receding-mode characterisation studies whose numbers are quoted inline; those studies are not part of this repository.

| Phase | Document |
|---|---|
| A audit | `TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md` |
| B grasp front end and funnel | `PHASE_B_GRASP_FAMILY_AND_FUNNEL.md` |
| C interception solver | `PHASE_C_INTERCEPTION_SOLVER.md` |
| D authority demands, 𝓕_C | `PHASE_D_AUTHORITY_DEMANDS.md` |
| E receding execution, variants | `PHASE_E_RECEDING_EXECUTION.md` |
| F baselines, rollout fix | this file; campaigns `evidence/phaseF_sim_prefix/` and `evidence/phaseF_sim/` |

## 1. Audit result (Phase A)

**Verdict:** a defensible formulation exists: PROCEED, with three modifications.
1. An ascending first-feasible scan replaces the secant intersection, because the feasibility predicate is non-monotone.
2. A velocity-matched Hermite rendezvous reference is shared by certification and execution.
3. The authority demands are phase-specific.

**Interface limit, stated before implementation:** MovePregrasp / CaptureTransfer cannot acquire a moving object. Acquisition therefore still requires rest; H1 is testable only on approach, meeting and time to commit.

## 2. Exact equations

**Grasp family** (`DERIVED`).
- Closing direction: n(θ) = e1 cos θ + e2 sin θ.
- Contacts: c± = p_H + s ĥ ± R_H n(θ).
- Gripper frame: x_M = n(θ), z_M = σĥ, y_M = z_M × x_M, so the roll ψ is fixed.
- Mouth poses: capture / standoff / retreat = p_H + s ĥ + d·y_M with d = 6 / 120 / 180 mm.
- Axial bound: |s| ≤ min(L_H − w_f − m, 35 mm) = 35 mm.
- θ resolution floor: Δθ ≤ min(ε_p/ℓ, ε_R), giving N_θ = 53 per sign.
- Remaining DOF: (σ, θ, s). Setting `axialMax: 0` imposes s ≡ 0.

**Reachability surrogate.** The Gen3 joint-7 origin sits at p_wrist = p_tool + R_tool (0, 0, −0.061525).
- SDF(ρ, z) of its reachable solid of revolution; ρ = √(x² + y²) in the base frame.
- Score = min(SDF at the standoff tool pose, SDF at the capture tool pose). It is a necessary condition only.
- Prune when score < −20 mm.
- Order by (first surrogate-feasible event, −score), then a diversity pass over (σ, s, θ-bin), then fill.

**Interception.** T_G(g, τ) = T̂_O(t+τ) · ᴼT_G(g), where T̂_O is the constant-twist prediction.

(g, τ) ∈ 𝓕_I requires all of:
- **ground:** all mouth centres above the ground;
- **pursuit bound:** ‖p_G(τ) − p_start‖ ≤ v̄(τ − L_calc) + lead, and angle ≤ ω̄(τ − L_calc) + lead (moving objects only);
- **surrogate:** SDF ≥ −20 mm;
- **exact layers** at T_G(g, τ);
- **timing:** τ ≥ armScale·T_reach + L_calc + L_entry;
- **timed rollout** of p_ref = p_G(t) + h00 e0 + D h10 ė0 and R_ref = R_G(t) Exp(h00 Log(R_G0ᵀ R0)) under the receding-mode reach law, ending within 12 mm / 0.05 rad with clearance ≥ 20 mm;
- **terminal relative motion:** ‖v_M − v_G‖ ≤ 0.04 m/s and ‖ω_M − ω_O‖ ≤ 0.08 rad/s.

Here v_G = v_O + ω_O × (p_G − p_O), h00 = 2u³ − 3u² + 1 and h10 = u³ − 2u² + u.

**Earliest encounter.** τ*_g = the first feasible τ_j among τ_j = L_calc + L_entry + jΔτ.
- Δτ = max(20 ms, min(ε_p/|v_G|, ε_R/|ω_O|)), with |v_G| ≤ |v_O| + |ω_O| r_max.
- At rest the schedule has a single event and τ*_g = armScale·T_reach + L_calc + L_entry.
- The timing skip jumps δ/(1 + r), with r = armScale |v_G| / v̄.

**Authority.**
- κ*(y, q) = sup{κ : min over q̇ ∈ 𝔅(q) of ‖W(Jq̇ − κy)‖ ≤ ε}, with 𝔅 = 0.95·[v_l, v_u] ∩ damper and W = diag(L, L, L, 1, 1, 1).
- κ(g, τ) = min over:
  - y_path: commanded step twists on the rollout, [Log(R_{k+1} R_kᵀ)/dt; (p_{k+1} − p_k)/dt] of consecutive commanded body poses;
  - y_sync = [ω_O; v_O + ω_O × (p_B − p_O)] at the rendezvous q;
  - y_insert = [0; −v_far y_M] at the standoff and capture q.
- 𝓕_C = {(g, τ) ∈ 𝓕_I : κ ≥ 1}.

**Selection** (no weights).
1. Admissible: 𝓕_I for B1/B2, 𝓕_C for FULL.
2. Earliest τ.
3. Inside Δτ (at rest ε_p/v_far): B1 by clearance; B2 by condition index, then clearance; FULL by min(κ, 2), then clearance.
4. Grasp id.

Hysteresis is the unchanged `updateGraspSelector` with a 0.3 s dwell.

**Receding execution.**
- The plan is kept while ‖aimed − meeting‖ ≤ ε_p and angle ≤ ε_R, where aimed = T̂_O(t_R) · ᴼT_M and the prediction is the newest one.
- Otherwise the plan is patched from the executed reference (p, v).
- Results with latency > L_calc are refused. Budget-limited empty sweeps are inconclusive.
- Replanning closes when now + L_calc + L_entry ≥ t_R.
- After t_R the object-relative tracking law runs. Acquisition freezes the grasp.

## 3. Provenance of every rule

| Rule | Provenance |
|---|---|
| rendezvous at the pregrasp on the predicted trajectory; earliest feasible rendezvous; travel time ≤ arrival time with a planning-time shift; replanning patches with continuity; fine motion after the rendezvous | `LIT` Croft, Fenton, Benhabib TSMC 1998; Hujić et al. T-Mech 1998 |
| SDF ranking, then bounded exact IK; incumbent retention | `LIT` Akinola/Xu dynamic grasping (code, `5c1e01f`) |
| boundary-condition polynomial reference | `LIT` form (Hujić patches); boundary properties `DERIVED` and unit-tested |
| (σ, θ, s) family, fixed ψ, axial bound, N_θ = 53 | `DERIVED` (geometry; TRIAD Phase 4 resolution) |
| wrist (ρ, z) SDF | `DERIVED` (joint 1 about base z); necessary condition only; false negatives `EXP` |
| event spacing, rest collapse | `DERIVED` (TRIAD Phase 3) |
| pursuit bound | `DERIVED` from the rate cap and lead tube of the shared reach policy |
| ascending best-first sweep | numerical method; not a bank; guarantee stated in Phase C §1 |
| timing skip | `DERIVED` under the straight-line motion-time model; lossless on 2590 paired events (`EXP`) |
| authority solver (box, damper, BCLS, bisection) | `CODE` mc_rtc Tasks constraint mirror (04e9efc, unchanged) |
| path / sync / insertion demands | `DERIVED` from the commanded laws (`commandMouthTargetWithWorldVelocity`, rigid-object feed-forward, MovePregrasp far speed) |
| κ as a filter, not a decider; lexicographic order | user specification; tie-break order `NUM`/transparent |
| aimed-state retention | Audit §3.6.1 (Hujić) |
| replanning closure inside L_calc + L_entry | `DERIVED` (no result can arrive before t_R) |
| latency guard, inconclusive budget results | logic (the rollout start assumption; an incomplete search proves nothing) |

## 4. Parameters

| Class | Parameters |
|---|---|
| **Physical / model-derived** | handle R_H 11.25 mm, L_H 68.7 mm (`CODE` planner model); finger half-width 12.5 mm (Robotiq tip lattice); wrist offset 61.525 mm (URDF); URDF joint position and velocity limits; QP velocityPercent 0.95, damper [0.1, 0.01, 0.5] (controller YAML); derived: axial limit 35 mm, N_θ 53, Δτ, r_max, pursuit bound, tie band ε_p/|v_G| |
| **Literature-derived** | earliest rendezvous objective; travel ≤ arrival; planning-time shift; incumbent retention; SDF prefilter concept |
| **Inherited interface / empirical** (`CONFIG` / `EMP`) | ε_p 15 mm, ε_R 0.12 rad; reach tolerances 12 mm / 0.05 rad; terminal relative speed 0.04 m/s / 0.08 rad/s; far speeds 0.38 m/s / 1.65 rad/s; tracking leads; clearance floor 25 mm; transit minimum 20 mm; runtime reserve 8 mm; L_entry 0.05 s; armScale 1.25 (`EMP`); capture / standoff / retreat 6 / 120 / 180 mm; axial margin 3 mm |
| **Numerical, characterized** | SDF grid 10 mm and 4·10⁶ samples; prune tolerance 20 mm (deepest miss 12.9 mm); θ-bin 13.6°; K = 40 (recall and best-clearance retention); L_calc 0.80 s (p90 moving-job wall); budgets 450 exact / 40 rollouts (observed first-feasible needs); authority stride 1 (stride 5 missed peaks); minimum step 20 ms |
| **Numerical, not derived** | horizon 8 s; κ_max 8; residual tolerance 1e-4 |
| **Experimental / tunable** | κ_min = 1 (definitional, but its use on path demands is the H2 question); switch dwell 0.3 s; tie bands (clearance 5 mm, κ 0.1, saturation 2, capability 0.01) |

## 5. Changed files (04e9efc → final)

- `src/ReceivingGraspFamily.h` (new): family, frames, mechanical screen, surrogate query, shortlist.
- `src/Gen3WristReachabilityMap.h` (new, generated).
- `src/PredictiveInterception.h` (new): pursuit bound, event schedule, Hermite reference, demand twists, earliest-encounter selection.
- `src/HandoverInterceptionController.h`: parameters, job/request/result structs, execution state.
- `src/ReceiverV2.cpp`:
  - shared exact-layer helper, extracted without behaviour change;
  - funnel;
  - interception sweep and rollout;
  - authority demands;
  - predictive selection and execution;
  - logging.
- `etc/HandoverInterceptionController.in.yaml`: `graspFamily`, `receivingFamily`, `reachabilityFunnel`, `interception`, `variant` (defaults keep 04e9efc behaviour).
- `tools/test_receiving_grasp_family.cpp`, `tools/test_predictive_interception.cpp`, `tools/run_control_aware_supervisor_unit_tests.sh`.
- `supervisory_mode/tools/`:
  - `build_gen3_wrist_reachability.py`, `characterize_reachability_surrogate.py`, `funnel_characterization.py`;
  - `interception_characterization.py`, `interception_latency.py`, `authority_characterization.py`;
  - `phaseF_outcomes.py`, `demand_vs_execution.py`.

## 6. Tests, build, checkers

**Unit tests** (`tools/run_control_aware_supervisor_unit_tests.sh`, `-Wall -Wextra -Werror -pedantic`): all pass.

| Requested validation | Where | Result |
|---|---|---|
| cylinder / Robotiq frame construction | `testFrameAndContacts` | pass |
| task constraints reduce the manifold | `testTaskConstraintReduction` | pass |
| sampling-resolution convergence | `thetaSamplesForResolution` test; Phase B sub-family analysis | the median converges; **the worst case is not demonstrated** |
| reachability prefilter false negatives vs exact IK | Phase B `characterizeAll` | 0 pruned grasps feasible (1020 / 737 / 363 pruned at 0 / −20 / −50 mm) |
| seeded IK consistency | — | **not implemented.** IK always starts from the frozen state (purity rule). Near-goal re-solves disagree with the executing plan (Phase E), handled by the replanning closure. |
| interception on analytic moving targets | `testEarliestAndImpossibleInterception` | pass |
| earliest encounter | same | pass (within one step, meeting pose within ε_p) |
| impossible interception | same | pass (∞) |
| changed motion → replanning | same (unit); `plan_patched` events in situ | pass; 6 patches in Phase F |
| no chatter | `testSelectionAndHysteresis` (0 switches in 30 alternating updates) | pass in the unit test. **In situ FULL shows adopt/abort cycling (§11).** |
| authority identical for the same y | `testAuthorityDemands` (bit-identical κ and residual) | pass |
| demand matches the downstream reference | `demand_vs_execution.py` | executed / rollout peak ratio median 0.84 (n = 38; min 0.21) |
| bank_search unaffected when disabled | code path unchanged (helper extraction only); smoke runs | bank_search 2/4 completed (near-ground, lateral-low) |

**Build and checkers:** full controller build clean; `check_worker_snapshot_purity` PASS (106 functions); `check_planner_core_purity` PASS; markdown links PASS; documentation claims PASS.

## 7. Timing per stage (`EXP`, this machine)

| Stage | Median | p90 | Max |
|---|---|---|---|
| front end (G0 → G_K, 530 hypotheses × events) | 0.4–0.5 ms | ≤ 1.6 ms | 1.7 ms |
| exact layers per (g, τ) | 1.0–1.4 ms | 3.3 ms | |
| rollout per (g, τ), stride 5 / stride 1 | 6.0 / 12.7 ms | 7.4 / 15.8 ms | |
| sweep, moving-object job (held arm) | 529 ms | 752 ms | 3574 ms |
| sweep, at-rest job | 75 ms | 141 ms | 181 ms |
| selection job wall in situ: B0 | 0.064 s | 0.113 s | 0.249 s |
| selection job wall in situ: B1 | 0.139 s | 0.609 s | 1.069 s |
| selection job wall in situ: B2 | 0.136 s | 0.515 s | 1.105 s |
| selection job wall in situ: FULL | 0.052 s | 0.167 s | 1.278 s |

## 8. Candidate-count reduction per layer (median per job, held arm)

G0 530 → G_mech 420–530 → G_R 420–530 → G_K 40 → 1–5 pass the exact layers at some τ → 𝓕_I 1–5 within the tie band → 𝓕_C: about 35–43 % of 𝓕_I encounters have κ < 1.

## 9. One full trace

File: `evidence/phaseD_sim/full_trace_filter_lateral_low_gen1.txt` (not included in this repository) (FULL filter, held arm, lateral-low, object at 0.08 m/s).

- **G0 → G_mech → G_R → G_K:** 530 → 420 → 420 → 40.
- **Sweep:** 39 events at Δτ = 0.1875 s; 344 exact evaluations, 19 rollouts; 442 ms.
- **Timing failures** at τ = 1.975–2.163 s (required 2.90–3.20 s).
- **𝓕_I:** 7 grasps at τ = 3.475 s.
- **Authority filter:** grasps 332 (κ_path = 0.935) and 386 (κ_path = 0.989) rejected.
- **𝓕_C:** grasps {200, 202, 249, 251, 254} at τ* = 3.475 s.
- **Selection:**
  - B1 (clearance) → 200 (78.2 mm);
  - B2 (condition index) → 200;
  - FULL (κ) → 249 (κ = 1.467, clearance 76.9 mm).

## 10. B0 / B1 / B2 / FULL switches

```yaml
configs:
  HandoverInterceptionController_ReceiverV2:
    supervisorMode: control_aware
    controlAware:
      graspFamily: receiving
      variant: reactive | predictive | predictive_capability | full
```

Overrides: `evidence/phaseF_sim/overrides/`. The default `supervisorMode: bank_search` and `variant: reactive` with `graspFamily: legacy_ring` keep the receding-mode behaviour.

## 11. Phase F in situ (`evidence/phaseF_sim`, `EXP`)

**Setup.**
- 4 scenarios × 3 repeats. B0 and the bank_search regression come from `phaseF_sim_prefix`; that code path is unaffected by the fix.
- **The simulation is near-deterministic.** Repeats differ only by wall-clock jitter, so each cell is effectively one scenario outcome, not three samples.
- No statistical claim is made.

**Rollout fix before these results.** The first campaign (`phaseF_sim_prefix`) had a rollout defect on replans from a moving arm: the command chain started at the shifted reference instead of the integrated arm. It produced spurious path-demand peaks of 4.4–4.9 m/s and gave FULL 12/12, B1 9/12, B2 9/12. **After the fix those numbers changed materially.** The pre-fix results are kept as evidence of that sensitivity and are not used below.

**Completions:**

| Scenario | B0 reactive | B1 predictive | B2 capability | FULL |
|---|---|---|---|---|
| longitudinal | 0/3 | 3/3 | 3/3 | 2/3 |
| near-ground | 2/3 | 2/3 | 1/3 | 1/3 |
| lateral-low | 3/3 | 3/3 | 3/3 | 0/3 |
| diagonal | 0/3 | 3/3 | 3/3 | 3/3 |
| **total** | **5/12** | **11/12** | **10/12** | **6/12** |

**Meeting and timing** (`outcomes.md`):
- **B1/B2/FULL:** in longitudinal and diagonal the velocity-matched rendezvous happened **1.62 s before the giver stopped**, at 1.5–1.9 mm / 0.023 rad; in lateral-low about 0.6 s before, at 2.6–3.1 mm.
- **B0** first entered the terminal tube 2.63 s before the stop in lateral-low (11.3 mm), and never in longitudinal or diagonal. There it failed on `control_aware_track_clearance_reserve` / no certified plan.
- **Freeze after giver stop:** diagonal B1 / FULL 0.08 s, B2 1.37 s. Longitudinal B1 4.16 s, B2 3.87 s, FULL about 6.0 s (aborts before the successful plan). Lateral-low B0 0.08 s, B1/B2 0.83 s.

**Aborts:** most predictive-variant aborts were TERMINAL_CERTIFY failures (mouth corridor axial offset, pad pair / acquisition tube). All 29 terminal-certification aborts after the fix were on s ≠ 0 grasps (`axial_offset_aborts.txt`; 264 s ≠ 0 vs 21 s = 0 adoptions, pooled over the predictive variants and inflated by FULL cycling). Before the fix: 28 of 29 (83 vs 23 adoptions; `phaseF_sim_prefix/axial_offset_aborts.txt`). Terminal-certification aborts coincide with s ≠ 0; whether that is causal, or reflects the family's s ≠ 0 majority (4 of the 5 axial values), is not separated here.

**FULL failure mechanism.** FULL shows adopt → abort limit cycles (`full_abort_cycles.txt`): longitudinal 46–50 initial selections / 44–49 aborts per run; lateral-low about 15 / 14.
- From the held arm an 𝓕_C encounter exists, so FULL adopts.
- About 0.1 s later the re-solve from the now-moving arm finds none, so it aborts to hold.
- The authority predicate is **start-state dependent**. Its moving-start rollout mixes the snapshot configuration with the shifted reference.
- Abort on infeasibility has no dwell.

**Demand vs execution.** The executed commanded body-speed peak / rollout demand peak ratio is 0.84 median (0.21–1.01). The low ratios are replans from a moving arm, the same approximation.

## 12. Unsupported / approximate

1. **No moving acquisition.** Acquisition requires rest (interface limit). H1 is observed only on approach and meeting.
2. **Constant-twist prediction.** The object always stops in these scenarios. Encounters planned 2–3.5 s ahead rely on predictions beyond their measured validity (Phase 2: 0.10–0.65 s at 15 mm). Aimed-state patching and the terminal certification catch the consequences.
3. **Rollout start while the arm moves.** The snapshot configuration and the latency-shifted reference are mixed. This demonstrably affects κ (FULL cycling) and the demand match.
4. **Timing model.** armScale × preview convergence time (`EMP`) barely shrinks with progress, so a fresh τ recedes. Aimed-state retention avoids the recession but does not fix the model.
5. **IK seeding.** IK is not seeded from the previous solution. Near-goal re-solves can reject the executing grasp; this is handled by closing replanning, not by consistency.
6. **Latency.** L_calc is a machine-specific p90, so about 10 % of moving jobs are refused.
7. **Sampling convergence.** θ and s worst-case convergence is not demonstrated; K is chosen in-sample.
8. **Axial offset.** The s ≠ 0 grasps pass the selection-time layers but fail TERMINAL_CERTIFY more often. The consistency of the axial bound with the certification corridor after settling is open.
9. **Evidence scope.** Four near-deterministic scenarios, one machine, kinematic simulation. No hardware.

## 13. Closing separation

**Established robotics:**
- earliest-feasible rendezvous interception with travel-time ≤ arrival-time and receding patches (Croft / Hujić);
- reachability-map ranking before exact IK (Akinola / Xu);
- antipodal parallel-jaw grasp parametrisation on a cylinder;
- QP joint-velocity bounds with dampers (mc_rtc / Tasks);
- manipulability / condition-index grasp preference.

**Project-specific implementation (this repository):**
- the instantiation of those pieces with the controller's own exact layers and reach law;
- the event-ordered best-first sweep with necessary screens and a timing skip;
- phase-specific authority demands computed from the commanded laws;
- the receding execution rules (aimed-state retention, latency guard, inconclusive budgets, replanning closure);
- the B0/B1/B2/FULL switches and characterization tooling.

**Unproven hypotheses:**
- **H1 (predictive interception differs from reactive tracking):** *different behaviour observed, benefit not established.*
  - In these four near-deterministic scenarios the predictive variants met the grasp pose before the giver stopped (1.5–3.1 mm, up to 1.6 s early) and completed 11/12 (B1) vs 5/12 (B0).
  - The B0 failures are clearance-reserve and certification failures of the reactive tracker, not a demonstrated lack of prediction.
  - Acquisition still waits for rest, and the scenarios are not independent samples, so no general claim follows.
- **H2 (the constrained reserve κ adds information beyond reachability / motion-aware selection):**
  - *Information:* yes. κ is limited by the commanded path demand the preview model does not bound (35–43 % of 𝓕_I encounters have κ < 1), and it is anti-correlated with the condition index (Spearman median −0.83).
  - *Benefit:* **not supported.** Used as a hard filter (FULL) it made selection start-state dependent and produced adopt/abort cycles: 6/12 vs 11/12 for B1. The pre-fix 12/12 was an artefact.
  - Any future test of H2 needs the moving-start rollout approximation removed and hysteresis on authority infeasibility, both designed before running.
