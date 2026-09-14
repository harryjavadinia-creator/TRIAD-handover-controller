# FINAL TRIAD — Phase 3: the temporal candidate law I_k → T_k

**Inputs.**
- Phase 1 principles A1–A4 and B.
- Phase 2 measurements: prediction validity, preview/runtime parity, latency sensitivity.
- The dense temporal characterization of runs_ae1e824 (`char_T`): 191 leads on [0.50, 10.00] s at 0.05 s, with every grasp and route certified at the moving and at the rest epoch of all four scenarios. Every coarser lead set is an exact subsample of the same search (checked in `V2_COMPUTATION_AND_CANDIDATE_SPACE.md` §5).

**Tool and evidence.**
- Tool: `tools/tk_law_study.py`. It uses the selector replica of `tools/replay_search_policies.py`: timing admission, J, tie-break.
- Evidence: `research/triad_v2/evidence/phase3_tk/` (`tk.json`, `TK_SUMMARY.md`).

**Evidence tags.**
- CODE
- DERIVED: follows from code semantics and stated assumptions.
- MEASURED
- OFFLINE: replay over logged certifications, with a modelled worker wall.
- INFERENCE

**Scope.** This phase derives and evaluates the law offline. It does not yet change the controller.
- In-situ evidence is confounded by the recertification inconsistency found in Phase 2, whose repair the author deferred. No in-situ claim is made here.
- Selection uses the current J. Phase 6 may replace it. Where the law depends on J, that is stated.

---

## 1. What T_k must satisfy (from Phases 1–2)

| requirement | source |
|---|---|
| A candidate interception time τ is useful only if the complete action is certifiable **and** timing-admissible at the time the decision is made: τ − t_dec ≥ L_commit and T_pres(g,r) + L_entry ≤ τ − t_dec. | CODE (`selectFiniteEventPlan`), Phase 1 A1 |
| t_dec = t_k + computation time. While the object moves, admission and computation interact. | MEASURED (Track 1, Phase 2 §0.3) |
| The prediction at τ is commitment-grade only for τ − t_k ≤ h_valid(ε). The worst case is 0.10–0.65 s in the validated envelope; the nominal case is 0.35 s for ε = 15 mm. | REPLICA (Phase 2 §1.3) |
| At rest the prediction is exact at every horizon (error ≤ 0.9 mm). | MEASURED (Phase 2 §1.4) |
| Consecutive hypotheses should differ by no more than a capture-relevant pose change. | Phase 1 B; `V2_COMPUTATION` §5.1 |

## 2. Derivations

### 2.1 Rest regime: the time dimension collapses exactly (DERIVED + MEASURED)

When |v̂| ≤ v_rest and |ω̂| ≤ ω_rest, the V2 prediction record carries zero twist (CODE `currentObjectPredictionV2`). Every τ therefore has the same presentation pose.

**Consequences.**
1. Every copied-state certification is independent of τ: the rollouts use times relative to the presentation (CODE, the memoization comment in `HandoverInterceptionController.h`). Hence T_pres(g,r), J_motion(g,r), clearances and feasibility do not depend on τ.
   - MEASURED: the records are identical across all 191 leads in 4/4 rest searches (`tk.json`, `recordsIdenticalAcrossLeads`).
2. J_global = J_motion + w_T·(τ − t − T_pres)/T_ref with w_T > 0, so J_global is strictly increasing in τ for every (g,r).
   - MEASURED: 0 pairs violate this in 4/4.
3. So for each certified (g,r), the best admissible τ is the earliest admissible one: **τ*(g,r) = t_dec + max(L_commit, T_pres(g,r) + L_entry).**

The argmin over (g,r) of J_global at τ*(g,r) is the argmin over the continuum of τ. Any finite lead bank can only select the same (g,r) at a later or equal τ, or a different (g,r) with a J at least as high.

**Rest law.** Certify a **single** presentation pose; assign τ per certified record; select.
- This is lossless with respect to any lead set, including continuous τ, and it is minimal in computation (one hypothesis).
- It replaces 14 memoized hypotheses and the rounding of τ up to the next bank lead.

OFFLINE, at a decision time equal to epoch + modelled wall:

| scenario | policy | wall (s) | τ − t_epoch (s) | ΔJ vs zero-latency 191-lead argmin |
|---|---|---:|---:|---:|
| diagonal | BANK14 | 0.19 | 2.800 | +0.0211 |
| diagonal | 191 leads | 0.19 | 2.600 | +0.0105 |
| diagonal | **REST law** | 0.16 | **2.546** | **+0.0077** |
| lateral-low | BANK14 | 0.56 | 2.800 | +0.0289 |
| lateral-low | 191 leads | 0.56 | 2.800 | +0.0289 |
| lateral-low | **REST law** | 0.50 | **2.727** | **+0.0251** |
| longitudinal | BANK14 | 0.66 | 3.250 | +0.0491 |
| longitudinal | 191 leads | 0.66 | 2.850 | +0.0368 |
| longitudinal | **REST law** | 0.59 | **2.735** | **+0.0308** |
| near-ground | BANK14 | 0.20 | 3.700 | +0.0185 |
| near-ground | 191 leads | 0.20 | 3.350 | +0.0105 |
| near-ground | **REST law** | 0.18 | **3.314** | **+0.0086** |

In all four scenarios the REST law selects the same (g,r) as the 191-lead grid. It gives the earliest τ, and its only residual versus the unachievable zero-latency reference is its own computation time.

### 2.2 Moving regime: commitment-grade candidates do not exist (DERIVED from Phase 2)

- **Lower bound.** Admission requires τ − t_dec ≥ L_commit = 1.6 s. It also requires T_pres + L_entry ≤ τ − t_dec, where the measured static reach time T_s (a lower bound on T_pres, Track 1 premise, CODE) makes the smallest admissible lead **2.1–3.9 s** in the four scenarios (`V2_COMPUTATION` §5.1).
- **Upper bound.** For commitment-grade prediction error ε = 15 mm, the worst-case h_valid is ≤ 0.65 s for every tested stop duration at 0.08 m/s and ≤ 0.55 s at 0.04 m/s.

Since h_lo > h_valid throughout the validated envelope, **T_k^commit(moving) = ∅**. The current controller already refuses commitment while the object moves (`objectStopped` in the terminal gate, CODE), so this is consistent.

Any τ generated while the object moves is therefore a **provisional hypothesis** under the explicit motion-class assumption "no acceleration onset within τ − t_k". The frozen architecture allows provisional motion. Whether provisional adoption while moving helps completion is a supervisory question for Phase 7: Phase 2 measured 17/41 vs 7/8. T_k only has to generate the provisional set well.

### 2.3 Temporal spacing from pose change (DERIVED)

Adjacent hypotheses along the constant-twist prediction differ by |v̂|·Δτ in position and |ω̂|·Δτ in angle. To keep consecutive hypotheses within the capture/freshness tube (ε_p, ε_R):

**Δτ = min(ε_p/|v̂|, ε_R/|ω̂|).**

At 0.08 m/s and ε_p = 15 mm this gives 0.19 s. It agrees with the measured convergence at Δτ ≤ 0.2 s (`V2_COMPUTATION` §5.1: Δτ ≤ 0.2 s recovers the earliest admissible τ in 4/4 with ΔJ ≤ 0.0075; 0.45 s loses 0.15–0.35 s).

ε_p and ε_R are the commit-freshness tube (15 mm / 0.12 rad, inherited). They are derived or validated in Phase 9, and T_k inherits that classification.

### 2.4 Time-consistency of a moving search (DERIVED + OFFLINE)

Let c be the computation per certified lead. While the ladder walks up by Δτ per lead, the admission boundary moves by c. An ascending search that starts below the admissible region catches up only if c < Δτ. Otherwise the decision time outruns the leads it evaluates.

**Measured.** With the exact Track 1 prune, c at leads that may still be admissible is **0.18–0.59 s per lead** (modelled from the job profiles). That exceeds Δτ_phys = 0.19 s at 0.08 m/s.

**OFFLINE consequences** (`TK_SUMMARY.md`):
- Every no-stopping grid (Δτ = 0.05–0.20 s) and every unanchored ascending ladder at Δτ ≤ 0.10 s ends with **no admissible plan** in 3–4 of 4 scenarios.
- The full 191-lead grid needs 44–78 s, or 7.9–8.4 s with the prune, and is never admissible at its decision time.
- BANK14 (3.1–5.0 s) is admissible in 3/4 scenarios, at leads 5.5–8.0 s with ΔJ +0.10…+0.29.

Hence the moving ladder must be **anchored**: generate τ_j at the time the decision will be made, not at t_k. Choose the first lead so that it is admissible when its certification finishes.

### 2.5 Anchored provisional ladder (DERIVED structure, EMPIRICAL constants)

For the current state I_k (object moving):

1. **Latency allowance A.** An upper bound on the search's own computation. It is consistent iff the modelled or measured wall ≤ A. Otherwise the law declares the set stale and re-anchors with A := measured wall.
2. **Lower lead.** h_lo = max(L_commit, min_g T_s(g, p̂(t_k + A + L_commit)) + L_entry). It comes from one static-screen probe of all grasps (≈ 32 static screens). It is exact because route T_pres ≥ T_s (CODE, Track 1 proof).
3. **Ladder.** τ_j = t_k + A + h_lo + j·Δτ, with Δτ from §2.3.
4. **Stop** at the first τ_j that has a certified record admissible at the current decision time (window W = 0).
   - Rationale: J is increasing in τ at fixed (g,r) (§2.1), and the worst-case prediction error is non-decreasing in the horizon (Phase 2 §1.3). Both favour the earliest admissible τ.
   - Measured: W = 0.5 s never lowered J and made the set stale in 2/4 scenarios at A = 1.0.
5. Apply the exact timing prune to every (τ_j, g).
6. **Upper cap:** the workspace-exit horizon (no grasp passes the static screen). MEASURED not binding up to 10 s in 3/4 scenarios, so it is a safety cap, not an active constraint.

**OFFLINE result at A = 1.0 s** (Δτ = 0.15 s on the 0.05 s grid, W = 0):

| scenario | leads certified | modelled wall (s) | consistent (wall ≤ A) | selected lead / grasp / route | ΔJ | Δτ vs zero-latency reference (s) | reach / retreat clearance (mm) |
|---|---:|---:|---|---|---:|---:|---|
| diagonal | 1 | 0.21 | yes | 3.95 / axisP_side_337deg / ring80mm_2of8 | +0.0038 | +0.05 | 74.5 / 215.9 |
| lateral-low | 1 | 0.39 | yes | 3.90 / axisN_side_337deg / direct | +0.0132 | +0.45 | 81.6 / 79.0 |
| longitudinal | 1 | 0.59 | yes | 3.80 / axisP_side_337deg / ring140mm_1of8 | +0.0547 | +0.95 | 80.9 / 219.8 |
| near-ground | 1 | 0.24 | yes | 3.80 / axisP_side_45deg / ring140mm_7of8 | +0.0533 | +0.60 | 67.1 / 100.5 |
| *BANK14, same data* | 14 | 3.08–5.01 | — | admissible in 3/4; leads 5.5–8.0 | +0.10…+0.29 | +1.6…+4.8 | |

**Sensitivity to A:**
- A = 0.5 s is inconsistent (wall > A) in 2/4 scenarios: diagonal, and near-ground, which has no admissible plan after 6.5 s of chasing. It is consistent but poor in lateral-low, where it picks a 46 mm-clearance record with ΔJ +0.155.
- A = 1.5–2.0 s is consistent in 4/4 but later: Δτ +0.10…+1.85 s, ΔJ up to +0.18.
- The selected grasp is the same axis/side family as the reference in 4/4 at A = 1.0.
- Δτ ∈ {0.15, 0.45} s gives identical results at A = 1.0, because the first anchored lead is already admissible. Δτ matters only when the first lead fails.

(INFERENCE) The anchored ladder turns the moving search from 3–5 s (bank) or 45–78 s (dense) into 0.2–0.6 s. Its τ is within 0.05–0.95 s of the unachievable zero-latency optimum. That makes provisional plans available while the object moves. Phase 2 showed such plans have large realised error, so this is a capability whose value Phase 7 must decide. It is not evidence of better completion.

## 3. The law

```
Input I_k: t_k, object pose and twist estimate (p, v̂, R, ω̂), robot state, last search wall ĉ.

if |v̂| ≤ v_rest and |ω̂| ≤ ω_rest:                       # rest regime
    T_k  = { t_k + L_commit }                             # one pose (all τ identical)
    after certification: τ(g,r) = t_dec + max(L_commit, T_pres(g,r) + L_entry)
    commitment-grade: yes
else:                                                    # moving regime
    A     = max(A_min, ĉ)                                 # latency allowance
    h_lo  = max(L_commit, min_g T_s(g, p̂(t_k + A + L_commit)) + L_entry)
    Δτ    = min(ε_p / |v̂|, ε_R / |ω̂|)
    T_k   = { t_k + A + h_lo + j·Δτ : j = 0, 1, ... } in ascending order, exact timing prune,
            stop at the first τ_j with an admissible certified record at the current decision time,
            or when the static screen reports workspace exit
    if wall > A: mark stale, set ĉ := wall, regenerate
    commitment-grade: no (h_lo > h_valid(ε_p) in the validated envelope); provisional only
```

N_T(k) is state dependent:
- 1 at rest;
- typically 1 while moving in the four scenarios (the first anchored lead is admissible);
- more when early leads are infeasible, bounded by the workspace-exit cap.

## 4. Parameter provenance

| symbol (config) | value | meaning | classification | evidence / note |
|---|---|---|---|---|
| v_rest, ω_rest (`presentationMaximumLinearSpeed/AngularSpeed`) | 0.004 m/s, 0.08 rad/s | regime switch | ENGINEERING FALLBACK (inherited) | Load-bearing for §2.1; Phase 9 must derive it from estimator noise and the capture tube. |
| L_commit (`minimumCommitRemainingTime`) | 1.6 s | admission lead | ENGINEERING FALLBACK (inherited) | Phase 2: insertion timing ±0.03 s, acquire and retreat conservative 0.5–1.8 s. Phase 9 derives it from the measured post-commit durations. |
| L_entry (`minimumReachEntryLead`) | 0.05 s | reach-entry reserve | ENGINEERING FALLBACK (inherited) | Phase 9. |
| T_s lower bound on T_pres | — | exact prune and h_lo | DERIVED (CODE; Track 1 proof, 0 premise violations) | |
| Δτ = min(ε_p/\|v̂\|, ε_R/\|ω̂\|) | 0.19 s at 0.08 m/s | temporal spacing | DERIVED form; EMPIRICALLY VALIDATED convergence (Δτ ≤ 0.2 s) | ε_p = 15 mm and ε_R = 0.12 rad inherited; Phase 9. |
| A_min | 1.0 s | minimum latency allowance | EMPIRICAL (≥ max measured anchored wall 0.59 s; A = 0.5 inconsistent in 2/4) | Load-bearing and machine dependent; must be re-measured on the final build. |
| W | 0 | window after first admissible lead | DERIVED under the current J (monotone in τ) + EMPIRICAL | Revisit if Phase 6 changes the ranking. |
| workspace-exit cap | static screen | upper horizon | DERIVED (exact) | not binding ≤ 10 s in 3/4 |
| h_valid(ε) | 0.10–0.65 s | commitment-grade horizon | EMPIRICALLY VALIDATED (REPLICA over the stated envelope) | Only for commit eligibility, not generation. |
| 14-lead bank, 0.45 s step, 8.0 s max | — | fixed schedule | **REMOVED** by this law | |

## 5. Limits and open items

- Offline over one giver speed (0.08 m/s), four start poses, one robot start. The wall is modelled from profile averages, not timed runs.
- The moving-regime law has not been run in situ. Two confounds apply there: the recertification defect (Phase 2 §2.4, deferred) and machine-speed sensitivity (Phase 2 §0.3).
- The rest law changes the selector's domain from bank leads to a per-record τ. Implementation must preserve the selector's admission test and tie-break exactly.
- Direction changes, noise and slow sensors are outside the envelope (Phase 2 §1.5). The law is stated for the validated envelope only.
