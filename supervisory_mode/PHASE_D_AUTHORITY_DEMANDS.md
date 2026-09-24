# Phase D: controller-authority demand from the actual downstream commands

Formulation: `TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md` §3.7.
The authority solver is unchanged: `qpJointVelocityBox`, `boxConstrainedLeastSquares`, `directionalReserve` (04e9efc).
Only the demand vectors y and the role of κ change.
Tags: `CODE` / `DERIVED` / `LIT` / `NUM` / `EXP`.

## 1. Demands (tool body `gen3_robotiq_85_base_link`, world frame, [ω; v])

| Phase | Configuration q | y_req | Source of the law |
|---|---|---|---|
| Interception path | every rollout configuration (stride `authorityStride`) | commanded step twist: finite difference of consecutive commanded body poses of the rate-limited, lead-bounded reference | `CODE`: the same twist `commandMouthTargetWithWorldVelocity` sends to the QP transform task; the rollout already computed it as feed-forward |
| Synchronization | rollout end (rendezvous) | `synchronizationTwist` = [ω_O; v_O + ω_O × (p_B − p_O)] at t+τ | `CODE`: rigid-object tracking feed-forward |
| Insertion | static standoff and capture configurations at the event pose | `insertionTwist` = [0; −v_far y_M] | `CODE`: MovePregrasp `farLinearSpeed`, object at rest (acquisition interface, Audit §6) |

**κ.** κ(g, τ) = min over the three phases of κ*(y, q). A configuration inside the QP security distance gives κ = 0.

**𝓕_C.** 𝓕_C = {(g, τ) ∈ 𝓕_I : κ(g, τ) ≥ κ_min}, with κ_min = 1 (definitional: the declared demand is realisable).

**Role of κ.**
- κ never decides interception existence.
- `authorityFilter: true` (FULL) makes τ*_g the earliest event in 𝓕_C; otherwise κ is logged and used only as the FULL tie-break inside Δτ.
- Selection has no weights: earliest τ first, then the tie-break inside Δτ.

**Replaced demand.** The 04e9efc sum y_follow + 0.38·y_insert mixed two phases that cannot co-occur, because insertion starts only once the object is at rest.

**Unit tests** (`testAuthorityDemands`):
- at rest the 04e9efc `followInsertTwist` equals the Phase D insertion twist, and gives bit-identical κ and residual through the unchanged solver;
- while moving, follow_insert = synchronization + insertion exactly;
- the commanded-step twist of a known motion is recovered.

The legacy demand now calls the same pure function, with identical arithmetic.

## 2. Characterization (`evidence/phaseD_sim` (not included in this repository), held arm, `EXP`)

**Setup.** 4 scenarios × {log only (stride 5), FULL filter (stride 5), log only (stride 1)}.

**Deduplication.** Statistics use every moving-prediction job plus only the first at-rest job of each run. Held-arm runs repeat identical at-rest jobs, and `SCOPE=all` shows how those repeats inflate the numbers. Output: `authority_characterization.txt`.

| | log, stride 5 | log, stride 1 | FULL filter, stride 5 |
|---|---|---|---|
| feasible encounters | 149 | 114 | 102 |
| κ median / p10 / min | 1.051 / 0.762 / 0.453 | 1.025 / 0.792 / **0.346** | 1.091 / 1.020 / 1.001 |
| encounters in 𝓕_I with κ < 1 | 34.9 % | **43.0 %** | 0 % (filtered) |
| limiting phase | path 137, insertion 12, sync 0 | path 108, insertion 6 | path 96, insertion 6 |
| sync reserve median / min | 8.0 (cap) / 3.2 | 7.8 / 3.2 | 7.9 / 3.2 |
| insertion reserve median / min | 1.65 / 0.89 | 1.61 / 0.89 | 1.58 / 1.01 |
| rollout cost per (g, τ) | 6.0 ms | 12.7 ms | 5.8 ms |
| moving-job sweep median / p90 | 511 / 643 ms | 567 / 1125 ms | 519 / 1206 ms |

**Findings.**
1. **The interception path limits authority, not synchronization.** The commanded body twist peaks mid-path at about 0.15–0.57 m/s. The preview IK tracks it because it clips joint velocities component-wise and has no damper, while the QP's 0.95 box ∩ damper cannot realise it in direction in 35–43 % of 𝓕_I encounters. This is the information κ adds beyond the rollout: the rollout model is more permissive than the QP velocity constraints. Whether that matters for the executed motion is tested in Phase E/F, not assumed.
2. **Stride.** Stride 5 misses demand peaks (min κ 0.45 vs 0.35; 35 % vs 43 % below κ_min, distributional). The default is **stride 1**, which doubles rollout cost (12.7 ms per (g, τ)).
3. **Filter effect** (FULL): 50 grasp encounters in 𝓕_I had no 𝓕_C encounter within the horizon, and 6 moved later. 2/28 jobs had 𝓕_I ≠ ∅ but 𝓕_C = ∅.
4. **H2 information check.** Spearman(κ, condition index at the rendezvous) per job has median −0.83 (p10 −1.00, p90 +0.79; 23 jobs with ≥ 3 feasible grasps). κ is **not** a proxy for the generic manipulability-type measure here. The FULL and B2 tie-break selections differ in 15/30 jobs, but FULL and B1 (clearance) differ in only 4/30, because both break ties only inside Δτ. Different selections are not better selections: this shows information, not benefit.

## 3. Unsupported / approximate

- **Interpretation of κ_min = 1 on the path.** The QP task is soft, so κ < 1 on a commanded step means lag inside the lead tube, not failure. Whether demanding κ ≥ 1 on the path improves anything is the H2 question for Phase F.
- **Configurations and timing.** Insertion κ is evaluated at the static standoff and capture configurations of the event pose, not at the rollout end. The synchronization demand uses the predicted twist at t+τ.
- **Evidence scope.** Stride comparisons and filter vs log comparisons come from different simulations and are distributional only.
