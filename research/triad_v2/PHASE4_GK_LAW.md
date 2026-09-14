# FINAL TRIAD — Phase 4: the grasp candidate law (I_k, τ) → G_k(τ)

**Inputs.**
- The CALL handle and Robotiq 2F-85 capture geometry (CODE).
- Phase 1 principles C1–C5 and D.
- The 64-angle characterization of runs_ae1e824 (`char_G`): 64 approach angles per handle-axis sign at 5.625°, 14 leads, every grasp statically screened and every statically feasible grasp route-certified, moving and rest epochs, 4 scenarios. Coarser angle sets are exact subsamples.

**Tool and evidence.**
- Tool: `tools/gk_law_study.py`, with the selector replica from `replay_search_policies.py`.
- Evidence: `research/triad_v2/evidence/phase4_gk/` (`gk.json`, `GK_TABLES.md`).

**Scope.** As in Phase 3, the analysis is offline. The rest epoch uses the Phase 3 rest law (one pose, τ per record); the moving epoch uses zero-latency selection over the logged leads.

## 1. Grasp parameterization (CODE: `buildCandidate`)

The handle is a cylinder of radius 11.25 mm with axis a_H.
- A candidate is (θ, s): approach angle θ about the handle axis, measured from the robot-relative outward direction, and axis sign s ∈ {+,−}.
- The mouth frame has z_M = s·a_H and y_M = Rot(a_H, θ)·outward.
- Capture point: p_H + y_M·captureDepth (0.006 m).
- Standoff: capture + y_M·0.120 m. Retreat: capture + y_M·0.180 m.

Rotation about the handle axis leaves the grasp itself unchanged (a cylinder under a parallel gripper). Feasibility in θ is decided entirely by external constraints: arm reachability, collision with the rest of the CALL object and the ground, corridor, closure and retreat. Those are what the certification stages test.

## 2. Derived resolution floor (DERIVED)

At the standoff, the first pose the robot must reach, neighbouring samples differ by R·Δθ in position and Δθ in orientation, with R = captureDepth + standoff = 0.126 m.

Apply the same pose-change criterion as T_k (Phase 3 §2.3), with the capture/freshness tube (ε_p, ε_R) = (15 mm, 0.12 rad). Samples closer than this are operationally indistinguishable at execution:

**Δθ_min = min(ε_p / R, ε_R) = min(0.119, 0.120) rad = 6.8°**, i.e. at most ⌈360/6.8⌉ = 53 useful samples per sign.

On the nested power-of-two grid used by the controller, that is 64 per sign. Sampling finer than Δθ_min cannot be justified from the execution tolerance.

## 3. Measured feasible arcs (MEASURED)

Contiguous feasible runs on the 64 grid, per lead and axis sign:

| scenario / epoch | statically feasible arcs: count, min, median, singletons | complete-action arcs: count, min, median, singletons |
|---|---|---|
| diagonal / moving | 22, 1, 3, 3 | 19, 1, 2, 6 |
| diagonal / rest | 1, 4, 4, 0 | 1, 4, 4, 0 |
| lateral-low / moving | 31, 1, 4, 5 | 31, 1, 4, 5 |
| lateral-low / rest | 3, 1, 3, 1 | 3, 1, 3, 1 |
| longitudinal / moving | 31, 1, 5, 5 | 26, 1, 2.5, 8 |
| longitudinal / rest | 3, 7, 8, 0 | 3, 1, 7, 1 |
| near-ground / moving | 21, 1, 5, 1 | 17, 2, 4, 0 |
| near-ground / rest | 1, 5, 5, 0 | 1, 4, 4, 0 |

- The median arc is 3–5 samples (17–28°).
- **Single-sample arcs (< 5.6°) exist in 6 of 8 epochs.** They are narrower than the execution tolerance Δθ_min. So a feasible arc of width < Δθ_min is feasible only on the exact sampled pose, and a runtime deviation still inside the tolerance could make it infeasible.
- (INFERENCE; to decide in Phase 8, not here) Candidates from arcs narrower than Δθ_min are fragile. A robustness requirement "arc width ≥ Δθ_min" would be the physically consistent certification rule, but it is not adopted without evidence.

## 4. Convergence and alternative laws

Reference: 64 angles per sign, zero-latency selection (rest: per-record τ). Full table in `GK_TABLES.md`.

| law (per sign) | static screens (fraction of 64) | route rollouts (fraction) | ΔJ vs 64 (8 epochs) | earliest admissible τ loss |
|---|---|---|---|---|
| UNIFORM(8) | 0.12 | 0.00–0.20 | 0 to +0.124; misses the only family in diagonal/rest | +0.45…+1.35 s in 3/8 |
| **UNIFORM(16)** (current) | 0.25 | 0.20–0.33 | 0 to +0.0055 | **+0.90 s in lateral-low/moving** |
| **UNIFORM(32)** | 0.50 | 0.40–0.58 | **0 to +0.0055** | **0 in 8/8** |
| UNIFORM(64) | 1.00 | 1.00 | 0 | 0 |
| ADAPT(16): coarse 16, refine around static passes to 64 | 0.29–0.44 | 0.87–1.00 | 0 in 8/8 | +0.45 s in lateral-low/moving |
| ARCREP(1): 64-angle static screen, route-certify one sample per arc | 1.00 | 0.12–0.25 | +0.0004 to +0.052 | 0 |
| ARCREP(3): arc ends + centre | 1.00 | 0.38–0.75 | 0 to +0.052 (near-ground/moving) | 0 |

**Findings.**
- **UNIFORM(32) is the coarsest uniform law that loses no earliest admissible τ in 8/8** (MEASURED). Its ΔJ ≤ 0.0055 comes from a single case (longitudinal moving: an earlier τ with slightly higher J).
- UNIFORM(16) is not converged: it loses 0.90 s in lateral-low/moving (also reported in `V2_COMPUTATION` §5.2).
- Adaptive refinement saves static screens, but these are cheap (≈ 1 ms). Route certification dominates cost (≈ 5 ms per rollout, one per route × static pass), and refinement concentrates on exactly the grasps that need routes (87–100% of the route rollouts). It still loses τ in 1/8. **Rejected.**
- Arc representatives cut route rollouts by 25–88% but lose up to ΔJ 0.052 (near-ground moving). The best grasp inside an arc is not predictable from the arc alone. **Rejected** as a lossless law; kept as an optional anytime ordering.
- The axis-sign dominance per scenario is strong but not exclusive: singletons and some complete records of the other sign exist. **No sign conditioning.**

## 5. The law

```
Input: predicted presentation pose p̂(τ) (from T_k), robot state, geometry (R, handle axis).

Δθ_min = min(ε_p / R, ε_R)                         # resolution floor, 6.8° here
N_θ    = 32 per axis sign (11.25°)                  # coarsest converged uniform grid (§4); 64 if §6 fallback triggers
G_raw(τ) = { (θ_i = 2π i / N_θ, s) : i < N_θ, s ∈ {+,−} }
G_k(τ)   = { g ∈ G_raw(τ) : exact static screen at p̂(τ) passes reach standoff, reach capture,
                             closure sweep and carried retreat }     # variable count N_G(k,τ)
Route certification (Phase 5) is applied only to g ∈ G_k(τ).
```

**State conditioning.**
- The set is regenerated around the predicted pose at every τ in T_k.
- Its size N_G(k,τ) is the number of grasps passing the exact static screen. It varies by epoch: in the data, statically feasible arcs total 1–31 per lead/sign group.
- The static screen is the reachability conditioning of Phase 1 principle D, and it is exact rather than learned.

## 6. Parameter provenance

| symbol | value | classification | evidence |
|---|---|---|---|
| grasp parameterization (θ about handle axis, sign) | — | DERIVED from handle symmetry and parallel-gripper symmetry (CODE) | §1 |
| R = captureDepth + standoffDistance | 0.126 m | inherited geometry constants (captureDepth 0.006, standoff 0.120) | Phase 9 derives the standoff from the gripper and handle |
| Δθ_min | 6.8° | DERIVED (pose-change floor), dependent on ε_p and ε_R (Phase 9) | §2 |
| N_θ = 32 per sign | 11.25° | EMPIRICALLY VALIDATED: coarsest uniform grid with no earliest-τ loss in 8/8 and ΔJ ≤ 0.0055 vs 64 (which satisfies Δθ_min) | §4 |
| N_θ = 16 per sign (current) | 22.5° | **REPLACED** (τ loss 0.90 s) | §4 |
| static screen as reachability filter | — | DERIVED (exact; A1: F1 USEFUL, F3 USEFUL) | A1 |

**Why 32 and not 64.**
- 64 meets the derived floor; 32 does not (11.25° > 6.8°).
- The data show 32 changes neither the earliest admissible τ nor the J minimum beyond 0.0055 in the eight epochs available.
- 64 doubles static and route work per τ, which feeds the latency allowance A of the T_k law (Phase 3 §2.5).
- This is a stated trade-off: 32 is EMPIRICALLY VALIDATED on four scenarios, not derived. Phase 11 must include out-of-sample poses. If a feasible arc narrower than 11.25° decides an outcome there, the law reverts to 64.

## 7. Limits

- One object, one gripper, one robot start, four scenarios, one speed. Offline selection with zero latency for the moving epoch.
- Arc fragility (§3) is identified but not resolved. It belongs with the certification robustness questions of Phase 8.
