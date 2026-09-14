# FINAL TRIAD — Phase 5: the route candidate law (I_k, τ, g) → R_k(τ, g)

## Inputs

- **Phase 1** principles E1 (direct first, alternatives only on failure) and E2.
- **`runs_ae1e824/char_R`**: route bank of direct + 16 directions × {40, 80, 140, 200} mm (65 routes), 4 scenarios, moving and rest epochs.
- **New `runs_phase5_routes/char_stretch`** (this phase):
  - the current 17 routes (direct + 8 directions × {80, 140} mm);
  - plus **pure time-stretched direct routes** `directx105 … directx170`: the same straight path, with only the allotted reach duration multiplied by 1.05, 1.10, 1.20, 1.35, 1.50 and 1.70.
  - Enabled by `transitPlanning.directTimeStretch`, which is characterization-only with an empty default (commit 0d1247d).
  - V1 `FrozenPlanRecord` sets are identical on this build (4/4).

**Tool:** `tools/rk_law_study.py`. **Evidence:** `research/triad_v2/evidence/phase5_rk/` (`RK_RING65_TABLES.md`, `RK_STRETCH_TABLES.md`, JSON) and `runs_phase5_routes/`.

**Scope:** offline. The moving epoch uses zero-latency selection; the rest epoch uses the Phase 3 rest law. The in-situ confounds of Phase 2 still apply.

## 1. How a route is certified (CODE)

- **Route definition.** A route is a smooth curve from the planning-start mouth pose to the certified standoff. It bows by an apex offset in one of the directions around the chord (`transitRouteBank`, `reachCurvePose`). "Direct" means no offset.
- **Allotted reach duration** (`beginPredictiveRouteCandidate`): `T_pres = timingArmScale · T_s,standoff · max(1, pathStretch)`.
  - T_s,standoff comes from the static IK screen.
  - pathStretch = curve length / chord length, capped at 1.90.
- **Pass condition.** The governed rollout must track the timed reference under the speed limits (far/near 0.38 / 0.16 m/s) and lead bounds, end within 12 mm / 0.05 rad of the standoff, and keep the clearance reserves.
- **Consequence.** A curved route receives more time purely because it is longer. So a route can "succeed where direct fails" for two different reasons: geometry (it avoids something) or time (the straight path was given too little time).

## 2. Why direct fails, and what rescues it (MEASURED)

Moving epoch, counts of (τ, g) pairs that passed the static screen:

| scenario | direct succeeds | direct fails: reach_tracking | direct fails: clearance reserve | direct fails: other |
|---|---:|---:|---:|---:|
| longitudinal | 15 | 10 | 9 | 1 |
| lateral-low | 8 | **34** | 1 | 0 |
| near-ground | 19 | 2 | 2 | 1 |
| diagonal | 16 | 6 | 2 | 0 |

**After a direct `reach_tracking` failure** (success / attempts per alternative, `char_stretch`):

| scenario | time stretch ×1.05 | ×1.10 | ×1.20 | ×1.35 | ×1.50 | ×1.70 | ring 80 mm (8 dirs) | ring 140 mm (8 dirs) | rescued by stretch / by any ring |
|---|---|---|---|---|---|---|---|---|---|
| lateral-low | 1/34 | 23/34 | **28/34** | 26/34 | 26/34 | 25/34 | 89/272 | 156/272 | 28/34 / 33/34 |
| diagonal | 4/6 | 4/6 | 4/6 | 4/6 | 3/6 | 3/6 | 15/48 | 9/48 | 5/6 / 5/6 |
| near-ground | 0/2 | 0/2 | 0/2 | 1/2 | 1/2 | 1/2 | 0/16 | 0/16 | 1/2 / 0/2 |
| longitudinal | 0/10 | 0/10 | 0/10 | 0/10 | 0/10 | 1/10 | 0/80 | 3/80 | 1/10 / 2/10 |

**After a direct clearance-reserve failure** (14 pairs):
- Time stretch rescues only 3 (longitudinal runtime reserve 2/3; one longitudinal robust-transit reserve at ×1.5).
- Rings rescue 5.
- In 65-route `char_R` (longitudinal, after direct reach_tracking), 200 mm rings succeed in 26/160 attempts vs 5/160 at 140 mm; after clearance failures, 17/144 vs 17/144.

**Findings.**
- **In lateral-low, "route required" is predominantly a reach-duration artefact.** The same straight path succeeds for 28 of 34 pairs once given 20% more time. Spatial rings rescue 33/34, but they also carry 4–44% more time through their path stretch.
  - This answers the open question from `V2_COMPUTATION` §5.3 (MEASURED).
  - (INFERENCE) The static IK screen's reach time underestimates the time the speed-governed reach needs. That is a timing-model mismatch, not obstacle avoidance.
- **Direction matters when geometry is the problem.** At the same radius, and hence similar stretch, only some directions succeed: lateral-low 140 mm 8–13/16; diagonal 1–7/16 (`char_R`).
- **Longitudinal failures are mostly genuine.** Neither stretch nor 80/140 mm rings rescue 8/10 reach_tracking pairs. Only large-radius (200 mm) curved paths help.
- **When direct succeeds, rings still change J.** The unchanged J prefers a ring or a stretched direct in 7/8 epochs, by up to 0.065 (near-ground). At rest it can prefer a ×1.5 stretched direct that intercepts ≈ 1 s later: diagonal τ − t 2.94 s vs 1.97 s; longitudinal 3.20 s vs 2.15 s.
  - This is a property of the cost (slower motion scores better on effort/utilization terms than the time term penalizes). It is not a feasibility property.
  - It is handed to Phase 6 as a measured pathology candidate.

## 3. Laws compared

Complete-action coverage = (τ, g) pairs with at least one complete route. ΔJ is relative to all routes of the dataset.

**65-route data** (`char_R`):

| law | route rollouts (fraction) | coverage (8 epochs) | ΔJ range | earliest admissible τ change |
|---|---|---|---|---|
| direct only | 0.02 | 1/1 … 19/21; **8/43 lateral-low moving** | 0 … +0.065 | +0.45 s lateral-low moving |
| current 17 (8 dirs × 80,140) | 0.26 | **20/28 longitudinal moving**, 4/6 longitudinal rest, ≥ 20/21 elsewhere | 0 … +0.010 | 0 |
| 9 (4 dirs × 80,140) | 0.14 | 19/28 … 41/43 | 0 … +0.017 | +0.45 lateral-low moving |
| lazy: direct, then all 64 rings on failure | 0.02–0.82 | **100% in 8/8** | 0 … +0.065 (J preference) | 0 |

**17 + stretch data** (`char_stretch`):

| law | route rollouts (fraction) | coverage (8 epochs) | ΔJ range | earliest τ change |
|---|---|---|---|---|
| rings only (current 17) | 0.74 | 20/21 near-ground moving, else 100% | 0 … +0.048 | 0 |
| direct + stretch ladder, no rings | 0.30 | 19/20 … 36/42 | 0 … +0.054 | 0 |
| **failure-conditioned: direct; reach_tracking → stretch ladder (ascending, stop at first success), then rings; other failure → rings** | **0.04–0.50** | **100% in 8/8** | 0 … +0.065 (J preference for non-direct when direct succeeds) | **0** |

**Interpretation.**
- The failure-conditioned generator is the only law with full coverage in both datasets. Its cost grows only where direct fails: 4% in near-ground/diagonal, 25–50% in lateral-low and longitudinal.
- Its ΔJ is not a coverage or earliest-τ loss. It comes entirely from the unchanged J preferring a non-direct (often slower) motion when direct already works. Whether that preference is desirable is decided in Phase 6.
- If Phase 6 keeps a cost that genuinely prefers longer motions, the law must add the stretch/ring alternatives even when direct succeeds. That is a cost choice, not a feasibility need.

## 4. The law

```
Input: certified static grasp g at τ (from G_k(τ)), start mouth pose, predicted standoff.

R_k(τ, g) = [ direct ]
if direct fails with reach_tracking (timing):
    for κ in (1.10, 1.20, 1.35, 1.50, 1.70):          # stretch ladder, ascending
        R_k += [ direct × κ ];  stop at the first success
if direct still fails, or failed on clearance / other geometric reasons:
    R_k += rings(8 directions × {80, 140, 200} mm)      # spatial alternatives
N_R(k, τ, g) = 1 when direct succeeds; grows only on failure.
```

**Stage order.** The generator uses the certification's own failure reason, so it adds no new geometry predicate. With the Track 1 exact timing prune, any alternative whose stretched duration already fails admission at the decision time is skipped. This is lossless, because T_pres grows with κ and with stretch.

## 5. Parameter provenance

| symbol | value | classification | evidence / note |
|---|---|---|---|
| direct first, alternatives on failure | — | LITERATURE-SUPPORTED (Yang 2021 E1) + MEASURED coverage 8/8 | §3 |
| reach-duration model `timingArmScale · T_s · stretch` | inherited | **defect identified**: underestimates governed reach time for direct lateral-low (MEASURED) | A derived replacement (duration from the governor speed and lead limits over the chord) is the principled fix. It changes timing admission and belongs to the Phase 9 timing constants. |
| stretch ladder κ ∈ {1.10, 1.20, 1.35, 1.50, 1.70} | — | EMPIRICAL (≥ 1.10 needed; 1.20 rescues 82% in lateral-low; no gain beyond) | Superseded if the reach-duration model is repaired. |
| ring radii {80, 140, 200} mm × 8 directions | — | EMPIRICAL: 200 mm needed for longitudinal coverage; 40 mm rescues 10/544; 80/140 cover the rest | The radius depends on scenario geometry; no derivation from obstacle geometry is available (the log reason names the limiting sample/obstacle, not a direction). |
| maximumPathStretch 1.90 | inherited | ENGINEERING FALLBACK | not binding in the data |

## 6. Limits

- Offline, four scenarios, one start pose. Longitudinal failures that no tested alternative rescues are not explained.
- The stretch ladder treats a timing-model defect symptomatically. The repair (duration derived from governor limits) must be implemented and re-characterized before the final freeze (Phase 9/10).
- The J preference for slower or curved motions (§2) is unresolved until Phase 6.
