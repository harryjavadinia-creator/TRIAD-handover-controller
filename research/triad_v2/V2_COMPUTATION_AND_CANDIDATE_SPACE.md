# TRIAD V2 — computational credibility, stage certification and T/G/R characterization

Branch `research/triad-v2-dynamic-control`. Evidence built from clean HEAD `ae1e824`:
34 runs, logs xz-compressed in [`evidence/runs_ae1e824`](evidence/runs_ae1e824), with generated tables in `analysis/`.
The frozen V2 hypothesis, ξ=(τ,g,r), cost, terminal controller and bank semantics are unchanged.
No constant was tuned. The bank values 14/32/17 are **not** replaced here.

Evidence tags: **[M]** measured in these logs · **[C]** follows from the code · **[I]** inference, not yet tested.

## 1. Code changes (commits after `883c14e`)

| Commit | Change |
|---|---|
| `439bf77` | **Cancellation of superseded generations.** A pending job is cancel-requested, without blocking, in three cases: (a) the receiver state generation has advanced; (b) FULL_SEARCH only: the newest independent prediction differs from the certified pose at any bank instant by more than the commit-freshness tube (15 mm / 0.12 rad, existing `predictiveReachPolicy`); (c) FULL_SEARCH only: the held arm left the snapshot start pose by more than the reach tracking tolerances. The worker polls `plannerCancel_` at every bounded work unit: every `stepFiniteTriadSearch` step for searches, and every `routeWorkUnitsPerCycle` (128) route units for certification rollouts, previously 4096. A cancelled result is discarded on arrival whether or not the worker finished. `[V2SnapshotAudit]` logs snapshot age, robot drift, object displacement and the snapshot prediction's error at every outcome. Checker invariant **I9** plus mutation tests cover it. |
| `461aa0f` | **Instrumentation** (worker thread of V2 runs only; no decision reads it). `[V2JobProfile]` gives wall time and counts per stage. `[CertStage]` gives the deepest certified stage per evaluated candidate. `routeStepFail` now records the phase at failure. |
| `e57f6eb` | **`characterizeOnly` mode.** One FULL_SEARCH at the moving epoch and one at rest; geometry is evaluated at every lead and nothing is adopted. Adds `[V2CompleteRecord]` and `[V2CharacterizationSelection]` (unchanged selector at epoch and at receipt), the `tools/analyze_v2_characterization.py` analysis tool (`timing`, `stages`, `subset`, `tgr`) and `TRIAD_MAX_WAIT`. |
| `1139d31` | **Defect fix, found by the 191-lead run.** The shared search tests the hypothesis budget before it detects end-of-schedule, so the V2 budget `max(leads,15)` made any bank of ≥15 leads end in `budget_exhausted`. The budget is now `leads+1`, which is the same value (15) for the default bank. |
| `ae1e824` | Evidence summary reports cancellation coverage; `subset` reproducibility check. **Evidence HEAD.** |
| `18012fa` | Evidence summary I9 column; counterfactual rows tagged by run. Tools only; the analysis tables in the evidence were generated with this version. |

**V1 regression [M]:** all four FrozenPlanSet sha256 hashes are identical to the recorded evidence (198/283/432/233 records). V1-independent completes 1/4, as before. Cross-run giver truth is bit-identical between V1-independent and V2 in all four scenarios.

## 2. Handover outcomes with cancellation (V2, independent giver)

| Scenario | Run | Repeat | Notes |
|---|---|---|---|
| longitudinal | completed | completed | 36–37 superseded searches cancelled while the giver decelerates |
| near-ground | completed | completed | 1 replacement each |
| lateral-low | **failed closed** | completed | Run: a plan certified at rest was invalidated mid-reach (recertification: `closure/right_inner_pad`). The replacement's terminal certificate failed at τ (`closure/pad_pair`) 0.19 s before the window closed. |
| diagonal | completed | completed | Robot and object moved concurrently (24 samples) |

- **Invariants [M]:** I1–I9 PASS on every completed run; I9 also passes on the failed run. The injected stale terminal result and the injected supersession were both rejected with no effect, and both runs completed.
- **Lateral-low [M]:** 0/2 before cancellation (`runs_d9e8886`); with cancellation 1/2 in this campaign, plus 1/1 in a development run. The difference is attributable: the 6.5 s search launched during the stopping transient is now cancelled.
- **Remaining failure [M]:** start-state-dependent closure certification combined with a hold-to-settle delay of about 3 s before replacement. Too few runs to claim a rate.

## 3. Timing results [M]

Source: `analysis/TIMING.md`, 8 V2 scenario runs. Controller time runs at about wall time.

| Job | Outcome | n | Worker wall median / p95 / max | Latency max |
|---|---|---:|---|---:|
| FULL_SEARCH | accepted | 12 | 0.55 / 3.32 / 3.59 s | 3.59 s |
| FULL_SEARCH | cancelled | 211 | 0.015 / 0.032 / 3.90 s | 3.90 s |
| RECERTIFY_ACTIVE | accepted | 4408 | 4.2 / 8.2 / 15.4 ms | 50 ms |
| TERMINAL_CERTIFY | accepted | 210 | 2.4 / 3.4 / 4.4 ms | 5 ms |

**FULL_SEARCH.** A search with 14 distinct poses takes 2.9–3.9 s; with the object at rest (memoized) it takes 0.47–0.57 s. Share of accepted-search wall time:

- route reach phase: 59% (governor + IK + swept-volume query at every preview step);
- static reach-to-standoff: 16%;
- finalize / terminal timing audit: 6%;
- route closure: 5%;
- everything else: 14%;
- nested within these: IK steps 42%, swept-volume queries 40%.

**RECERTIFY_ACTIVE.** Route reach 49%, finalize 19%, closure 16%, retreat 9%.

**TERMINAL_CERTIFY.** Finalize 37%, closure/contact 31%, carried retreat 17%, insertion 12%.

**Cancellation.**
- 211 FULL_SEARCH cancellations. Controller-time latency from request to discard: median 1 ms, max 5 ms.
- The worker was stopped by the flag in 211/211 cases.
- 4 RECERTIFY_ACTIVE jobs had already finished before cancellation (a phase change); they were discarded.

**Snapshot age at receipt.**
- FULL_SEARCH accepted: age ≤ 3.59 s; object displacement since snapshot ≤ 0.287 m; prediction error ≤ 0.1 mm (constant-velocity segment).
- RECERTIFY_ACTIVE: age ≤ 50 ms, but **robot drift from the snapshot ≤ 14.8 mm**. The certificate describes a state up to 15 mm behind the arm (limitation, §6).

**Supersession thrash [M].** While the giver decelerates (about 0.85 s), each new search is superseded within 12–40 ms. That is 33–37 cancelled searches per run, each costing 15 ms median. It ends when the object is at rest.

**Instrumentation overhead [M].** Not resolvable. The moving-epoch 14-lead search took 3.59–3.90 s here versus 3.6–3.7 s before instrumentation.

## 4. Candidate-stage certification (goal C)

Stage ladder, taken from the certifier's own phase order:

- **NONE** — reach failed;
- **REACH** — standoff reached within tracking tolerance and clearance reserve;
- **INSERTION** — capture pose reached through the corridor; for routes, also the capture dwell/velocity gate;
- **CLOSURE_CONTACT** — bilateral pad contact with all hard constraints;
- **CARRIED_RETREAT** — carried retreat certified; COMPLETE additionally requires a valid cost audit.

Transfer readiness has **no candidate-dependent test** in the certifier: it is implied by bilateral contact. It therefore cannot discriminate candidates and is not a separate rung **[C]**.

### 4.1 Static screen: every grasp at every event

Accepted searches, memoized hypotheses expanded [M]:

| Set | n | NONE | REACH | INSERTION | CLOSURE_CONTACT | CARRIED_RETREAT | Rejected only after reach |
|---|---:|---:|---:|---:|---:|---:|---|
| V2 scenario runs | 5 376 | 57.9% | 3.1% | 26.7% | 3.1% | 9.2% | **32.9%** of evaluated = **78.1%** of reach-certified |
| characterization, default bank | 3 584 | 56.1% | 2.5% | 29.7% | 3.6% | 8.2% | 35.7% / 81.3% |
| characterization, 191 leads | 48 896 | 57.9% | 2.7% | 27.7% | 3.5% | 8.3% | 33.9% / 80.4% |
| characterization, 64 angles | 14 336 | 57.3% | 2.4% | 29.1% | 3.3% | 7.9% | 34.8% / 81.4% |

Downstream rejection reasons, 191-lead set:

| Rung reached | Reason | Count |
|---|---|---:|
| INSERTION | `closure/pad_pair/blue_handle_acquisition_tube` | 11 705 |
| CLOSURE_CONTACT | `ik_preview_no_convergence` (carried retreat) | 1 678 |
| INSERTION | `closure/*_pad_shoulder_low/robot_blue_handle` | 1 609 |

**About 4 in 5 grasps that the arm can reach are rejected by closure/contact or carried-retreat geometry** **[M]**.

### 4.2 Route rollouts

Route rollouts run only for grasps that already passed the complete static screen, so these counts are conditional on it:

- 51% fail in governed transit reach (NONE), mostly `reach_tracking`;
- 0.6–1.6% fail only downstream, at closure (`static_acquire/closure/*`), from the governed arm state;
- the rest are complete.

### 4.3 Reach-only selection versus complete-action certification [M]

Reach-only rule: earliest timing-admissible event, then shortest reach time, then largest clearance. The same timing admission as the selector is applied at the search epoch.

| Evidence set | Reach-only (static reach to standoff) picks an action complete certification rejects | Reach-only (governed route reach) picks a rejected action |
|---|---|---|
| V2 scenario searches | **9 of 12** | 2 of 12 (near-ground: `axisN_side_293deg/direct`, rejected at closure) |
| characterization, default bank | 6 of 8 | 1 of 8 |
| characterization, 191 leads | 6 of 8 | 1 of 8 |
| characterization, 64 angles | 5 of 8 | 1 of 8 |

**At the event reach-only picks, few or none of the reach-feasible grasps are complete.** Typical counts are 0/1, 0/5, 1/5 and 2/20.

**Conclusion [M].** On this robot, object and certifier, a reach/IK-only receiver would usually commit to an action that cannot be grasped or carried away, and only closure/contact and retreat certification exclude it.

**Scope.** This supports the scientific value of certifying through closure and carried retreat. It does **not** support a separate transfer-readiness stage, and it does not show that the full route rollout needs its own closure re-check. That re-check changes the outcome in only 1–2 of 8–12 searches.

## 5. T/G/R candidate spaces (goal B)

**Method.**
- Each axis was refined independently on a superset grid; the other two stayed at the default.
- Every coarser resolution is an **exact subsample of the same search**.
- Moving-epoch searches of all 12 superset runs reproduce every default-bank complete record with identical objective (`analysis/SUBSET_CHECKS.txt`: 12/12 PASS, 0 missing, 0 differing).
- "Best J" is argmin of the unchanged global objective over admissible cost-valid records. A Python re-implementation matches the logged selector in all cases.

**Caveats.**
- These are four deterministic scenarios at one giver speed (0.08 m/s), one robot start and one object.
- The prediction is constant-twist. The scripted giver actually stops after 0.40 m, about 4.1 s after the moving epoch, so poses at larger moving-epoch leads are never realized.

### 5.1 Temporal interval and Δτ

Source: `analysis/TGR_T_*.md`, 191 leads on [0.5, 10] s [M].

| Scenario | Complete leads (any action) | Timing-admissible complete intervals | Earliest admissible τ, 0.05 grid | …V1 bank | ΔJ, V1 bank | ΔJ, Δτ = 0.1 / 0.2 / 0.45 / 0.9 / 1.8 |
|---|---|---|---:|---:|---:|---|
| longitudinal | 92/191 | [2.10,2.35] [2.85,6.00] [7.20,7.45] | 2.10 | 2.35 | 0.0101 | 0.0026 / 0.0026 / 0.0101 / 0.0101 / 0.0141 |
| near-ground | 159/191 | [2.90,5.30] [6.15,6.75] [7.10,7.35] [7.70,8.75] [9.00,10.00] | 2.90 | 3.25 | 0.0037 | 0 / 0.0075 / 0.0037 / 0.0037 / 0.0037 |
| lateral-low | 191/191 | [3.10,10.00] | 3.10 | 3.25 | 0.0081 | 0.0028 / 0.0028 / 0.0081 / 0.0229 / 0.0718 |
| diagonal | 129/191 | [3.90,10.00] | 3.90 | 4.15 | 0.0062 | 0 / 0 / 0.0062 / 0.0062 / 0.0588 |

- **Lower bound of the useful interval [M].** Set by timing admission, not geometry. Complete actions exist at leads down to 0.5–2.45 s but are inadmissible (reach time + 0.05 s and the 1.6 s commit lead). The admissible lower bound is 2.1–3.9 s.
- **Upper bound [M, I].** Not reached by geometry in 3 of 4 scenarios; complete actions persist to 10 s. The physical bound is when the giver stops, which the constant-twist prediction cannot know.
- **Feasible set is fragmented [M].** Longitudinal and near-ground have gaps of 0.25–0.85 s. A coarse grid can straddle a gap: the V1 bank's earliest admissible τ is 0.15–0.35 s later than the 0.05 s grid in all 4 scenarios.
- **Convergence [M].**
  - Δτ ≤ 0.2 s recovers the earliest admissible τ in 4/4 scenarios, with ΔJ ≤ 0.0075.
  - Δτ = 0.45 s (the V1 spacing) loses 0.15–0.35 s of opportunity in 4/4.
  - J changes little because the objective is dominated by the event time.
- **Physical argument [C].** At object speed v, adjacent hypotheses differ by v·Δτ. The commit-freshness tube is 15 mm, so Δτ ≤ 0.015/v, which is **0.19 s at 0.08 m/s**. The measured convergence at Δτ ≈ 0.1–0.2 s is consistent with this.
- **Cost [M].** Search time grows linearly with leads: 191 leads took 45–78 s. **None of the 12 superset moving-epoch searches (T191, G64, R65) had a timing-admissible plan at receipt**, against 3 of 4 default-bank searches. Fine temporal resolution is unusable at receipt unless the per-lead cost falls or leads are ordered and truncated **[I]**.

### 5.2 Receiver approach family and angular resolution

Source: `analysis/TGR_G_*.md`, 64 angles × 2 axis signs, 5.625° spacing. Angle 0 is robot-relative (outward from handle to mouth) [M].

- **Approach family.** Complete grasps occupy a few contiguous arcs around the handle axis. One handle-axis sign dominates per scenario:
  - axisP in longitudinal and diagonal: 988/1007 and 989/998 records;
  - axisN in lateral-low: 1418/1500.
- **Arc widths at rest** (the committed geometry):
  - longitudinal: axisP 7 and 8 samples, plus an axisN singleton;
  - near-ground: axisP 4 samples (45–62°);
  - lateral-low: axisN 8 and 3 samples, plus an axisP singleton;
  - diagonal: axisP 4 samples (326–343°).
  - The narrowest non-singleton arcs are 3–4 samples, **about 17–23°**. Single-sample arcs (below 5.6°) exist, so N_θ=64 has not converged for those.

**Coverage versus N_θ per sign** (ΔJ against 64; earliest admissible τ):

| Scenario / epoch | 4 | 8 | 16 (current) | 32 |
|---|---|---|---|---|
| longitudinal moving | ΔJ 0.131, τ 4.60 | 0.124, 3.70 | 0.0055, 2.35 | 0.0055, 2.35 |
| longitudinal rest | none | 0.033 | 0.0026 | 0.0009 |
| near-ground moving / rest | none | 0 | 0 | 0 |
| lateral-low moving | 0.194, τ 4.60 | 0.041, 3.70 | 0, **τ 3.25** | 0, **τ 2.35** |
| lateral-low rest | none | 0.093 | 0 | 0 |
| diagonal moving | 0.166, 5.95 | 0.083, 4.60 | 0, 4.15 | 0, 4.15 |
| diagonal rest | none | **none** | 0.0128, **τ 2.35** | 0.0128, **τ 1.90** |

- **N_θ = 8 (45°) [M].** Misses the whole feasible family in diagonal-rest and is ≥0.03 worse in 5 searches.
- **N_θ = 16 (22.5°) [M].** Always finds a complete action. It finds a later earliest admissible τ than 32 in 2 of 8 searches (by 0.45 s and 0.9 s).
- **N_θ = 32 → 64 [M].** Never changes the earliest admissible τ; ΔJ ≤ 0.013.
- **Geometric argument [C, I].** Sampling is guaranteed to hit an arc of width w only if spacing ≤ w. Measured narrowest non-singleton arcs of about 17–23° imply ≥ 16–21 samples per sign; 32 gives margin.
- **What sets w [I].** The arc width should be derivable from the Robotiq opening (the pad-pair acquisition tube) and the CALL handle radius and corridor angle limits. That derivation is not done here.
- **Cost [M].** Static records scale linearly with N_θ; 64 angles took 11–19 s per moving search.

### 5.3 Direct motion versus route alternatives

Source: `analysis/TGR_R_*.md`, 16 directions × {40, 80, 140, 200} mm + direct = 65 routes [M]. Counts are (event, grasp) pairs that passed the static screen.

| Scenario / epoch | Direct sufficient | Route required | No complete route | Leads with a complete action: direct only / 65 routes | ΔJ, direct only | ΔJ, V1 17 routes | ΔJ, 4 dirs × {80,140} (9 routes) |
|---|---|---|---|---|---:|---:|---:|
| longitudinal moving | 15 | 13 | 7 | 8 / 9 | 0.0063 | 0.0063 | 0.0063 |
| longitudinal rest | 42 | 42 | 0 | 14 / 14 | 0.0030 | 0.0030 | 0.0030 |
| near-ground moving | 19 | 2 | 3 | 11 / 11 | 0.0650 | 0 | 0.0169 |
| near-ground rest | 14 | 0 | 0 | 14 / 14 | 0.0688 | 0.0090 | 0.0090 |
| lateral-low moving | 8 | **35** | 0 | **7 / 14** | 0.0022 | 0.0022 | 0.0022 |
| lateral-low rest | 28 | 28 | 0 | 14 / 14 | 0.0024 | 0.0024 | 0.0024 |
| diagonal moving | 16 | 8 | 0 | 8 / 8 | 0.0490 | 0 | 0 |
| diagonal rest | 14 | 0 | 0 | 14 / 14 | 0.0071 | 0.0034 | 0.0035 |

- **Routes change feasibility only where direct transit fails [M].** Lateral-low moving: direct-only covers 7/14 events; routes cover 14/14. Longitudinal moving: 8/9. Elsewhere routes only lower J, by up to 0.069.
- **How the direct route fails [M].** At the NONE stage, almost always `predictive_static/reach_tracking` (the governed reach did not arrive within tolerance in its allotted duration). A few failures are clearance reserves.
- **Why a route may succeed [C, I].** Route reach duration is the static reach duration × path stretch, so a curved route receives more time. "Route required" may therefore partly reflect the reach-timing model rather than obstacle avoidance. This needs an ablation: direct transit with equal duration.
- **Radius versus direction count [M].** Radius matters more than direction count.
  - 4 directions reproduce 8 or 16 in most cases.
  - The best radius depends on the scenario: 40 mm (longitudinal, lateral-low), 80 mm (near-ground; diagonal at rest), 140 mm (diagonal moving).
  - 200 mm is never best.
- **Cost [M].** About 5 ms per route rollout. Cost scales with #routes × #static-feasible grasps × #leads. 65 routes took 8.5–16 s per moving search, versus about 3 s for 17.

## 6. Failures and limitations observed

1. **Lateral-low failed closed in one of two runs [M].** Mechanism: certification depends on start state, the arm is held ~3 s to settle before replacement, and the terminal certificate failed at τ.
2. **Moving-epoch full search (14 leads) costs 2.9–3.9 s [M].** During giver deceleration it is superseded roughly every 15–40 ms; 33–37 cancelled searches per run. Cancellation removes wasted latency but cannot produce a plan while the prediction changes faster than a search completes.
3. **RECERTIFY_ACTIVE certificates are up to 50 ms / 14.8 mm stale against the moving arm [M].** They are not re-anchored before retention. Commit still uses a fresh TERMINAL_CERTIFY with drift checks.
4. **Search-time failure at finer resolution [M].** All 12 superset moving-epoch searches (8.5–78 s) had zero timing-admissible plans at receipt; the default bank had plans in 3 of 4.
5. **Defect found and fixed (`ae1e824`):** hypothesis budget with ≥15 leads.
6. **Tooling defects found and fixed:** evidence-summary header omitted I9; counterfactual rows lacked run tags.
7. **Scope.** Four deterministic scenarios, one speed, simulation only. Characterization runs apply timing admission at the search epoch. Rest-epoch searches start at different times across runs, so only within-run subsampling is exact.

## 7. Recommendations (not implemented)

1. **Justify N_τ from v·Δτ ≤ freshness tube** (Δτ ≤ 0.015/v). The data show Δτ ≤ 0.2 s suffices at 0.08 m/s. Bound the interval below by timing admission and above by the prediction's validity horizon, not by a fixed 8 s. Because search cost is linear in leads, evaluate leads in ascending order and stop at the first admissible complete event **[I]**. The objective already chooses at or near the earliest admissible event.
2. **Justify N_θ from the narrowest complete-feasible arc.** Derive the arc from gripper opening, handle radius and corridor limits, then confirm by convergence (32 per sign converged here; 16 did not in 2 of 8 searches). Report singleton arcs as an unresolved sensitivity.
3. **Before sizing N_r, run the missing ablation:** direct transit with the same duration as the curved route. If `reach_tracking` failures disappear, the route family compensates for the reach-timing model rather than geometry. Otherwise, radius (not direction count) is the dimension to justify.
4. **Keep complete-action certification through closure/contact and carried retreat** (4.1, 4.3). Drop the claim of a separate transfer-readiness stage unless a candidate-dependent transfer test is added.
5. **Computation.**
   - Route reach (59%) and swept-volume/IK queries (together about 80% nested) dominate; they are the only levers that would make finer banks admissible at receipt.
   - Supersession thrash suggests using a prediction-change rate, not only a tube, to decide when a full search is worth starting **[I]**.
6. **Evidence still needed:** repeated runs with timing noise and varied giver speeds and stop times, to turn the lateral-low 1/2 and the counterfactual counts into rates.

## 8. Reproduce

```bash
TRIAD_RECEIVER_MODE=v2 scripts/run_scenario.sh diagonal out/v2/diagonal
TRIAD_MAX_WAIT=900 TRIAD_RECEIVER_MODE=v2 \
  TRIAD_EXTRA_OVERRIDE=research/triad_v2/evidence/runs_ae1e824/overrides/char_T.yaml \
  scripts/run_scenario.sh diagonal out/char_T/diagonal
python3 tools/analyze_v2_characterization.py timing out/v2/*/*.log
python3 tools/analyze_v2_characterization.py stages out/v2/*/*.log
python3 tools/analyze_v2_characterization.py tgr --search moving out/char_T/diagonal/diagonal.log
python3 tools/analyze_v2_characterization.py subset --search moving out/char_default/diagonal/diagonal.log out/char_T/diagonal/diagonal.log
python3 tools/summarize_v2_evidence.py out
```

The campaign script is `evidence/runs_ae1e824/campaign.sh`.
