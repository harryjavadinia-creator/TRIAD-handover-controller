# A1: complete-action certification depth ablation

**Scope.** Offline replay over the frozen 14×32×17 bank. No controller behaviour was changed. Exact timing prune OFF throughout. The bank is a frozen experimental reference, not the final bank.

Tags: **CODE** · **MEASURED** · **COUNTERFACTUAL** · **INFERENCE**.

## 1. Checkpoint and sources

**Git.**
- Checkpoint tag `triad-v2-checkpoint-a1-start` = `11ea52a`.
- Logging-only instrumentation `ba7f887`, run from a clean worktree (`evidence/runs_a1_ba7f887/HEAD.txt`).
- Analysis tools: `tools/a1_certification_depth.py`, `tools/a1_summarize.py`.

**Instrumentation (logging only).** Retreat-certified route records log the terminal timing audit result (ran, success, reason, duration), the pre-audit execution-time estimate, whether all cost inputs are finite, and a PROXY objective. The proxy is computed on a copy of the candidate using the pre-audit time and is never read by any decision.

**Equivalence proof (MEASURED; `runs_a1_ba7f887/LOGGING_ONLY_EQUIVALENCE.txt`).**
- Moving-epoch search, 4 scenarios: every `[CertStage]` record is identical to the Phase-D build (856–1 179 per scenario; 0 differing, 0 extra).
- Zero-latency selections of the moving and at-rest searches are identical in 4/4 scenarios.
- V1 FrozenPlanSet hashes identical (198/283/432/233).
- I1, I9, I10 and I11 pass; completed in-situ runs pass all invariants.

**Evidence** (frozen bank, prune off):

| Set | Contents |
|---|---|
| `runs_a1_ba7f887` | char ×4, in-situ ×4, V1 ×4 |
| `runs_phaseD_16fbdde` | char_a, char_b, insitu_a, insitu_b |
| `runs_prune_7b312d6` | char_off, off_a, off_b, off_c |

- **Total:** 40 V2 logs. Identical searches are de-duplicated by record fingerprint, leaving **27 unique completed searches** (71 occurrences):
  - 4 moving-epoch;
  - 23 at-rest, including searches from moved arm states after runtime invalidation.
- **Per-candidate listing:** `analysis/a1_candidates.csv`, 6 030 evaluated candidates with population, τ, g, r, deepest stage passed, first failing stage, reason, cost validity, J, reach and retreat clearance, and audit fields.
- **Tables:** `analysis/A1_TABLES.md`.

## 2. How the certificate is structured, and one limit on the ablation (CODE)

**The certificate as implemented.**
- **STATIC screen** per (τ, g): F1 reach standoff → F2 reach capture (corridor) → F3 closure sweep to bilateral contact → F5 carried retreat.
- **ROUTE rollout** per (τ, g, r), only for grasps passing the whole static screen: F1 governed transit reach → F2 approach + capture dwell → F3 closure → F5 carried retreat.
- **Finalize:** the terminal timing audit (`previewVelocityGateParityShadow`, 4.0 s limit), then the seven-term cost.
- **F4 transfer readiness does not exist** and is not reported as a stage.

**Consequence.** `computeCompletePlanAuditCost` sets `completeCostAuditValid` only if the audit succeeded and every whole-action input (effort, path, reach and retreat clearance, joint margin, conditioning, velocity utilization) is finite. `auditEstimatedTime` is written only on audit success. `selectFiniteEventPlan` discards records without a valid finite cost. Therefore:

- **(a) Literal reading.** C1–C4 with the unchanged objective and selector select *exactly the C5 winner* in every search, because only fully certified records have a cost. Under the current objective, certification depth cannot be ablated at the selection level. The objective's validity condition already embeds F1–F5 plus the audit. **CODE**
- **(b) Ranking-free reading (used below).** Suppose a shallower policy could rank its extra candidates somehow. Its winner is then:
  - **DETERMINED_SAME** when no shallow-only candidate can be admitted at the decision time with a J lower bound ≤ the full winner's J. The bound J ≥ w_T(lead + 0.70 s)/T_ref (CODE) turned out never to exclude anyone.
  - **DETERMINED_INVALID** when the full certificate has no admissible action but observed shallow-only records are admissible, so any ranking picks an action the full certificate rejects.
  - **NOT DETERMINED** otherwise.
  - For statically rejected grasps whose routes were never rolled out, admissibility uses the exact necessary condition T_route ≥ T_static (Track 1 proof).

## 3. Summary table

27 unique completed searches, both epochs pooled. Selection columns are ranking-free statuses at the receipt time (as executed); zero-latency results follow the table. Computation is the worker wall summed over these searches.

| Level | STATIC (τ,g) surviving / 2 528 | P(full rejection \| STATIC accepted) | ROUTE (τ,g,r) surviving / 3 502 | P(full rejection \| ROUTE accepted) | Selected tuple changes (literal / ranking-free determined) | Shallower winner determined later-rejected | Main later rejection stage of shallow-only contenders | Computation of the stage (share of wall) |
|---|---|---|---|---|---|---|---|---|
| C1 = F1 | 1 075 (42.5%) | 907/1 075 = **84.4%** | 1 764 (50.4%) | 53/1 764 = 3.0% | 0 / 1 (26 not determined) | 1 search (9 occ.), longitudinal moving | F3 closure (route and static), static F2 | static F1 2.84 s (13%); route F1 14.26 s (64%) |
| C2 = F1+F2 | 1 000 (39.6%) | 832/1 000 = 83.2% | 1 764 (50.4%) | 53/1 764 = 3.0% | 0 / 1 (26 not determined) | same search | F3 closure | static F2 0.57 s (3%); route F2 0.44 s (2%) |
| C3 = +F3 | 275 (10.9%) | 107/275 = 38.9% | 1 725 (49.3%) | 14/1 725 = 0.8% | 0 / 0 (21 not determined; 5 same; 1 none) | 0 | static F5 (unrolled grasps) | static F3 0.49 s (2%); route F3 1.03 s (5%) |
| C4 = +F5 | 206 (8.1%) | 38/206 = 18.4% | 1 725 (49.3%) | 14/1 725 = 0.8% | 0 / 0 (26 same, 1 none) | 0 | timing audit timeout | static F5 0.43 s (2%); route F5 0.63 s (3%) |
| C5 = +audit (current) | 168 with ≥1 complete route (6.6%) | 0 | 1 711 (48.9%) | 0 | — | — | — | audit + cost 1.26 s (6%, 1 725 finalizations) |

**Reading the conditional rejection columns.**
- "Full rejection" of a STATIC (τ, g) means no fully certified route exists for it.
- ROUTE rows are conditional on the static screen having passed. That is why STATIC and ROUTE percentages differ by an order of magnitude and must not be combined.

**At the zero-latency decision time** (all records admitted at the search epoch):

| Level | Determined same | Not determined |
|---|---:|---:|
| C4 | 26 | 1 (near-ground moving: 14 audit-rejected records without J; the PROXY winner is the same tuple) |
| C3 | 4 | 23 |
| C2 | 0 | 27 |
| C1 | 0 | 27 |

## 4. Per-scenario counterfactual winners (COUNTERFACTUAL)

Representative search per scenario; all tables are in `analysis/A1_TABLES.md`. Winners are (lead s, g, r); clearances are reach/retreat in m.

| Scenario | Epoch, decision | C5 winner, J, clearances | C4 | C3 | C2 | C1 |
|---|---|---|---|---|---|---|
| diagonal | moving, receipt | 5.5, axisP_337, ring80_2; 0.775; 0.074/0.223 | same | not det. (3 static-F5-rejected grasps) | not det. (5 route closure fails + 51 grasps) | as C2 |
| diagonal | moving, epoch | 4.15, axisP_337, ring140_2; 0.682; 0.072/0.218 | same | not det. (13) | not det. (152) | not det. (152) |
| longitudinal | moving, receipt | **none admissible** | none | none | **INVALID**: 15 admissible route records failing F3 closure (+10 grasps) | **INVALID** (15 route records + 15 grasps) |
| longitudinal | moving, epoch | 2.35, axisP_337, direct; 0.621; 0.079/0.210 | same | not det. (10) | not det. (152) | not det. (163) |
| near-ground | moving, receipt | 8.0, axisP_247, ring140_5; 1.037; 0.082/0.088 | same (PROXY same) | not det. (2) | not det. (15) | not det. (37) |
| near-ground | moving, epoch | 3.25, axisP_45, ring80_7; 0.754; 0.071/0.106 | not det. (14 audit timeouts; **PROXY same**) | not det. (19) | not det. (68) | not det. (117) |
| lateral-low | moving, receipt | 8.0, axisN_23, ring80_6; 0.932; 0.069/0.099 | same | **same** | not det. (6) | not det. (6) |
| lateral-low | moving, epoch | 3.7, axisN_337, direct; 0.678; 0.082/0.077 | same | not det. (10) | not det. (135) | not det. (135) |
| lateral-low | at rest after invalidation (8 searches), receipt/epoch | 1.8, axisN_337, direct; 0.471; 0.067/0.077 | same | **same** | not det. (107–108) | not det. |
| longitudinal / near-ground / diagonal | at rest (normal), receipt | 2.8 / 3.7 / 2.8 (see tables) | same | not det. (10 / 10 / 21) | not det. | not det. |

**Whether a fully certified alternative existed.** At every decision time in the evidence except the longitudinal moving search at receipt. There, the full certificate correctly returned no plan, and C1/C2 would necessarily have selected a closure-infeasible action.

## 5. STATIC screen rejection breakdown (MEASURED)

| Stage | Moving (1 792 screens) | At rest (736 screens) | Top reasons |
|---|---:|---:|---|
| F1 reach | 1 029 (57.4%) | 424 (57.6%) | IK non-convergence 305 / 107; ground-plane clearance of fingertips; object/handle clearance |
| F2 insertion | 61 (3.4%) | 14 (1.9%) | IK non-convergence 57 / 14; corridor axial / lateral 4 |
| F3 closure/contact | 532 (29.7%) | 193 (26.2%) | `closure/pad_pair/blue_handle_acquisition_tube` 462 / 169; pad-shoulder contact with blue handle 68 / 23 |
| F5 carried retreat | 44 (2.5%) | 25 (3.4%) | IK non-convergence 43 / 25; wrist–ground 1 |

## 6. ROUTE rejection breakdown (MEASURED; population = routes of static-screen passes)

| Stage | Moving (2 142 rollouts) | At rest (1 360) | Top reasons |
|---|---:|---:|---|
| F1 transit reach | 966 (45.1%) | 772 (56.8%) | runtime clearance reserve 419 / 255; reach tracking 349 / 162; robust transit clearance reserve 151 / 82; path stretch limit 0 / 256 |
| F2 insertion + dwell | **0** | **0** | — |
| F3 closure/contact | 39 (1.8%) | **0** | `static_acquire/closure/pad_pair` 22; pad shoulder 17 |
| F5 carried retreat | **0** | **0** | — |
| Terminal timing audit / cost validity | 14 (0.7%) | 0 | `timeout` 14 (all near-ground, τ = 5.05–5.95 s); cost inputs finite in all |

## 7. Terminal timing audit (analysed separately)

1. **Rejections.** In planning it rejects **14 of 1 725** F1–F5-certified route records (0.8%). All are `timeout` (the terminal controller replay does not settle within 4.0 s). All belong to the near-ground moving search: 1 at τ = 5.05 s, and all 7 at 5.5 s and all 6 at 5.95 s. **MEASURED**
2. **Selection effect.**
   - Its rejections never changed a winner determinably: DETERMINED_SAME at receipt in 27/27 searches.
   - At near-ground's zero-latency epoch the effect is not determined because J is undefined without the audit. Under the PROXY objective (pre-audit time), the winner is unchanged: proxy J of the rejected records 0.92–1.20 vs winner J 0.754. **COUNTERFACTUAL**
3. **Runtime.** Inside RECERTIFY_ACTIVE the same audit **invalidated the executing, fully certified near-ground plan (τ = 8.0 s) in 6/6 prune-off in-situ runs** (`recertification_infeasible/cost_invalid/timeout`). A replacement plan (τ = 3.7 s) then completed in all 6. **MEASURED**
   - Whether the invalidated plan would have failed at the terminal gate is **NOT EXECUTED**. No run executed an audit-rejected action to capture.
4. **Cost.** Finalization (audit + cost) is 6% of search worker wall (1.26 s over 1 725 finalizations, about 0.7 ms each). It was 19% of RECERTIFY_ACTIVE wall in the Phase B profile. **MEASURED**
5. **Stale comment.** The code comment says the audit "never [changes] hard feasibility". Through cost validity it is a hard selection gate. **CODE**

## 8. Runtime connection (MEASURED vs NOT EXECUTED)

24 prune-off in-situ runs with the frozen bank:

| Scenario | Runs | Completed | Runtime invalidations of executing full-certificate actions |
|---|---:|---:|---|
| diagonal | 6 | 6 | none |
| longitudinal | 6 | 6 | none |
| near-ground | 6 | 6 | 6× timing audit timeout during PROVISIONAL_REACH, then replacement, then completion |
| lateral-low | 6 | 2 | 4 runs: F3 closure (`static_acquire/closure`) at PROVISIONAL_REACH **and** at TERMINAL_TRACK, then the no-plan window expired |

- **Only full-certificate actions were executed** (V2 adopts only C5 records). Every shallower-level winner in §3–§4 is **COUNTERFACTUAL / NOT EXECUTED**.
- **Runtime invalidations are real behaviour changes by F3 and the audit** during recertification from moved arm states. They are *not* evidence that a physical failure was prevented: the invalidated actions were never carried to capture.
- **No runtime invalidation by F1, F2 or F5 occurred** in prune-off runs. Route F1 `reach_tracking` invalidations occurred only with the prune enabled (Track 1) and are excluded here.

## 9. Stage verdicts

| Stage | Verdict | Basis |
|---|---|---|
| **F1 reach** | **USEFUL** | Rejects 57% of static screens and 45–57% of route rollouts (MEASURED). Every downstream stage rolls out from F1's reached state, so F1 cannot be removed as an independent test in this implementation (CODE). No observed runtime failure is attributable to its absence, and no selection change is determined at C1 beyond the F3 case. It changes candidate sets massively but is not shown NECESSARY. |
| **F2 insertion** | **REDUNDANT IN CURRENT EVIDENCE** | Never rejects a route rollout (0/3 502). Rejects 75 static screens (3.0%), 71 of them IK non-convergence of the insertion segment. It never produces a determined selection difference (C1 and C2 statuses coincide in every search), and no runtime invalidation. |
| **F3 closure/contact** | **USEFUL** (strongest; not provably NECESSARY) | The largest downstream filter: 725 static rejections (28.7%) and 39 route rejections; P(full rejection) drops from 83% to 39% at the static level (MEASURED). It is the only stage with a **determined** selection consequence: in the longitudinal moving search at receipt, C2/C1 must select a closure-infeasible action where C3–C5 select none (9 occurrences, COUNTERFACTUAL). At runtime it invalidated executing plans in 4 lateral-low runs (MEASURED). A prevented physical failure is not shown. |
| **F5 carried retreat** | **NOT TESTABLE FROM CURRENT DATA** | Never rejects a route rollout (0/3 502). Its only rejections are 69 static screens (2.7%, IK non-convergence during carried retreat), whose routes were never rolled out, so its effect on selection is unobservable. C3 is DETERMINED_SAME in the 5 receipt-time searches where no such grasp was admissible. No runtime invalidation. |
| **Terminal timing audit** | **USEFUL** | Planning: 14 rejections (0.8%), no determined or proxy winner change, so redundant for planning selection (MEASURED / COUNTERFACTUAL). Runtime: it invalidated the executing near-ground plan in 6/6 runs, which were then re-planned and completed (MEASURED). About 6% of search computation. A prevented terminal failure is NOT EXECUTED. |

## 10. What the evidence does and does not prove

**Does prove.**
1. The seven-term objective is defined only for fully certified actions. With the unchanged selector, shallower certification cannot change the selected tuple. Certification depth and the objective are coupled, so any shallow-certification TRIAD would require a different objective. **CODE**
2. Actions accepted at F1 are overwhelmingly rejected later at the grasp level (84% static). Downstream rejection comes almost entirely from F3 closure/contact (static) and F1 transit reach (route). F2 and F5 never reject a route rollout. **MEASURED**
3. In one scenario, at the executed decision time, stopping before F3 necessarily selects a closure-infeasible action where the full certificate correctly reports no plan. **COUNTERFACTUAL**
4. The terminal timing audit never changes a planning winner in this evidence, but it does change runtime behaviour: it invalidates executing plans. **MEASURED**

**Does not prove.**
1. That any stage prevented a physical/runtime failure. No shallow-certified action was executed; runtime invalidations stopped actions before capture.
2. That F2 or F5 is unnecessary in general. Their rejections are confined to static screens whose counterfactual routes were never rolled out. Four scenarios and one object are a narrow sample.
3. That the static screen is not over-conservative. Static rejections are not tested for false rejection here.
4. Anything about F4 transfer readiness. It is not implemented.
5. Absence of observed runtime failures without a stage is not proof that the stage is safe to remove.

## 11. Recommendation

1. **Preserve F3 closure/contact.** It is the principal complete-action filter and the only stage with a determined selection consequence. Upgrading its verdict to NECESSARY requires a runtime outcome ablation.
2. **Preserve F1.** It is structurally required by the rollout. Its route-level cost (64% of search wall) is a computational, not scientific, concern.
3. **Require runtime outcome ablation before simplifying or removing F2 or F5.**
   - F2 is redundant in current evidence at the route level.
   - F5 is not testable from current data.
   - Removing either requires a pre-registered execution experiment with shallow-certified actions actually executed, in simulation or on hardware, and a defined failure taxonomy. That is outside A1.
4. **Terminal timing audit.**
   - Preserve it as a runtime recertification gate; its runtime behaviour change is the evidence.
   - Its planning-time contribution is redundant in current evidence. Whether it can be simplified in planning needs the same runtime outcome ablation.
   - Correct the stale "never changes hard feasibility" comment in a separate, reviewed change (no behaviour change).
5. **Scientific claim wording.** Describe the certificate as F1–F3 + F5 + terminal timing audit, coupled to a cost defined only on complete actions. Do not claim independent transfer-readiness certification.

**Stopped after A1.** No A2/A4/A6/A9, threshold, bank or architecture work was started.
