# TRIAD V2 — prediction updates during object motion

**Question.** Can the minimum computational change let prediction updates during object motion stop forcing full FULL_SEARCH restart/cancellation cycles, so that a moving object does not imply waiting for it to stop before a usable provisional plan exists?

**Fixed.** T/G/R resolutions, cost, selector, terminal controller, novelty claim and all tolerances are unchanged. The checkpoint is preserved as tag `triad-v2-checkpoint-cancel-restart` (commit `7c7ea61`). The characterization evidence (`runs_ae1e824`) is untouched.

Evidence tags: **[M]** measured · **[C]** follows from code · **[I]** inference.

## 1. Answer

1. **Mechanism (implemented, selectable).** `fullSearchPredictionUpdate: select_then_certify` separates what a prediction change actually invalidates, using only existing protections and tolerances. It removes restart cycles triggered by prediction changes and never adopts a stale target. I1–I10 hold in every run.
2. **Effect (measured).** In these four scenarios it does **not** make a usable plan available earlier while the object moves:
   - longitudinal and lateral-low obtain their first plan only after the giver is at rest, under every variant;
   - near-ground and diagonal obtain it during motion under every variant, including the checkpoint.
3. **Why [M].** Restarts are not the binding constraint. The first search from the moving epoch takes 2.9–4.5 s, while the constant-velocity presentation lasts 3.67 s after that epoch. Once its result is received, either:
   - no timing-admissible complete action remains (longitudinal: 0 of 232 records admissible at receipt, in every run), or
   - the giver has started to decelerate. The targets of every still-admissible hypothesis then leave the 15 mm tube within 15–100 ms. A hypothesis takes 0.1–0.3 s to certify and a single action about 5 ms.

   The worker time spent before the first plan that did not produce it is 4.5–6.5 s in longitudinal and lateral-low, in every variant.
4. **Default restored to `cancel_and_restart`.** The new mechanism brought no gain in time to first plan. Variants of it lost 1–2 s at rest in longitudinal, and one variant failed runs that the checkpoint completes (§4). It remains available for further study.

## 2. Mechanism

Classification when the independent prediction changes while a FULL_SEARCH runs (`src/ReceiverV2.cpp`, *FULL_SEARCH prediction updates*):

| Class | Condition (existing tolerances) | Action |
|---|---|---|
| Unsafe / stale job | held arm left the snapshot start (reach tracking tolerances), or receiver state generation advanced | cancel the job (unchanged) |
| Obsolete job | no hypothesis that can still be admitted (event time − now ≥ the selector's minimum safe commit lead, 1.6 s) keeps its predicted target inside the commit-freshness tube (15 mm / 0.12 rad) | cancel; search a new epoch |
| Record whose target moved | certified presentation pose outside the tube at its event instant | kept; if the **unchanged selector** picks it, only that action is certified at the newest prediction (`CERTIFY_SELECTED`: the RECERTIFY_ACTIVE rollout from the held start, reach index 0); a failed certificate excludes the record and the selector runs again |
| Record within the tube | — | adopted exactly as before |

**Why this is defensible.** It uses the same guarantees V2 already relies on.
- A record inside the tube is exactly what the checkpoint accepts.
- A record whose target moved is adopted only with the single-action certificate that V2 already uses to retain an active plan whose target moved.
- Adoption logs `[V2AdoptFreshness]`.
- Checker invariant **I10** requires that every adoption is fresh, that every certified adoption cites a successful certificate, and that a refused record is never selected again. Mutation tests cover all three.
- **I9** now also covers adoption certificates.

**Commits.**
- `a272cbc`: select-then-certify.
- `05b5b4c`: record-index fix, found in testing. The selector returns an index into its filtered record list, and under exclusions that differs from the plan-set index.
- `73f05a5`: obsolete-job rule.

**Design steps rejected on evidence** (development runs, not in the comparison):
- **Re-queueing affected hypotheses inside the running search.** During deceleration the target of an active hypothesis left the tube before its 0.1–0.3 s certification finished, so work was aborted repeatedly. The fixed epoch let early events expire, and the first plan came 0.3 s *later* than with the checkpoint.
- **Re-targeting not-yet-started hypotheses** through a worker mailbox. No benefit, and the first search grew from 3.9 s to 4.6 s.

## 3. Comparison

Sources:
- `evidence/runs_05b5b4c`: cancel_and_restart ×2 and select_then_certify without the obsolete-job rule ×2.
- `evidence/runs_73f05a5`: final select_then_certify ×3, plus stale and supersession injections.

The table is `runs_73f05a5/analysis/COMPARISON_TABLE.md`, produced by `tools/analyze_v2_characterization.py supersession`. V1 FrozenPlanSet hashes are identical on both builds.

Metric definitions:
- *First adoption − rest*: first provisional adoption time minus the giver's scripted rest time; negative means obtained while the object moves.
- *Discarded work units*: bounded planner steps in cancelled searches or failed selected-action certificates.
- *Unused worker wall*: all worker time before the first plan other than the search (and certificate) that produced it, whether cancelled or completed but not used.
- *Concurrent motion*: 50 ms samples with robot and object both moving, before commit.

| Scenario | Variant | Completed | First plan before rest | First adoption − rest (s) | Searches / cancelled | Discarded units (median) | Unused wall before first plan (s) | Worker wall to commit (s) | Concurrent motion (s) |
|---|---|---:|---:|---|---|---:|---:|---:|---:|
| longitudinal | cancel_and_restart | 2/2 | 0/2 | 0.57, 0.57 | 74/71 | 179 770 | 4.49 | 7.05 | 0 |
| longitudinal | select_then_certify, no obsolete rule | 1/2 | 0/2 | 2.05, 2.60 | 10/0 | 0 | 6.21 | 8.74 | 0 |
| longitudinal | select_then_certify (final) | 3/3 | 0/3 | 2.15, 0.57, 1.76 | 35/29 | 123 469 | 5.69 | 8.30 | 0 |
| near-ground | cancel_and_restart | 2/2 | 2/2 | −1.38, −1.16 | 71/67 | 54 866 | 0 | 7.77 | 0 |
| near-ground | no obsolete rule | 2/2 | 2/2 | −1.39, −1.26 | 6/0 | 2 361 | 0 | 11.16 | 0 |
| near-ground | final | 3/3 | 3/3 | −1.41, −1.27, −1.44 | 33/27 | 33 997 | 0 | 7.81 | 0 |
| lateral-low | cancel_and_restart | 1/2 | 0/2 | 0.48, 0.45 | 76/73 | 297 468 | 4.49 | 7.45 | 0 |
| lateral-low | no obsolete rule | 2/2 | 1/2 | −0.01, 1.13 | 3/0 | 918 | 2.48 | 7.80 | 0 |
| lateral-low | final | 0/3 | 0/3 | 1.18, 0.72, 0.71 | 36/30 | 282 889 | 4.76 | 8.08 | 0 |
| diagonal | all three | 2/2, 2/2, 3/3 | all | −1.49 … −1.67 | 1 search each, 0 cancelled | 0 | 0 | 5.2 | 1.2 |

**Readings [M].**
- **Time to first plan while moving.** No variant obtains a plan before rest in longitudinal. In lateral-low, only one pure select-then-certify run does, 10 ms before rest: a certified far-lead action, with 13 of 14 selected-action certificates failing across the two runs. Near-ground and diagonal are unchanged.
- **Discarded versus unused work.** Not cancelling drives *discarded* work to about 0. The work is still *unused*: stale searches run to completion and are never adopted. Pure select-then-certify has the highest worker wall to commit (near-ground 11.2 s vs 7.8 s). The obsolete-job rule brings load back to the checkpoint's level; it cancels about 3× less often (every 50–100 ms instead of 15–40 ms during deceleration) but discards similar work.
- **Failures.**
  - Without the obsolete-job rule, two runs failed closed because a search stayed alive on stale targets: longitudinal (its search finished at 15.49 s with 0 records inside the tube, then the late plan failed) and the supersession-injection run (a search started at the end of deceleration never completed before the window closed).
  - With the rule, both pass: 3/3 longitudinal, injection completed.
  - Lateral-low failed 3/3 in the final variant against 1/2 for the checkpoint. Every failure adopts the same at-rest action (`axisN_side_337deg/direct`, lead 2.8 s) and loses it at `closure/right_inner_pad` during reach, the known start-state-dependent mechanism. It occurs after an at-rest adoption, which prediction handling does not change. Three runs cannot separate this from the base rate of about 1/3.
- **Concurrent motion.** Diagonal only, 1.2 s, identical in all variants.

## 4. Root cause, quantified [M]

Timeline: giver start 8.12 s; moving epoch (first full search) 9.02 s; deceleration begins 12.70 s; rest 13.55 s.

| Scenario | First search latency | At receipt | Consequence |
|---|---|---|---|
| diagonal | 2.9–3.0 s | 12.0 s, constant velocity, admissible plans | adopted while moving, all variants |
| near-ground | 3.1–3.4 s | 12.1–12.4 s, admissible plans | adopted while moving, all variants |
| longitudinal | 3.6–3.8 s | 12.6–12.8 s, **0 of 232 records timing-admissible** | the complete events (leads 2.35–5.95 s in the bank) expired during computation |
| lateral-low | 3.9–4.5 s | after deceleration is detected (≈12.92 s); every still-admissible target outside the tube within 30–80 ms | no fresh admissible record; certificates of far-lead stale records mostly fail (`reach_tracking`) |

- **Latency.** The window in which a moving-epoch result can still be adopted closes during the search itself (longitudinal), a matter of latency relative to timing admission.
- **Deceleration.** It reopens only when the target is stationary enough for the 15 mm tube (lateral-low). Under a constant-twist predictor, the target at an admissible horizon h ≥ 1.6 s + reach time moves by Δv·h. At the giver's deceleration (0.094 m/s²) that exceeds 15 mm within about 0.1 s, shorter than one hypothesis certification.

Neither limit depends on how prediction updates are scheduled.

## 5. Recommendations (not implemented)

1. **Search latency.** Getting a plan while moving in longitudinal/lateral-low requires the moving-epoch result to arrive while complete, admissible events remain. Two options exist:
   - evaluate hypotheses in ascending lead and publish them as they complete ("anytime" provisional selection);
   - reduce the per-hypothesis cost (§3 of `V2_COMPUTATION_AND_CANDIDATE_SPACE.md`: route reach and swept-volume/IK queries dominate).

   The first changes the domain over which the unchanged selector chooses, and interacts with V2's retain-while-certified rule. **It is a decision for the author, not a computational detail.**
2. **Keep the three-way classification as analysis instrumentation.** Its receipt logs showed where stale records come from. Keep `select_then_certify` selectable for runs with noisy, non-decelerating predictions, where far-lead drift without an overall target change would make it worthwhile **[I]**. Those runs are not part of this evidence.
3. **Evidence gap.** The giver model has one speed and a scripted stop. The constraint found here, a constant-velocity window of 3.67 s against a 3–4.5 s search, is specific to this script and bank size. Varying giver speed and stop time is required before generalizing.

## 6. Reproduce

```bash
TRIAD_RECEIVER_MODE=v2 scripts/run_scenario.sh lateral-low out/restart/lateral-low
TRIAD_RECEIVER_MODE=v2 TRIAD_EXTRA_OVERRIDE=select.yaml scripts/run_scenario.sh lateral-low out/select/lateral-low
#   select.yaml: configs: {HandoverInterceptionController_ReceiverV2: {fullSearchPredictionUpdate: select_then_certify}}
python3 tools/check_v2_run_log.py out/select/lateral-low/lateral-low.log
python3 tools/analyze_v2_characterization.py supersession out/*/*/*.log
```
