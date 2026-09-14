# Track 1: runtime exact timing prune of route certification

**Checkpoint.** `triad-v2-checkpoint-phase-e-start` (`f0acc46`). Implementation commit `7b312d6`. Evidence: `evidence/runs_prune_7b312d6`.

**Setting.** `configs.HandoverInterceptionController_ReceiverV2.fullSearchExactTimingPrune`, **default `false`**.

**Unchanged.** 14×32×17 bank, seven-term cost, selector, prediction model, complete-action stages, terminal controller, commitment semantics and V1. The V1 FrozenPlanSet hashes are identical on this build.

Tags: **[M]** measured · **[C]** from code · **[P]** proved.

## 1. Rule

In FULL_SEARCH, after the static screen of grasp g at event τ succeeds at worker step k, route certification of (τ, g) is skipped if

  τ_same − t_k + ε < L_commit   **or**   T_s + L_entry > τ_same − t_k + ε,   ε = 10⁻¹².

**Symbols.**
- **T_s** = `timingArmScale × static reach-to-standoff duration`: the static candidate's `predictedPresentationTime` (cpp `finishCurrentPlanningCandidate`).
- **L_commit** = max(`minimumCommitRemainingTime`, `presentationDecelerationDuration` + 0.25) = 1.6 s.
- **L_entry** = `minimumReachEntryLead` = 0.05 s.
- Both are the selector's own arguments (rv2 `adoptFullSearchResultV2`).
- **t_k** is the controller clock the control thread last published to the worker (`v2ControllerClockForWorker_`, written at the top of every `stepReceiverV2` cycle).
- **τ_same** is the latest event in the frozen bank whose presentation pose is *bitwise identical* to τ's. Exact memoization reuses a hypothesis's route results for such events; while the object moves, τ_same = τ.

Code: rv2 `exactTimingPruneRoutesV2`; call site cpp `finishCurrentPlanningCandidate`. The pruned candidate is completed as infeasible with reason `v2_exact_timing_prune` and logged `[V2TimingPrune]`.

## 2. Why no admissible route can be removed

**Selector gate [C]** (`src/FiniteEventPlanSelector.h`, `selectFiniteEventPlan`). A record with event time τ and presentation duration T_pres is admissible at decision time t_d iff

 (A) τ − t_d + ε ≥ L_commit  and  (B) T_pres + L_entry ≤ τ − t_d + ε.

**Premise [C].** For every route r of (τ, g):

- **Plan reach duration.** `beginPredictiveRouteCandidate` computes base = max(2·dt, T_s) and reachDuration = base × max(1, stretch). `makeInterceptionPlan` stores plan.reachDuration = max(2·dt, reachDuration).
- **Presentation duration.** `finalizePredictiveRouteCandidate` sets predictedPresentationTime = plan.reachDuration (cpp 6995), and no other code writes a route candidate's `predictedPresentationTime` (grep).
- **Selector input.** The selector reads `record.predictedPresentationDuration = candidate.predictedPresentationTime`.
- Therefore **T_pres(r) ≥ max(2·dt, T_s) ≥ T_s**.

**Proof [P].** Let t_d be any time at which a record of (τ', g) with pose(τ') = pose(τ) is selected.

1. The record cannot exist before its route certification, which would start after step k. The controller clock is monotone, and t_k was published before step k. Hence t_d ≥ t_k.
2. τ' ≤ τ_same by definition.
3. So τ' − t_d ≤ τ_same − t_k.
4. If the first disjunct holds: τ' − t_d + ε ≤ τ_same − t_k + ε < L_commit, so (A) fails.
5. If the second holds: T_pres + L_entry ≥ T_s + L_entry > τ_same − t_k + ε ≥ τ' − t_d + ε, so (B) fails.
6. In both cases every route of (τ', g) is inadmissible at every possible decision time, so removing it cannot change any selection. ∎

**Scope of the proof.** It covers selection by `selectFiniteEventPlan` (both provisional adoption and characterization). It does not claim anything about downstream execution: provisional motion begins at a different time, see §4.

**Runtime verification [M].**
- Every complete route record is checked for T_pres ≥ T_s (`[V2TimingPrunePremiseViolation]`). **0 violations over 9 093 complete route records** in this campaign.
- Checker invariant **I11** requires:
  - no premise violation;
  - no route certified for a pruned (generation, hypothesis, grasp);
  - no adoption from a pruned (generation, hypothesis, candidate);
  - every `[V2TimingPrune]` line satisfying its own inequality with τ_same ≥ τ.
- Mutation tests cover three violations. **I11 PASS on all 32 V2 logs.**

**Search-level losslessness [M]** (`analysis/SEARCH_LOSSLESS.txt`, `tools/compare_timing_prune.py search`). Characterization logs, prune off vs on, same scenario, 8 searches (4 moving + 4 at rest). At the prune-on receipt time, the set of admissible records, their J, and the selector's choice are **identical in 8/8** (0 missing, 0 extra, 0 J differences).

## 3. Search-level effect (characterization, moving-epoch search)

| Scenario | Pruned (τ, g) | Search wall off → on (s) | Complete records off → on | Admissible at on-receipt (both) |
|---|---:|---|---|---:|
| near-ground | 14 | 3.27 → **1.77** | 256 → 80 | 28 |
| lateral-low | 7 | 4.69 → 3.95 | 382 → 310 | 37 |
| longitudinal | 6 | 3.93 → 3.35 | 232 → 227 | 9 |
| diagonal | 0 | 2.87 → 2.86 | 267 → 267 | 176 |

At-rest searches: 0 prunes, because memoized events up to 8 s share the pose, and wall is unchanged (0.15–0.53 s).

**Stage counts in FULL_SEARCH jobs up to the first provisional plan [M]** (median over 3 in-situ repeats; `analysis/RUNS_COMPARISON.md`):

| Scenario | Prune | Static screens (NONE / REACH / INSERTION / CLOSURE_CONTACT / CARRIED_RETREAT) | Pruned (τ, g) | Route rollouts | Complete route records |
|---|---|---|---:|---:|---:|
| near-ground | off | 448 (307 / 50 / 62 / 5 / 24) | 0 | 408 | 242 |
| near-ground | on | 448 (307 / 50 / 62 / 5 / 24) | 14 | **170** | 66 |
| longitudinal | off | 716 (309 / 11 / 320 / 13 / 63) | 0 | 756 | 317 |
| longitudinal | on | 448 (227 / 11 / 164 / 11 / 35) | 6 | 510 | 227 |
| lateral-low | off | 922 (505 / 0 / 356 / 13 / 48) | 0 | 696 | 378 |
| lateral-low | on | 448 (248 / 0 / 146 / 11 / 43) | 7 | 612 | 310 |
| diagonal | off / on | 448 (247 / 0 / 160 / 17 / 24) | 0 | 408 | 267 |

The prune removes only route rollouts. The static screen (F1 through the static retreat) is unchanged per hypothesis. Off counts for longitudinal and lateral-low include the later at-rest search, because the first plan came from it.

## 4. In-situ before/after

4 scenarios × 3 repeats, default `cancel_and_restart` mode; `analysis/RUNS_COMPARISON.md`.

| Scenario | Prune | Completed | First plan before giver rest | First adoption − rest (s) | Worker wall to first plan (s, median) | Work units to first plan (median) | First tuple (lead, g, r), J | Reach / retreat clearance (m) | Concurrent motion (s) |
|---|---|---:|---:|---|---:|---:|---|---|---:|
| near-ground | off | 3/3 | 3/3 | −1.44, −1.45, −0.88 | 3.09 | 242 458 | 8.00, axisP_247, ring140_5, 1.037 (×3) | 0.082 / 0.088 | 0 |
| near-ground | on | 3/3 | 3/3 | **−2.69, −2.64, −2.43** | **1.88** | 172 004 | 5.05, axisP_45, ring80_1, **0.949** (×2); 8.00 … 1.037 (×1) | 0.053 / 0.090 (×2) | 0 |
| longitudinal | off | 3/3 | 0/3 | +0.52, +0.50, +0.60 | 5.01 | 358 336 | 2.80, axisP_337, direct, 0.650 (×3) | 0.080 / 0.221 | 0 |
| longitudinal | on | **1/3** | **2/3** | −1.24, −1.28, +0.60 | 3.28 | 244 695 | 5.95, axisP_337, ring80_1, 0.852 (×2); 2.80 … 0.650 (×1) | 0.081 / 0.177 (×2) | 0.95 |
| lateral-low | off | 2/3 | 0/3 | +0.46, +0.46, +0.51 | 4.95 | 329 609 | 2.80, axisN_337, direct, 0.622 (×3) | 0.082 / 0.081 | 0 |
| lateral-low | on | **1/3** | **2/3** | −0.75, +0.56, −0.72 | 3.80 | 232 035 | 6.85, axisN_45, ring80_6, 0.896 (×2); 2.80 … 0.622 (×1) | 0.064 / 0.100 (×2) | 0.40 |
| diagonal | off | 3/3 | 3/3 | −1.69, −1.64, −1.42 | 2.88 | 222 160 | 5.50, axisP_337, ring80_2, 0.775 (×3) | 0.074 / 0.223 | 1.20 |
| diagonal | on (0 prunes) | 3/3 | 3/3 | −1.76, −1.73, −1.79 | 2.77 | 222 160 | 5.05, axisP_315, ring80_2, 0.770 (×2); 5.50 … 0.775 (×1) | 0.077 / 0.211 (×2) | 1.50 |

**Totals.**
- Completion: off 11/12, on 8/12.
- First plan while the object moves: off 6/12, on 10/12.
- Worker wall until commit, or until log end for failed runs (median over the scenario medians): off 7.0 s, on 6.9 s (per run in `RUNS_COMPARISON.md`).

**What changed, and why [M].**
1. **near-ground: earlier and cheaper first plan, completion unchanged.** Pruning 13–14 timing-dead (τ, g) cuts the moving search from 3.1 to 1.9 s. The plan arrives 1.2 s earlier, and J falls from 1.037 to 0.949 because the earlier receipt still admits τ = 5.05 s. Reach clearance falls from 82 to 53 mm (still above the 20 mm hard margin). The committed tuple is identical (axisP_45 / ring140_7).
2. **longitudinal: new failures (on_a, on_b).**
   - The pruned moving search completes at 12.26–12.31 s, before the prediction is superseded at about 12.9 s, with 9 admissible records.
   - It adopts τ = 5.95 s: a target on the constant-velocity extrapolation, which the scripted giver never reaches because it stops at 13.55 s.
   - The arm moves (0.95 s concurrent with the object).
   - During deceleration, recertification fails (`reach_tracking`, 13.66 s). The arm needs 2.3 s to settle below 2.5 mm/s.
   - From that arm state every subsequent FULL_SEARCH finds **0 complete actions** (0.13 s each), and the 7 s window closes.
   - on_c's search took 3.57 s, had no admissible plan, and behaved like prune-off.
3. **lateral-low: two failures on vs one off.**
   - on_a and on_c adopt τ = 6.85 s during motion; recertification fails at 13.12 s (`reach_tracking`).
   - The replacement adopted at rest then fails through the known start-state-dependent closure mechanism (`pad_pair`, `right_pad_shoulder_low`). The same final failure class occurs in the prune-off run off_c.
   - on_b's prune-on search ran 3.9 s, was cancelled like prune-off, and completed.
4. **diagonal: no prunes.** The tuple change in on_a and on_c comes from worker-latency jitter. A 2.77 s search, instead of 2.84 s, is received 70 ms earlier, when τ = 5.05 s is still admissible (196 vs 176 admissible plans). That plan later needed one replacement (retreat IK) and still completed.

**Interpretation.**
- The prune is lossless for **selection** [P, M].
- It is not neutral for **outcome**. Making a certified plan available before deceleration lets V2 start provisional motion toward a far target predicted under constant velocity. When the giver stops, that plan is invalidated, and the arm state it leaves can have no certifiable complete action (longitudinal) or leads into the known closure failure (lateral-low).
- The failures are caused by the interaction of early provisional motion, a constant-twist prediction, and the no-recovery hold, not by any lost admissible action.
- Three repeats per arm and four deterministic scenarios are not enough to estimate rates.

## 5. Verdict and recommendation

1. **Keep the prune implemented but default off.** It is provably lossless for selection and removes 0–1.5 s of route certification. But in these scenarios it converted earlier availability into worse completion (8/12 vs 11/12).
2. **The newly exposed question is scientific, not computational:** should provisional motion start from a plan whose target lies on an unconfirmed constant-velocity extrapolation? It maps directly onto inventory ablations A4 (commitment timing / provisional motion onset) and A6 (prediction sensitivity). A defensible next experiment runs the prune-on configuration with a motion-onset condition that is explicit and pre-registered (e.g. start provisional reach only when τ is within the observed motion horizon). That is an architectural decision for the author, not part of this track.
3. **No tolerance or bank change was made to recover the lost completions.**

## Reproduce

```bash
TRIAD_RECEIVER_MODE=v2 TRIAD_EXTRA_OVERRIDE=evidence/runs_prune_7b312d6/overrides/prune_on.yaml \
  scripts/run_scenario.sh longitudinal out/on/longitudinal
python3 tools/check_v2_run_log.py out/on/longitudinal/longitudinal.log      # I11
python3 tools/compare_timing_prune.py search OFF_CHAR.log ON_CHAR.log        # losslessness
python3 tools/compare_timing_prune.py runs out/off_a out/off_b --on out/on_a out/on_b
```
