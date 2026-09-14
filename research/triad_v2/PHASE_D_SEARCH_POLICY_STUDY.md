# TRIAD V2 Phase D: first-action latency and search-policy study

**Question.** Is the exhaustive FULL_SEARCH scientifically necessary, or can a bounded/incremental search expose a complete-action-certified action substantially earlier without materially sacrificing feasibility or decision quality?

**Scope.** Offline replay only. No runtime search policy was implemented.

**Unchanged.** T/G/R banks, hard checks, seven-term objective, selector, terminal controller, prediction model, commitment semantics and V1 are all unchanged.

Evidence tags: **[M]** measured · **[C]** follows from code · **[I]** inference.

## A. Freeze

- **Checkpoint.** Tag `triad-v2-checkpoint-phase-d-start` = `dceaccc` (default `fullSearchPredictionUpdate: cancel_and_restart`). Earlier tags and all evidence directories are untouched.
- **Source changes, logging only** (commit `16fbdde`):
  - `[CertStage]` carries worker wall time and cumulative bounded work units since job start, a per-stage count/wall snapshot, and for complete records the motion cost, the selector's global objective (`extendMotionCostToSearchEpoch`), and presentation and execution durations.
  - Memoized hypotheses log one `path=memo` record each.
  - `[V2FullSearchSelection]` logs selector wall time and the selected tuple.
  - V2 worker only; nothing reads these values. One call line was added inside the existing V2-only memo branch of `HandoverInterceptionController.cpp`.
- **V1 regression [M].** The four FrozenPlanSet sha256 hashes are identical (198/283/432/233 records) on the logging build.
- **Tools.**
  - `tools/replay_search_policies.py` replays the policies over logged record availability, with an exact Python replica of `selectFiniteEventPlan`. Replayed FULL_ARGMIN matches the controller's logged selection in **all** replayed searches: 16 characterization searches and 220 in-situ jobs, 0 mismatches.
  - `tools/summarize_search_policy_replay.py` produces the tables.
- **Evidence.** `evidence/runs_phaseD_16fbdde`:
  - V1 regression ×4;
  - `characterizeOnly` ×2 per scenario (full moving-epoch search plus full at-rest search, no cancellation);
  - in-situ V2 ×2 per scenario.
  - Repeats choose identical records and tuples; wall times agree within about 10%.

**Method details.**
- The existing enumeration (ascending lead, then grasp index, then route) is already temporal order. **No ordering was changed.**
- A record becomes available when its route rollout, or memo reuse, is logged.
- Decision time is job submission time plus worker wall; latency and worker wall agree to about 1 ms.
- Timing admission is the selector's: τ − t ≥ 1.6 s and presentation duration + 0.05 s ≤ τ − t.
- Two references:
  - **FULL_ARGMIN (as executed)**: exhaustive argmin at its own receipt time;
  - **zero-latency argmin**: exhaustive argmin with every record admitted at the search epoch, i.e. the decision exhaustive search would make if computation were free.
- Transfer readiness has no candidate-dependent test in the certifier. Every selected record is complete through carried retreat (deepest stage CARRIED_RETREAT) by construction.

## B/C. Results

### Concise comparison: moving-epoch searches (the latency-critical case)

8 searches (4 scenarios × 2 repeats); `analysis/REPLAY_char.md`.

| Policy | Admissible plan & before reach-start deadline | Median wall to plan (s) | Median t_plan − t_rest (s) | Tuple = zero-latency argmin | Median / max ΔJ vs zero-latency argmin | Median / max ΔJ vs FULL_ARGMIN as executed | Median work avoided |
|---|---|---:|---:|---|---|---|---:|
| FULL_ARGMIN (as executed) | 6/8 (longitudinal: none admissible at receipt) | 3.21 | −1.31 | 0/6 | +0.254 / +0.283 | 0 | 0 |
| FIRST_COMPLETE_FEASIBLE | 4/8 (τ = 1.8 s records not admissible) | 0.12 | −4.41 | 2/8 | +0.025 / +0.150 | −0.105 / −0.044 | 0.95 |
| FIRST_ADMISSIBLE_COMPLETE (extra) | 8/8 | 0.61 | −3.91 | 2/8 | +0.173 / +0.502 | +0.042 / +0.219 | 0.74 |
| EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 8/8 | 0.81 | −3.71 | **6/8** | **0.000** / +0.349 | −0.093 / +0.066 | 0.69 |
| ANYTIME_INCUMBENT (first incumbent) | 8/8 | 0.61 | −3.91 | 2/8 | +0.173 / +0.502 | +0.042 / +0.219 | 0.74 |

**At-rest searches** (8): memoized exact-pose reuse makes the full search cost 0.16–0.57 s. **No bounded policy saves time** (0% of work avoided). EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR differs from executed FULL_ARGMIN by ΔJ ≤ +0.047, and FIRST_COMPLETE_FEASIBLE is never admissible (0/8).

### Per scenario: moving-epoch search

Characterization run a; run b and the in-situ runs give identical tuples.

| Scenario | Policy | Wall to plan (s) | t − t_rest (s) | Admissible | τ lead (s), g, r | J | ΔJ vs executed / zero-latency | Reach / retreat clearance (m) | Hypotheses / grasp screens / routes | Work avoided |
|---|---|---:|---:|---|---|---:|---|---|---|---:|
| longitudinal | FULL_ARGMIN | 3.79 | — | **none** | — | — | — | — | 14/448/595 | 0 |
| longitudinal | FIRST_COMPLETE | 0.17 | −4.36 | yes | 2.35, axisP_337, direct | 0.621 | n/a / 0 | 0.079 / 0.210 | 3/80/1 | 0.91 |
| longitudinal | EARLIEST_TAU | 0.39 | −4.14 | yes | 2.35, axisP_337, direct | 0.621 | n/a / **0** | 0.079 / 0.210 | 3/96/34 | 0.87 |
| near-ground | FULL_ARGMIN | 3.20 | −1.32 | yes | 8.00, axisP_247, ring140_5 | 1.037 | 0 / +0.283 | 0.082 / 0.088 | 14/448/408 | 0 |
| near-ground | FIRST_COMPLETE | 0.02 | −4.51 | **no** | 1.80, axisP_45, direct | 0.681 | — | 0.074 / 0.108 | 1/3/1 | 0.99 |
| near-ground | FIRST_ADMISSIBLE | 2.65 | −1.87 | yes | 6.40, axisP_68, direct | 1.255 | +0.219 / +0.502 | **0.030** / 0.049 | 12/356/341 | 0.19 |
| near-ground | EARLIEST_TAU | 2.78 | −1.74 | yes | 6.40, axisP_68, ring80_2 | 1.102 | **+0.066** / +0.349 | **0.047 / 0.049** | 12/384/357 | 0.14 |
| lateral-low | FULL_ARGMIN | 4.64 | **+0.12** | yes | 8.00, axisN_23, ring80_6 | 0.932 | 0 / +0.254 | 0.069 / 0.099 | 14/448/731 | 0 |
| lateral-low | FIRST_COMPLETE | 0.08 | −4.44 | **no** | 1.80, axisN_68, ring140_2 | 0.827 | — | 0.067 / 0.109 | 1/20/12 | 0.98 |
| lateral-low | FIRST_ADMISSIBLE | 0.84 | −3.69 | yes | 3.70, axisN_45, ring80_1 | 0.974 | +0.042 / +0.296 | 0.078 / 0.105 | 6/179/122 | 0.76 |
| lateral-low | EARLIEST_TAU | 1.13 | −3.39 | yes | 3.70, axisN_337, direct | 0.678 | −0.254 / **0** | 0.082 / 0.077 | 6/192/170 | 0.69 |
| diagonal | FULL_ARGMIN | 2.82 | −1.70 | yes | 5.50, axisP_337, ring80_2 | 0.775 | 0 / +0.093 | 0.074 / 0.223 | 14/448/408 | 0 |
| diagonal | FIRST_COMPLETE | 0.37 | −4.15 | yes | 4.15, axisP_337, direct | 0.731 | −0.044 / +0.049 | 0.070 / 0.217 | 7/208/1 | 0.73 |
| diagonal | EARLIEST_TAU | 0.47 | −4.05 | yes | 4.15, axisP_337, ring140_2 | 0.682 | −0.093 / **0** | 0.072 / 0.218 | 7/224/17 | 0.70 |

**In situ [M]** (`analysis/REPLAY_insitu_key.md`):
- The same tuples appear. In lateral-low and longitudinal (run a) the real moving search was **cancelled at 3.89 s** (giver deceleration) having exposed nothing to the receiver. EARLIEST_TAU had an admissible plan at t = 10.20 s and 9.43 s respectively.
- The first in-situ provisional plans were adopted at t − t_rest = −1.56 s (diagonal), −1.28 s (near-ground), +0.48 s (lateral-low), +0.54 s (longitudinal).

**ANYTIME_INCUMBENT evolution [M]** (moving searches; incumbent J at 10/25/50/75/100% of the full wall):
- **longitudinal:** 0.621 (τ 2.35) → 0.663 → 0.722 → 0.817 → none. The incumbent degrades as earlier events expire.
- **lateral-low:** none → 0.678 → 0.722 → none → 0.932. The incumbent was lost to expiry 6 times.
- **diagonal:** none → 0.682 → 0.691 → 0.720 → 0.775.
- **near-ground:** none until 83% of the search, then 1.037.

**More computation makes the executed decision worse in every moving scenario.** The objective favours early events and those expire while the search runs.

## D. Does exhaustive argmin behave like "earliest timing-admissible τ, then best (g, r)"?

| Comparison | Moving epoch | At rest | All |
|---|---|---|---|
| **Zero latency** (both policies admit records at the search epoch): P(τ equal) / P(g) / P(r) / P(tuple) | 6/8, 6/8, 6/8, 6/8 | 6/8, 8/8, 6/8, 6/8 | **12/16, 14/16, 12/16, 12/16** |
| **As executed** (FULL_ARGMIN at receipt vs EARLIEST_TAU at its stop), where both exist | 0/6, 2/6, 0/6, 0/6 | 4/8, 8/8, 4/8, 4/8 | 4/14, 10/14, 4/14, 4/14 |

**Answer [M].**
1. **The audit observation is supported as a statement about the exhaustive decision rule.** With latency removed, the exhaustive argmin *is* "earliest admissible τ, then best (g, r)" in 12 of 16 searches. All four exceptions are near-ground; there the argmin prefers a later, cheaper τ (moving: 3.25 over 2.35–2.8 s; at rest: 3.25 over 2.35 s).
2. **It does not describe the controller as executed.** In moving-epoch searches the executed exhaustive decision never matches EARLIEST_TAU's τ (0/6). Its own 2.8–4.6 s latency expires the events the rule would have chosen, and it selects later events with higher J (+0.09 to +0.28 against its own zero-latency decision).

## E. Where the computation goes

**Full moving-epoch search** (`analysis/FULL_SEARCH_BREAKDOWN.md`, char_a):

| Scenario | Wall (s) | Grasp/static reach screening | Insertion/closure | Route (transit reach) certification | Carried retreat | Cost / terminal timing audit | Hypothesis setup | Static screens / route rollouts |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| longitudinal | 3.79 | 17% | 10% | **64%** | 4% | 4% | 0% | 448 / 595 |
| near-ground | 3.21 | 16% | 15% | **57%** | 4% | 6% | 0% | 448 / 408 |
| lateral-low | 4.64 | 7% | 10% | **73%** | 4% | 6% | 0% | 448 / 731 |
| diagonal | 2.82 | 19% | 12% | **56%** | 6% | 6% | 0% | 448 / 408 |

Nested inside these phases: IK integration steps are 32–45% and swept-volume clearance queries 37–48% of search time (measured on near-ground, lateral-low and diagonal). Selection overhead is 0.012 ms median, 0.024 ms max, for 200–600 records.

**Per candidate [M].**
- A static grasp screen costs 0.4–2.0 ms (median; rejected screens are the cheaper).
- A route rollout costs 3.3–4.5 ms when rejected and 5.8–6.6 ms when complete through carried retreat.
- The first complete certified record appears after 15–400 ms. Its successful route costs 5–9 ms; the rest is 2–207 failed static screens.

**Before the first *admissible* plan (EARLIEST_TAU)** the time goes to evaluating τ that cannot be admitted, not to certifying candidates:
- **near-ground:** τ = 1.8–2.8 s have 31–43 complete records each, but presentation durations exceed the lead. Certifying them takes 1.43 s; by then τ = 3.25–5.05 s have negative slack. The first admissible τ is 6.4 s at 2.78 s (`analysis/HYPOTHESIS_SLACK.md`).
- **lateral-low:** τ = 1.8–3.25 s cost 0.80 s for the same reason.

**Latency is therefore caused by N_τ × N_g static screening plus N_r route certification of every statically feasible (τ, g)** (N_r accounts for 56–73% alone), and specifically by certifying events that are geometrically feasible but can never be timing-admissible. It is not caused by the cost of certifying an individual candidate.

## F. Verdict

**F3 (candidate certification is the bottleneck): rejected [M].** A complete certified action costs ≤ 9 ms of route certification plus a few ms of screening; the first one appears within 15–400 ms.

**F2 (exhaustive search materially changes decisions for the better): not supported as a general conclusion [M].**
- Against the zero-latency argmin, exhaustive search adds information over "earliest admissible τ, best (g, r)" in 2 of 8 moving searches (near-ground).
- As executed, its latency makes decisions worse in 4 of 4 moving scenarios (ΔJ +0.09 to +0.28), and in longitudinal it yields no admissible plan at all.
- **Counterexample:** near-ground moving. The executed exhaustive search finds τ = 8.0 s with J = 1.037 and reach/retreat clearance 0.082/0.088 m. EARLIEST_TAU stops at τ = 6.4 s with J = 1.102 (+0.066) and clearance 0.047/0.049 m (−35 / −39 mm). There, continuing the search found a cheaper and safer action, only 0.43 s later.

**F1 (bounded search strongly supported): supported for EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR in 3 of 4 scenarios, but not unconditionally.**
- **longitudinal, lateral-low, diagonal:**
  - a complete, admissible plan in 0.33–1.17 s instead of 2.8–4.6 s or none;
  - 3.4–4.2 s before the object rests;
  - tuple identical to the zero-latency exhaustive argmin (ΔJ = 0, identical clearances);
  - better J than exhaustive as executed (−0.09, −0.25) or a plan where exhaustive had none.
- **near-ground:** material cost and clearance loss (above), with little time gained (2.78 vs 3.20 s).
- **Other stopping rules fail the feasibility or quality requirement:**
  - FIRST_COMPLETE_FEASIBLE is inadmissible in 12 of 16 searches;
  - FIRST_ADMISSIBLE_COMPLETE and the first ANYTIME incumbent incur ΔJ up to +0.50 and reach clearance as low as 30 mm.
- **Scale:** four deterministic scenarios (repeats are identical), so there are four independent moving-epoch decisions.

**Decision gate G: not triggered.** The result is not a *very clear* F1, because of the near-ground counterexample and n = 4. No runtime anytime/bounded search was implemented.

### Exploratory, reported separately: exact timing prune (no ordering change)

This is a certification-order change, so it is reported apart from the four required policies. It is not implemented.

**Rule.** After a grasp's static screen at time t, skip its route rollouts at event τ if τ − t < 1.6 s or static reach time + 0.05 s > τ − t.

**Why it is exact.**
- A route's presentation duration is max(2·dt, static reach time) × max(1, path stretch) ≥ static reach time **[C]**, verified on all 4 973 logged complete route records **[M]**.
- Admissibility only decreases with time.
- So no skipped rollout could ever be admitted at any later decision. Replayed on moving searches only, where no memo reuse occurs, by removing the measured wall and work of skipped rollouts (`analysis/REPLAY_char_exact_timing_prune.md`).

**Effect [M, replay estimate].**
- It removes 0 / 0.40–0.48 / 0.66 / 1.41–1.43 s of route certification (diagonal / longitudinal / lateral-low / near-ground).
- **EARLIEST_TAU + prune** reaches an admissible plan in **0.28–0.63 s in all 4 scenarios**, 3.9–4.2 s before rest.
- Tuple equals the zero-latency argmin in 3 of 4 scenarios. Near-ground selects τ = 3.7 s, ΔJ +0.042 vs the zero-latency argmin, clearance 0.065/0.101 m vs 0.071/0.106 m.
- **FULL_ARGMIN + prune** still takes 1.8–4.0 s and, as executed, selects later events with ΔJ +0.09 to +0.25 vs zero latency.
- This estimate ignores scheduling and logging overhead. It is not a runtime measurement.

## Failures and counterexamples

1. **near-ground moving:** early stop loses 0.066 in J and 35–39 mm of clearance against exhaustive as executed (above).
2. **Non-admissible early records:** FIRST_COMPLETE_FEASIBLE returns τ = 1.8 s records that cannot be admitted in 12 of 16 searches (all 8 at rest; near-ground and lateral-low moving).
3. **Anytime incumbents lost to expiry:** 6 times in lateral-low and 2 in longitudinal; the incumbent can disappear, not only improve.
4. **At rest, no bounded policy saves time**, because exact-pose memoization already makes the full search ≈ one hypothesis. EARLIEST_TAU costs up to ΔJ +0.047 and 23 mm reach clearance (near-ground at rest).
5. **Lateral-low in situ failed** both runs through the known start-state-dependent closure mechanism, after an at-rest adoption. That is unrelated to this study.
6. **Replay limits:** the replay cannot show what happens after an early moving adoption (recertification during deceleration, terminal gate). Availability and certification are measured; downstream success is not.

## Recommendation for the next scientific step

1. **Author decision before any runtime change.** The evidence supports evaluating "earliest admissible τ, then best (g, r)" as a runtime candidate. It does not support calling it better unconditionally: near-ground shows that stopping can lose cost and clearance.
2. **Broaden the evidence first.** Vary giver speed, stop time and start pose so that P(τ equal) and the ΔJ/clearance loss distribution are estimated over many independent decisions, not four.
3. **Measure the exact timing prune in a logging-only runtime ablation before adopting it.** It is lossless by construction for any decision taken after the prune, removes the dominant waste (certifying timing-dead events), and makes the near-ground counterexample small (ΔJ +0.042).
4. **If a runtime anytime/bounded search is later approved**, it must satisfy the gate-G conditions:
   - motion only from a complete-action-certified plan;
   - exhaustive mode retained as reference;
   - no cost-driven switching (retain while certified);
   - stale-generation protections unchanged.
5. **Do not choose new N_τ, N_g, N_r from this study.** The latency is dominated by N_r × (statically feasible τ, g) and by timing-dead events, which a prune or stopping rule addresses without changing resolution.

## Reproduce

```bash
TRIAD_RECEIVER_MODE=v2 TRIAD_EXTRA_OVERRIDE=evidence/runs_phaseD_16fbdde/overrides/char_default.yaml \
  scripts/run_scenario.sh near-ground out/char/near-ground
python3 tools/replay_search_policies.py --json out/replay.json out/char/near-ground/near-ground.log
python3 tools/replay_search_policies.py --exact-timing-prune --json out/replay_prune.json out/char/near-ground/near-ground.log
python3 tools/summarize_search_policy_replay.py out/replay.json
```
