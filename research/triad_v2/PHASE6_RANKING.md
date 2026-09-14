# FINAL TRIAD — Phase 6: ranking of certified complete actions

**Question.** Among certified, timing-admissible complete actions, what should decide? The comparison is the current seven-term objective J7 against simpler rules. Its weights (T 8, E 2, L 2, C 3, Q 1.6, K 1.4, V 1)/19 have no derivation (Track 2 inventory; Phase 1 §H).

**Data.** Five characterization datasets of the same four scenarios, moving and rest epochs (40 search instances; they are *not* independent, since the scenarios repeat across datasets):
- A1 default bank;
- 64-angle G;
- 65-route R;
- 17 routes + time stretch;
- 191-lead T.

**Method.**
- Every complete record is joined with its logged cost terms. Recomputed J7 matches the logged motion objective within 4·10⁻⁵ (term rounding) in all 40 instances.
- All rankers see the same admissible set: zero-latency admission at the epoch while moving; the Phase 3 rest law (τ per record) at rest.
- **Tool:** `tools/ranking_study.py`. **Evidence:** `research/triad_v2/evidence/phase6_ranking/` (`ranking.json`, `RANKING_TABLES.md`).

**Metrics.** Completion C = time from the epoch until the action completes = (τ − t) − T_pres + T_exec, the quantity J7's time term measures.
- Completion regret = C − min C over the admissible set.
- Clearance = min(reach, carried-retreat) certified clearance.
- Clearance deficit = best clearance available within +0.5 s of the minimum completion, minus the selected clearance.

**Not measured here.** Completion rate and switching require in-situ runs. These are confounded by the Phase 2 recertification defect (deferred to Phase 8), so they are out of scope for this offline phase and noted as a Phase 7/11 obligation.

## 1. Results (pooled over 40 instances; per-search detail in `RANKING_TABLES.md`)

| ranker | same tuple as J7 | completion regret mean / max (s) | clearance min / median (mm) | clearance deficit vs best within +0.5 s: mean / max (mm) | J7 regret mean / max |
|---|---:|---|---|---|---|
| **J7** (current) | 40 | 0.21 / **1.38** | 70.6 / 80.3 | 1.4 / 5.0 | 0 / 0 |
| EARLIEST_TAU (τ, then completion, then clearance) | 9 | 0.03 / 0.58 | **47.9** / 75.6 | 7.6 / 34.3 | 0.033 / 0.133 |
| EARLIEST_DONE (completion, then clearance) | 10 | **0 / 0** | **47.9** / 75.6 | 7.9 / 34.3 | 0.031 / 0.133 |
| LEX(completion + 0.2 s → clearance → effort) | 7 | 0.05 / 0.19 | 47.9 / 80.2 | 3.5 / 34.3 | 0.023 / 0.133 |
| **LEX(completion + 0.5 s → clearance → effort)** | 6 | 0.30 / **0.50** | **73.1** / 80.8 | **0.0 / 0.1** | 0.037 / 0.388 |
| LEX(completion + 1.0 s → clearance → effort) | 5 | 0.62 / 0.98 | 75.0 / 81.2 | −0.6 / 0.0 | 0.053 / 0.388 |
| SAFE_EARLIEST (clearance ≥ 80 mm, then completion) | 12 | 1.15 / 5.04 | 79.0 / 80.7 | −1.7 / 1.2 | 0.088 / 0.528 |
| J_T+C (time and clearance terms only) | 14 | 0.07 / 0.47 | 73.1 / 80.2 | 0.6 / 3.7 | 0.039 / 0.396 |
| J7 without V | 28 | 0.09 / 1.38 | 70.6 / 80.2 | 1.4 / 5.0 | 0.003 / 0.018 |
| J7 without E, L | 18 | 0.26 / 1.37 | 70.6 / 80.4 | 1.4 / 7.1 | 0.003 / 0.030 |
| J7 without Q, K | 30 | 0.13 / 0.67 | 70.6 / 80.3 | 1.5 / 5.0 | 0.003 / 0.044 |

### J7 weight sensitivity

Halving or doubling one weight, with renormalisation, changes the selected tuple in:
- E×2: 21/40
- T×0.5 and L×0.5: 19/40 each
- Q×2 and V×2: 15/40 each
- L×2: 11/40
- C×0.5: 8/40
- K: 1/40

Consequences of these changes:
- T×0.5 delays completion by up to 1.52 s.
- C×0.5 lowers clearance by up to 26 mm.
- T×2 lowers clearance by up to 26 mm (while completing up to 1.38 s earlier).

### Where J7's regret comes from

- **1.38 s** (64-angle G, lateral-low moving): J7 picks a later event with the same grasp family; clearance changes by 4 mm.
- **0.96–1.05 s** (stretch data, rest): J7 picks a ×1.5 time-stretched direct route. That is a slower reach, and its lower effort, velocity and conditioning terms outweigh the time term (Phase 5 §2).

So J7 trades up to about 1 s of interception for effort and smoothness terms whose physical value in this task is not established.

## 2. Findings

1. **J7 provides no material benefit over simple rules on the measured axes** (MEASURED, offline).
   - Its worst completion regret (1.38 s) is larger than LEX(0.5)'s (0.50 s).
   - Its worst clearance (70.6 mm) is lower than LEX(0.5)'s (73.1 mm).
   - Its selection is fragile to unjustified weights: up to 21/40 changes under a single ×2.
   - Its only advantage is J7 itself, which is circular.
2. **Pure earliest completion is not safe enough as the only criterion.**
   - It picks 48–51 mm clearance actions in lateral-low (moving) where 73–82 mm actions complete ≤ 0.5 s later.
   - Both clearances are above the certification reserves, so this is a robustness trade-off, not a feasibility issue.
3. **A two-level lexicographic rule captures the useful part of J7 transparently.**
   - Completion within δ of the earliest, then maximum certified clearance, then minimum effort.
   - With δ = 0.5 s, it bounds completion regret by construction.
   - It has the best clearance floor of all rules that bound regret, and zero clearance deficit.
4. **Dropping single terms from J7** (V, E+L, Q+K) changes little. That is consistent with the time and clearance terms carrying the decision (J_T+C: regret ≤ 0.47 s, clearance ≥ 73.1 mm).

## 3. Decision (subject to in-situ confirmation)

**Replace J7 by a lexicographic ranking:**

```
Among certified, timing-admissible complete actions A:
  C_min = min_{a∈A} C(a)
  B     = { a ∈ A : C(a) ≤ C_min + δ }
  select argmax_{a∈B} clearance_min(a); ties → min effort; ties → deterministic order
```

**Why δ = 0.5 s is provisional.**
- δ = 0.5 s is EMPIRICAL: it is the smallest tested band with zero clearance deficit against the +0.5 s best.
- δ = 0 (earliest completion) and δ = 0.5 s differ only in the clearance–time trade-off: ≤ 0.5 s later for up to +34 mm clearance.
- Choosing between them needs outcome evidence (completion, runtime clearance, post-commit failures). Offline data cannot supply that.
- Given Phase 2's measured clearance optimism of the preview (up to 17 mm reach, 13 mm retreat), a larger certified clearance has plausible value. The claim stays INFERENCE until Phase 7 measures it.

**Derivable alternative to a soft band.** A hard clearance floor c_floor = runtime reserve (8 mm) + measured preview optimism (17 mm) = 25 mm, then earliest completion.
- In the current data every admissible record exceeds 47 mm, so this floor is inactive and reduces to EARLIEST_DONE.
- It is recorded as the DERIVED safety floor that must hold regardless of δ.

**Consistency with Phases 3–5.**
- Completion time is non-decreasing in τ at fixed (g, r). The Phase 3 rest law therefore remains lossless.
- For δ > 0, the moving ladder window W must be ≥ δ, and the Phase 5 route law must still evaluate alternatives within the band when direct succeeds.
- With δ = 0, the Phase 3 W = 0 and the Phase 5 direct-first rule are exact.

## 4. Parameter provenance

| symbol | value | classification |
|---|---|---|
| seven J7 weights, T_ref, soft clearance, soft joint margin, soft condition index | inherited | **REMOVED** (no material benefit; fragile) |
| completion time C | — | DERIVED (the time-to-completion of the certified action) |
| clearance_min = min(reach, retreat) | — | DERIVED (certified hard-stage quantities) |
| δ | 0.5 s (or 0) | EMPIRICAL / to be decided in situ (Phase 7) |
| c_floor | 25 mm | DERIVED from runtime reserve 8 mm + Phase 2 parity 17 mm; inactive in current data |

## 5. Limits

- Offline selection over overlapping datasets of four scenarios. No completion or switching outcomes yet.
- Effort is used only as a final tie-break. Its physical meaning (joint-velocity integral) is not validated as a preference.
- J7's timing audit is still what makes a record cost-valid (A1). Removing the J7 weights does not remove that certification stage.
