# TRIAD V2 — evidence (source `d9e8886`)

All runs come from one build of commit `d9e8886` (`runs_d9e8886/HEAD.txt`,
clean worktree, installed binary hashes in `installed_binaries.sha256`).
Logs are stored xz-compressed; decompress with `xz -dk` before running the tools.
The full generated tables are in [`evidence/runs_d9e8886/SUMMARY.md`](evidence/runs_d9e8886/SUMMARY.md),
produced by `tools/summarize_v2_evidence.py` (exit status PASS).

Exploratory runs used during development are **not** part of this evidence
(see `V2_IMPLEMENTATION.md` §5 for what they changed).

## 1. Required evidence, item by item

| Required | Result | Where |
|---|---|---|
| Object/giver truth independent of the selected receiver plan | **PASS**. The static check shows the truth path references no plan, commit, τ, grasp or route. Replay: every truth sample recomputes from the logged script (max \|dp\| ≤ 2.1e-13 m). Cross-run: V1-independent vs V2, same scenario, **bit-identical** truth at 786–1291 common samples in all 4 scenarios, although V1 and V2 chose different plans. No commit created a plan-driven truth. | SUMMARY §cross-run; I2; `tools/check_giver_truth_independence.py` |
| Robot moves while the object is still moving | **Observed** in diagonal (24 concurrent 50 ms samples; run, repeat and injection run). **Not observed** in longitudinal, near-ground, lateral-low: adoption happened only after the object came to rest. | D1 |
| Planning generations while the robot moves | **Observed** in 8 of 9 V2 runs (diagonal: 218 jobs submitted with robot **and** object moving). | D2 |
| Provisional plan update/replacement before commitment | **Replacement**: near-ground 2× (run and repeat; reasons `cost_invalid/timeout`, `predictive_static/reach_tracking`), lateral-low 1×. **Retained with prediction update**: near-ground, diagonal. | D3 |
| Stale worker results cannot commit | **PASS**. Injection run: the injected stale terminal certificate (generation 537) was rejected with `canCommit=false`, and the commit used certificate 569. Stale rejections also occurred naturally at phase changes in 5 further runs; no committing certificate was ever a rejected generation. | I6, D4; `v2_inject_stale` |
| Exactly one final commitment | **PASS** in every completed V2 run (1 V2 commit, 0 V1 commit markers, 0 second attempts). | I3 |
| No global reselection after commitment | **PASS** (0 post-commit job submissions, adoptions or selections). | I5 |
| Capture / load transfer / carried retreat | **PASS** in 7 of 7 completed V2 runs, in order. | I8 |
| V1 regression unchanged | **PASS**. All four FrozenPlanSet sha256 hashes are identical to the recorded V1 evidence (198/283/432/233 records). The V1 completion pattern matches pristine `main` (lateral-low fails: see §3). | SUMMARY §V1 regression |
| Worker reads only immutable snapshot state | **PASS**. `tools/check_worker_snapshot_purity.py`: 85 worker-reachable functions, 2 roots, no unguarded live access. The checker fails on V1 source (15 accesses) and on an injected V2 live read. | `tools/test_v2_checkers.py` |

## 2. First architectural comparison (independent giver, identical bank, checks, objective, QP, terminal controller)

| Scenario | V1 frozen pre-reach | V2 receding (run / repeat) |
|---|---|---|
| longitudinal | planning result at t=12.38 s, committed, then **post-commit failure**: object left V1's 15 mm committed-prediction tube (giver stopped short of V1's chosen τ) | **completed / completed**; 3 full searches (3.7, 2.2, 0.55 s), commit after rest, capture at 12.9 s after giver start |
| near-ground | **post-commit failure** (same cause) | **completed / completed**; 2 provisional replacements, capture at 15.3 s |
| lateral-low | **rejected at commit** (prediction drift) | **failed closed / failed closed**: no certified plan within the presentation window (§3) |
| diagonal | **completed**, capture at 8.8 s | **completed / completed**; robot moved while object moved; capture at 9.7 s |

Completion: V1 **1/4**, V2 **3/4** in both runs. V1 post-commit failures:
**2**. V2 post-commit failures: **0**. Minimum live clearance during motion:
V1 0.078 m (diagonal); V2 0.079–0.082 m (completed runs).

**Interpretation, bounded.** Four deterministic scenarios, one run each for V1
and two for V2, a single speed and a scripted giver. The difference is
consistent with the architectural variable: V1's two failures are
post-commitment tube violations caused by committing to a τ the independent
giver did not honour; V2 never committed before the object was observed at rest.
On diagonal, where both succeed, V2 captured 0.9 s later. This is **not**
evidence of a general advantage: no statistics, no variation of giver timing or
noise, no human.

## 3. Failures and limitations observed in the evidence

1. **lateral-low fails in both architectures.**
   - **V1:** commit rejected. With 0.9 s observation plus 4.5 s planning at 0.08 m/s, the object passes the simulator's 0.40 m travel point and stops, so the prediction drifts. This confirms the audit's travel-cap inference (43.9 mm drift in the `main` baseline run).
   - **V2 run:** the initial plan was adopted at t=13.46 s and invalidated 6 ms later (`reach_tracking`, the object began stopping). The replacement search, launched during the stopping transient, took 6.51 s; the next, memoized, took 0.51 s. Its replacement plan was adopted at t=20.49 s and failed re-certification from the moving arm state (`closure/pad_pair/blue_handle_acquisition_tube`) before the window closed at 20.54 s.
   - **V2 repeat:** the provisional plan was adopted 18 ms before the object began stopping, and its first re-certification failed (`reach_tracking`). The replacement search launched during the stopping transient, where the 14 predicted poses still differ and memoization cannot apply, and it ran past the 7 s window.
2. **Latest-only scheduling cannot cancel a superseded in-flight search.** A full search of the inherited bank takes 2.2–6.5 s while poses differ. This is the dominant V2 failure mechanism observed.
3. **Replacement requires holding the arm.** A full bank result is certified from a held start state; concurrent motion therefore continues only while the incumbent stays certified.
4. **Certification is start-state dependent** (local IK with redundancy): the same grasp can pass from one arm state and fail closure centering from another.
5. **Capture is quasi-static** (unchanged terminal controller). V2 intercepts the presentation, not a moving handle at contact.
6. **Simulation only**: virtual load transfer; no hardware or human.

## 4. Reproduce

```bash
TRIAD_RECEIVER_MODE=v2 scripts/run_scenario.sh diagonal out/v2/diagonal
TRIAD_RECEIVER_MODE=v1-independent scripts/run_scenario.sh diagonal out/v1_independent/diagonal
python3 tools/check_v2_run_log.py out/v2/diagonal/diagonal.log
python3 tools/check_giver_truth_independence.py --cross out/v1_independent/diagonal/diagonal.log out/v2/diagonal/diagonal.log
python3 tools/summarize_v2_evidence.py out
```
