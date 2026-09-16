# Phase E: receding predictive interception execution and baseline variants

Formulation: `TRIAD_PREDICTIVE_INTERCEPTION_AUDIT.md` §3.6.
Code: `ReceiverV2.cpp` (`handlePredictiveSelectionV2`, `setInterceptionExecutionV2`, `interceptionExecutionReferenceV2`, execution branch in `stepControlAwareTrackV2`).
Tags: `CODE` / `DERIVED` / `LIT` / `NUM` / `EXP`.

## 1. Variants (`controlAware.variant`)

All variants share the same perception and prediction, Phase B grasp family and funnel (K = 40), exact controller layers, low-level tracking law (rate cap, lead tube, clearance governor, safety filter, QP task) and acquisition interface (MovePregrasp / CaptureTransfer, object at rest).

| Variant | Selection | 𝓕_C filter | Tie-break inside Δτ | Execution before rendezvous |
|---|---|---|---|---|
| `reactive` (B0, default) | TRIAD-lite 04e9efc: admissible → clearance → reserve at the current object pose | TRIAD-lite κ gate | — | tracks the live object-relative standoff |
| `predictive` (B1) | earliest τ in 𝓕_I | no (κ logged) | clearance | velocity-matched Hermite rendezvous patch |
| `predictive_capability` (B2) | earliest τ in 𝓕_I | no | condition index, then clearance | same |
| `full` (FULL) | earliest τ in 𝓕_C | κ ≥ κ_min (path, sync, insertion) | min(κ, κ_sat), then clearance | same |

`requireObjectStopped` is kept at the acquisition interface for every variant (Audit §6): no moving acquisition.

## 2. Receding loop (predict → intercept → move concurrently → update → correct)

1. **Solve** (worker thread, non-blocking):
   - the interception sweep of Phase C/D runs from the snapshot;
   - when a plan executes, the rollout starts from the executed reference state predicted at t_snap + L_calc (Hujić planning-time shift);
   - the incumbent grasp is always re-solved, even outside the shortlist, and is never marked dominated.
2. **Latency guard.** A result with realised latency > L_calc (0.80 s) is refused. Its rollout start assumption is violated.
3. **Inconclusive results.** When the budget is exhausted and there is no feasible encounter, or the incumbent is unresolved, nothing changes: a budget-limited sweep proves no infeasibility.
4. **Selection** uses the unchanged `updateGraspSelector` hysteresis (dwell 0.3 s). A switch happens only when:
   - the incumbent is infeasible (`switch_inadmissible`), or
   - a challenger outside the tie bands persists for the dwell (`switch_dominated`), or
   - nothing is feasible (`abort_to_hold`).
5. **Aimed-state retention** (Audit §3.6.1). The plan is kept while the newest prediction places the grasp, at the *planned* t_R, within ε_p = 15 mm / ε_R = 0.12 rad of the planned meeting pose. Otherwise it is patched: a new Hermite patch starting from the executed reference pose and velocity.
6. **Execution** (control thread, every cycle). Before t_R the reference goal is the Hermite patch evaluated with the *live* prediction. It passes through the same rate cap, lead tube and safety filter as the TRIAD-lite tracker.
7. **Replanning closed.** A re-solve returns τ ≥ L_calc + L_entry, so once now + L_calc + L_entry ≥ t_R no result can patch the approach (`DERIVED`). No more selections are submitted.
8. **Local synchronization.** After t_R the existing object-relative tracking law runs (Hujić fine motion).
9. **Freeze.** The existing measured gate plus TERMINAL_CERTIFY leads to `grasp_freeze` at MovePregrasp commit.

## 3. Design corrections found in pilots (`evidence/phaseE_sim/pilot*`, lateral-low)

| Pilot observation | Cause | Correction | Provenance |
|---|---|---|---|
| Moving jobs returned no encounter | exact budget 240 below the need (up to 434, one job 1078) | budget 450 / rollouts 40 | `NUM` from the phaseC/D logs |
| t_R receded by about 0.15 s per job; the arm never met the grasp | τ re-derived every job while T_reach (preview convergence × armScale) barely shrinks | aimed-state retention instead of τ comparison | Audit §3.6.1 (was implemented wrongly) |
| Incumbent flagged infeasible within about 0.5 s of t_R (clearance floor, closure, terminal relative motion from a near-goal start); repeated switching; no admission | the planning model is evaluated from a near-goal state it was not designed for | replanning closed inside L_calc + L_entry of t_R | `DERIVED` (no result can arrive in time) |
| `abort_to_hold` on a budget-exhausted empty result, then no certifiable grasp from the mid-approach hold pose | an incomplete search was treated as proof | result inconclusive | logic |

**After the corrections** (single runs, descriptive only):
- **B1:** rendezvous with the decelerating object at 1.45 mm / 0.025 rad; local synchronization; then TERMINAL_CERTIFY failed (`mouth_corridor/blue_handle_axial_offset`), leading to abort, re-plan at rest, admit and freeze. Completed.
- **FULL:** the same pattern. Rendezvous at 5.4 mm; abort at terminal certification; freeze on the second plan. Completed.

Both first grasps had |s| > 0. Whether the Phase B axial bound (35 mm) is consistent with the terminal certification corridor after the object settles is **open**. It is not tuned here.

## 4. Unsupported / approximate

- **Rollout start state.** The arm configuration is the snapshot, while the reference state is shifted by L_calc. With a moving arm this mixes two instants (lag ≤ tracking lead).
- **Refused results.** 10 % of moving jobs exceed L_calc and are refused (latency is machine-dependent).
- **Held-arm dead end.** After an abort mid-approach, the held pose can leave no certifiable grasp (pilot 2, all 40 shortlisted grasps fail closure / tube). This is the same failure mode as V2 Track 1.
- **Acquisition and prediction.** Acquisition of a moving object is not implemented (interface limit). The prediction is constant twist.
