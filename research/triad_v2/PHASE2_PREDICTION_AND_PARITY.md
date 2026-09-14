# FINAL TRIAD — Phase 2: Prediction validity and preview/runtime parity

Branch `research/triad-v2-dynamic-control`.

**Instrumentation.** Commits 53ec3d3 and 39d7553 add logging-only parity traces, plus the diagnostics committed with this report. No decision reads any of it.

**Evidence** is in `research/triad_v2/evidence/runs_phase2_parity/`:
- logs;
- `analysis/PREDICTION_TABLES.md`, `analysis/PARITY_TABLES.md`, `analysis/parity.json`, `analysis/adoptions.jsonl`, `analysis/replica_validation.jsonl`;
- `CAMPAIGN_NOTES.txt`, which lists two confounds.

**Tools.** `tools/prediction_error_study.py` and `tools/parity_analysis.py`.

**Evidence tags.**
- CODE: read in source.
- MEASURED: in-situ simulation logs.
- REPLICA: exact offline replication of controller code, validated against logs.
- OFFLINE_ONLY: conditions the simulator cannot produce, evaluated on the validated replica.
- INFERENCE: interpretation.

**Unchanged.** Architecture, bank, cost, selector, predictor, certification stages and commitment. V1 `FrozenPlanRecord` sets are byte-identical to the A1 build in all four scenarios (MEASURED: 198 / 283 / 432 / 233 records, identical SHA-256).

---

## 0. Summary

1. **The constant-twist prediction is accurate only while the object keeps a constant velocity.** The horizon over which it is valid is set by the giver's unannounced deceleration, not by the estimator.
   - Noise-free, inside a constant-velocity segment, the error is ≤ 0.23 mm at h = 3 s (REPLICA).
   - With the scripted stop, the worst-case horizon with e_p ≤ 15 mm is:
     - 0.35 s at 0.08 m/s (the nominal scenarios);
     - 0.55 s at 0.04 m/s;
     - 0.15 s at 0.24 m/s;
     - ≤ 0.65 s for any stop duration tested at 0.08 m/s.
   - V2's lead bank starts at 1.80 s.
2. **In situ, every provisional adoption made while the object moved used a horizon of 2.35–5.33 s.** Its realised target error at τ was up to 313 mm (median 148 mm). Adoptions at rest: ≤ 0.9 mm (MEASURED, 101 adoptions, 49 runs).
   - When the first adoption happened while the object moved, 17 of 41 runs completed. When it happened at rest, 7 of 8 completed (MEASURED).
3. **Runs are not reproducible across machine speed.** The first FULL_SEARCH now takes 12–15% less wall time than in the A1/Track 1 campaigns. That flips which generation is adopted: an early far-lead plan while moving instead of a later at-rest plan. In-situ completion fell to 11/24 traced runs (parity_a–f) and 5/12 untraced control runs (ctrl_d–f).
   - The untraced controls fail at the same rate as the traced runs (5/12 vs 11/24 overall; 5/12 vs 6/12 for the matched runs d–f). The instrumentation is therefore not the cause (MEASURED).
   - V2 in-situ outcomes depend on wall-clock computation relative to giver motion. Track 1 found the same thing.
4. **The estimator's validity envelope is narrow** (REPLICA / OFFLINE_ONLY).
   - It needs a fresh noise-free pose every 1 ms control tick.
   - Per-tick position noise of σ = 0.2 mm, or a sensor at ≤ 100 Hz with sample-and-hold, drives the velocity estimate error to 80–93 mm/s at a true 80 mm/s. The cause is 1 kHz finite differencing plus the 0.30 m/s raw-speed rejection.
   - A compensated 0.22 s delay costs 7.2 mm at h = 0. Uncompensated, it costs 17.6 mm.
5. **Terminal parity is good for insertion, and the timing model is conservative elsewhere** (MEASURED, 19 committed runs).
   - Insertion duration: predicted within ±0.03 s of runtime.
   - Acquire: over-predicted by 0.51–0.53 s.
   - Carried retreat: over-predicted by 0.51–1.78 s.
   - Insertion path: spatial deviation ≤ 14.9 mm. Retreat path: ≤ 12.5 mm.
   - Retreat clearance: the preview is optimistic by 2–13 mm.
6. **Reach parity depends on the route.**
   - Ring-140 near-ground: spatial deviation ≤ 13.4 mm, arrival within 0.03 s, clearance within 0.5 mm.
   - Direct lateral-low: the runtime runs up to 164 mm ahead of the time-aligned preview (spatial 50 mm, 0.24 rad). Runtime clearance is 17 mm lower than the preview, even though arrival matches (1.90 s vs 1.94 s).
7. **Certification is not consistent between the from-start rollout and a rollout resumed from the runtime state.** This is the load-bearing defect found here.
   - 39 runtime invalidations hit plans adopted at rest, so with no prediction change (MEASURED).
   - In a reproducible longitudinal case (7 runs), three things hold at once:
     - the from-start rollout reaches the standoff within 1.6 mm;
     - the runtime arm tracks that rollout within 4.7 mm;
     - yet the rollout resumed from the runtime state at step 15–16/78 ends 142–249 mm from the standoff and invalidates the plan.
   - Mechanism (MEASURED + CODE): the preview IK's elbow joint (Gen3 joint 4) moves off its branch, −1.58 → −0.73 rad, while the reference, command and posture target are unchanged. Both rollouts run with joint 6 at ≈ 91% of its ±2.23 rad range, inside the preview's joint-limit barrier (activation 0.65). There the weighted DLS redistributes the task, and a ≈ 0.02 rad difference in the start state selects a different branch. The runtime QP enforces the same limits as hard bounds with an interpolated posture task and does not flip.
   - Part of the "runtime invalidation" evidence used in A1 (stage verdicts) and Track 1 may therefore be certification artefacts rather than physical infeasibility.

No result contradicts the frozen architecture. Results 1–3 constrain the T_k law, and result 7 must be repaired before the in-situ evidence of Phases 3–8 is trustworthy (section 3).

---

## 1. Phase 2A — prediction error versus horizon

### 1.1 What is being evaluated (CODE)

**Estimator** (`updateObjectMotionEstimate`).
- Every 1 ms control tick, the raw twist is the finite difference of two consecutive (possibly delayed) measurements.
- It is rejected if |v| > 0.30 m/s or |ω| > 1.5 rad/s.
- Otherwise it is low-pass filtered with α = dt/(τ+dt), τ = 0.10 s.
- The pose is the measurement, plus age·v̂ when latency compensation is on.
- The prediction record is valid after 0.70 s and 350 samples.
- V2 zeroes the record twist below 0.004 m/s and 0.08 rad/s (`currentObjectPredictionV2`).

**Predictor.** p̂(t_k+h) = p(t_k) + h·v̂; R̂ = exp(h·ω̂)·R, with "no stop assumption" (`predictionPoseAtV2`).

**Truth.** The independent scripted giver (`IndependentGiverModel.h`):
- constant twist from the start pose;
- travel of 0.40 m;
- a C² quintic stop of duration D (0.85 s nominal), with peak deceleration 1.875·v/D.
- It starts at full speed, has no direction change, and adds no noise.

### 1.2 Replica validity (REPLICA vs MEASURED)

`prediction_error_study.py` reimplements the chain above tick by tick. It was compared against 77 in-situ V2 runs covering:
- 67 nominal runs;
- speeds of 0.04 and 0.16 m/s;
- stop 0.425 s;
- delay 0.22 s, compensated and uncompensated.

Two comparisons were made:
- **14,407 `[V2Motion]` estimate-speed samples**: max difference 4.96·10⁻⁶ m/s, the 5-decimal logging resolution.
- **51,522 `[V2SnapshotAudit]` prediction-error samples** (each an in-situ e_p(h) for h = snapshot age): max difference 8.3·10⁻⁷ m, the 6-decimal resolution.

The replica therefore **is** the controller's predictor for the purposes of this envelope.

The in-situ audits alone could not give the envelope: their largest logged error is 0.79 mm, because the jobs they audit are short or at rest.

### 1.3 Envelope

The decision times t_k are every 10 ms while the truth moves and the record is valid. Horizons are 0 to 8 s in 0.05 s steps. h_valid(ε) is the largest h such that the worst e_p over all decision times is ≤ ε at every horizon up to h. The full tables are in `analysis/PREDICTION_TABLES.md`.

| condition | source | worst h_valid for e_p ≤ 5 / 10 / 15 / 30 mm (s) | worst e_p at h = 0.5 / 1.0 / 1.8 / 3.0 s (mm) | estimate speed error max (mm/s) |
|---|---|---|---|---:|
| 0.04 m/s, stop 0.85 s | SIMULATOR | 0.25 / 0.40 / 0.55 / 1.00 | 12 / 30 / 60 / 108 | 8.1 |
| **0.08 m/s, stop 0.85 s (nominal)** | SIMULATOR | **0.15 / 0.25 / 0.35 / 0.55** | **24 / 60 / 121 / 215** | 16.2 |
| 0.12 m/s | SIMULATOR | 0.10 / 0.20 / 0.25 / 0.40 | 36 / 90 / 181 / 322 | 24.3 |
| 0.16 m/s | SIMULATOR | 0.10 / 0.15 / 0.20 / 0.35 | 48 / 119 / 242 / 430 | 32.4 |
| 0.24 m/s | SIMULATOR | 0.05 / 0.10 / 0.15 / 0.25 | 71 / 179 / 363 / 645 | 48.6 |
| 0.08 m/s, stop 0.425 s | SIMULATOR | 0.10 / 0.20 / 0.25 / 0.45 | 31 / 69 / 132 / 227 | 27.6 |
| 0.08 m/s, stop 1.7 s | SIMULATOR | 0.25 / 0.35 / 0.50 / 0.75 | 14 / 44 / 101 / 192 | 8.6 |
| 0.08 m/s, stop 3.0 s | SIMULATOR | 0.35 / 0.50 / 0.65 / 1.00 | 9 / 28 / 75 / 160 | 5.0 |
| 0.08 m/s, delay 0.22 s compensated | SIMULATOR | 0.00 / 0.05 / 0.10 / 0.35 | 39 / 76 / 138 / 232 | 48.9 |
| 0.08 m/s, delay 0.22 s uncompensated | SIMULATOR | 0.00 / 0.00 / 0.00 / 0.55 | 24 / 60 / 121 / 215 (17.6 at h = 0) | 48.9 |
| per-tick noise σ_p = 0.2 mm (1 kHz) | OFFLINE_ONLY | 0.05 / 0.10 / 0.15 / 0.30 | 47 / 93 / 167 / 279 | 93.0 |
| sample-and-hold 100 Hz, noise-free | OFFLINE_ONLY | 0.05 / 0.10 / 0.15 / 0.35 | 40 / 80 / 144 / 240 | 80.0 |
| sample-and-hold 30 Hz, noise-free | OFFLINE_ONLY | 0.00 / 0.05 / 0.15 / 0.30 | 43 / 83 / 147 / 243 | 80.0 |
| heading turn 0.25 rad/s, no stop | OFFLINE_ONLY | 0.60 / 0.90 / 1.10 / 1.60 | 4 / 12 / 36 / 94 | 2.0 |
| heading turn 0.5 rad/s, no stop | OFFLINE_ONLY | 0.40 / 0.60 / 0.75 / 1.10 | 7 / 24 / 70 / 179 | 4.0 |

**Error decomposition.** When the whole horizon stays inside the constant-velocity segment, the worst e_p is small:

| condition | worst e_p |
|---|---|
| nominal, h = 3.0 s | 0.23 mm |
| 0.22 s compensated delay, h = 3.0 s | 2.17 mm |
| 0.22 s uncompensated delay | 17.6–19.6 mm (= v·delay) |

So the error is almost entirely model error from the unmodelled acceleration.

**Rotation.** At 0.30 rad/s the orientation error ≤ 0.12 rad holds for 0.60 s. The four nominal scenarios have no rotation, so this was not exercised in situ.

**Timing of the worst case** (`PREDICTION_TABLES.md`, last table). The worst e_p at h = 1.8 s occurs for decisions 0.5 s before to 0.5 s after the stop onset (≈ 121 mm nominal). It is zero only when the decision is ≥ 2 s before the onset.

(INFERENCE) The predictor carries no information about when the giver will stop. Any guarantee on a moving target therefore holds only under a declared motion-class assumption, "no acceleration onset within h". It cannot hold unconditionally.

### 1.4 What V2 actually decided (MEASURED)

`analysis/adoptions.jsonl` covers 101 provisional adoptions in the 49 Phase 2 V2 runs (campaign 1 parity a–c; campaign 2 parity d–f, ctrl d–f, variants, resume g–i).

| decision state | adoptions | horizon τ − t (s) | realised \|p_O(τ) − predicted presentation\| |
|---|---:|---|---|
| object moving | 42 | 2.35 – 5.33 | up to 313 mm; median 148 mm |
| object at rest | 59 | 1.6 – 4.0 | ≤ 0.9 mm |

Outcome by the state at the first adoption:

| first adoption | runs | completed |
|---|---:|---:|
| object moving | 41 | 17 |
| object at rest | 8 | 7 |

An adoption while moving is never prediction-valid at its own τ. Its benefit can only come from moving the arm toward a region that stays useful; see Track 1 and section 3.

### 1.5 Validated envelope (to carry into Phase 3)

- **SIMULATOR, noise-free 1 kHz measurement.** Moving object, constant velocity, giver speed 0.04–0.16 m/s, stop duration 0.425–3.0 s, delay ≤ 0.22 s with compensation.
  - Worst-case 15 mm validity horizon: 0.10–0.65 s.
  - Under the explicit assumption "no acceleration onset within h", the envelope extends to ≥ 3 s with errors ≤ 2.2 mm.
- **At rest.** Error ≤ 0.9 mm in situ at every horizon. The rest-zeroing makes the at-rest prediction exact for a static giver.
- **Not validated.** Measurement noise, sensor rates below the control rate, direction changes (offline only), start transients (the simulator starts at full speed), and rotation in situ.
  - The current estimator is not usable with a real sensor without change (OFFLINE_ONLY evidence).
  - Per the phase instructions, no new predictor is introduced here. This is a stated limitation and a requirement for any hardware claim.

---

## 2. Phase 2B — preview/runtime parity

### 2.1 Instrumentation (CODE; logging only)

`ReceiverV2.parityTrace` (default `false`) enables four logs:

1. `[V2ParityPreviewReach]` — for RECERTIFY_ACTIVE jobs that start at reach index 0: the copied-state rollout samples (plan time, mouth pose, clearance, reference / rate-limited / command positions, clearance scale, and joint vector). The last such trace before reach start is logged when execution begins (`[V2ParityReachStart]`).
2. `[V2ParityCommitPreview]` / `[V2ParityPreviewTerminal]` — at commitment: the terminal certificate's predicted durations and clearances, and its insertion / closure / retreat rollout.
3. `[V2ParityRuntime]` — every 10 ms: FSM state, V2 phase, measured mouth pose, estimated and true object pose, runtime reference, clearance scale, and current clearance (pose safety, or attached-retreat safety once carried).
4. `[V2ParityResumeFailure]` — when a resumed recertification fails: the rejected rollout next to the last accepted one. Also `reachEndPositionError` / `reachEndOrientationError` on every `[V2CertificationDetail]`.

Traces are recorded only inside certification jobs (never FULL_SEARCH), and no decision reads them.

**Equivalence.**
- Selection code is untouched.
- V1 record sets are identical.
- Traced vs untraced completion does not differ beyond run-to-run variation: 6/12 traced vs 5/12 untraced for matched sets d–f.

**Campaign.**
- Campaign 1: nominal scenarios × 3 (parity_a–c).
- Campaign 2: nominal × 3 traced (parity_d–f) and × 3 untraced (ctrl_d–f); variants speed 0.04 / 0.16 m/s, stop 0.425 s and delay 0.22 s compensated / uncompensated, on longitudinal and near-ground.
- Diagnostics: longitudinal × 3 (resume_g–i).

**Recorded confounds** (`CAMPAIGN_NOTES.txt`):
- a compile overlapped parity_c/longitudinal;
- the first-pass variant runs had broken override files (V2 refused to start). They are archived as invalid and were rerun.

### 2.2 Reach parity (MEASURED; `analysis/PARITY_TABLES.md`)

**Method.** Only plans executed through the standoff window are compared, in two groups:
- object static during the window (a true parity measurement);
- object moving (confounded by prediction updates; reported separately).

**Object static during reach:**

| route family (runs) | time-aligned max / RMS path dev (mm) | spatial max dev (mm) | max orientation dev (rad) | standoff error preview / runtime (mm) | arrival in tolerance preview / runtime (s) | min clearance preview / runtime (mm) |
|---|---|---:|---:|---|---|---|
| near-ground `axisP_side_45deg` / ring140mm_7of8 (9) | 11.0–13.7 / 6.8–7.4 | 10.9–13.4 | 0.052–0.064 | 1.1–1.2 / 5.4–5.5 | 3.04–3.06 / 3.01–3.05 | 79.0–79.4 / 79.5 |
| near-ground ring140mm_0of8, delay 0.22 s (1) | 17.4 / 9.8 | 10.7 | 0.038 | 1.4 / 5.4 | 3.08 / 3.06 | 81.2 / 79.1 |
| near-ground direct, speed 0.04 m/s (1) | 2.4 / 1.5 | 0.7 | 0.046 | 1.5 / 2.2 | 0.48 / 0.51 | 80.6 / 79.3 |
| **lateral-low `axisN_side_337deg` / direct (3)** | **163.5 / 74.3** | **49.8** | **0.236** | 1.8 / 6.0 | 1.94 / 1.90 | **81.6 / 64.6** |
| lateral-low direct, short terminal reach after replacement (3) | 4.9–5.3 / 3.2–3.4 | 2.3 | 0.056–0.061 | 1.9–2.0 / 3.6–3.9 | 0.70–0.76 / 0.71–0.77 | 56.4–66.8 / 56.4–66.8 |

**Lateral-low direct.** The runtime moves faster than the preview between 0.6 s and 1.5 s into the reach: 156 mm ahead at 1.1–1.2 s, with the preview nearly stalled at 0.61–1.01 s. It also takes a lower path (z 30 mm lower). Both converge by 1.8 s.
- Arrival timing agrees.
- Runtime clearance is 17 mm below the certified value, while still above the 8 mm runtime reserve.
- (INFERENCE) The certified reach-clearance margin is not conservative for this route. The preview integrator and the QP take different transient paths.

**Object moving during reach** (longitudinal direct or ring80). The runtime standoff error is 107–109 mm, equal to the object displacement during the window (70–71 mm, plus the approach geometry). This is a prediction effect and is not attributed to parity.

**Plans executed only partially** (invalidated or replaced) with the object static: time-aligned deviation up to the invalidation is 3.9–29.3 mm.

### 2.3 Terminal parity after commitment (MEASURED; 19 committed runs)

| quantity | predicted | runtime | error (runtime − predicted) |
|---|---|---|---|
| insertion duration (terminal timing audit) | 1.04–1.12 s | 1.069–1.088 s | −0.03 … +0.03 s |
| acquire duration | 2.668–2.694 s | 2.154–2.165 s | −0.51 … −0.53 s (conservative) |
| carried-retreat duration | 1.10–1.225 s (2.40 in 1 run) | 0.564–0.616 s | −0.51 … −1.78 s (conservative) |
| insertion path: spatial max deviation / end-point error | — | — | 13.9–14.9 mm / 11.5–12.2 mm |
| retreat path: spatial max deviation / end-point error | — | — | 8.7–12.5 mm / 2.8–8.3 mm |
| carried-retreat clearance | 80.8–219.3 mm | 79.1–211.3 mm | runtime 2–13 mm lower (preview optimistic) |

The insertion end-point error of 11.5–12.2 mm is the distance between the preview's insertion end and the runtime MovePregrasp exit. The runtime exits on its velocity-gate/dwell condition while the preview uses the convergence tolerance, so this is partly a phase-boundary definition difference (INFERENCE).

**Timing model.** Of the terminal durations used by the selector's timing and the commitment reserve:
- insertion is accurate;
- acquire and retreat are conservative by ~0.5 s and 0.5–1.8 s.

Any timing reserve derived in Phase 3 or 9 must use the measured components, not the conservative totals.

### 2.4 Recertification consistency (MEASURED + CODE)

**Invalidations of plans adopted at rest.** These occurred with no prediction change during execution. 39 runtime invalidations out of 77 fall in this class:

| count | reason |
|---:|---|
| 15 | reach_tracking |
| 10 | static_acquire (closure) |
| 6 | runtime_clearance_reserve |
| 4 | reach (ground plane) |
| 4 | terminal static_acquire |

The remaining 38 hit plans adopted while moving, where prediction updates are a legitimate cause (23 reach_tracking, 12 terminal-audit timeout, 3 closure).

**Reproducible case: longitudinal plan 2** (`axisP_side_23deg` direct, lead 1.80 s). It appears in 7 runs: parity_a/b/d/f and resume_g/h/i.
- The from-start rollout reaches the standoff with 1.5–1.6 mm error.
- The runtime arm follows that rollout within 4.6 mm.
- The resumed recertification at index 15–16/78 (arm 14 mm from the reach start) ends with 142–249 mm / 0.06–0.12 rad error and invalidates the plan.

**Diagnostics** (resume_i: `[V2ParityResumeRejected/Accepted]` and `...Joints`):
- The rejected and accepted resumed rollouts use identical reference and command sequences and the same clearance scale (1.0).
- They start from joint states differing by ≈ 0.02 rad (5 mm at the mouth).
- After ~20 steps, the rejected rollout's joint 4 moves from −1.58 rad toward −0.73 rad and joint 2 from 0.88 to 1.64 rad. The mouth then leaves the commanded direction (y 0.18 → 0.31 m while the command decreases).
- The accepted rollout keeps joint 4 at −1.61 rad throughout.
- Both use the null-space posture target (`plannedStandoffArmPosture`: −1.14, 1.19, 4.13, −1.61, −1.95, 1.79, 1.00).

**Code difference.**
- The preview IK (`previewReachStep`) uses weighted damped least squares with a joint-limit barrier and a null-space posture term toward the final standoff posture only.
- The runtime (`executeProvisionalReachV2`) commands an arm posture interpolated from hold → transit → standoff by phase progress, as a QP task.

**Joint-limit check** (CODE: `previewReachStep`; URDF joint 6 limit ±2.23 rad, margin 0.015, barrier activation s > 0.65). Joint 6 is 2.00–2.02 rad in both rollouts (s ≈ 0.91; logged minimum joint-margin ratio 0.092 rejected vs 0.095 accepted). The barrier multiplies the joint's DLS weight by 1/(1 + 0.8·s⁶/(1−s²)²) and adds an avoidance velocity. The posture targets are identical in both rollouts, so the posture-model difference is a parity difference but **not** the trigger here.

(INFERENCE) Near a joint-limit barrier the preview integrator has a branch point that a small change in its initial state can cross. The runtime controller, which handles the limit differently, does not follow that branch. A rollout resumed from the runtime state can therefore reject a plan the runtime is executing correctly.

**Consequence.** Retain-while-certified is only as valid as recertification. Some of the "runtime invalidations" counted as certification value in A1 (for example the lateral-low closure invalidations) and Track 1 may be artefacts of this inconsistency. A1's candidate-level stage statistics come from from-start FULL_SEARCH rollouts and are not affected. Its runtime-outcome connection is (INFERENCE; to re-examine in Phase 8).

---

## 3. Consequences for the program

| finding | what it changes | phase |
|---|---|---|
| Worst-case prediction validity 0.10–0.65 s while moving; errors 76–313 mm at the horizons V2 uses | The upper T_k horizon for **commitment-grade** candidates cannot exceed the validity horizon while the object moves. Beyond it, candidates are provisional hypotheses under an explicit "no acceleration onset" assumption, and T_k must say so. The fixed 1.8–8.0 s bank is not justified by prediction validity. | 3 |
| Outcome depends on search wall time (adoption generation flips at ~12–15% CPU speed change) | A law I_k → T_k must bound computation, or make adoption independent of arrival time (e.g. no adoption of predictions outside the validity envelope). Otherwise in-situ results are not reproducible. All later in-situ comparisons must control machine load and report first-search wall time. | 3, 7, 11 |
| Estimator valid only for noise-free 1 kHz measurement | A hardware or noisy-sensor claim needs an estimator change. No simulation claim beyond the envelope in 1.5. | 9, limitations |
| Insertion timing accurate; acquire and retreat timing conservative by 0.5–1.8 s | L_commit and the timing admission reserve must be rederived from the measured components. | 3, 9 |
| Reach clearance optimistic by up to 17 mm (direct lateral-low); retreat clearance by up to 13 mm | Clearance margins used as hard gates (runtime reserve 8 mm, transit 12 mm) need a parity allowance, or the preview must reproduce the runtime path. | 5, 9 |
| **Recertification inconsistency (resumed rollout vs from-start and runtime)** | **Must be repaired before in-situ evidence in Phases 3–8 is used.** The repair needs a decision on what recertification certifies. The candidates are:
1. the remaining action integrated from the measured runtime state (current semantics, which exposes the integrator's branch sensitivity);
2. the certified trajectory re-checked under the newest prediction, plus a runtime-conformance tube (needs a parity-derived tube; lateral-low direct deviates up to 164 mm time-aligned);
3. a preview integrator that handles joint limits like the runtime QP (changes every certification, FULL_SEARCH included).

It touches certification semantics and implementation, not the architecture. Whatever is chosen, re-measure 2.4 with the same tools, with V1 hash evidence. | next step, before Phase 3 in-situ work |

No finding requires changing the frozen architecture: provisional motion, recertification, a single late commitment, and complete-action certification all remain.

## 4. Reproduction

```bash
# 4A
python3 tools/prediction_error_study.py validate <V2 logs...>        # replica vs in-situ
python3 tools/prediction_error_study.py grid OUT.json && python3 tools/prediction_error_study.py report OUT.json
python3 tools/prediction_error_study.py adoptions <V2 logs...>       # realised error of adoptions
# 4B (runs with ReceiverV2 parityTrace: true)
python3 tools/parity_analysis.py OUT.json <logs...> && python3 tools/parity_analysis.py report OUT.json
```

Campaign scripts: `evidence/runs_phase2_parity/campaign.sh` and `campaign2.sh`. Binary hashes: `installed_binaries*.sha256`. The diagnostic resume_g–i runs used the build with this commit's joint-trace logging.
