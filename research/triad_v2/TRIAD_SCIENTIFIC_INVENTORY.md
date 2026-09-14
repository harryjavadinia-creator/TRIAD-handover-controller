# TRIAD scientific inventory (mechanisms beyond T/G/R)

**Scope.** Measurement and documentation only; no mechanism was changed. The source is the checkpoint `triad-v2-checkpoint-phase-e-start` (`f0acc46`) plus the Track 1 prune commit (default off). Paths are relative to the repository root:
- `cpp` = `src/HandoverInterceptionController.cpp`;
- `rv2` = `src/ReceiverV2.cpp`;
- `yaml` = `etc/HandoverInterceptionController.in.yaml`;
- `st/X` = `src/states/HandoverInterceptionController_X.cpp`.

Line numbers refer to that source.

**Provenance classes** (strictest supportable label):

| Label | Meaning |
|---|---|
| **PHY** | follows from physics with stated inputs |
| **GEO** | follows from measured/CAD geometry |
| **LIT** | taken from cited literature (none of the constants below cite a source in code or config) |
| **EMP-V** | empirically validated in this repository's evidence, *re-verified in this project* |
| **EMP-C** | a code/config comment claims calibration or validation, but the supporting evidence was not located or re-verified |
| **ENG** | inherited engineering choice with no derivation |

**Role:**
- **C**: part of the proposed scientific contribution (the V2 receding complete-action receiver: complete-action certification, provisional vs committed plan, single terminal commitment);
- **I**: implementation infrastructure;
- **C/I**: both.

## 1. Complete-action certification stages (F1…F5)

Stages are those of the certifier as logged by `[CertStage]` (Phase C). All checks run on a copied robot state with the preview IK integrator (§10), not on the QP.

| Stage | Location | Meaning | Provenance | If removed/relaxed | Needed to defend | Role |
|---|---|---|---|---|---|---|
| F1 reach | static: `cpp` stepCapturePlanning ReachStandoff (≈9212); route: stepPredictiveRouteCandidate Reach (≈6560–6760) | the arm reaches the object-relative standoff within position 10 mm / 0.055 rad (static) or 12 mm / 0.050 rad (route governor), with swept-volume clearance and joint limits | ENG; the geometry inside is GEO | reach-only selection picks actions later rejected in 9/12 searches (Phase C) | done in part (Phase C); needs broader scenarios | C |
| F2 insertion | static ReachCapture (≈9232); route Approach + Dwell (≈6770–6860) | capture pose reached inside the grasp corridor, then an 80 ms capture dwell under a velocity gate | GEO corridor, ENG dwell | closure attempted from poses outside the corridor | corridor derivation from Robotiq/CALL geometry; dwell sensitivity | C |
| F3 closure/contact | previewClosureStep (5782); previewDynamicClosureSafety (5326) | closure swept in ≥120 samples to bilateral pad contact within contact 1.0 mm / penetration 0.25 mm and an aperture-dependent centering tube | GEO pads, ENG tolerances | the largest downstream rejection class (`closure/pad_pair/blue_handle_acquisition_tube`, 11 705 records in the 191-lead study) | pad/handle contact model vs a real gripper; tolerance sensitivity | C |
| F4 transfer readiness | **no candidate-dependent test** in the certifier (implied by F3) | — | — | removing it changes nothing in selection [C] | a real transfer model (object mass, friction, grip force) would be a new check, not a defence of an existing one | C (claim must be dropped or implemented) |
| F5 carried retreat | static Retreat (≈9265) using previewAttachedRetreatSafe (5089); route Retreat (≈6900); carriedObjectGroundSafe / carriedObjectArmSafe (4986 / 5024) | the arm carrying the attached object reaches the retreat pose (180 mm) clear of ground and arm | GEO retreat model, ENG distance | 0.6–1.6% of route records rejected only here | retreat-distance sensitivity; whether carried-object collisions occur without it | C |
| Terminal timing audit | finalizePredictiveRouteCandidate (6984) → previewVelocityGateParityShadow (5899) | replays the MovePregrasp terminal controller on the copied state; fails the record if the velocity gate does not settle (hard-coded 0.040 m/s, cpp:5936) | ENG; the constant duplicates the MovePregrasp config value | a cost-valid record could still time out at the terminal gate | parity evidence (preview vs runtime terminal durations) | I |
| Cost-audit validity | computeCompletePlanAuditCost (7618) | record is admitted only if every cost term is finite and the timing audit succeeded | ENG | non-finite terms would enter the argmin | none (defensive) | I |

## 2. Seven-term objective

Location: `cpp` computeCompletePlanAuditCost (7618) and barriers (7597); global extension in `src/FiniteEventPlanSelector.h` `extendMotionCostToSearchEpoch`. Config: `yaml` 158–183.

J_motion = Σ w_i φ_i, with weights normalized to sum to 1.

J_global = J_motion + w_T (τ − t_epoch − T_pres) / T_ref.

| Term | φ (dimensionless) | Reference / soft | Weight | Provenance | If removed | Needed to defend | Role |
|---|---|---|---|---|---|---|---|
| T time | audit execution time / 8 s | T_ref 8 s | 0.4211 (= 8/19) | ENG | argmin loses its time preference; the audit and Phase D show it dominates (earliest-τ ≈ argmin in 12/16) | weight sweep; compare with a lexicographic time-first selector | C/I |
| E effort | predicted effort / 8 | 8 | 0.1053 (2/19) | ENG | small | sensitivity | I |
| L path | transit path length / 0.50 m | 0.50 m | 0.1053 (2/19) | ENG | routes with detours no longer penalized | sensitivity | I |
| C clearance | −log((c − 0.020)/(0.080 − 0.020)) for 20 mm < c < 80 mm; 0 above 80 mm; 1e6 below 20 mm; c = min(reach, retreat clearance) | hard 20 mm (= transit minimum), soft 80 mm | 0.1579 (3/19) | ENG form; the 20 mm lower bound is shared with a hard check | loses the preference for safer routes (near-ground: 35 mm clearance difference) | clearance–outcome relation; soft-limit sweep | C/I |
| Q joint margin | −log(ratio / 0.20) | 0.20 | 0.0842 (1.6/19) | ENG | joint-limit-near postures become equal-cost | sensitivity | I |
| K conditioning | −log(index / 0.10), angular rows scaled by 0.20 m | 0.10; 0.20 m | 0.0737 (1.4/19) | ENG | ill-conditioned postures become equal-cost | sensitivity; tracking-error correlation | I |
| V velocity reserve | (joint velocity utilization)⁴ | — | 0.0526 (1/19) | ENG; the exclusion of terminal utilization is EMP-C (comment cpp≈7665) | near-limit trajectories become equal-cost | sensitivity | I |
| R rotation | (angle/π)², **diagnostic, weight 0** | — | 0 | EMP-C ("never changed the winner", comment) | none (already excluded) | re-verify the claim | I |
| Tie tolerance | 1e-9 on J, then a deterministic secondary order (completion time, τ, T_pres, clearance, names) | — | — | ENG | nondeterministic ties | none | I |

**Prior evidence [EMP-V].**
- Audit: time-first lexicographic selection equals FULL argmin on 76–100% of the planner-time grid.
- Phase D: zero-latency argmin = earliest admissible τ then best (g, r) in 12/16 searches.

**The weights have no derivation.** The config calls them "controller-specific engineering preferences … not literature-derived" (yaml 148–150).

## 3. Timing gates and reserves

| Item | Location | Value, meaning | Provenance | If relaxed | Needed | Role |
|---|---|---|---|---|---|---|
| Minimum safe commit lead L_commit | yaml 472; rv2 selector call and bank (≈466, ≈1070) | max(1.6 s, decel 0.85 + 0.25 s) = 1.6 s; the event must be ≥ L_commit ahead at selection | ENG | selects events too close to execute | reach-time and actuation-latency statistics | C/I |
| Minimum reach entry lead L_entry | yaml 476 | 0.05 s; T_pres + L_entry ≤ τ − t | ENG | reach starts late | tracking-latency measurement | I |
| Execution time scale | yaml 229–237 `executionTiming` | armScale 1.25 (preview→runtime), effective gripper rate 0.32 /s, priority blend 0.20 s, capture lock 0.15 s, capture dwell 0.08 s, bilateral dwell 0.10 s, confirmation dwell 0.25 s | EMP-C ("calibrated execution-time model", yaml 226) | predicted times are wrong, so admissions are wrong | re-run the calibration (preview duration vs measured mc_rtc phase durations) | I |
| Bounded lead bank | yaml 459–467 | 14 leads [1.8, 8.0] s, step 0.45 s | ENG (see V2_COMPUTATION_AND_CANDIDATE_SPACE §5) | — | done in part (Δτ ≤ 0.015/v) | C |
| Presentation deceleration | yaml 271 | 0.85 s | ENG (giver model) | — | human giver data | I |
| Presentation acquisition window / V2 no-plan bound | yaml 275; rv2 374, 424 | 7.0 s after quasi-static / after τ | ENG | fails earlier or later | human waiting-tolerance literature | I |
| Maximum event search wall time | yaml 463 | 7.0 s (V1 per-search budget; unused in the V2 worker) | ENG | — | — | I |
| Exact timing prune (Track 1, default off) | rv2 `exactTimingPruneRoutesV2` | uses L_commit and L_entry | derived, lossless [C] | — | — | I |

## 4. Prediction and freshness tolerances

| Item | Location | Value, meaning | Provenance | If relaxed | Needed | Role |
|---|---|---|---|---|---|---|
| Prediction model | rv2 86–120; `propagatePoseConstantTwist` | constant twist from the latest estimate; zero twist below the rest thresholds | ENG (no stop model) | — | a giver motion model; compare with human hand-over data | C |
| Velocity estimator | cpp 3153–3215 | first-order low-pass of finite-difference twist, τ_f = 0.10 s; raw samples rejected above 0.30 m/s or 1.50 rad/s | ENG (standard filter form) | noise or lag in prediction | delay/noise sensitivity (§8, A6) | I |
| Observation | yaml 251–253; st/ObserveObject | 0.90 s observation, ≥ 0.70 s and ≥ 350 samples; moving if displacement ≥ 25 mm and speed > 10 mm/s | ENG | misclassification | classification ROC on giver data | I |
| Perception latency model | yaml 259–263 | off; 0.220 s delay option with compensation | ENG | — | delay experiment | I |
| Rest thresholds (presented / stopped) | yaml 281–282 | 0.004 m/s, 0.08 rad/s | ENG | commits on a moving object | estimator noise floor | C/I |
| Commit-freshness tube | yaml 223–224, 498–499; used in rv2 1440 (commit), 1560 (supersession), adoption freshness | 15 mm / 0.12 rad object drift | ENG | stale certificates accepted | grasp-corridor tolerance derivation (e.g. the pad capture depth is 6 mm) | C |
| Measurement freshness | rv2 1394 | estimate age ≤ perception buffer 1.0 s | ENG | stale estimate used at commit | sensor timing | I |

## 5. Provisional plan: persistence, replacement and recertification

| Item | Location | Meaning | Provenance | If changed | Needed | Role |
|---|---|---|---|---|---|---|
| Persistence rule | rv2 `processReceiverJobResultV2` (RecertifyActive branch), `invalidateProvisionalPlanV2` (1217) | retain while certified; replace only after a certification failure (hold, then full search); no cost-driven switching, no hysteresis parameter | ENG (V2 design decision) | cost-driven replacement could oscillate and delay | A3 ablation | C |
| Recertification frequency | rv2 412 | RECERTIFY_ACTIVE resubmitted whenever the worker is idle during PROVISIONAL_REACH, about every 5 ms (4.2 ms median worker time) | ENG (as fast as compute allows) | lower frequency → larger stale windows (already 14.8 mm robot drift per certificate, Phase B) | A5 | C |
| Recertification content | rv2 `runRecertifyActiveRolloutV2` (≈725) | the same (τ, g, r), resumed at the time index from the moving arm under the newest prediction, through retreat and timing audit | ENG | — | parity with a full search | C |
| State/plan generations, stale rejection, cancellation | rv2 `checkSupersessionV2` (1552), stale checks (≈930) | results used only if their generation matches; cancel on state change or arm drift (12 mm / 0.05 rad) | ENG (standard concurrency hygiene) | stale commits | done (I6, I9 invariants) | I |
| Prediction-update mode | yaml 640 | cancel_and_restart (default) / select_then_certify | EMP-V (V2_PREDICTION_UPDATE_MECHANISM) | — | — | I |
| Adoption start-state premise | rv2 ≈1085 | mouth within 12 mm / 0.05 rad of the snapshot start | ENG | reach starts from an uncertified state | — | I |

## 6. Terminal commitment gate

Location: rv2 `executeTerminalTrackV2` (1351), `stepReceiverV2` TERMINAL_TRACK (≈420–470), `commitProvisionalReceiverPlanV2` (1411). Config: PresentationHold section (yaml 488–500).

| Condition | Value | Provenance | If relaxed | Needed | Role |
|---|---|---|---|---|---|
| t ≥ τ | — | ENG | commits before the planned event | A4 | C |
| Mouth at object-relative standoff | 12 mm / 0.050 rad | ENG | corridor entry misaligned | tolerance sweep | C |
| Object stopped | 0.004 m/s, 0.08 rad/s | ENG | capture of a moving handle (out of scope) | A4 | C |
| Gripper open | closure ≤ 0.05 | ENG | — | — | I |
| Estimate fresh, live clearance safe | age ≤ 1.0 s | ENG | — | — | I |
| Stable dwell | 0.10 s | ENG | chattering commits | sensitivity | I |
| Fresh TERMINAL_CERTIFY taken during the gate | same generation and plan | ENG | stale commit | done (I6) | C |
| Drift since certificate | object 15 mm / 0.12 rad; mouth 12 mm / 0.05 rad | ENG | — | A4 | C |
| Capture corridor and clearance safe now | geometry §9 | GEO/ENG | — | — | I |
| Single latch | `v2CommitLatched_` | design | multiple commits | done (I3, I5) | C |

## 7. Capture, closure, contact and transfer thresholds

Execution states run after commitment; they are **unchanged V1 terminal control**.

| Item | Location | Value | Provenance | If relaxed | Needed | Role |
|---|---|---|---|---|---|---|
| MovePregrasp terminal approach | yaml 502–542; st/MovePregrasp | speeds 0.38 / 0.14 m/s; terminal speed tolerances 0.040 m/s, 0.080 rad/s; stable dwell 0.08 s; tolerances 3 mm / 0.025 rad; centering 0.45 mm | ENG | capture pose error | A7 | I |
| Pad contact model | yaml 287–312; cpp 5326 | pad points (URDF), capture depth 6 mm, contact tolerance 1.0 mm, penetration 0.25 mm, pad centering 4 mm (preview), target tip gap 26 mm | GEO (pad points), ENG (tolerances) | false contact acceptance or rejection | real gripper contact measurements | C/I |
| Acquisition centering tube | yaml 338–341; cpp 1722 | 1.5 mm (far) → 0.55 mm (near) over 10 mm | ENG | closure rejects or off-centre grasps | A7 | C/I |
| CaptureTransfer lock and confirmation | yaml 544–588; st/CaptureTransfer | lock dwell 0.15 s, lock tolerances 4 mm / 0.030 rad / 0.35 mm centre, confirmation dwell 0.25 s, timeouts 2 / 4 / 20 s, centering servo gains 6.0 / 1.2 | ENG | — | A7 | I |
| Virtual load transfer | yaml 353–380; st/CaptureTransfer (≈500–700) | mass 0.346 kg; full support force 3.39426 N (= m·g, **PHY**); virtual contact 800 N/m and 25 N·s/m; release share 0.85 for 0.30 s; timeout 5 s; force/moment limits 80 N / 8 N·m; admittance 1 kg, 35 N·s/m, 80 N/m | PHY (m·g); **simulation-only**, ENG otherwise | the transfer claim is unsupported without physical sensing | hardware force-sensor experiment | I (not evidence of physical transfer) |
| Gripper actuation | yaml 300–333 | close rate 1.6; min rate 0.2; slow distance 6 mm; command leads 0.18 / 0.025 s; closure guard 0.08; joint weights 25 / 2400 | ENG; hardware bridge values measured (0.87% open, yaml 328–333) | — | — | I |

## 8. Retreat

| Item | Location | Value | Provenance | If relaxed | Needed | Role |
|---|---|---|---|---|---|---|
| Retreat distance | yaml 81 | 0.180 m retreat offset (candidate retreat pose; direction set in `buildCandidate`, cpp 7314) | ENG | carried-retreat certification weaker or stronger | A8 sweep | C/I |
| Carried-object safety | cpp 4986, 5024, 5089 | carried-object capsules vs ground and arm | GEO | collisions after grasp | — | C |
| Retreat state | yaml 590–614; st/Retreat 254 | success within 12 mm / 0.060 rad while safe; speeds 0.50 m/s, 1.25 rad/s | ENG | — | — | I |

## 9. Safety and clearance geometry

| Item | Location | Value | Provenance | If relaxed | Needed | Role |
|---|---|---|---|---|---|---|
| Object model | yaml 55–68; cpp ≈4270 | CALL: sensor capsule r 17 mm, half-length 18.2 mm; handles r 11.25 mm, half-length 68.7 mm; O_T_H 86.9 mm | GEO | — | CAD check | I |
| Gripper model | cpp `refreshGripperGeometry` (≈3633) | hand-placed proxy spheres approximating robotiq_base.stl (base 32 mm, palm 20 mm, corners 13 mm, …) | ENG approximation of GEO | clearance is only as conservative as the proxies | bound the proxy error against the STL | I |
| Mouth frame | yaml 59–61 `B_T_M` | 98.3 mm offset | GEO | — | — | I |
| Safety margins | yaml 68, 73–74, 293 | object margin 3 mm, ground margin 15 mm (object) / 10 mm (arm), corridor margin 3 mm | ENG | collision risk | A8 | I |
| Grasp corridor | yaml 292–298; cpp 4061, ≈5426 | max angle 10°, entry depth 135 mm, palm limit 4 mm, axial ±35 mm, finger inset 10 mm | GEO-motivated ENG | closure outside the valid mouth | derive from Robotiq 2F-85 opening (85 mm) and handle radius | C/I |
| Transit clearance (hard) | yaml 193 | minimum predicted route clearance 20 mm | ENG | — | A8 | C/I |
| Reach governor | yaml 201–224 (shared V1/V2 `predictiveReachPolicy`) | slowdown starts at 35 mm clearance, hard margin 16 mm, runtime minimum 8 mm, speeds 0.38 / 0.16 m/s far/near, tracking leads 35 / 12 mm | ENG | — | A8 | I |
| Route shape | yaml 189–196; cpp 1882 | ring of cubic Bézier offsets (8 directions × 80/140 mm), max path stretch 1.90, clearance preference band 10 mm (only affects per-grasp best in the legacy path, not the V2 global selector [C, cpp 7945 vs selector]) | ENG | see V2_COMPUTATION_AND_CANDIDATE_SPACE §5.3 | — | C (bank) / I (band) |
| Standoff distance | yaml 80 | 0.120 m | ENG | — | sweep | C/I |

## 10. mc_rtc execution assumptions

| Item | Location | Value | Provenance | If violated | Needed | Role |
|---|---|---|---|---|---|---|
| Control period | `configs/mc_rtc.yaml.example` Timestep | 0.001 s | ENG (mc_rtc default) | timing-model mismatch | — | I |
| QP constraints | yaml 382–386 | kinematics constraint with damper [0.1, 0.01, 0.5] and 95% velocity limits; **no collision constraints** (`collisions: []`); safety is a supervisory filter | ENG | collisions not prevented by the QP | stated as an assumption | I |
| Task gains | yaml per state (e.g. ReceiverV2 terminal 40 / 5400, reach 58 / 6600, hold 18 / 4200) | stiffness / weight | ENG | tracking lag vs preview | preview/runtime parity study | I |
| Preview IK integrator (certification model) | yaml 89–126 `preview`; cpp `previewReachStep` (5452) | dt 0.02 s, gains 3.5, damping 0.04, ≤ 700 iterations per segment, speed caps 0.70 m/s / 2.2 rad/s | ENG | certification ≠ execution | **parity evidence required**: the preview is a separate kinematic integrator, not the QP | C/I |
| Simulation | runner | kinematic mc_rtc_ticker, kinematic object attachment, virtual transfer sensor | — | no dynamics or contact physics | hardware or dynamics simulation | I |
| Worker concurrency | cpp runPlannerWorker; rv2 runReceiverWorkerJobV2 | one background thread on frozen snapshots; wall-clock worker vs controller clock (≈ real time) | ENG | results arrive later if the controller runs faster than wall time | — | I |

## 11. State machine

`yaml` 647–678.
- **V1:** Initial → ObserveObject → SolveInterception → ExecuteCommittedReach → PresentationHold → MovePregrasp → CaptureTransfer → Retreat → Completed.
- **V2:** ObserveObject → ReceiverV2 → MovePregrasp → CaptureTransfer → Retreat → Completed.
- Every state has FAIL → Failure. There is no recovery path after commitment and no re-grasp.
- Provenance ENG; role C for the V2 single-commitment structure, I otherwise.
- Defence needed: failure-mode taxonomy, including whether a post-commit abort or retry is required for human hand-over.

## 12. Other numerical constants of note

| Constant | Location | Value | Remark |
|---|---|---|---|
| Hard-coded terminal velocity tolerance | cpp 5936, 7060 | 0.040 m/s | duplicates MovePregrasp `terminalLinearSpeedTolerance`; can drift out of sync |
| Hypothesis budget (V2) | rv2 bank | leads + 1 | fixed in Phase B (`1139d31`) |
| Settle tolerances (arm held) | yaml 431–432 | 2.5 mm/s, 0.015 rad/s | full search waits for these |
| Memoization | cpp stepFiniteTriadSearch V2 branch | bitwise-identical pose | exact, computational |
| Corridor angle weighting in worst clearance | cpp ≈5438 | 0.05 × angle clearance | ENG scaling of rad into metres in a min() |

## 13. What is claimed vs what is infrastructure

- **Scientific contribution candidates (C).**
  - Complete-action certification through F1–F3 and F5, plus the terminal timing audit as its execution-parity guard.
  - Provisional vs committed plans with single terminal commitment.
  - Persistence/recertification.
  - The commit gate and freshness tube.
  - The prediction interface.
  - The T/G/R bank (already studied).
- **Not defensible as currently claimed.**
  - F4 transfer readiness: no test.
  - Physical load transfer: simulation-only virtual sensor.
  - The seven weights: no derivation.
  - The execution-time calibration: EMP-C, evidence not located.
- **Infrastructure (I).** Estimator, observation gating, gains, state-level servo parameters, mc_rtc assumptions, concurrency hygiene.

## 14. Minimal ablation matrix (proposed, not implemented)

**Scoring.**
- **Scientific importance** (S1 highest): how directly the ablation tests a contribution claim or a load-bearing assumption.
- **Computational cost:** Low = config/replay only; Medium = new runs with existing code paths plus small switches; High = new mechanism or hardware.
- **Runs:** in units of the current 4-scenario × 3-repeat set (12 runs ≈ 13 min). Offline replays reuse Phase D logs where marked.

| Rank | ID | Ablation | Arms | Primary measures | Importance | Cost | Notes |
|---:|---|---|---|---|---|---|---|
| 1 | A1 | Certification depth F1…F5 | select using F1 only; F1–F2; F1–F3; F1–F3 + F5; full (+ timing audit) | P(selected action later rejected by full certification); completion; failure stage | S1 | **Low** offline (Phase C logs already give F1-only and full); Medium for runtime arms | the core claim; offline already shows 9/12 rejections for F1-only |
| 2 | A6 | Prediction delay/noise sensitivity | perception delay 0 / 0.11 / 0.22 s (existing option), ± compensation; additive pose noise σ ∈ {0, 1, 3} mm | completion, freshness rejections, commit drift, time to first plan | S1 | Medium (delay exists; noise needs a small injection) | tests the receding claim under realistic sensing |
| 3 | A4 | Commitment timing | early commit (V1 pre-reach, existing), current terminal gate, later commit (require longer stable dwell or post-contact) | post-commit failures, completion, capture time | S1 | Low–Medium (V1 vs V2 already exists; "later" needs a dwell change) | the single-commitment contribution; V1-independent vs V2 already measured on 4 scenarios |
| 4 | A2 | Selector | seven-term J; time-first lexicographic; earliest admissible τ + max clearance; J without C, K, Q, V (T+E+L); single-term T | J regret, clearance, completion, P(tuple equal) | S2 | **Low** (offline replay over Phase D records) | prior evidence: time dominates; decides whether seven terms are needed |
| 5 | A3 | Persistence vs replacement | retain-while-certified (current); replace when a new search finds ΔJ < −δ (δ ∈ {0.05, 0.1}) | switches, oscillation, completion, time to capture | S2 | Medium (needs background searching during reach, which the architecture currently blocks) | only if persistence is claimed as a contribution |
| 6 | A5 | Recertification policy | continuous (current); every 50 ms; every 200 ms; only at standoff (TERMINAL only) | invalidation latency, stale-certificate drift, worker load, completion | S2 | Medium (rate limiter) | quantifies the value of continuous recertification |
| 7 | A7 | Terminal/transfer thresholds | centering tube ×{0.5, 1, 2}; commit rest threshold ×{0.5, 1, 2}; confirmation dwell {0.1, 0.25, 0.5} s | capture success, closure failures, time | S3 | Medium | infrastructure robustness; unchanged V1 terminal control |
| 8 | A8 | Safety margins | hard transit clearance {10, 20, 30} mm; retreat distance {0.12, 0.18, 0.24} m; object margin {0, 3, 6} mm | feasibility rate, clearance distribution, completion | S3 | Medium–High (needs runs across scenarios, and collision ground truth for 0 mm) | margins are ENG; results change feasible sets, not the architecture |
| — | A9 | Execution-time calibration check | measure preview vs runtime phase durations for committed actions | calibration error | S2 (prerequisite for timing gates) | Low (logs exist) | validates EMP-C armScale 1.25 |

**Ordering by cost within importance.**
- **S1:** A1 offline (now) → A4 (reuse V1/V2) → A6.
- **S2:** A2 offline → A9 → A5 → A3.
- **S3:** A7, A8.

**Not proposed.**
- Changing the prediction model or the terminal controller (out of scope).
- F4 as an ablation (it has no test to remove).
