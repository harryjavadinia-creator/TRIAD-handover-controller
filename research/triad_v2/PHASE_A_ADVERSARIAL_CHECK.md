# TRIAD V2 — Phase A adversarial check

Baseline audited: `main @ 21d70226e4afe681e513433e9dfc3c29f70497e6` (TRIAD V1).
Branch for V2: `research/triad-v2-dynamic-control`.
Inputs read: current source and FSM, background-worker code, the TRIAD
Foundations Audit (2026-09-13), and primary literature.

Evidence tags: **[CODE]** source at baseline, **[LIT]** primary literature
(full text unless marked *abstract*), **[INF]** inference.

Issue classes: **FATAL** · **MUST FIX** · **SHOULD FIX** · **NON-BLOCKING**.

Verdict: **no FATAL blocker.** The frozen V2 architecture is implementable and
scientifically coherent once the MUST-FIX corrections below are applied. Each
correction is the smallest change that keeps the architecture's own
requirements consistent. None of them introduces a new research story.

---

## A. Scientific blockers

| # | Issue | Class | Evidence | Smallest correction |
|---|---|---|---|---|
| A1 | **Terminal capture cannot close on a moving handle.** The frozen terminal chain is quasi-static: PresentationHold requires object speed ≤ 0.004 m/s and ≤ 0.08 rad/s before release; MovePregrasp inserts to a fixed world pregrasp pose locked from the stopped object; the preview's terminal chain holds the virtual object at the presentation anchor. Under an independent constant-velocity prediction, no candidate would ever certify, and the robot would never move. | **MUST FIX** | [CODE] `PresentationHold.cpp:187–192`; `lockStaticPresentationToCurrentObject cpp:3186–3212`; `stepPredictiveRouteCandidate cpp:6578, 6621–6806` | Make terminal certification **explicitly conditional**: “if the object is presented (quasi-static) at the predicted pose at τ, the complete action (reach → insertion → closure → transfer readiness → carried retreat) is feasible.” The **commit gate verifies the premise from current measurements** (object speed, pose freshness, robot at standoff). Terminal controller, capture and transfer are unchanged, as required. |
| A2 | **An independent giver must still present.** With A1, capture succeeds only if the giver eventually comes to rest; a giver that never stops can never be captured by the frozen terminal controller. | **MUST FIX** (scenario definition) | [CODE] as A1 | The V2 simulated giver is a **robot-independent scripted presentation**: constant initial velocity, then a smooth quintic stop to rest after travelling `movingObject.maximumSimulatedTravel` (0.40 m, already present), over `presentationDecelerationDuration` (0.85 s, already present). No new tuned parameter is introduced, and nothing in it reads a receiver plan. This is a *scope* condition (dynamic presentation ending in a quasi-static transfer), not a relabelling of τ. |
| A3 | **V1's execution guards contradict V2 semantics.** V1 fails closed when the object leaves a 15 mm tube around the *committed* prediction and rejects commit on prediction drift. Applied pre-commit in V2, the first prediction update would abort the handover instead of re-certifying it. | **MUST FIX** | [CODE] `ExecuteCommittedReach.cpp:277–289`; `commitGlobalTimePlanSelection cpp:8587–8608` | V2's provisional phase replaces “deviation ⇒ fail” with “deviation ⇒ re-certify; infeasible ⇒ replace.” Deviation limits are applied **once**, at the commit gate, against the terminal-certification snapshot. **Safety guards (live clearance, swept-command filter) stay fail-closed.** |
| A4 | **V1's robot-imposed giver must be removed from V2 and from the comparison.** V1 writes the committed τ and deceleration into the simulated truth. Comparing V2 (independent giver) with V1 (obedient giver) would confound architecture with environment. | **MUST FIX** | [CODE] `cpp:3009–3016, 8817–8828` | The giver model becomes a configuration choice: default `committed_plan` reproduces V1 history byte-for-byte; `independent_scripted` is mandatory in V2 and is also used for **V1-under-independent-giver** comparison runs. V1 code paths are not deleted. |
| A5 | **The “complete-action” extension may be empty.** If retreat and transfer-readiness checks never reject a candidate that reach + insertion + closure accept, the distinguishing part of the contribution is vacuous. | **SHOULD FIX** (measure) | [INF]; failure reasons are already logged per route rollout (`predictive_static/static_retreat/…`) | Log the rejecting stage of every certification attempt and report the fraction rejected **only** by closure / retreat stages. Required before any contribution claim. |
| A6 | **Dependence on one predictor.** V1 bakes a stopping giver into `predictPresentationPose`. | **MUST FIX** | [CODE] `cpp:2101–2117` | V2 plans against an abstract prediction record (generation, timestamp, pose(t) query). The default implementation is constant-twist extrapolation of the existing estimator, with **no stop assumption**; any WP-supplied predictor can replace it without touching certification. |

## B. Prior art threatening the proposed contribution

Hypothesis tested: *TRIAD maintains and updates complete receiver-action
candidates while the robot moves; each candidate is certified through
terminal acquisition, closure/contact, transfer readiness and safe carried
retreat; irreversible commitment occurs only at the terminal capture
boundary.*

| Work | Robot moves while object moves? | What is maintained / re-selected | Certification horizon before commitment | Commitment boundary | After commitment |
|---|---|---|---|---|---|
| Yang et al. 2021, ICRA [LIT] | Yes (tracks human at ~10 Hz) | Grasp re-scored every step (quality, temporal consistency, home bias); first grasp with IK + collision-free **joint path** and collision-free **Cartesian standoff→grasp path** | Reach + insertion path (kinematic, collision). No closure, transfer or retreat check | Arrival at standoff (10 cm) → open-loop grasp | Close; drop at preset place; closure-width failure check |
| Yang et al. 2022, ICRA [LIT] | Yes (stochastic MPC, 2 s horizon) | Goal set over grasps inside MPC cost; learned reachability/manipulability ranking; hand-collision constraints | Motion to standoff (MPC constraints) + learned reachability | Standoff (15 cm) → **blocking** grasp policy | Close; retreat to standoff; **retry** Approach on missed grasp |
| Christen et al. 2023, CVPR [LIT] | Yes (“moving simultaneously with the human”) | Closed-loop learned approach policy | **Learned** predictor of forward-grasp success, trained by executing the grasp from pre-grasp poses | Pre-grasp reached **and** predicted success > threshold → open-loop grasp | Close; predetermined retract (not certified) |
| Akinola et al. 2021, IROS [LIT] | Yes | Grasps re-filtered each loop by reachability SDF + motion-aware grasp-quality net; seeded replanning | Reachability + learned grasp success under motion | `CanGrasp` (≤1.1 b, ≤20°) → final 1 s prediction → blocking grasp | Close while moving 0.1 s; lift check |
| Marturi et al. 2019, Auton. Robots [LIT] | Yes (arbitrary 6-DoF human motion) | Grasp re-selection by reachability + collision-free trajectory; local/global planner switching | Reach to pre-grasp | Not a formal boundary (tracking; grasp when feasible) | Grasp |
| Burgess-Limerick et al. 2023, ICRA [LIT] | Yes (base and arm on the move) | Reactive tracking | None beyond reactive control | Distance < 0.1 m → final-phase grasp controller | Servo grasp |
| Salehian et al. 2016, RSS [LIT] | Yes | Intercept time T\* recomputed by workspace likelihood | Reachability of fixed object reaching points | Interception (DS convergence) | – |
| Islam et al. 2020/2021, RSS/IJRR [LIT] | Yes (conveyor) | Constant-time replanning to updated goals | Path to pregrasp (preprocessed coverage) | **Replan cutoff t_rc**, after which the last path is executed | Grasp controller |
| Menon et al. 2014, ICRA [LIT *abstract*] | Yes | Time-parameterised kinodynamic plan, earliest feasible pickup | Arm + end-effector trajectory | – | – |
| Oelerich et al. 2024 [LIT] | Yes | Path-following MPC with GPR handover-location prediction | Motion within adaptive path bounds | None (continuous) | – |
| Djeha et al. 2022, RO-MAN [LIT] | Yes | Task-space QP with handover constraints | Instantaneous QP constraints | None (continuous) | Haptic exchange |
| Yan et al. 2024 T-RO; 2026 catching (arXiv 2605.28462) [LIT *abstract*] | Yes (catching) | Interception motion | Includes **post-impact capture stability** | Impact | Compliant capture |

**Answer to the exact question.** In the works inspected at mechanism level,
three ingredients are each already established:

- concurrent motion with continuous re-selection;
- a late commitment boundary at standoff or pre-grasp (Yang 2021/2022, Christen 2023, Akinola 2021, Islam 2020);
- certification of the terminal grasp *before* commitment (Christen 2023, learned; Yang 2021, path-level).

**No inspected work maintains, while the receiver moves, an explicit
model-based certification that extends through closure/contact, transfer
readiness and carried-object retreat and gates a single late commitment on
it.** That absence is *not* proof of novelty. Downstream-feasibility-aware
grasp selection is classical in static pick-and-place / manipulation planning,
and catching work now certifies post-impact capture (abstract level).

**Remaining defensible distinction (to be tested, not claimed):** the
*certification horizon* (through carried retreat), *maintained while moving*,
with *explicit time-indexed interception candidates* along an independent
prediction, and a *single model-verified commitment at terminal capture*.
Its value depends on A5: showing that the post-grasp stages actually reject
candidates, and that doing so changes outcomes.

## C. Control-theoretic issues

| # | Issue | Class |
|---|---|---|
| C1 | **Not MPC.** No finite-horizon optimal control problem over inputs is re-solved each sample; the incumbent is *retained* while feasible rather than re-optimised; decisions are discrete plan identities; the only optimisation per cycle is the instantaneous task QP. Calling V2 “MPC” would be false. | NON-BLOCKING (terminology) |
| C2 | **“Receding complete-action supervisory controller over a task-space QP.”** Defensible *if qualified*. The candidate horizon recedes with t_k and a supervisor selects references for a lower QP layer. More precise: **an event-triggered receding-horizon supervisory planner with plan certification and a single irreversible commitment transition (a hybrid supervisor) over a task-space QP.** “Event-triggered” because re-search happens on invalidation; “supervisor” because it switches discrete modes and plans. Avoid “controller” in formal claims unless closed-loop properties are shown. Closest control/planning lineage: real-time replanning with safe partial commitment (Hauser 2012, Auton. Robots) and run-time-assurance fail-closed switching. | SHOULD FIX (wording) |
| C3 | **No recursive feasibility.** Nothing guarantees that a plan certified at generation k remains certifiable at k+1 (no terminal invariant set, no certified fallback). “Maintaining feasibility” means *monitoring and re-certification*, not invariance. | MUST STATE (limitation) |
| C4 | **Fallback while replacing.** Holding the arm while the object moves is not certified safe against the object; V1 has the same exposure during observation and planning. The live clearance monitor stays fail-closed. | SHOULD FIX (documented; later: certified braking fallback) |
| C5 | **Switching chatter.** Persistence (retain while certified) removes cost-driven switching; infeasibility-boundary oscillation remains possible. Every replacement is logged with its reason; no hysteresis parameter is added. | NON-BLOCKING (measure) |
| C6 | **Certification latency versus motion.** A full V1 bank generation takes 2–4 s of wall time (evidence/async worker_timing). A result computed from a snapshot of a moving robot is stale by up to ~1 m of arm travel at 0.38 m/s. | **MUST FIX** (see D-list) |

## D. Code-level blockers

| # | Blocker | Class | Code |
|---|---|---|---|
| D1 | **Worker reads live robot state.** The corridor hard check uses `liveMouthHalfGap()` → `livePadCenters()` → `robot().frame(…)`; IK and limit check read `robot().ql()/qu()`; the kinematic cache may read `robot().vl()/vu()` on first use. With a moving robot these are data races and non-snapshot inputs. The header-inline call is invisible to the existing purity checker. | **MUST FIX** | `cpp:3935, 3449–3453, 5091–5092, 5426–5427, 3861–3862`; `h:636` |
| D2 | **Worker reads estimator members the control thread keeps writing in V2**: `objectLinear/AngularVelocityEstimate_` (plan twist), `objectMotionEstimateValid_` (route-bank gate), `observedObjectMode_`. Race-free in V1 only because estimation pauses during planning. | **MUST FIX** | `cpp:2009, 2096–2097, 7574–7576, 3057` |
| D3 | **Worker writes controller-owned selection members** (`capturePlanningStatus_`, `planningCostSelection*`, `planningBaseOutward_`, …) that V1's control thread reads only after `Ready`. V2's control thread must never read these while a job runs. | **MUST FIX** | see worker-read audit |
| D4 | **Staleness is undefined for repeated jobs.** Generation IDs exist for one submission only (`plannerRequestGeneration_`); there is no state/prediction generation and no rule preventing an old result from changing a newer plan or committing. | **MUST FIX** | `cpp:8061–8163`; `SolveInterception.cpp:669–688` |
| D5 | **Plan start-state premise.** Every rollout seeds from the frozen `q` with zero velocity and starts the reach curve at the frozen mouth pose; V1 aborts launch if the robot is > 12 mm from that anchor. A full-bank result therefore cannot be adopted by an arm that moved during a multi-second search (C6). | **MUST FIX** | `cpp:6381–6385, 7393–7398`; `ExecuteCommittedReach.cpp:318–331` |
| D6 | **No provisional/committed distinction.** `candidateSelected_`, `interceptionCommitted_` and `committedInterceptionPlan_` are set together by `commitCandidate`. | **MUST FIX** | `cpp:8719–8939` |
| D7 | **Legacy commit side effects.** `commitCandidate` creates the simulated-truth plan from the commit; binding-cost commit proofs assume one pre-reach selection. | **MUST FIX** (in V2 path) | `cpp:8726–8748, 8817–8834` |
| D8 | **Timing-audit constants duplicated as literals; unused YAML keys; dead refresh after `return`.** | NON-BLOCKING (leave V1 untouched) | `cpp:5767–5791, 3329–3331` |

## E. Minimal implementation plan

Constraints honoured: V1 default behaviour unchanged; bank resolution, route
geometry, weights, hard checks, terminal controller, capture, transfer and
retreat unchanged.

1. **Worker snapshot hardening (D1–D3), behaviour-preserving.** Copy into the planning snapshot at freeze: open-mouth half-gap, joint position and velocity limits, object pose, twist, estimate validity and observed mode, and prediction generation/timestamp. Replace the worker reads. Extend the purity checker to follow header-inline helpers and to forbid the estimator members. V1 regression: FrozenPlanSet hashes unchanged.
2. **Independent giver (A2, A4).** Pure function `giverTruthPoseAt(t − t_obs, scenario)` in a dependency-free header with a unit test. Config `movingObject.giverTruthModel ∈ {committed_plan (default, V1), independent_scripted}`. In independent mode, no plan-to-truth path exists; each cycle logs a truth sample and asserts that it equals the pure function.
3. **Prediction record (A6).** `ObjectPredictionRecord {generation, stamp, pose, twist}`, with constant-twist `poseAt(t)`. The V2 bank uses `poseAt(t_k + h)` for the same 14 leads; certification uses the conditional presentation model (constant velocity up to τ, then presented at rest).
4. **Provisional vs committed (D6, D7).** New `ProvisionalReceiverPlan` owned by the control thread, copied out of the worker result. `commitProvisionalReceiverPlanV2()` is the only path to `committedInterceptionPlan_` in V2, guarded by a once-only latch. No simulated-truth plan is created.
5. **Generations (D4).** `planningGeneration` (per job) and `receiverStateGeneration` (incremented on every plan adoption, replacement, phase change or commit). A result is used only if its generation matches the latest submitted job **and** its state generation matches the current one. Latest-only: at most one job in flight, and new work is submitted only when the worker is idle.
6. **Job types (C6, D5).**
   - **FULL_SEARCH** (unchanged V1 finite search from a snapshot of a *held* arm): used initially and after invalidation.
   - **RECERTIFY_ACTIVE** (one (τ,g,r) rollout from the *current moving* state, resuming the plan at the current time index, with the object at the newest prediction): used continuously while moving. Under the persistence rule, non-incumbent candidates are needed only for replacement, so the bank is regenerated only when the incumbent fails.
   - **TERMINAL_CERTIFY** (grasp g from the current state at the current object estimate: standoff → corridor insertion → closure sweep → carried retreat, same hard checks): used after τ and at the commit gate.
7. **V2 FSM state `ReceiverV2`** (`ObserveObject` emits `V2` only when `receiverArchitecture: v2_receding`). Phases: `SEARCH_HELD` → `PROVISIONAL_REACH` (same governor, lead tube and safety filter as ExecuteCommittedReach; closure unauthorised) → `TERMINAL_TRACK` (track the object-relative standoff of the current estimate) → `COMMIT` (current-state gate + fresh TERMINAL_CERTIFY) → `OK` → existing MovePregrasp → CaptureTransfer → Retreat. Invalidation leads to hold and FULL_SEARCH. Unsafe conditions lead to Failure. The no-plan budget reuses `maximumEventSearchWallTime` (7 s); the terminal wait reuses `presentationAcquisitionWindow` (7 s).
8. **Evidence checkers.** `tools/check_v2_run_log.py`: giver independence, concurrent motion, generations while moving, provisional update/replacement, stale rejection, exactly one commit, no post-commit reselection, capture, transfer and retreat. `tools/check_giver_truth_independence.py`: cross-run truth identity. Fault injection: a forced stale generation must be rejected.
9. **Runs.** V1 regression (default config, 4 scenarios, hashes); V2 on the 4 scenarios; V1 under the independent giver on the same 4.
