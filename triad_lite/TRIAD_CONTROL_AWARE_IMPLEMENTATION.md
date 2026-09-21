# TRIAD_CONTROL_AWARE_IMPLEMENTATION

TRIAD-lite: TRIAD as a **control-aware supervisory grasp/entry selector**, implemented behind a feature flag. Audit and verdict (IMPLEMENT WITH MODIFICATIONS): [TRIAD_CONTROL_AWARE_SUPERVISOR_AUDIT.md](TRIAD_CONTROL_AWARE_SUPERVISOR_AUDIT.md).

Branch `research/triad-control-aware-supervisor`, created from `research/triad-continuation-control` @ `0e26083`. **Not validated for handover outcome.**

## 1. Switch

`etc/HandoverInterceptionController.in.yaml`, state `HandoverInterceptionController_ReceiverV2`:

```yaml
supervisorMode: bank_search   # default: TRIAD V2 unchanged
# supervisorMode: control_aware   # TRIAD-lite
```

TRIAD-lite also requires `receiverArchitecture: v2_receding` (run with `TRIAD_RECEIVER_MODE=v2`). Override used in the simulation evidence: `evidence/runs_sim_20260916/overrides/control_aware.yaml`.

## 2. Files changed

| File | Change |
|---|---|
| `src/ControlAwareGraspSupervisor.h` | **new**, pure Eigen: grasp family, QP joint-velocity box, box-constrained least squares, directional reserve, lexicographic selection, hysteresis |
| `src/HandoverInterceptionController.h` | new job type `ControlAwareSelect`, phase `ControlAwareTrack`, `ControlAwareCandidateEvalV2`, parameters, members, declarations (additive) |
| `src/ReceiverV2.cpp` | gated hooks in 8 existing places + new functions at the end of the file (see §4) |
| `etc/HandoverInterceptionController.in.yaml` | `supervisorMode` and `controlAware:` block (default `bank_search`) |
| `tools/test_control_aware_grasp_supervisor.cpp`, `tools/run_control_aware_supervisor_unit_tests.sh` | **new** unit tests |
| `.github/workflows/source-checks.yml` | installs `libeigen3-dev`; runs the new unit tests |
| `research/triad_lite/` | audit, this report, offline check (`tools/authority_offline.py` + `evidence/authority_offline.json`), log summarizer, simulation evidence |

## 3. Exact equations implemented

### 3.1 Grasp family (`generateGraspFamily`, geometry from the existing `buildCandidate`)

g = (σ, φ), σ ∈ {+1, −1}, φ_k = 2πk/N (N = `graspsPerSign` = 32).
z_M = σ z_H; y_M = Rot(z_M, φ)·outward, orthogonalized; x_M = y_M × z_M.
Capture point = p_H + 6 mm·y_M; standoff = capture + 0.12 m·y_M; retreat = capture + 0.18 m·y_M.
Object-relative grasp frame: ^O T_G = (W T_O)⁻¹ · W T_M,capture. It is logged per candidate.

The pure `graspFrameRotation` is checked against `buildCandidate` in every selection (`frameConsistency`; 0.000 in all simulation runs).

### 3.2 Layers (`runControlAwareSelectionV2`, worker thread, frozen current state)

1. **Robot reach**: `previewReachSegment` to the standoff. Failure is classified as `ik` (no convergence), `joint_limits`, `collision` (ground plane) or `geometry` (object/handle interference).
2. **Geometry**: `previewReachSegment(requireCorridor = true)` to the capture pose, then `previewClosureSweep` (bilateral pad contact inside the acquisition tube, no penetration).
3. **Robot, carried retreat**: `previewReachSegment(attachedRetreat = true)`.
4. **Security distance**: no limited joint within s_Dist of a limit at q_standoff or q_capture.
5. **Clearance floor**: c(g) = min(reach clearance, carried-retreat clearance) ≥ 0.025 m.
6. **Authority**: κ*(g) ≥ κ_min = 1 (§3.3).

### 3.3 Authority (`qpJointVelocityBox`, `boxConstrainedLeastSquares`, `directionalReserve`)

**QP joint-velocity box** for the 7 arm joints, read from the controller's own `constraints: kinematics` entry (damper [0.1, 0.01, 0.5], velocityPercent 0.95). This mirrors `tasks::qp::DamperJointLimitsConstr::update`:

- l_i = 0.95·vl_i, u_i = 0.95·vu_i;
- for a limited joint with range r, i_D = 0.1 r and s_D = 0.01 r:
  - if d_low < i_D: l_i ← max(l_i, −ξ (d_low − s_D)/(i_D − s_D));
  - else if d_up < i_D: u_i ← min(u_i, ξ (d_up − s_D)/(i_D − s_D));
- ξ = damper offset 0.5, the history-free lower bound of Tasks' ξ, giving an inner approximation.

Acceleration and jerk bounds are ±∞ for this robot module, so the velocity level is exact for this QP.

**Demand** (tool body `gen3_robotiq_85_base_link`, world frame, [ω; v]):

- `follow`: ω = ω̂_O, v = v̂_O + ω̂_O × (p_B − p_O);
- `follow_insert`: the same, plus v_ins·(−y_M), with v_ins = MovePregrasp `farLinearSpeed` = 0.38 m/s;
- optional `disturbanceSpeed` adds ±x, ±y, ±z demands (default off).

**Residual**: ε = min over q̇ ∈ box of ‖W(J q̇ − y)‖, with W = diag(L, L, L, 1, 1, 1), L = 0.20 m. Solved by a primal active-set method on ½‖A x − b‖² + ½ρ‖x‖², ρ = 10⁻¹⁰.

**Reserve**: κ* = max{κ ∈ [0, 8] : ε(κ y) ≤ 10⁻⁴}, found by bisection. This is exact because ε(κ) is convex, so its sublevel set is an interval. κ* exceeds the true supremum by at most tol/‖W y‖.

The per-candidate reserve is the minimum over both demands at both configurations.

**Runtime gate** (control thread): ε(κ_min·y) ≤ tol at the live robot state for the same demands, computed with one bounded least squares per demand (no bisection).

### 3.4 Selection and hysteresis (`selectGraspLexicographic`, `updateGraspSelector`)

Cascade, no weights:

1. S1 = admissible;
2. S2 = within 5 mm of the best clearance;
3. S3 = within 0.10 of the best min(κ*, 2);
4. shortest reach distance (translation + L·rotation), ties by id.

Hysteresis rules:

- an incumbent inside S3 is kept;
- a dominated incumbent is replaced only after the same challenger stays best for 0.30 s;
- an incumbent re-evaluated as inadmissible is replaced immediately (`trustIncumbentReevaluation: true`, see §6.3);
- with no admissible grasp, the selector aborts to hold.

### 3.5 τ as a feedback event (`stepControlAwareTrackV2`)

**Tracking.** The reference is rate-limited toward W T̂_O(t)·^O T_M,standoff (live estimate), using the V2 clearance governor and speed/lead limits (`predictiveReachPolicy`), then the geometric safety filter. Feedforward is the reference motion. The posture target is the certified standoff posture.

**Gate** (τ_g = first time it holds for `terminalStableDwell` 0.10 s):

- ‖e_p‖ ≤ 12 mm and e_R ≤ 0.05 rad to the live standoff;
- object quasi-static (|v̂| ≤ 4 mm/s, |ω̂| ≤ 0.08 rad/s; `requireObjectStopped`);
- gripper open, perception fresh, command safe;
- runtime authority gate satisfied.

**Admission.** A fresh `TERMINAL_CERTIFY` issued within the stable gate, then the unchanged `commitProvisionalReceiverPlanV2` (drift, corridor and clearance checks). Commit freezes the grasp and hands authority to MovePregrasp.

**Abort.**
- Before commit: an inadmissible incumbent, a failed terminal certificate or no admissible grasp returns to hold, and selection resumes.
- After commit: no retry (the FSM has no path back; that belongs to the acquisition-controller work).

## 4. Exact mc_rtc constraints used by the evaluator

Only `constraints[type = kinematics]` (Tasks backend → `DamperJointLimitsConstr`): position damper plus velocityPercent × URDF velocity limits. The QP has no collision constraints (`collisions: []`) and no contacts. Acceleration and jerk bounds are infinite (mc_rbdyn defaults; the Gen3 module declares none).

Excluded on purpose: task weights, posture task (soft), and the external geometric filter, which is handled by the clearance layer.

## 5. What remains approximate

- **Damper gain.** ξ is lower-bounded by the damper offset (inner approximation); Tasks' actual ξ depends on the joint velocity at damper activation.
- **Evaluated configuration.** q_standoff and q_capture come from the preview IK (barrier-DLS), not from the configuration the QP actually reaches; the IK branch can differ.
- **Kortex saturation.** Kortex-internal velocity/acceleration saturation (POSITION mode) is not modelled.
- **Demand model.** The demand is the current object-twist estimate plus nominal insertion speed; there is no prediction and no disturbance set by default.
- **Inherited closure test.** The geometric closure layer inherits the preview certifier's sub-millimetre acquisition tube, which dominates rejections (§6).
- **Simulation fidelity.** The simulation is kinematic, contact is geometric, force is virtual.
- **Grasp coordinates.** Axial position s along the handle is not a decision variable.

## 6. Validation

### 6.1 Build and checkers (all PASS)

- Out-of-tree RelWithDebInfo build: 0 errors, 0 warnings.
- `tools/run_control_aware_supervisor_unit_tests.sh`: PASS. It covers:
  - grasp family and frame geometry;
  - the exact damper/velocity box and the security-distance flag;
  - active-set least squares against **brute-force enumeration** (60 random problems);
  - well-conditioned vs constrained direction, rank deficiency, and the obvious joint-limit case;
  - cascade and saturation;
  - hysteresis with no chatter under reach-distance alternation (0 switches in 40 updates);
  - dwell-gated switching, immediate switch on inadmissible, abort to hold;
  - freeze (no change even when inadmissible);
  - the untrusted-re-evaluation variant.
- Existing tests: `run_binding_cost_checks.sh`, bounded-lead-schedule, independent-giver model, timing-frontier replay, module setup, latency-matrix, scenario identity, override YAML, markdown links, documentation claims.
- Static checks: `check_worker_snapshot_purity.py` (100 worker-reachable functions, including the new worker function), `check_planner_core_purity.py`, `check_giver_truth_independence.py --static`.

**Identical behavior when disabled.** Every new path is gated by `v2ControlAware_` (false unless `supervisorMode: control_aware`). The only non-gated edits relax a submission precondition for the new job type and add name-table entries. V1 is untouched. This is established by construction and the static checks; **no byte-level log comparison against a pre-change build was run**.

### 6.2 Simulation sanity runs (kinematic simulation, one run per cell — not a powered comparison)

Controller installed temporarily into `~/mc_rtc_ws/install`. The prior install was backed up to `~/TRIAD_LITE_INSTALL_BACKUP_20260916_163345`, **restored afterwards and checksum-verified**.

Campaign 1, `evidence/runs_sim_20260916`, default semantics:

| scenario | V2 bank_search | TRIAD-lite control_aware | TRIAD-lite supervisor events |
|---|---|---|---|
| longitudinal | FAIL (no certified plan in window; 11 full searches, 362 recertifications) | FAIL (same reason) | 2 initial, 1 switch, 2 abort-to-hold |
| near-ground | **completed**, commit t = 17.34 s | **completed**, commit t = 15.68 s | 1 initial, 18 switches, admit, freeze |
| lateral-low | FAIL (39 full searches) | **completed**, commit t = 19.30 s | 4 initial, 3 abort-to-hold, admit, freeze |
| diagonal | FAIL | FAIL (no admissible grasp in window) | 1 initial, 1 switch, 1 abort |

Authority characterization among grasps that passed geometry, robot and clearance layers (`evidence/summary.txt`):

| scenario | passed other layers | rejected by authority | κ* min / median / max | max within-decision κ* ratio |
|---|---:|---:|---|---:|
| near-ground | 318 | **89 (28%)** | 0.73 / 1.08 / 1.60 | 1.97 |
| longitudinal | 15 | **7 (47%)** | 0.79 / 1.04 / 1.85 | 1.66 |
| diagonal | 35 | 4 (11%) | 0.93 / 1.29 / 1.81 | 1.71 |
| lateral-low | 686 | 24 (3.5%) | 0.86 / 1.95 / 2.70 | 2.89 |

Sanity conclusion: the directional authority metric **does differ** among otherwise-feasible grasps, and in closed-loop simulation it rejects a non-trivial share. The dominant rejections are nevertheless geometric: the inherited closure acquisition tube, and standoff interference with the modelled grey handle and sensor core.

### 6.3 A rule tested and rejected by evidence

Campaign 2 (`evidence/runs_sim_20260916b`) set `trustIncumbentReevaluation: false`, so an executing incumbent could be removed only by the terminal certificate, runtime safety or a dominating challenger. The motivation was Phase 2's resumed-evaluation defect.

Result: **1/4 completed. Longitudinal, near-ground and diagonal all failed on `control_aware_track_clearance_reserve`** (e.g. 7.9 mm < 8 mm): the arm kept tracking grasps that the re-evaluation had rejected. The default is therefore `true`; the option remains for study.

### 6.4 Example trace

`evidence/example_trace.txt`, lateral-low, campaign 1:

1. **Candidates.** One decision evaluated 64 candidates, classified as `[collision:7, geometry:47, ik:1, none:9]`. Among them:
   - `axisP_side_0deg` admissible (clearance 80.4 mm, κ* = 1.92);
   - `axisP_side_11deg` rejected (geometry, `closure/pad_pair/blue_handle_acquisition_tube`);
   - `axisP_side_68deg` rejected (collision in carried retreat, wrist link);
   - `axisP_side_281deg` rejected (IK, retreat no convergence).
2. **Selection.** `decision = initial` selected grasp 0.
3. **Tracking to admission.** Tracking reached the gate (dist 0.24 mm, angle 0.002 rad). `acquisition_admit` fired at t = 19.296, followed by `V2TerminalCommit committed=true` and `grasp_freeze`.
4. **Outcome.** `[Completed] … carried-object retreat finished`.

Authority rejection example (near-ground): `axisP_side_68deg`, clearance 79.4 mm. The `follow_insert` demand at the standoff gives κ* = 0.82 (residual 0.039 m/s), so it was rejected with `kappa=0.8192<kappaMin=1.000`.

## 7. Old TRIAD components: bypassed or retained (control_aware mode)

| Component | Status |
|---|---|
| τ lead bank, event hypotheses, FULL_SEARCH | **bypassed** (τ = measured gate event) |
| Route bank / ring routes / time stretch | **bypassed** (direct tracking) |
| Weighted J₇ / binding cost / global argmin | **bypassed** (lexicographic cascade) |
| Timed provisional reach, RECERTIFY_ACTIVE resumed rollouts | **bypassed** |
| Exact timing prune, select-then-certify | **bypassed** |
| Grasp geometry (`buildCandidate`), preview kinematics, closure sweep, carried retreat | **retained** as feasibility layers |
| Clearance governor, safety filter, TERMINAL_CERTIFY, single commit | **retained** |
| MovePregrasp / CaptureTransfer / Retreat | **retained, unchanged** |
| V2 bank_search path, V1 | **retained, default** |

## 8. Reproduce

```bash
tools/run_control_aware_supervisor_unit_tests.sh
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$HOME/mc_rtc_ws/install && cmake --build build -j && cmake --install build   # back up the install first
research/triad_lite/evidence/runs_sim_20260916/campaign.sh research/triad_lite/evidence/<new_dir>
python3 research/triad_lite/tools/summarize_triad_lite_logs.py <logs...>
python3 research/triad_lite/tools/authority_offline.py
```

## 9. Classification

**Standard robotics.**
- Grasp selection by robot capability (reachability- and manipulability-aware selection, e.g. Akinola ICRA 2021, Yang ICRA 2022).
- Velocity-level capability under joint velocity/position limits (velocity polytopes).
- Box-constrained least squares.
- Lexicographic selection with hysteresis.
- Task-space tracking with a governor and safety filter.
- Event-triggered admission.

**New implementation.**
- An authority test built from the *exact* hard bounds of this controller's mc_rtc Tasks kinematics constraint, in the direction of the declared acquisition demand, per receiving grasp.
- A layered, fully logged rejection taxonomy (geometry / IK / joint limits / collision / security distance / clearance / authority).
- A feature-flagged TRIAD-lite supervisor that replaces the (τ, g, r) bank search with continuous tracking and measured admission, reusing the existing certificate and commit.

**Unproven scientific hypothesis.**
- **Hypothesis.** Among grasps passing identical geometric and robot feasibility, selecting by constrained directional authority improves closed-loop acquisition completion and/or time over reachability-only selection. The difference must persist after the insertion speed is tuned competently.
- **Evidence so far.** The simulation shows the criterion discriminates and rejects 3.5–47% of otherwise-feasible grasps. It does **not** show an outcome benefit: single kinematic runs, no matched reachability-only arm, no contact physics.
- **Next experiment.** An A/B with the authority layer disabled (`kappaMin` → 0) versus enabled, N ≥ 20 per scenario, with matched presentations. Add a slower-insertion arm to exclude the engineering explanation.
