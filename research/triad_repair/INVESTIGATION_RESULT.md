# Gated scientific repair: result at the diagnostic gate

Date: 2026-09-17. Reference: `bd815da31326c0d32dfb41a60345d38b81fe249f`.
Branch: `research/triad-scientific-repair`. This is an engineering/diagnostic
checkpoint, NOT completion of the proposed H1/H2 scientific campaign.
Evidence labels: CODE, DERIVED, EMPIRICAL, LITERATURE, HYPOTHESIS.

## A. Repair report

**CODE.** The original branch still points to the frozen reference. Old Phase A–F
reports and evidence were not edited. The pre-existing untracked
`research/triad_control_shot/` was not touched.

Separate repair commits:

- `85b00d4`: object-fixed receiving-grasp basis. `objectFixedReceivingGraspPoses`
  in `src/ReceivingGraspFamily.h` removes robot-position dependence. The basis
  projects object +X perpendicular to the handle axis, falling back to object +Y.
  Fixed family parameters and ID now identify an immutable object-relative pose.
- `48fb76c`: retention/patch geometry guard. `matchesActiveGraspGeometryV2` checks
  standoff, capture and retreat before selector mutation. The patched execution
  reference uses the stored plan transform. Same-ID geometry disagreement is
  rejected, not silently accepted.
- `ebbc593`: snapshot-epoch initialization. `submitReceiverCertificationV2` saves
  the current command state, and `rolloutInterceptionV2` starts from the matching
  snapshot robot state/time. Snapshot velocity/acceleration are no longer reset
  in this moving rollout. Result age and state drift are logged.
- `977ac75`: `terminalFromStateV2` factors the existing terminal checker; paired
  diagnostics compare the snapshot posture and timed-rollout terminal posture.
  Normal terminal execution uses its original inputs and rules.
- `eec8299`: opt-in matched reactive/predictive/lookahead interfaces with explicit
  common grasp membership and authority disabled for selection/admission.
- `bc6528e`: opt-in capability diagnostics on identical executed configurations.

**DERIVED.** The temporal repair chooses a consistent *counterfactual snapshot-start
proposal*, not propagation of the actual QP through computation latency:

\[
 (x_k,\eta_k,t_k)\xrightarrow{\text{preview}}\widetilde x(t),\widetilde\eta(t),
 \qquad (x_u,\eta_u,t_u)\ne(\widetilde x(t_u),\widetilde\eta(t_u))\text{ in general}.
\]

Actual execution reanchors the patch to its current command state at use. The
latency allowance remains a timing/age rule, not a proof of use-time feasibility.
The preview is still damped IK, not the mc_rtc QP. That fidelity gate is open.

No new controller, predictor, contact model, cost tuning or hard-kappa policy was
introduced. Historical FULL behavior remains available for reference; matched
experiments explicitly disable its authority veto.

## B. Regression / unit test report

**CODE/EMPIRICAL.** All three C++ test suites passed, and the complete isolated
Release build passed after each substantive source repair. Tests cover:

- all 530 physical grasp transforms, object-frame equivariance, and the old
  robot-relative basis as a negative regression;
- same-grasp transform comparisons, axial/orientation mismatch and nonfinite data;
- snapshot command boundaries and reanchoring at different result ages;
- generic capability metrics on a known full-rank and rank-deficient Jacobian.

Worker snapshot purity passed for 107 reachable functions. Independent-giver
static/replay checks passed. The new failed-run checker accepts legitimate
no-commit failures while rejecting injected geometry mismatch, premature closure,
V1 commitment and multiple commitments. Source-wiring checks pass, including no
scripted stop-time reads in the inspected decision functions.

Limitations: source checks are not proofs of arbitrary future code behavior.
Predictive logs do not emit the candidate O_T_G records used by the runtime ID
checker, so `observed_physical_ids=0` there is NOT independent runtime identity
coverage. Identity is covered by unit tests, guards, and held/reactive candidate
logs. The legacy V2 freshness-mutation suite needs a legacy V2 log; its mutation
is a no-op on TRIAD-lite logs. It passed with the retained legacy reference log.

The isolated ticker crashes with GUI disabled and also faults at normal run-for
teardown. Those logs remain. The final wrapper stops on terminal events, records
termination separately, and needed SIGKILL after SIGTERM in the final smoke.
Simulated completion must not be reported as clean process exit.

## C. Root-cause map

**EMPIRICAL.** See `DIAGNOSTIC_SUMMARY.json`, `evidence/*/run.log`, manifests and
`evidence/checks/checks.json`; `summarize_diagnostics.py` regenerates the counts.

| Observation | Established mechanism / classification | Not established |
|---|---|---|
| Same-ID physical drift in original implementation | Representation/interface defect, J; repaired | Need for a new controller |
| Snapshot/use joint drift up to 0.682 rad in a diagnostic | Robot evolves during computation, I/model mismatch | This is NOT preview error or proof of an unsafe action |
| 158 noncancelled paired terminal evaluations: 150 pass/pass, 5 pass/fail, 3 fail/fail | Full downstream checker does not necessarily agree across start postures or with initial layers | Independent trial statistics or physical contact failure |
| All 5 pass/fail disagreements occur at +35 mm | Rendezvous-start terminal corridor rejects `blue_handle_axial_offset`, G/interface or numerical margin | A nullspace-control limitation; causality of all nonzero offsets |
| Three fail/fail cases | Initial-layer acceptance insufficient for full terminal semantics; snapshot reasons include cost timeout | Pure posture causality |
| One additional pair cancelled | Censored diagnostic; excluded explicitly | Feasibility failure |
| +17.5 mm terminal pad rejection after good rendezvous; later another +17.5 mm grasp completes | Grasp/terminal-path compatibility is candidate-specific, B/G | Every nonzero axial grasp is invalid |
| Reactive/lookahead smoke runs fail to commit; predictive smoke completes | Real implementation diagnostic outcomes | H1 benefit, baseline competence, population performance |
| Final capability smoke: 107 samples, minimum kappa 1.08259, completion event | Same-state logging works | Kappa predicts failure; no kappa<1 example exists in this run |

The paired accepted encounters include -17.5, 0, +17.5 and +35 mm; they do not
provide a complete paired -35 mm characterization. That requested five-level
investigation is therefore incomplete. Repeated resting generations are not
independent observations. The smoke trio also precedes the final common
replacement-reference continuity change; their manifests capture the exact diff.
Do not silently relabel them as final-branch outcome evidence.

No residual failure has yet been uniquely attributed to constrained task-direction
tracking, redundant posture, or physical moving-contact interaction. H2 cycling
in the old branch remains confounded by old identity/model defects.

## D. Clean H1 design

**CODE/HYPOTHESIS.** Three interfaces now exist, sharing physical candidate IDs,
perception, geometry checks, low-level QP, stopped-object acquisition and safety:

1. Current object-relative tracking with observed rigid-motion feedforward.
2. The same tracker targeting a fixed short lookahead (0.20 s diagnostic value).
3. Budgeted sampled future-rendezvous selection with a receding reference patch.

Authority is diagnostic only. The common pool is chosen from current-pose
screening before downstream outcomes, not copied from predictive winners.
The treatment includes future-time feasibility/selection and reference generation;
it is not honestly described as changing prediction alone.

`STUDY_PROTOCOL.md` fixes completion from trial start as primary, proposes a
+0.10 practical superiority margin against both simpler arms, and specifies
paired randomized independent realizations, excluded pilot seeds, multiplicity
handling and power-based replication. The four old scenarios remain regression
cases. No clean H1 outcome campaign has been launched. Baseline *competence*, not
just wiring parity, and preview/runtime interpretation must be established first.

## E. Clean H2 design

**CODE/DERIVED.** At the same executed q and task frame, log

\[
 \rho(q,y)=\min_{v\in B(q)}\|W(J(q)v-y)\|,\qquad
 \kappa(q,y)=\sup\{c:\rho(q,cy)\le\epsilon\}.
\]

Compare with sigma_min(WJ), condition index, manipulability, and a velocity-box
width weighted alternative. Use identical temporal samples/aggregation. The
implemented box is conservative/history-free and reserve is capped at 8; neither
is an exact model of all active QP constraints. Requested twist and J(q)dq are
logged with q, dq and the box.

The new runtime samples cover precommit tracking at 50 ms. Existing path/sync/
insertion preview values remain model quantities. Actual acquisition/contact
phase capability logging and held-out H2 evaluation are not completed.
Split validation by independent trajectory, not sample rows; compare added
out-of-sample predictive value conditional on speed, time, error and clearance.
Only then consider intervention. No hard-kappa filter is justified now.

## F. Residual robotics problem

**HYPOTHESIS.** Does a snapshot-model rendezvous proposal, after actual QP motion
and use-time reference reanchoring, reliably reach a state from which the existing
stopped-object terminal acquisition can proceed?

This is currently a model/interface validation question, not an established novel
control problem. It may reduce entirely to engineering consistency.

## G. Control necessity verdict

**No new controller is presently justified.** The observed disagreements do not
separate geometry tolerance, preview dynamics, timing and posture sufficiently
to demonstrate failure of a competent existing controller. Gate status: continue
engineering diagnostics; stop controller invention and confirmatory claims.

## H. Controller formulation

Not opened. The repaired system can be described without inventing a new law:

- Physical/simulated state: x_R=(q,dq), object pose/twist x_O=(T_O,V_O), gripper
  configuration and existing acquisition mode. Simulator truth is not a measured
  physical contact force.
- Available information: I_k=(x_R,k, estimated T_O,V_O, timestamp, command state
  eta_k, incumbent object-relative geometry, mode). Prediction uses observed
  history, not the giver script's future stop.
- Supervisory decision a=(g,tau), g=(sigma,theta,s) on a finite mechanical family.
  Matching experiments use fixed common membership.
- Kinematics: dot(q)=v, V_E=J(q)v. Actual discrete evolution is the existing
  mc_rtc solver/integrator map F_QP(x_R,eta); preview uses a different map
  F_preview. No equality or closed-loop error bound has been demonstrated.
- Predictions: T_G(t_k+tau)=T_hat_O(t_k+tau) O_T_G(g).
- Objective: among the budget-explored model-feasible events find tau_min;
  retain the configured tau tie band and apply existing clearance selection and
  incumbent retention. This is not continuous time optimization or a proof of
  exhaustive earliest interception.
- Hard model checks: mechanical bounds, ground/pursuit/reachability screening,
  existing IK/collision/clearance/terminal layers, timing, and timed-rollout pose
  and relative-motion tolerances. A separate stopped terminal checker still
  authorizes actual commitment; linked-q_R checks are diagnostic, not enforced.
- Robot commands: existing body/task pose and velocity references to the same
  QP; unchanged downstream gripper/acquisition/transfer/retreat commands.
- Memory: incumbent physical grasp, reference patch and rendezvous time,
  pending generation/snapshot, terminal commitment latch.
- Assumptions: known handle geometry, estimated object motion, existing model
  limits, hypothetical stopped acquisition and simulator contact/attachment
  semantics. Long-horizon constant-twist accuracy is not guaranteed by replanning.
- LITERATURE: interception, receding replanning, reachability screening, QP tracking.
  DERIVED: object-fixed identity and consistent epochs. CODE: finite search,
  existing controller and diagnostics. EMPIRICAL: listed diagnostic outcomes.
  NEW scientific method: none established.

## I. Closest prior art

**LITERATURE.** Hujic et al. (1998), §§II–V, already formulate earliest predicted
pregrasp rendezvous through an outer encounter search and inner robot-motion
problem, followed by replanning and fine tracking. Thus sampled rendezvous and
patching are not a new control contribution. Their predictable-motion setting
differs from unannounced human stops; that difference alone establishes no new
method here.
https://cimlab.mie.utoronto.ca/wp-content/uploads/2016/11/Hujic-1998-The-robotic-interception-of-moving-objects-in-industrial-settings-strategy-development-and-experiment.pdf

Akinola/Xu et al., Dynamic Grasping with Reachability and Motion Awareness (2021),
provide the directly relevant dynamic grasp-selection/replanning comparison.
Reachability screening plus online grasp selection is established, not a novelty
inferred from the new object-fixed representation.
https://arxiv.org/abs/2103.10562
https://github.com/jingxixu/dynamic-grasping

Djeha et al., Human-Robot Handovers using Task-Space Quadratic Programming (2022),
and Yang et al., Model Predictive Control for Fluid Human-to-Robot Handovers
(2022), are the relevant existing QP and predictive handover comparators.
https://arxiv.org/abs/2206.09185
https://arxiv.org/abs/2204.00134

No repaired diagnostic here demonstrates a mathematical capability missing from
those families. The interaction-controller literature gate was not reopened:
these experiments still use stopped-object acquisition and do not isolate a
moving-contact or load-transfer failure. This is not a claim that all possible
handover control problems are solved.

## J. Novelty verdict

**No novelty gap established.** Neither snapshot/posture disagreement nor a
successful predictive smoke run demonstrates a distinct missing control method.

## K. Article type

**E — NO PAPER YET.** A system/validation paper remains contingent on clean
matched outcome evidence. No supervisory/control/hybrid-method paper is earned.

## L. Paper-ready contribution statement

None yet. Defensible present engineering outputs are (1) repaired physical grasp
and plan-geometry contracts, (2) explicit model/use-time and linked-terminal
compatibility diagnostics, and (3) baseline interfaces with diagnostic-only
capability measurements. These are not three claimed research novelties.

## M. Paper outline

Not supplied: the article gate is E.

## Outstanding work / scope boundary

The broader investigation is unfinished: full five-offset causal characterization,
adequate preview/execution correspondence, competent-baseline validation, clean
H1 and held-out H2 remain. This checkpoint does not claim those gates passed.
The next useful experiment is a paired replay of the SAME physical grasp and
reference from recorded snapshot and result-use states, followed by the SAME
terminal checker, comparing QP execution to preview. Include interior and axial
boundary grasps before attributing failures to nullspace control. Do not add an
architecture to compensate for an unisolated defect.
