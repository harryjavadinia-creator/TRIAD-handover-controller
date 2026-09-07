# Architecture

## Pipeline

```text
Observe
  -> Predict
  -> Generate bounded (event time, grasp, route) alternatives
  -> Copied-state preview
  -> Hard physical feasibility
  -> Cost-valid complete-plan records
  -> Final timing admission at selection time
  -> Exact finite argmin
  -> Commit once
  -> mc_rtc FSM/QP execution
```

TRIAD separates high-level plan selection from low-level execution. The finite
planner chooses **what** and **when**: the event time, grasp orientation and
transit route. The mc_rtc FSM/QP layer determines **how** to track the
committed references. The QP does not choose the event time or minimize the
high-level objective.

The interface is narrow and worth stating exactly. Per control cycle TRIAD
supplies one `TransformTask` target pose, a body-frame reference velocity, a
zero reference acceleration, an arm-posture target and a six-joint gripper
target. It also configures one kinematics constraint, **no collision constraint
and no contacts**. No event time, grasp, route or objective value ever crosses
that boundary. Joint limits are therefore enforced twice and independently —
constructively inside TRIAD's own differential-IK step scaling, and again by the
QP — whereas collision safety is enforced **once, by TRIAD alone**.

mc_rtc's internal QP formulation is not reproduced in this repository and no QP
equation is claimed from it.

## One frozen decision state, one final timing gate

Candidate generation and copied-state feasibility evaluation use one frozen
decision state `s0` and one frozen bounded prediction schedule. This makes
geometric and cost comparisons across event hypotheses refer to a common
search epoch.

Timing is different: after the full bounded schedule has been inspected,
`FiniteEventPlanSelector` reapplies the timing-admission rule using the
controller time `now` at final selection. A complete plan must therefore pass:

1. copied-state hard physical feasibility;
2. finite/valid objective construction; and
3. the final selection-time timing gate.

See [`mathematics.md`](mathematics.md) for the corresponding sets and
[`timing_frontiers.md`](timing_frontiers.md) for the hardware-facing replay.

## Controller structure

- `src/HandoverInterceptionController.{h,cpp}` contains candidate generation,
  copied-state preview, feasibility tests, objective construction and commit
  support.
- `src/FinitePlanSelector.h` is the within-event finite selector. It is also
  used by the event-time refinement path.
- `src/FiniteEventPlanSelector.h` is the cross-event selector used by
  `global_time_plan`; it reapplies final timing admission and selects the
  finite global minimum.
- `src/states/HandoverInterceptionController_SolveInterception.cpp` builds the
  bounded event schedule, evaluates every configured event, pools complete
  alternatives and performs the one-time global selection.
- `src/states/` contains the compiled mc_rtc FSM states. The active state list
  is defined in `src/states/CMakeLists.txt`; transitions and configuration are
  in `etc/HandoverInterceptionController.in.yaml`.

TRIAD is the public method name. `call_handover` and
`HandoverInterceptionController` are retained implementation identifiers from
the CALL project lineage.

## Active FSM

```text
Initial
  -> ObserveObject
  -> SolveInterception
  -> ExecuteCommittedReach
  -> PresentationHold
  -> MovePregrasp
  -> CaptureTransfer
  -> Retreat
  -> Completed
```

Any rejected or unsafe execution path enters `Failure`. `CaptureTransfer`
owns closure, bilateral confirmation and load transfer continuously in the
compiled release.

## Configuration

`etc/HandoverInterceptionController.in.yaml` is a CMake `configure_file`
template. Build-time placeholders are replaced with the actual mc_rtc runtime
install locations. Scenario-specific object pose/velocity values are applied
through a temporary per-controller override by `scripts/run_scenario.sh`, so
the tracked template is not edited during reproduction.

## Runtime verification

`tools/check_global_time_plan_log.py` independently inspects a completed run.
It checks schedule completeness, reconciles pooled and excluded alternatives,
reconstructs the frozen seven-term binding objective from logged terms, and,
when timing-diagnostic records are present, independently verifies the exact
argmin over the cost-valid and final-timing-admissible set.

Scenario identity is checked separately by
`tools/verify_scenario_identity.py`. This separation prevents a scientifically
valid log from being mistaken for evidence from the wrong scenario.

## Asynchronous planning

The complete finite search runs on one background worker. The control thread
freezes a snapshot, submits one planning generation and returns; on later cycles
it polls an atomic state and never waits. When a result appears it applies
current timing admission, takes the exact finite argmin and commits once, or
fails closed.

The handoff is a single result buffer with release/acquire publication. There is
never more than one job in flight; the worker writes the result and its
generation before the release store, and the control thread reads them only
after the paired acquire load. **No mutex, condition variable, future or join is
reachable from the 1 kHz callback.** Generation identity is carried on the
result and checked on arrival, an exception inside the worker is caught at the
thread boundary, and the worker is cancelled and joined in the planning state's
teardown, in `reset()` and in the destructor; `detach()` appears nowhere.

The worker pins the admission instant to the frozen search epoch, so no
hypothesis is skipped because the worker took time to compute. That is the
intended asynchronous semantics: enumerate the complete frozen bank, then apply
current timing admission once, at result receipt.

## Copied-state discipline, stated in its narrow form

Candidate certification runs on a copied `MultiBodyConfig` taken once at the
search epoch, together with a planner-owned robot model and a planner-owned
object/handle world. The repository ships a static guard,
`tools/check_planner_core_purity.py`, with a mutation test that reintroduces
nine different live-state reads and requires all nine to be caught.

The property that guard establishes must be stated narrowly:

> The copied-state planner reads no live robot **pose or configuration** after
> the snapshot; every kinematic quantity it decides from comes from the frozen
> `MultiBodyConfig`. It does still read three categories of model-constant live
> data — joint position limits, joint velocity limits, and the open-gripper
> mouth half-gap — through accessors the guard does not cover.

The first two are constants of the robot model. The third is held constant
during planning by three independent interlocks: the gripper is commanded fully
open every cycle while planning, a closure-authority interlock clamps any
positive command to zero, and the observation stage already fails the run if the
aperture drifts. Repeated runs produce bit-identical frozen plan sets.

**Do not write "the planner is copied-state pure" or "the planner reads no live
state" without that qualification.**
