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
