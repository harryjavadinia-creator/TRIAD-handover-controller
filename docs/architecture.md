# Architecture

## Pipeline

```
Observe -> Predict -> Generate(tau,g,r) -> Preview -> Hard feasibility
  -> Soft ranking -> Exact finite argmin -> Commit -> mc_rtc execution
```

A discrete deliberative layer (the finite planner) chooses *what* and
*when* — the event time, grasp orientation and transit route. mc_rtc's
FSM/QP layer executes *how* — joint-level tracking of the committed plan.
The QP is not asked to choose the event time or evaluate the objective; it
tracks the references handed to it once a plan is committed. See
`docs/mathematics.md` for the exact formulation at each stage.

## Controller structure

- `src/HandoverInterceptionController.{h,cpp}`: the controller, including
  candidate generation, hard feasibility, the objective, and the finite
  and global selectors.
- `src/FinitePlanSelector.h`, `src/FiniteEventPlanSelector.h`: the
  within-event and cross-event selector logic, also covered by standalone
  regression tests in `tools/test_finite_plan_selector.cpp` and
  `tools/test_finite_event_plan_selector.cpp`.
- `src/states/`: the mc_rtc FSM states (Initial, ObserveObject,
  SolveInterception, ExecuteCommittedReach, PresentationHold,
  MovePregrasp, CaptureTransfer, Retreat, Completed, Failure). The FSM
  transition table is in `etc/HandoverInterceptionController.in.yaml`.

## Configuration

`etc/HandoverInterceptionController.in.yaml` is a CMake `configure_file`
template. At configure time it is instantiated into the generated,
installed configuration, with install-location placeholders (states
libraries/files paths, the bundled object description path) substituted for
the actual install prefix on your machine. See `docs/simulation.md` for how
scenario-specific fields (object initial pose and velocity) are selected
without editing this file.

## Runtime verification

`tools/check_global_time_plan_log.py` independently reconstructs the
selector's proof from a run's log: it re-derives the reported cost from the
frozen seven-term weight vector (hard-coded in the checker itself, not
imported from the controller), reconciles per-hypothesis pooled/rejected
candidate counts, and confirms the committed plan is the exact argmin over
the cost-valid and timing-admissible set. See `docs/simulation.md` for
exactly what it checks.
