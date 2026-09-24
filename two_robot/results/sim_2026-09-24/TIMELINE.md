# Two-robot handover, simulation, 2026-09-24 (branch triad/two-robot-2026-09-24, run from the build tree, nothing installed)

Scenario `pure_x`: the object starts at [0.92, 0, 0.55] m; Robot B presents it at 0.08 m/s along −x and stops at [0.55, 0, 0.55] m. Robot A receives. Time from the 1 kHz log index. Phase codes from `DualGiverCoordinator::Phase` (1 Prepositioning, 2 StartSettling, 3 Ready, 4 Executing, 5 TerminalSettling, 6 Holding, 7 Failed).

## Robot A (receiver) states

| t (s) | state |
|---|---|
| 0.00 | Initial |
| 7.78 | ObserveObject |
| 9.02 | SolveInterception |
| 12.06 | ExecuteCommittedReach |
| 14.52 | PresentationHold |
| 14.62 | MovePregrasp |
| 15.70 | CaptureTransfer |
| 19.38 | Retreat |
| 19.94 | Completed |

## Robot B (giver) phases

| t (s) | phase |
|---|---|
| 0.00 | 1 — Prepositioning |
| 2.70 | 2 — StartSettling |
| 2.90 | 3 — Ready (waiting for Robot A's observation to start) |
| 8.12 | 4 — Executing (presenting: constant velocity, then deceleration) |
| 13.17 | 5 — TerminalSettling |
| 13.37 | 6 — Holding |

Result line in the log: `[Completed] full plan-once handover completed: grasp confirmed and carried-object retreat finished mode=MOVING`.

TimingSummary from the log: planning 3.039 s, reach 2.225 s, approach 1.074 s, acquire 2.155 s, transfer 1.527 s, retreat 0.564 s, execution 7.545 s (predicted 8.903 s).

The two regression variants run on the same build also completed: the plain single-robot longitudinal scenario, and the second robot loaded with `dualHandover.enabled: false`.
