# Two-robot handover, simulation, 2026-09-24 (run from the build tree, nothing installed)

Scenario `pure_x`: the object starts at [0.92, 0, 0.55] m; Robot B presents it at 0.08 m/s along −x and stops at [0.55, 0, 0.55] m. Robot A receives: it observes the carried object, evaluates its complete grasp × route bank once at Robot B's fixed endpoint and time, commits, reaches, captures and retreats with the object. Time from the 1 kHz log index. Phase codes from `DualGiverCoordinator::Phase` (1 Prepositioning, 2 StartSettling, 3 Ready, 4 Executing, 5 TerminalSettling, 6 Holding, 7 Failed).

## Robot A (receiver) states

| t (s) | state |
|---|---|
| 0.00 | Initial |
| 7.78 | ObserveObject |
| 9.02 | SolveInterception (one event: Robot B's endpoint, lead 4.15 s) |
| 9.51 | ExecuteCommittedReach |
| 13.17 | PresentationHold |
| 13.27 | MovePregrasp |
| 14.34 | CaptureTransfer |
| 18.03 | Retreat (object released by Robot B, carried by Robot A) |
| 18.59 | Completed |

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

TimingSummary from the log: planning 0.490 s, reach 2.895 s, approach 1.067 s, acquire 2.160 s, transfer 1.527 s, retreat 0.564 s, execution 8.213 s (predicted 9.597 s).

Robot A observes the object that Robot B carries (`movingObject.simulateMotion: false`); the object pose written by the giver module and the pose Robot A plans and retreats with differ by 0.13 mm over the run, with no discontinuity. The single-robot longitudinal and near-ground scenarios run on the same build also complete.
