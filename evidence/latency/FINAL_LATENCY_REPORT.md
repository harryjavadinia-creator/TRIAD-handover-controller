# Corrected perception-latency ablation

Fix commit: f56add35cb66e1e495c21502d60d29d6e000902f
Scenario: canonical_yz, position [0.55, -0.56, 0.15], velocity [0, 0.08, 0].
No scientific parameter changed. Owned-binary mechanism, exclusive machine.

## Bug and fix
selectDelayedObjectMeasurement() and the perception buffer trim read
plannerConfig_.perceptionLatencySeconds, a mirror refreshed only at planning
freeze and calibration, therefore still holding its declared default of 0.220 s
when the per-cycle perception path first used it. The logging accessor read the
controller member, so logs showed the requested delay while the buffer used
0.220 s. Introduced by 7b6c272. The fix points those two reads at the live
members; nothing else changed.

## Regression: configured delay versus actual measurement age
| configured | measurement age | raw error | v*tau | compensated estimate error |
|---|---|---|---|---|
| 0.00 | 0.000 | 0.0000 | 0.0000 | 0.0000 |
| 0.10 | 0.100 | 0.0080 | 0.0080 | 0.0000 |
| 0.22 | 0.220 | 0.0176 | 0.0176 | 0.0000 |
| 0.30 | 0.300 | 0.0240 | 0.0240 | 0.0001 |
| 0.40 | 0.400 | 0.0320 | 0.0320 | 0.0002 |
| 0.50 | 0.500 | 0.0400 | 0.0400 | 0.0007 |
| 0.60 | 0.600 | 0.0480 | 0.0480 | 0.0024 |

Uncompensated raw error follows e = v*tau with ratio 1.000 at every delay.
Compensation drives the estimate error to zero, with a residual that grows
slowly with delay, consistent with the velocity filter time constant of 0.10 s.

## Full sweep, compensation ON versus OFF
| tau | comp | admissible | winner | committed | completed | outcome |
|---|---|---|---|---|---|---|
| 0.00 | -   | 53 | h14 | yes | yes | completed |
| 0.10 | ON  | 53 | h14 | yes | yes | completed |
| 0.10 | OFF | 38 | h14 | yes | yes | completed |
| 0.22 | ON  | 53 | h14 | yes | yes | completed |
| 0.22 | OFF | 40 | h14 | yes | yes | completed |
| 0.30 | ON  | 44 | h14 | yes | yes | completed |
| 0.30 | OFF | 44 | h12 | yes | no  | execution failed |
| 0.40 | ON  | 39 | h14 | yes | no  | execution failed |
| 0.40 | OFF | 38 | h12 | yes | no  | execution failed |
| 0.50 | ON  | 31 | h14 | yes | no  | execution failed |
| 0.50 | OFF | 31 | h13 | yes | no  | execution failed |
| 0.60 | ON  | 31 | h12 | no  | no  | freshness rejected |
| 0.60 | OFF | -  | -   | no  | no  | rejected upstream at motion classification |

## Compensated-PASS / uncompensated-FAIL split
A natural split exists at tau = 0.30 s and was not tuned for. Repeated three
times per arm on top of the sweep cell:

  compensated   4/4 completed
  uncompensated 4/4 committed then execution failed

At tau >= 0.40 s compensation is no longer sufficient and both arms fail. At
tau = 0.60 s uncompensated, TRIAD rejects upstream at the observation stage:
"[PresentationMode] rejected=AMBIGUOUS moved=0.0241/0.0250m", because the
0.6 s stale measurement makes the object look nearly stationary. That is a safe
fail-closed rejection before any planning.

## Consistency with the earlier 0.22 s evidence
Identical before and after the fix: mode DELAYED_COMPENSATED, age 0.220,
raw error 0.0176, compensated estimate error 0.0, FrozenPlanSet hash
6345250e2cf2ef21, handover completed. The earlier 0.22 s results stand.
