## V1 regression (FrozenPlanSet sha256, default configuration)

| scenario | recorded evidence | this run | records | identical | completed |
|---|---|---|---:|---|---|
| longitudinal | `e8356b1acd200e11…` | `e8356b1acd200e11…` | 198 | YES | yes |
| near-ground | `d6dbdfdb5b1c4511…` | `d6dbdfdb5b1c4511…` | 283 | YES | yes |
| lateral-low | `01022c98c76b334d…` | `01022c98c76b334d…` | 432 | YES | no |
| diagonal | `81124fd94cb46aca…` | `81124fd94cb46aca…` | 233 | YES | yes |

## V2 invariants and demonstrations

| run | completed | I1 | I2 | I3 | I4 | I5 | I6 | I7 | I8 | D1 concurrent motion | D2 gens while moving | D3 update/replacement | D4 stale rejected |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v2/longitudinal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | no | yes |
| v2/near-ground | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | yes | no |
| v2/lateral-low | no | PASS | — | — | PASS | — | — | — | — | no | yes | yes | no |
| v2/diagonal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | yes | yes | yes | yes |
| v2_repeat/longitudinal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | no | yes |
| v2_repeat/near-ground | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | yes | yes |
| v2_repeat/lateral-low | no | PASS | — | — | PASS | — | — | — | — | no | no | no | no |
| v2_repeat/diagonal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | yes | yes | yes | yes |
| v2_inject_stale/diagonal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | yes | yes | yes | yes |

(— = invariant not applicable because the run failed closed before commitment.)

### Required coverage across the V2 evidence set

- COVERED: robot moves while object moves — v2/diagonal, v2_repeat/diagonal, v2_inject_stale/diagonal
- COVERED: planning generations while robot moves — v2/longitudinal, v2/near-ground, v2/lateral-low, v2/diagonal, v2_repeat/longitudinal, v2_repeat/near-ground, v2_repeat/diagonal, v2_inject_stale/diagonal
- COVERED: provisional plan replaced before commitment — v2/near-ground, v2/lateral-low, v2_repeat/near-ground
- COVERED: provisional plan retained with prediction update — v2/near-ground, v2/diagonal, v2_repeat/near-ground, v2_repeat/diagonal, v2_inject_stale/diagonal
- COVERED: stale worker results rejected — v2/longitudinal, v2/diagonal, v2_repeat/longitudinal, v2_repeat/near-ground, v2_repeat/diagonal, v2_inject_stale/diagonal

## Cross-run giver truth independence (V1-independent vs V2, same scenario)

- cross: PASS longitudinal.log vs longitudinal.log: 1291 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True
- cross: PASS near-ground.log vs near-ground.log: 1289 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True
- cross: PASS lateral-low.log vs lateral-low.log: 786 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True
- cross: PASS diagonal.log vs diagonal.log: 1035 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True

## First architectural comparison: V1 frozen pre-reach vs V2 receding (independent giver)

| run | completed | commit | failure | planningLatency | planningGenerations | provisionalSwitches | minClearance | deviationTolerated | timeToCaptureSinceGiverStart | postCommitFailure | capture | transfer | retreat | concurrentMotionSamples |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v1_independent/longitudinal | no | yes | object left committed prediction tube positionError=0.0150 rotationError=0.0000; no replan | [3.35] | 1 | 0 | n/a | 0.013 | n/a | yes | no | no | no | n/a |
| v1_independent/near-ground | no | yes | object left committed prediction tube positionError=0.0150 rotationError=0.0000; no replan | [3.66] | 1 | 0 | n/a | 0.011 | n/a | yes | no | no | no | n/a |
| v1_independent/lateral-low | no | no | committed=false reason=global_commit_validation_failed detail=global_event_prediction_drif | [4.50] | 1 | 0 | n/a | n/a | n/a | no | no | no | no | n/a |
| v1_independent/diagonal | yes | yes | n/a | [2.58] | 1 | 0 | 0.078 | 0.006 | 8.821 | no | yes | yes | yes | n/a |
| v2/longitudinal | yes | yes | n/a | [3.66, 2.23, 0.55] | 455 | 0 | 0.082 | 0.000 | 12.911 | no | yes | yes | yes | 0 |
| v2/near-ground | yes | yes | n/a | [3.19, 3.39, 0.17] | 748 | 2 | 0.080 | 0.002 | 15.261 | no | yes | yes | yes | 0 |
| v2/lateral-low | no | no | reason=no_certified_provisional_plan_within_presentation_window window=7.000s quasiStaticS | [4.44, 6.51, 0.51] | 250 | 1 | 0.065 | 0.000 | n/a | no | no | no | no | 0 |
| v2/diagonal | yes | yes | n/a | [2.85] | 585 | 0 | 0.079 | 0.004 | 9.721 | no | yes | yes | yes | 24 |
| v2_repeat/longitudinal | yes | yes | n/a | [3.74, 2.32, 0.55] | 497 | 0 | 0.082 | 0.000 | 13.081 | no | yes | yes | yes | 0 |
| v2_repeat/near-ground | yes | yes | n/a | [3.07, 3.41, 0.17] | 766 | 2 | 0.080 | 0.002 | 15.271 | no | yes | yes | yes | 0 |
| v2_repeat/lateral-low | no | no | reason=no_certified_provisional_plan_within_presentation_window window=7.000s quasiStaticS | [4.50] | 3 | 0 | 0.230 | n/a | n/a | no | no | no | no | 0 |
| v2_repeat/diagonal | yes | yes | n/a | [3.06] | 518 | 0 | 0.079 | 0.004 | 9.721 | no | yes | yes | yes | 24 |

Units: latency s (V1: worker wall of the single search; V2: latency of each full search), clearance m (live gripper clearance during reach/provisional motion), deviation m (V1: largest object-model error tolerated during committed reach; V2: largest retained prediction update), time to capture s since giver start (last truth sample before bilateral grasp confirmation).

EVIDENCE SUMMARY: PASS
