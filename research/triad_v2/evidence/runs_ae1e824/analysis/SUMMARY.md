## V1 regression (FrozenPlanSet sha256, default configuration)

| scenario | recorded evidence | this run | records | identical | completed |
|---|---|---|---:|---|---|
| longitudinal | `e8356b1acd200e11…` | `e8356b1acd200e11…` | 198 | YES | yes |
| near-ground | `d6dbdfdb5b1c4511…` | `d6dbdfdb5b1c4511…` | 283 | YES | yes |
| lateral-low | `01022c98c76b334d…` | `01022c98c76b334d…` | 432 | YES | no |
| diagonal | `81124fd94cb46aca…` | `81124fd94cb46aca…` | 233 | YES | yes |

## V2 invariants and demonstrations

| run | completed | I1 | I2 | I3 | I4 | I5 | I6 | I7 | I8 | I9 | D1 concurrent motion | D2 gens while moving | D3 update/replacement | D4 stale rejected |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v2/longitudinal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | no | no |
| v2/near-ground | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | yes | no |
| v2/lateral-low | no | PASS | — | — | PASS | — | — | — | — | PASS | no | yes | yes | no |
| v2/diagonal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | yes | yes | yes | no |
| v2_repeat/longitudinal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | no | no |
| v2_repeat/near-ground | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | yes | no |
| v2_repeat/lateral-low | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | no | no |
| v2_repeat/diagonal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | yes | yes | yes | no |
| v2_inject_stale/diagonal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | yes | yes | yes | yes |
| v2_inject_supersede/longitudinal | yes | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | PASS | no | yes | no | no |

(— = invariant not applicable because the run failed closed before commitment.)

### Required coverage across the V2 evidence set

- COVERED: robot moves while object moves — v2/diagonal, v2_repeat/diagonal, v2_inject_stale/diagonal
- COVERED: planning generations while robot moves — v2/longitudinal, v2/near-ground, v2/lateral-low, v2/diagonal, v2_repeat/longitudinal, v2_repeat/near-ground, v2_repeat/lateral-low, v2_repeat/diagonal, v2_inject_stale/diagonal, v2_inject_supersede/longitudinal
- COVERED: provisional plan replaced before commitment — v2/near-ground, v2/lateral-low, v2_repeat/near-ground
- COVERED: provisional plan retained with prediction update — v2/near-ground, v2/diagonal, v2_repeat/near-ground, v2_repeat/diagonal, v2_inject_stale/diagonal
- COVERED: stale worker results rejected — v2_inject_stale/diagonal
- COVERED: superseded generations cancelled without effect (I9) — v2/longitudinal, v2/near-ground, v2/lateral-low, v2/diagonal, v2_repeat/longitudinal, v2_repeat/near-ground, v2_repeat/lateral-low, v2_inject_stale/diagonal, v2_inject_supersede/longitudinal

## Cross-run giver truth independence (V1-independent vs V2, same scenario)

- cross: PASS longitudinal.log vs longitudinal.log: 1289 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True
- cross: PASS near-ground.log vs near-ground.log: 1289 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True
- cross: PASS lateral-low.log vs lateral-low.log: 1291 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True
- cross: PASS diagonal.log vs diagonal.log: 1035 common elapsed samples before attachment, max |dp|=0.000e+00 m, identical script=True

## First architectural comparison: V1 frozen pre-reach vs V2 receding (independent giver)

| run | completed | commit | failure | planningLatency | planningGenerations | provisionalSwitches | minClearance | deviationTolerated | timeToCaptureSinceGiverStart | postCommitFailure | capture | transfer | retreat | concurrentMotionSamples |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| v1_independent/longitudinal | no | yes | object left committed prediction tube positionError=0.0150 rotationError=0.0000; no replan | [3.27] | 1 | 0 | n/a | 0.012 | n/a | yes | no | no | no | n/a |
| v1_independent/near-ground | no | yes | object left committed prediction tube positionError=0.0150 rotationError=0.0000; no replan | [3.43] | 1 | 0 | n/a | 0.012 | n/a | yes | no | no | no | n/a |
| v1_independent/lateral-low | no | no | committed=false reason=global_commit_validation_failed detail=global_event_prediction_drif | [4.45] | 1 | 0 | n/a | n/a | n/a | no | no | no | no | n/a |
| v1_independent/diagonal | yes | yes | n/a | [2.49] | 1 | 0 | 0.078 | 0.006 | 8.821 | no | yes | yes | yes | n/a |
| v2/longitudinal | yes | yes | n/a | [3.59, 0.53] | 528 | 0 | 0.082 | 0.000 | 11.541 | no | yes | yes | yes | 0 |
| v2/near-ground | yes | yes | n/a | [3.11, 0.18] | 752 | 1 | 0.080 | 0.002 | 12.471 | no | yes | yes | yes | 0 |
| v2/lateral-low | no | no | reason=no_certified_provisional_plan_within_presentation_window window=7.000s quasiStaticS | [0.47, 0.05] | 728 | 1 | 0.065 | 0.000 | n/a | no | no | no | no | 0 |
| v2/diagonal | yes | yes | n/a | [2.95] | 516 | 0 | 0.079 | 0.004 | 9.721 | no | yes | yes | yes | 24 |
| v2_repeat/longitudinal | yes | yes | n/a | [0.57] | 515 | 0 | 0.082 | 0.000 | 11.541 | no | yes | yes | yes | 0 |
| v2_repeat/near-ground | yes | yes | n/a | [3.07, 0.17] | 793 | 1 | 0.080 | 0.002 | 12.471 | no | yes | yes | yes | 0 |
| v2_repeat/lateral-low | yes | yes | n/a | [0.49] | 503 | 0 | 0.065 | 0.000 | 11.541 | no | yes | yes | yes | 0 |
| v2_repeat/diagonal | yes | yes | n/a | [2.86] | 516 | 0 | 0.079 | 0.004 | 9.721 | no | yes | yes | yes | 24 |

Units: latency s (V1: worker wall of the single search; V2: latency of each full search), clearance m (live gripper clearance during reach/provisional motion), deviation m (V1: largest object-model error tolerated during committed reach; V2: largest retained prediction update), time to capture s since giver start (last truth sample before bilateral grasp confirmation).

EVIDENCE SUMMARY: PASS
