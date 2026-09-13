## Prediction-update handling: per run

Times in s. timeToFirstPlan: first provisional adoption minus first FULL_SEARCH submission. firstAdoptMinusRest: negative means the plan was obtained before the giver came to rest. discardedUnits: bounded planner work units (static preview steps, route work units, hypothesis setups) belonging to cancelled searches or to failed/cancelled selected-action certifications. concurrentMotion: 50 ms samples with robot and object truth both moving, before commitment.

| run | mode | completed | timeToFirstPlan | firstAdoptMinusRest | firstAdoptWhileMoving | firstAdoptLead | firstAdoptSource | fullSearches | fullCancelled | selectedTargetMoved | selectedCertOk | selectedCertFailed | discardedUnits | discardedWall | fullUnits | fullWall | fullWallToFirstPlan | allWorkerWallToCommit | concurrentMotion | replacements |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| select_a/diagonal | select_then_certify | yes | 2.853 | -1.670 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.852 | 2.852 | 5.183 | 1.200 | 0 |
| select_a/lateral-low | select_then_certify | yes | 4.513 | -0.010 | yes | 8.000 | certified | 1 | 0 | 3 | 1 | 2 | 294 | 0.010 | 263056 | 4.492 | 4.492 | 7.643 | 0.000 | 0 |
| select_a/longitudinal | select_then_certify | yes | 6.576 | 2.053 | no | 2.800 | search | 3 | 0 | 0 | 0 | 0 | 0 | 0 | 526152 | 6.572 | 6.572 | 8.568 | 0.000 | 0 |
| select_a/near-ground | select_then_certify | yes | 3.128 | -1.395 | yes | 8.000 | search | 3 | 0 | 9 | 0 | 9 | 4615 | 0.094 | 510196 | 7.033 | 3.127 | 10.974 | 0.000 | 1 |
| select_b/diagonal | select_then_certify | yes | 3.033 | -1.490 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 3.032 | 3.032 | 5.246 | 1.200 | 0 |
| select_b/lateral-low | select_then_certify | yes | 5.653 | 1.130 | no | 3.250 | search | 2 | 0 | 11 | 0 | 11 | 1542 | 0.078 | 288570 | 5.563 | 5.563 | 7.967 | 0.000 | 0 |
| select_b/longitudinal | select_then_certify | no | 7.120 | 2.597 | no | 3.250 | search | 7 | 0 | 0 | 0 | 0 | 0 | 0 | 567313 | 7.693 | 7.116 | 8.906 | 0.000 | 0 |
| select_b/near-ground | select_then_certify | yes | 3.264 | -1.259 | yes | 8.000 | search | 3 | 0 | 1 | 0 | 1 | 107 | 0.004 | 515583 | 7.564 | 3.263 | 11.347 | 0.000 | 1 |
| restart_a/diagonal | cancel_and_restart | yes | 2.973 | -1.550 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.972 | 2.972 | 5.196 | 1.200 | 0 |
| restart_a/lateral-low | cancel_and_restart | no | 4.998 | 0.475 | no | 2.800 | search | 38 | 36 | 0 | 0 | 0 | 297679 | 4.488 | 327117 | 5.007 | 4.967 | 7.856 | 0.000 | 1 |
| restart_a/longitudinal | cancel_and_restart | yes | 5.092 | 0.569 | no | 2.800 | search | 36 | 35 | 0 | 0 | 0 | 295049 | 4.490 | 323819 | 5.067 | 5.067 | 7.051 | 0.000 | 0 |
| restart_a/near-ground | cancel_and_restart | yes | 3.147 | -1.376 | yes | 8.000 | search | 36 | 34 | 0 | 0 | 0 | 55355 | 0.583 | 313125 | 3.895 | 3.146 | 7.774 | 0.000 | 1 |
| restart_b/diagonal | cancel_and_restart | yes | 2.878 | -1.645 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.877 | 2.877 | 5.190 | 1.200 | 0 |
| restart_b/lateral-low | cancel_and_restart | yes | 4.977 | 0.454 | no | 2.800 | search | 38 | 37 | 0 | 0 | 0 | 297258 | 4.486 | 322781 | 4.948 | 4.948 | 7.052 | 0.000 | 0 |
| restart_b/longitudinal | cancel_and_restart | yes | 5.091 | 0.568 | no | 2.800 | search | 38 | 36 | 0 | 0 | 0 | 64491 | 0.791 | 349404 | 5.063 | 5.063 | 7.043 | 0.000 | 0 |
| restart_b/near-ground | cancel_and_restart | yes | 3.365 | -1.158 | yes | 8.000 | search | 35 | 33 | 0 | 0 | 0 | 54378 | 0.579 | 312148 | 4.060 | 3.324 | 7.757 | 0.000 | 1 |
| select_inject_stale/diagonal | select_then_certify | yes | 2.834 | -1.689 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.833 | 2.833 | 5.168 | 1.200 | 0 |
| select_inject_supersede/longitudinal | select_then_certify | no | n/a | n/a | no | n/a | n/a | 2 | 1 | 0 | 0 | 0 | 49372 | 0.501 | 320564 | 4.433 | 4.433 | 4.433 | 0.000 | 0 |

## Per mode (median over runs; counts are totals)

| mode | runs | completed | first plan while moving | median timeToFirstPlan | median firstAdoptMinusRest | total discardedUnits | total discardedWall | median fullWall | median concurrentMotion |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| cancel_and_restart | 8 | 7 | 4 | 4.171 | -0.352 | 1064210 | 15.417 | 4.504 | 0.000 |
| select_then_certify | 10 | 8 | 6 | 3.264 | -1.259 | 55930 | 0.687 | 5.027 | 0.000 |
