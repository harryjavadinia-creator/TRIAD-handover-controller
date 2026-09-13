## Prediction-update handling: per run

Times in s. timeToFirstPlan: first provisional adoption minus first FULL_SEARCH submission. firstAdoptMinusRest: negative means the plan was obtained before the giver came to rest. discardedUnits: bounded planner work units (static preview steps, route work units, hypothesis setups) belonging to cancelled searches or to failed/cancelled selected-action certifications. concurrentMotion: 50 ms samples with robot and object truth both moving, before commitment.

| run | mode | completed | timeToFirstPlan | firstAdoptMinusRest | firstAdoptWhileMoving | firstAdoptLead | firstAdoptSource | fullSearches | fullCancelled | selectedTargetMoved | selectedCertOk | selectedCertFailed | discardedUnits | discardedWall | fullUnits | fullWall | fullWallToFirstPlan | allWorkerWallToCommit | concurrentMotion | replacements |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| restart_a/diagonal | cancel_and_restart | yes | 2.973 | -1.550 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.972 | 2.972 | 5.196 | 1.200 | 0 |
| restart_a/lateral-low | cancel_and_restart | no | 4.998 | 0.475 | no | 2.800 | search | 38 | 36 | 0 | 0 | 0 | 297679 | 4.488 | 327117 | 5.007 | 4.967 | 7.856 | 0.000 | 1 |
| restart_a/longitudinal | cancel_and_restart | yes | 5.092 | 0.569 | no | 2.800 | search | 36 | 35 | 0 | 0 | 0 | 295049 | 4.490 | 323819 | 5.067 | 5.067 | 7.051 | 0.000 | 0 |
| restart_a/near-ground | cancel_and_restart | yes | 3.147 | -1.376 | yes | 8.000 | search | 36 | 34 | 0 | 0 | 0 | 55355 | 0.583 | 313125 | 3.895 | 3.146 | 7.774 | 0.000 | 1 |
| restart_b/diagonal | cancel_and_restart | yes | 2.878 | -1.645 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.877 | 2.877 | 5.190 | 1.200 | 0 |
| restart_b/lateral-low | cancel_and_restart | yes | 4.977 | 0.454 | no | 2.800 | search | 38 | 37 | 0 | 0 | 0 | 297258 | 4.486 | 322781 | 4.948 | 4.948 | 7.052 | 0.000 | 0 |
| restart_b/longitudinal | cancel_and_restart | yes | 5.091 | 0.568 | no | 2.800 | search | 38 | 36 | 0 | 0 | 0 | 64491 | 0.791 | 349404 | 5.063 | 5.063 | 7.043 | 0.000 | 0 |
| restart_b/near-ground | cancel_and_restart | yes | 3.365 | -1.158 | yes | 8.000 | search | 35 | 33 | 0 | 0 | 0 | 54378 | 0.579 | 312148 | 4.060 | 3.324 | 7.757 | 0.000 | 1 |
| select_a/diagonal | select_then_certify | yes | 2.853 | -1.670 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.852 | 2.852 | 5.183 | 1.200 | 0 |
| select_a/lateral-low | select_then_certify | yes | 4.513 | -0.010 | yes | 8.000 | certified | 1 | 0 | 3 | 1 | 2 | 294 | 0.010 | 263056 | 4.492 | 4.492 | 7.643 | 0.000 | 0 |
| select_a/longitudinal | select_then_certify | yes | 6.576 | 2.053 | no | 2.800 | search | 3 | 0 | 0 | 0 | 0 | 0 | 0 | 526152 | 6.572 | 6.572 | 8.568 | 0.000 | 0 |
| select_a/near-ground | select_then_certify | yes | 3.128 | -1.395 | yes | 8.000 | search | 3 | 0 | 9 | 0 | 9 | 4615 | 0.094 | 510196 | 7.033 | 3.127 | 10.974 | 0.000 | 1 |
| select_b/diagonal | select_then_certify | yes | 3.033 | -1.490 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 3.032 | 3.032 | 5.246 | 1.200 | 0 |
| select_b/lateral-low | select_then_certify | yes | 5.653 | 1.130 | no | 3.250 | search | 2 | 0 | 11 | 0 | 11 | 1542 | 0.078 | 288570 | 5.563 | 5.563 | 7.967 | 0.000 | 0 |
| select_b/longitudinal | select_then_certify | no | 7.120 | 2.597 | no | 3.250 | search | 7 | 0 | 0 | 0 | 0 | 0 | 0 | 567313 | 7.693 | 7.116 | 8.906 | 0.000 | 0 |
| select_b/near-ground | select_then_certify | yes | 3.264 | -1.259 | yes | 8.000 | search | 3 | 0 | 1 | 0 | 1 | 107 | 0.004 | 515583 | 7.564 | 3.263 | 11.347 | 0.000 | 1 |
| final_a/diagonal | select_then_certify | yes | 2.950 | -1.573 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.949 | 2.949 | 5.212 | 1.200 | 0 |
| final_a/lateral-low | select_then_certify | no | 5.702 | 1.179 | no | 2.800 | search | 12 | 10 | 0 | 0 | 0 | 308652 | 5.213 | 338081 | 5.728 | 5.691 | 8.554 | 0.000 | 1 |
| final_a/longitudinal | select_then_certify | yes | 6.677 | 2.154 | no | 2.800 | search | 12 | 10 | 0 | 0 | 0 | 158134 | 2.484 | 443047 | 6.664 | 6.664 | 8.665 | 0.000 | 0 |
| final_a/near-ground | select_then_certify | yes | 3.117 | -1.406 | yes | 8.000 | search | 11 | 9 | 0 | 0 | 0 | 47340 | 0.732 | 305108 | 4.008 | 3.116 | 7.852 | 0.000 | 1 |
| final_b/diagonal | select_then_certify | yes | 3.010 | -1.513 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 3.009 | 3.009 | 5.202 | 1.200 | 0 |
| final_b/lateral-low | select_then_certify | no | 5.246 | 0.723 | no | 2.800 | search | 12 | 10 | 0 | 0 | 0 | 281073 | 4.757 | 310503 | 5.277 | 5.236 | 8.069 | 0.000 | 1 |
| final_b/longitudinal | select_then_certify | yes | 5.092 | 0.569 | no | 2.800 | search | 11 | 9 | 0 | 0 | 0 | 39805 | 0.664 | 324718 | 5.083 | 5.083 | 7.070 | 0.000 | 0 |
| final_b/near-ground | select_then_certify | yes | 3.248 | -1.275 | yes | 8.000 | search | 11 | 9 | 0 | 0 | 0 | 33643 | 0.601 | 291413 | 4.015 | 3.246 | 7.795 | 0.000 | 1 |
| final_c/diagonal | select_then_certify | yes | 2.947 | -1.576 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.946 | 2.946 | 5.240 | 1.200 | 0 |
| final_c/lateral-low | select_then_certify | no | 5.230 | 0.707 | no | 2.800 | search | 12 | 10 | 0 | 0 | 0 | 282889 | 4.757 | 312319 | 5.260 | 5.219 | 8.082 | 0.000 | 1 |
| final_c/longitudinal | select_then_certify | yes | 6.282 | 1.759 | no | 2.800 | search | 12 | 10 | 0 | 0 | 0 | 123469 | 1.967 | 408382 | 6.272 | 6.272 | 8.297 | 0.000 | 0 |
| final_c/near-ground | select_then_certify | yes | 3.084 | -1.439 | yes | 8.000 | search | 11 | 9 | 0 | 0 | 0 | 33997 | 0.600 | 291767 | 3.855 | 3.083 | 7.808 | 0.000 | 1 |

## Per mode (median over runs; counts are totals)

| mode | runs | completed | first plan while moving | median timeToFirstPlan | median firstAdoptMinusRest | total discardedUnits | total discardedWall | median fullWall | median concurrentMotion |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| cancel_and_restart | 8 | 7 | 4 | 4.171 | -0.352 | 1064210 | 15.417 | 4.504 | 0.000 |
| select_then_certify | 20 | 16 | 11 | 3.889 | -0.634 | 1315560 | 21.961 | 5.171 | 0.000 |
