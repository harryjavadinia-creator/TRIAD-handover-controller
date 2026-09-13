## Prediction-update handling: per run

Times in s. timeToFirstPlan: first provisional adoption minus first FULL_SEARCH submission. firstAdoptMinusRest: negative means the plan was obtained before the giver came to rest. discardedUnits: bounded planner work units (static preview steps, route work units, hypothesis setups) belonging to cancelled searches or to failed/cancelled selected-action certifications. concurrentMotion: 50 ms samples with robot and object truth both moving, before commitment.

| run | mode | completed | timeToFirstPlan | firstAdoptMinusRest | firstAdoptWhileMoving | firstAdoptLead | firstAdoptSource | fullSearches | fullCancelled | selectedTargetMoved | selectedCertOk | selectedCertFailed | discardedUnits | discardedWall | fullUnits | fullWall | fullWallToFirstPlan | allWorkerWallToCommit | concurrentMotion | replacements |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| final_inject_stale/diagonal | select_then_certify | yes | 2.895 | -1.628 | yes | 5.500 | search | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 222160 | 2.894 | 2.894 | 5.163 | 1.200 | 0 |
| final_inject_supersede/longitudinal | select_then_certify | yes | 5.079 | 0.556 | no | 2.800 | search | 11 | 10 | 0 | 0 | 0 | 303397 | 4.505 | 332167 | 5.068 | 5.068 | 7.079 | 0.000 | 0 |

## Per mode (median over runs; counts are totals)

| mode | runs | completed | first plan while moving | median timeToFirstPlan | median firstAdoptMinusRest | total discardedUnits | total discardedWall | median fullWall | median concurrentMotion |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| select_then_certify | 2 | 2 | 1 | 3.987 | -0.536 | 303397 | 4.505 | 3.981 | 0.600 |
