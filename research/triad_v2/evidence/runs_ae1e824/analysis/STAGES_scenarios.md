## Deepest certified stage of every evaluated candidate (accepted FULL_SEARCH jobs, memoized hypotheses expanded)

- **static** (n=5376): NONE 3113 (57.9%), REACH 167 (3.1%), INSERTION 1434 (26.7%), CLOSURE_CONTACT 167 (3.1%), CARRIED_RETREAT 495 (9.2%)
  - rejected only downstream of reach: 1768 = 32.9% of evaluated, 78.1% of reach-certified
- **route** (n=8415): NONE 4298 (51.1%), REACH 0 (0.0%), INSERTION 53 (0.6%), CLOSURE_CONTACT 0 (0.0%), CARRIED_RETREAT 4064 (48.3%); COMPLETE (retreat + valid cost audit) 4036 (48.0%)
  - rejected only downstream of reach: 53 = 0.6% of evaluated, 1.3% of reach-certified

### Most frequent downstream-only rejection reasons

| path | deepest | reason | count |
|---|---|---|---:|
| static | INSERTION | `closure/pad_pair/blue_handle_acquisition_tube` | 1261 |
| static | CLOSURE_CONTACT | `ik_preview_no_convergence` | 167 |
| static | REACH | `ik_preview_no_convergence` | 159 |
| static | INSERTION | `closure/right_pad_shoulder_low/robot_blue_handle` | 130 |
| static | INSERTION | `closure/left_pad_shoulder_low/robot_blue_handle` | 39 |
| route | INSERTION | `predictive_static/static_acquire/closure/right_pad_shoulder_low` | 30 |
| route | INSERTION | `predictive_static/static_acquire/closure/pad_pair` | 23 |
| static | REACH | `mouth_corridor/blue_handle_axial_offset` | 6 |
| static | INSERTION | `closure/right_inner_pad/robot_blue_handle` | 2 |
| static | INSERTION | `closure/left_inner_pad/robot_blue_handle` | 2 |
| static | REACH | `mouth_corridor/blue_handle_lateral` | 2 |

Route rollouts are run only for grasps that already passed the complete static screen (direct copied-state reach, insertion, closure, carried retreat), so route-level stage counts are conditional on that screen.

### Reach-only selection versus complete-action certification

Reach-only rule (lexicographic): earliest timing-admissible event, then shortest reach time, then largest reach clearance; admissible at the search epoch when lead >= 1.6 s and reach time + 0.05 s <= lead. Static: grasp reach to standoff on the direct copied-state screen. Route: (grasp, route) whose route reach phase certified. 'complete?' = the chosen action has a COMPLETE record (static: any route of that grasp at that event).

| log | job | reach-only static choice | complete? | deepest | reach-only route choice | complete? | deepest (reason) | reach-feasible static actions at that event that are complete | reach-feasible route actions at that event that are complete |
|---|---|---|---|---|---|---|---|---|---|
| v2/diagonal | 1 | τ=1.900 axisP_side_0deg | **NO** | INSERTION | τ=4.150 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 14/14 |
| v2/lateral-low | 37 | τ=2.350 axisP_side_0deg | **NO** | INSERTION | τ=2.350 axisN_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 1/5 | 1/1 |
| v2/lateral-low | 296 | τ=1.800 axisN_side_0deg | **NO** | INSERTION | τ=1.800 axisN_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 1/5 | 1/1 |
| v2/longitudinal | 1 | τ=1.800 axisP_side_337deg | **NO** | INSERTION | τ=2.350 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 1/1 |
| v2/longitudinal | 39 | τ=2.350 axisP_side_337deg | yes | CARRIED_RETREAT | τ=2.350 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 2/3 | 2/2 |
| v2/near-ground | 1 | τ=2.800 axisN_side_293deg | yes | CARRIED_RETREAT | τ=2.800 axisN_side_293deg/direct | **NO** | INSERTION (predictive_static/static_acquire/closure/right_pad_shoulder_) | 1/1 | 0/1 |
| v2/near-ground | 121 | τ=2.800 axisN_side_0deg | **NO** | REACH | τ=3.250 axisP_side_45deg/direct | yes | CARRIED_RETREAT (feasible) | 0/2 | 7/7 |
| v2_repeat/diagonal | 1 | τ=1.900 axisP_side_0deg | **NO** | INSERTION | τ=4.150 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 14/14 |
| v2_repeat/lateral-low | 37 | τ=2.350 axisP_side_0deg | **NO** | INSERTION | τ=2.350 axisN_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 1/5 | 1/1 |
| v2_repeat/longitudinal | 37 | τ=2.350 axisP_side_337deg | yes | CARRIED_RETREAT | τ=2.350 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 2/3 | 2/2 |
| v2_repeat/near-ground | 1 | τ=2.800 axisN_side_293deg | yes | CARRIED_RETREAT | τ=2.800 axisN_side_293deg/direct | **NO** | INSERTION (predictive_static/static_acquire/closure/right_pad_shoulder_) | 1/1 | 0/1 |
| v2_repeat/near-ground | 125 | τ=2.800 axisN_side_0deg | **NO** | REACH | τ=3.250 axisP_side_45deg/direct | yes | CARRIED_RETREAT (feasible) | 0/2 | 7/7 |
