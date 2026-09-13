## Deepest certified stage of every evaluated candidate (accepted FULL_SEARCH jobs, memoized hypotheses expanded)

- **static** (n=3584): NONE 2009 (56.1%), REACH 89 (2.5%), INSERTION 1064 (29.7%), CLOSURE_CONTACT 128 (3.6%), CARRIED_RETREAT 294 (8.2%)
  - rejected only downstream of reach: 1281 = 35.7% of evaluated, 81.3% of reach-certified
- **route** (n=4998): NONE 2212 (44.3%), REACH 0 (0.0%), INSERTION 39 (0.8%), CLOSURE_CONTACT 0 (0.0%), CARRIED_RETREAT 2747 (55.0%); COMPLETE (retreat + valid cost audit) 2733 (54.7%)
  - rejected only downstream of reach: 39 = 0.8% of evaluated, 1.4% of reach-certified

### Most frequent downstream-only rejection reasons

| path | deepest | reason | count |
|---|---|---|---:|
| static | INSERTION | `closure/pad_pair/blue_handle_acquisition_tube` | 924 |
| static | CLOSURE_CONTACT | `ik_preview_no_convergence` | 127 |
| static | REACH | `ik_preview_no_convergence` | 85 |
| static | INSERTION | `closure/right_pad_shoulder_low/robot_blue_handle` | 78 |
| static | INSERTION | `closure/left_pad_shoulder_low/robot_blue_handle` | 46 |
| route | INSERTION | `predictive_static/static_acquire/closure/pad_pair` | 22 |
| route | INSERTION | `predictive_static/static_acquire/closure/right_pad_shoulder_low` | 17 |
| static | INSERTION | `closure/left_inner_pad/robot_blue_handle` | 15 |
| static | REACH | `mouth_corridor/blue_handle_axial_offset` | 3 |
| static | INSERTION | `closure/right_inner_pad/robot_blue_handle` | 1 |
| static | CLOSURE_CONTACT | `gen3_spherical_wrist_2_link/ground_plane` | 1 |
| static | REACH | `mouth_corridor/blue_handle_lateral` | 1 |

Route rollouts are run only for grasps that already passed the complete static screen (direct copied-state reach, insertion, closure, carried retreat), so route-level stage counts are conditional on that screen.

### Reach-only selection versus complete-action certification

Reach-only rule (lexicographic): earliest timing-admissible event, then shortest reach time, then largest reach clearance; admissible at the search epoch when lead >= 1.6 s and reach time + 0.05 s <= lead. Static: grasp reach to standoff on the direct copied-state screen. Route: (grasp, route) whose route reach phase certified. 'complete?' = the chosen action has a COMPLETE record (static: any route of that grasp at that event).

| log | job | reach-only static choice | complete? | deepest | reach-only route choice | complete? | deepest (reason) | reach-feasible static actions at that event that are complete | reach-feasible route actions at that event that are complete |
|---|---|---|---|---|---|---|---|---|---|
| char_default/diagonal | 1 | τ=1.900 axisP_side_0deg | **NO** | INSERTION | τ=4.150 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 14/14 |
| char_default/diagonal | 2 | τ=1.900 axisP_side_315deg | **NO** | CLOSURE_CONTACT | τ=2.350 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 1/1 |
| char_default/lateral-low | 1 | τ=2.350 axisN_side_337deg | **NO** | INSERTION | τ=3.250 axisN_side_68deg/ring80mm_5of8 | yes | CARRIED_RETREAT (feasible) | 0/5 | 1/1 |
| char_default/lateral-low | 2 | τ=2.350 axisP_side_0deg | **NO** | INSERTION | τ=2.350 axisN_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 1/5 | 1/1 |
| char_default/longitudinal | 1 | τ=1.800 axisP_side_337deg | **NO** | INSERTION | τ=2.350 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 1/1 |
| char_default/longitudinal | 2 | τ=2.350 axisP_side_337deg | yes | CARRIED_RETREAT | τ=2.350 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 2/3 | 2/2 |
| char_default/near-ground | 1 | τ=2.800 axisN_side_293deg | yes | CARRIED_RETREAT | τ=2.800 axisN_side_293deg/direct | **NO** | INSERTION (predictive_static/static_acquire/closure/right_pad_shoulder_) | 1/1 | 0/1 |
| char_default/near-ground | 2 | τ=2.800 axisN_side_0deg | **NO** | REACH | τ=3.250 axisP_side_45deg/direct | yes | CARRIED_RETREAT (feasible) | 0/2 | 7/7 |
