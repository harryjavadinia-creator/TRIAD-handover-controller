## Deepest certified stage of every evaluated candidate (accepted FULL_SEARCH jobs, memoized hypotheses expanded)

- **static** (n=48896): NONE 28293 (57.9%), REACH 1336 (2.7%), INSERTION 13524 (27.7%), CLOSURE_CONTACT 1699 (3.5%), CARRIED_RETREAT 4044 (8.3%)
  - rejected only downstream of reach: 16559 = 33.9% of evaluated, 80.4% of reach-certified
- **route** (n=68748): NONE 28913 (42.1%), REACH 0 (0.0%), INSERTION 1113 (1.6%), CLOSURE_CONTACT 5 (0.0%), CARRIED_RETREAT 38717 (56.3%); COMPLETE (retreat + valid cost audit) 38191 (55.6%)
  - rejected only downstream of reach: 1118 = 1.6% of evaluated, 2.8% of reach-certified

### Most frequent downstream-only rejection reasons

| path | deepest | reason | count |
|---|---|---|---:|
| static | INSERTION | `closure/pad_pair/blue_handle_acquisition_tube` | 11705 |
| static | CLOSURE_CONTACT | `ik_preview_no_convergence` | 1678 |
| static | REACH | `ik_preview_no_convergence` | 1263 |
| static | INSERTION | `closure/right_pad_shoulder_low/robot_blue_handle` | 936 |
| static | INSERTION | `closure/left_pad_shoulder_low/robot_blue_handle` | 673 |
| route | INSERTION | `predictive_static/static_acquire/closure/right_pad_shoulder_low` | 508 |
| route | INSERTION | `predictive_static/static_acquire/closure/pad_pair` | 500 |
| static | INSERTION | `closure/left_inner_pad/robot_blue_handle` | 199 |
| route | INSERTION | `predictive_static/static_acquire/closure/left_pad_shoulder_low` | 97 |
| static | REACH | `mouth_corridor/blue_handle_lateral` | 46 |
| static | REACH | `mouth_corridor/blue_handle_axial_offset` | 27 |
| static | CLOSURE_CONTACT | `gen3_spherical_wrist_2_link/ground_plane` | 20 |
| static | INSERTION | `closure/right_inner_pad/robot_blue_handle` | 10 |
| route | INSERTION | `predictive_static/static_acquire/closure/right_inner_pad` | 8 |
| route | CLOSURE_CONTACT | `predictive_static/static_retreat/ik_preview_no_convergence` | 5 |

Route rollouts are run only for grasps that already passed the complete static screen (direct copied-state reach, insertion, closure, carried retreat), so route-level stage counts are conditional on that screen.

### Reach-only selection versus complete-action certification

Reach-only rule (lexicographic): earliest timing-admissible event, then shortest reach time, then largest reach clearance; admissible at the search epoch when lead >= 1.6 s and reach time + 0.05 s <= lead. Static: grasp reach to standoff on the direct copied-state screen. Route: (grasp, route) whose route reach phase certified. 'complete?' = the chosen action has a COMPLETE record (static: any route of that grasp at that event).

| log | job | reach-only static choice | complete? | deepest | reach-only route choice | complete? | deepest (reason) | reach-feasible static actions at that event that are complete | reach-feasible route actions at that event that are complete |
|---|---|---|---|---|---|---|---|---|---|
| char_T/diagonal | 1 | τ=1.900 axisP_side_0deg | **NO** | INSERTION | τ=3.900 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 14/14 |
| char_T/diagonal | 2 | τ=1.900 axisP_side_315deg | **NO** | CLOSURE_CONTACT | τ=2.000 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 1/1 |
| char_T/lateral-low | 1 | τ=2.050 axisN_side_337deg | **NO** | INSERTION | τ=3.100 axisN_side_68deg/ring80mm_5of8 | yes | CARRIED_RETREAT (feasible) | 0/1 | 1/1 |
| char_T/lateral-low | 2 | τ=2.150 axisP_side_0deg | **NO** | INSERTION | τ=2.250 axisN_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/2 | 1/1 |
| char_T/longitudinal | 1 | τ=1.800 axisP_side_337deg | **NO** | INSERTION | τ=2.100 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 1/1 |
| char_T/longitudinal | 2 | τ=2.150 axisP_side_337deg | yes | CARRIED_RETREAT | τ=2.150 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 1/1 | 1/1 |
| char_T/near-ground | 1 | τ=2.800 axisN_side_293deg | yes | CARRIED_RETREAT | τ=2.800 axisN_side_293deg/direct | **NO** | INSERTION (predictive_static/static_acquire/closure/right_pad_shoulder_) | 1/1 | 0/1 |
| char_T/near-ground | 2 | τ=2.750 axisN_side_0deg | **NO** | REACH | τ=3.000 axisP_side_45deg/direct | yes | CARRIED_RETREAT (feasible) | 0/1 | 1/1 |
