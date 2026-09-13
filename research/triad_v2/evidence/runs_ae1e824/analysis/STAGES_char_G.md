## Deepest certified stage of every evaluated candidate (accepted FULL_SEARCH jobs, memoized hypotheses expanded)

- **static** (n=14336): NONE 8216 (57.3%), REACH 338 (2.4%), INSERTION 4173 (29.1%), CLOSURE_CONTACT 471 (3.3%), CARRIED_RETREAT 1138 (7.9%)
  - rejected only downstream of reach: 4982 = 34.8% of evaluated, 81.4% of reach-certified
- **route** (n=19346): NONE 9031 (46.7%), REACH 0 (0.0%), INSERTION 254 (1.3%), CLOSURE_CONTACT 0 (0.0%), CARRIED_RETREAT 10061 (52.0%); COMPLETE (retreat + valid cost audit) 9759 (50.4%)
  - rejected only downstream of reach: 254 = 1.3% of evaluated, 2.5% of reach-certified

### Most frequent downstream-only rejection reasons

| path | deepest | reason | count |
|---|---|---|---:|
| static | INSERTION | `closure/pad_pair/blue_handle_acquisition_tube` | 3660 |
| static | CLOSURE_CONTACT | `ik_preview_no_convergence` | 462 |
| static | REACH | `ik_preview_no_convergence` | 317 |
| static | INSERTION | `closure/right_pad_shoulder_low/robot_blue_handle` | 239 |
| static | INSERTION | `closure/left_pad_shoulder_low/robot_blue_handle` | 208 |
| route | INSERTION | `predictive_static/static_acquire/closure/pad_pair` | 91 |
| route | INSERTION | `predictive_static/static_acquire/closure/right_pad_shoulder_low` | 83 |
| route | INSERTION | `predictive_static/static_acquire/closure/left_pad_shoulder_low` | 79 |
| static | INSERTION | `closure/left_inner_pad/robot_blue_handle` | 62 |
| static | REACH | `mouth_corridor/blue_handle_axial_offset` | 14 |
| static | CLOSURE_CONTACT | `gen3_spherical_wrist_2_link/ground_plane` | 9 |
| static | REACH | `mouth_corridor/blue_handle_lateral` | 7 |
| static | INSERTION | `closure/right_inner_pad/robot_blue_handle` | 4 |
| route | INSERTION | `predictive_static/static_acquire/closure/right_inner_pad` | 1 |

Route rollouts are run only for grasps that already passed the complete static screen (direct copied-state reach, insertion, closure, carried retreat), so route-level stage counts are conditional on that screen.

### Reach-only selection versus complete-action certification

Reach-only rule (lexicographic): earliest timing-admissible event, then shortest reach time, then largest reach clearance; admissible at the search epoch when lead >= 1.6 s and reach time + 0.05 s <= lead. Static: grasp reach to standoff on the direct copied-state screen. Route: (grasp, route) whose route reach phase certified. 'complete?' = the chosen action has a COMPLETE record (static: any route of that grasp at that event).

| log | job | reach-only static choice | complete? | deepest | reach-only route choice | complete? | deepest (reason) | reach-feasible static actions at that event that are complete | reach-feasible route actions at that event that are complete |
|---|---|---|---|---|---|---|---|---|---|
| char_G/diagonal | 1 | τ=1.900 axisP_side_6deg | **NO** | INSERTION | τ=4.150 axisP_side_321deg/direct | yes | CARRIED_RETREAT (feasible) | 0/4 | 70/70 |
| char_G/diagonal | 2 | τ=1.900 axisP_side_326deg | yes | CARRIED_RETREAT | τ=1.900 axisP_side_326deg/direct | yes | CARRIED_RETREAT (feasible) | 1/3 | 1/1 |
| char_G/lateral-low | 1 | τ=2.350 axisN_side_332deg | yes | CARRIED_RETREAT | τ=2.350 axisN_side_332deg/direct | yes | CARRIED_RETREAT (feasible) | 2/20 | 18/18 |
| char_G/lateral-low | 2 | τ=2.350 axisP_side_11deg | **NO** | INSERTION | τ=2.350 axisN_side_332deg/direct | yes | CARRIED_RETREAT (feasible) | 3/19 | 3/3 |
| char_G/longitudinal | 1 | τ=1.800 axisP_side_337deg | **NO** | INSERTION | τ=2.350 axisP_side_337deg/direct | yes | CARRIED_RETREAT (feasible) | 0/4 | 1/1 |
| char_G/longitudinal | 2 | τ=2.350 axisP_side_332deg | yes | CARRIED_RETREAT | τ=2.350 axisP_side_332deg/direct | yes | CARRIED_RETREAT (feasible) | 8/12 | 8/8 |
| char_G/near-ground | 1 | τ=2.350 axisN_side_349deg | **NO** | REACH | τ=2.800 axisN_side_293deg/direct | **NO** | INSERTION (predictive_static/static_acquire/closure/right_pad_shoulder_) | 0/1 | 0/2 |
| char_G/near-ground | 2 | τ=2.800 axisN_side_0deg | **NO** | REACH | τ=3.250 axisP_side_51deg/direct | yes | CARRIED_RETREAT (feasible) | 0/5 | 28/35 |
