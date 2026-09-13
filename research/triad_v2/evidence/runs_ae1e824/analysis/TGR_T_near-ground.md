# T/G/R characterization — near-ground.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 52.681119s; hypotheses 191, memo reuses 0, static records 6112, route records 6273
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_45deg/ring80mm_7of8 τ=3.200 (timing-admissible plans 1286, cost-valid 3589)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_45deg/ring80mm_7of8 τ=3.20 J=0.749762 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 191 on [0.50, 10.00] s, finest step 0.050 s
- geometrically complete (any cost-valid complete action): 159 leads, intervals [0.50, 5.30], [6.15, 6.75], [7.10, 7.35], [7.70, 8.75], [9.00, 10.00]
- timing-admissible and complete: 111 leads, intervals [2.90, 5.30], [6.15, 6.75], [7.10, 7.35], [7.70, 8.75], [9.00, 10.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 191 | 159 | 111 | 2.90 | 0.749762 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.20 | 4.0 |
| 0.10 | 0.5 | 96 | 80 | 56 | 2.90 | 0.749762 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.20 | 8.0 |
| 0.20 | 0.5 | 48 | 41 | 29 | 2.90 | 0.757292 | 0.007529 | axisP_side_45deg/ring80mm_7of8 τ=3.30 | 16.0 |
| 0.45 | 3.25 | 22 | 19 | 13 | 3.25 | 0.753454 | 0.003692 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 36.0 |
| 0.90 | 3.25 | 11 | 9 | 6 | 3.25 | 0.753454 | 0.003692 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 72.0 |
| 1.80 | 3.25 | 5 | 4 | 3 | 3.25 | 0.753454 | 0.003692 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 144.0 |
| V1 14 | V1 bank | 14 | 11 | 7 | 3.25 | 0.753454 | 0.003692 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 5/16 angles; arcs (deg) samples 45.0..67.5 (2 samples, spacing 22.500), samples 225.0..270.0 (3 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 2/16 angles; arcs (deg) samples 292.5..315.0 (2 samples, spacing 22.500)
- complete grasps per complete lead: median 1, min 1, max 4 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 21 | 21 | 9.00 | 1.100267 | 0.350504 | axisP_side_270deg/ring140mm_5of8 | 1528 |
| 8 | 16 | 124 | 73 | 3.05 | 0.749762 | 0.000000 | axisP_side_45deg/ring80mm_7of8 | 3056 |
| 16 | 32 | 159 | 111 | 2.90 | 0.749762 | 0.000000 | axisP_side_45deg/ring80mm_7of8 | 6112 |

- complete records by handle-axis sign: {'axisP': 2582, 'axisN': 1007}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 369
  - direct sufficient: 241 (65.3%)
  - route alternative required: 46 (12.5%)
  - no complete route: 82 (22.2%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×20; INSERTION `predictive_static` ×14; NONE `predictive_static/runtime_clearance_reserve` ×6; NONE `predictive_static/robust_transit_clearance_reserve` ×4; CARRIED_RETREAT `feasible` ×2
- route wall time in this search 39.972 s over 6273 route rollouts (mean 6.37 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 241 | 147 | 0.793138 | 0.043375 | axisP_side_45deg/direct τ=3.05 | 369 |
| direct + 4 dirs × [80] mm | 5 | 286 | 158 | 0.766191 | 0.016429 | axisP_side_45deg/ring80mm_0of8 τ=3.20 | 1845 |
| direct + 8 dirs × [80] mm | 9 | 287 | 159 | 0.749762 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.20 | 3321 |
| direct + 4 dirs × [140] mm | 5 | 284 | 156 | 0.793138 | 0.043375 | axisP_side_45deg/direct τ=3.05 | 1845 |
| direct + 8 dirs × [140] mm | 9 | 286 | 158 | 0.771655 | 0.021893 | axisP_side_45deg/ring140mm_7of8 τ=3.45 | 3321 |
| direct + 4 dirs × [80, 140] mm | 9 | 286 | 158 | 0.766191 | 0.016429 | axisP_side_45deg/ring80mm_0of8 τ=3.20 | 3321 |
| direct + 8 dirs × [80, 140] mm | 17 | 287 | 159 | 0.749762 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.20 | 6273 |
