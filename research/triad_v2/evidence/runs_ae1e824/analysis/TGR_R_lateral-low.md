# T/G/R characterization — lateral-low.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 16.359360s; hypotheses 14, memo reuses 0, static records 448, route records 2795
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisN_side_337deg/ring40mm_7of16 τ=3.700 (timing-admissible plans 1029, cost-valid 1313)
- re-implemented argmin globalJ on admissible cost-valid records: axisN_side_337deg/ring40mm_7of16 τ=3.70 J=0.675545 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 10 leads, intervals [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 10 | 3.25 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 4.0 |
| 0.10 | 0.5 | 8 | 8 | 5 | 3.70 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 8.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 16.0 |
| 0.45 | 3.25 | 12 | 12 | 9 | 3.25 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 36.0 |
| 0.90 | 3.25 | 6 | 6 | 5 | 3.25 | 0.690086 | 0.014540 | axisN_side_337deg/ring40mm_7of16 τ=4.15 | 72.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.741496 | 0.065951 | axisN_side_337deg/ring140mm_6of16 τ=5.05 | 144.0 |
| V1 14 | V1 bank | 14 | 14 | 10 | 3.25 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 2/16 angles; arcs (deg) samples 270.0..292.5 (2 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 7/16 angles; arcs (deg) samples -22.5..112.5 (7 samples, spacing 22.500)
- complete grasps per complete lead: median 4, min 1, max 5 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 7 | 7 | 4.60 | 0.868017 | 0.192471 | axisN_side_90deg/ring200mm_14of16 | 112 |
| 8 | 16 | 9 | 9 | 3.70 | 0.719130 | 0.043585 | axisN_side_45deg/ring140mm_12of16 | 224 |
| 16 | 32 | 14 | 10 | 3.25 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 | 448 |

- complete records by handle-axis sign: {'axisN': 1248, 'axisP': 65}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 43
  - direct sufficient: 8 (18.6%)
  - route alternative required: 35 (81.4%)
  - no complete route: 0 (0.0%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×34; NONE `predictive_static/robust_transit_clearance_reserve` ×1
- route wall time in this search 15.765 s over 2795 route rollouts (mean 5.64 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 8 | 7 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 43 |
| direct + 4 dirs × [40] mm | 5 | 11 | 8 | 0.675594 | 0.000048 | axisN_side_337deg/ring40mm_8of16 τ=3.70 | 215 |
| direct + 8 dirs × [40] mm | 9 | 13 | 9 | 0.675594 | 0.000048 | axisN_side_337deg/ring40mm_8of16 τ=3.70 | 387 |
| direct + 16 dirs × [40] mm | 17 | 13 | 9 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 731 |
| direct + 4 dirs × [80] mm | 5 | 32 | 11 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 215 |
| direct + 8 dirs × [80] mm | 9 | 37 | 13 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 387 |
| direct + 16 dirs × [80] mm | 17 | 37 | 13 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 731 |
| direct + 4 dirs × [140] mm | 5 | 41 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 215 |
| direct + 8 dirs × [140] mm | 9 | 42 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 387 |
| direct + 16 dirs × [140] mm | 17 | 43 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 731 |
| direct + 4 dirs × [200] mm | 5 | 43 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 215 |
| direct + 8 dirs × [200] mm | 9 | 43 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 387 |
| direct + 16 dirs × [200] mm | 17 | 43 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 731 |
| direct + 4 dirs × [80, 140] mm | 9 | 41 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 387 |
| direct + 8 dirs × [80, 140] mm | 17 | 42 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 731 |
| direct + 16 dirs × [80, 140] mm | 33 | 43 | 14 | 0.677720 | 0.002174 | axisN_side_337deg/direct τ=3.70 | 1419 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 43 | 14 | 0.675594 | 0.000048 | axisN_side_337deg/ring40mm_8of16 τ=3.70 | 731 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 43 | 14 | 0.675594 | 0.000048 | axisN_side_337deg/ring40mm_8of16 τ=3.70 | 1419 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 43 | 14 | 0.675545 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 2795 |
