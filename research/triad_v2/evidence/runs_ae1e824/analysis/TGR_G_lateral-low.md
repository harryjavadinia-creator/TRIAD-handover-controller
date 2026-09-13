# T/G/R characterization — lateral-low.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 19.054458s; hypotheses 14, memo reuses 0, static records 1792, route records 2907
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisN_side_337deg/direct τ=3.700 (timing-admissible plans 1200, cost-valid 1500)
- re-implemented argmin globalJ on admissible cost-valid records: axisN_side_337deg/direct τ=3.70 J=0.677720 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 12 | 2.35 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 4.0 |
| 0.10 | 0.5 | 8 | 8 | 6 | 2.80 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 8.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 16.0 |
| 0.45 | 3.25 | 12 | 12 | 11 | 2.35 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 36.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.692601 | 0.014881 | axisN_side_337deg/direct τ=4.15 | 72.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.700215 | 0.022495 | axisN_side_51deg/ring140mm_6of8 τ=3.25 | 144.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 64 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 5/64 angles; arcs (deg) samples 270.0..270.0 (1 samples, spacing 5.625), samples 281.2..292.5 (3 samples, spacing 5.625), samples 348.8..348.8 (1 samples, spacing 5.625)
- axisN: complete at any evaluated lead for 28/64 angles; arcs (deg) samples -39.4..112.5 (28 samples, spacing 5.625)
- complete grasps per complete lead: median 12, min 7, max 19 (of 128)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 7 | 7 | 4.60 | 0.871790 | 0.194070 | axisN_side_90deg/ring140mm_7of8 | 112 |
| 8 | 16 | 9 | 9 | 3.70 | 0.719130 | 0.041410 | axisN_side_45deg/ring140mm_6of8 | 224 |
| 16 | 32 | 14 | 10 | 3.25 | 0.677720 | 0.000000 | axisN_side_337deg/direct | 448 |
| 32 | 64 | 14 | 12 | 2.35 | 0.677720 | 0.000000 | axisN_side_337deg/direct | 896 |
| 64 | 128 | 14 | 12 | 2.35 | 0.677720 | 0.000000 | axisN_side_337deg/direct | 1792 |

- complete records by handle-axis sign: {'axisN': 1418, 'axisP': 82}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 171
  - direct sufficient: 30 (17.5%)
  - route alternative required: 140 (81.9%)
  - no complete route: 1 (0.6%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×135; NONE `predictive_static/robust_transit_clearance_reserve` ×4; NONE `predictive_static/runtime_clearance_reserve` ×1
- route wall time in this search 16.778 s over 2907 route rollouts (mean 5.77 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 30 | 11 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 171 |
| direct + 4 dirs × [80] mm | 5 | 131 | 14 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 855 |
| direct + 8 dirs × [80] mm | 9 | 148 | 14 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 1539 |
| direct + 4 dirs × [140] mm | 5 | 168 | 14 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 855 |
| direct + 8 dirs × [140] mm | 9 | 170 | 14 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 1539 |
| direct + 4 dirs × [80, 140] mm | 9 | 168 | 14 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 1539 |
| direct + 8 dirs × [80, 140] mm | 17 | 170 | 14 | 0.677720 | 0.000000 | axisN_side_337deg/direct τ=3.70 | 2907 |
