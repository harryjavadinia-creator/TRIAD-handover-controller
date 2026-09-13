# T/G/R characterization — longitudinal.log, rest epoch search (generation 2)

- search epoch t=21.054 s; object estimate speed at epoch 0.0000 m/s; worker wall 2.065207s; hypotheses 14, memo reuses 13, static records 32, route records 390
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring40mm_2of16 τ=2.350 (timing-admissible plans 1698, cost-valid 2324)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring40mm_2of16 τ=2.35 J=0.623181 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 12 | 2.35 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 0.0 |
| 0.10 | 0.5 | 8 | 8 | 6 | 2.80 | 0.641091 | 0.017910 | axisP_side_337deg/ring80mm_2of16 τ=2.80 | 0.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.666930 | 0.043749 | axisP_side_337deg/ring140mm_3of16 τ=3.70 | 0.0 |
| 0.45 | 3.25 | 12 | 12 | 11 | 2.35 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 0.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 0.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.643246 | 0.020065 | axisP_side_337deg/ring140mm_3of16 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 4/16 angles; arcs (deg) samples 45.0..67.5 (2 samples, spacing 22.500), samples 315.0..337.5 (2 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 2/16 angles; arcs (deg) samples 247.5..270.0 (2 samples, spacing 22.500)
- complete grasps per complete lead: median 6, min 6, max 6 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 14 | 8 | 4.15 | 1.070018 | 0.446837 | axisN_side_270deg/ring200mm_9of16 | 112 |
| 8 | 16 | 14 | 12 | 2.35 | 0.655155 | 0.031974 | axisP_side_315deg/ring40mm_1of16 | 224 |
| 16 | 32 | 14 | 12 | 2.35 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 | 448 |

- complete records by handle-axis sign: {'axisP': 2226, 'axisN': 98}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 84
  - direct sufficient: 42 (50.0%)
  - route alternative required: 42 (50.0%)
  - no complete route: 0 (0.0%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×28; NONE `predictive_static/runtime_clearance_reserve` ×14
- route wall time in this search 1.980 s over 390 route rollouts (mean 5.08 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 42 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 84 |
| direct + 4 dirs × [40] mm | 5 | 42 | 14 | 0.624660 | 0.001479 | axisP_side_337deg/ring40mm_4of16 τ=2.35 | 420 |
| direct + 8 dirs × [40] mm | 9 | 42 | 14 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 756 |
| direct + 16 dirs × [40] mm | 17 | 42 | 14 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 1428 |
| direct + 4 dirs × [80] mm | 5 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 420 |
| direct + 8 dirs × [80] mm | 9 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 756 |
| direct + 16 dirs × [80] mm | 17 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 1428 |
| direct + 4 dirs × [140] mm | 5 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 420 |
| direct + 8 dirs × [140] mm | 9 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 756 |
| direct + 16 dirs × [140] mm | 17 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 1428 |
| direct + 4 dirs × [200] mm | 5 | 84 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 420 |
| direct + 8 dirs × [200] mm | 9 | 84 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 756 |
| direct + 16 dirs × [200] mm | 17 | 84 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 1428 |
| direct + 4 dirs × [80, 140] mm | 9 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 756 |
| direct + 8 dirs × [80, 140] mm | 17 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 1428 |
| direct + 16 dirs × [80, 140] mm | 33 | 56 | 14 | 0.626211 | 0.003030 | axisP_side_337deg/direct τ=2.35 | 2772 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 84 | 14 | 0.624660 | 0.001479 | axisP_side_337deg/ring40mm_4of16 τ=2.35 | 1428 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 84 | 14 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 2772 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 84 | 14 | 0.623181 | 0.000000 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 5460 |
