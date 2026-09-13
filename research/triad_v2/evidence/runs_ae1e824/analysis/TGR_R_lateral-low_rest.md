# T/G/R characterization — lateral-low.log, rest epoch search (generation 2)

- search epoch t=25.385 s; object estimate speed at epoch 0.0000 m/s; worker wall 1.817739s; hypotheses 14, memo reuses 13, static records 32, route records 260
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisN_side_337deg/ring40mm_7of16 τ=2.350 (timing-admissible plans 1368, cost-valid 2128)
- re-implemented argmin globalJ on admissible cost-valid records: axisN_side_337deg/ring40mm_7of16 τ=2.35 J=0.595439 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 12 | 2.35 | 0.595439 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=2.35 | 0.0 |
| 0.10 | 0.5 | 8 | 8 | 6 | 2.80 | 0.619123 | 0.023684 | axisN_side_337deg/ring40mm_7of16 τ=2.80 | 0.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.666492 | 0.071053 | axisN_side_337deg/ring40mm_7of16 τ=3.70 | 0.0 |
| 0.45 | 3.25 | 12 | 12 | 11 | 2.35 | 0.595439 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=2.35 | 0.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.595439 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=2.35 | 0.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.642808 | 0.047368 | axisN_side_337deg/ring40mm_7of16 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.595439 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=2.35 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 1/16 angles; arcs (deg) samples 292.5..292.5 (1 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 3/16 angles; arcs (deg) samples 45.0..67.5 (2 samples, spacing 22.500), samples 337.5..337.5 (1 samples, spacing 22.500)
- complete grasps per complete lead: median 4, min 4, max 4 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 14 | 10 | 3.25 | 0.690792 | 0.095352 | axisN_side_45deg/ring140mm_12of16 | 224 |
| 16 | 32 | 14 | 12 | 2.35 | 0.595439 | 0.000000 | axisN_side_337deg/ring40mm_7of16 | 448 |

- complete records by handle-axis sign: {'axisP': 630, 'axisN': 1498}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 56
  - direct sufficient: 28 (50.0%)
  - route alternative required: 28 (50.0%)
  - no complete route: 0 (0.0%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×28
- route wall time in this search 1.744 s over 260 route rollouts (mean 6.71 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 28 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 56 |
| direct + 4 dirs × [40] mm | 5 | 28 | 14 | 0.595543 | 0.000104 | axisN_side_337deg/ring40mm_8of16 τ=2.35 | 280 |
| direct + 8 dirs × [40] mm | 9 | 28 | 14 | 0.595543 | 0.000104 | axisN_side_337deg/ring40mm_8of16 τ=2.35 | 504 |
| direct + 16 dirs × [40] mm | 17 | 28 | 14 | 0.595439 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=2.35 | 952 |
| direct + 4 dirs × [80] mm | 5 | 42 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 280 |
| direct + 8 dirs × [80] mm | 9 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 504 |
| direct + 16 dirs × [80] mm | 17 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 952 |
| direct + 4 dirs × [140] mm | 5 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 280 |
| direct + 8 dirs × [140] mm | 9 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 504 |
| direct + 16 dirs × [140] mm | 17 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 952 |
| direct + 4 dirs × [200] mm | 5 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 280 |
| direct + 8 dirs × [200] mm | 9 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 504 |
| direct + 16 dirs × [200] mm | 17 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 952 |
| direct + 4 dirs × [80, 140] mm | 9 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 504 |
| direct + 8 dirs × [80, 140] mm | 17 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 952 |
| direct + 16 dirs × [80, 140] mm | 33 | 56 | 14 | 0.597838 | 0.002399 | axisN_side_337deg/direct τ=2.35 | 1848 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 56 | 14 | 0.595543 | 0.000104 | axisN_side_337deg/ring40mm_8of16 τ=2.35 | 952 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 56 | 14 | 0.595543 | 0.000104 | axisN_side_337deg/ring40mm_8of16 τ=2.35 | 1848 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 56 | 14 | 0.595439 | 0.000000 | axisN_side_337deg/ring40mm_7of16 τ=2.35 | 3640 |
