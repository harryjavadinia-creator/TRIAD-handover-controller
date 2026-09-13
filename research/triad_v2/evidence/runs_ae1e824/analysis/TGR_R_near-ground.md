# T/G/R characterization — near-ground.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 9.710581s; hypotheses 14, memo reuses 0, static records 448, route records 1560
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_45deg/ring80mm_14of16 τ=3.250 (timing-admissible plans 290, cost-valid 901)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_45deg/ring80mm_14of16 τ=3.25 J=0.753454 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 11 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [6.40, 6.40], [8.00, 8.00]
- timing-admissible and complete: 7 leads, intervals [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [6.40, 6.40], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 11 | 7 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 4.0 |
| 0.10 | 0.5 | 8 | 7 | 4 | 3.70 | 0.793113 | 0.039658 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 8.0 |
| 0.20 | 0.5 | 3 | 2 | 1 | 3.70 | 0.793113 | 0.039658 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 16.0 |
| 0.45 | 3.25 | 12 | 9 | 6 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 36.0 |
| 0.90 | 3.25 | 6 | 4 | 3 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 72.0 |
| 1.80 | 3.25 | 3 | 2 | 2 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 144.0 |
| V1 14 | V1 bank | 14 | 11 | 7 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 3/16 angles; arcs (deg) samples 45.0..67.5 (2 samples, spacing 22.500), samples 247.5..247.5 (1 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 2/16 angles; arcs (deg) samples 112.5..112.5 (1 samples, spacing 22.500), samples 292.5..292.5 (1 samples, spacing 22.500)
- complete grasps per complete lead: median 2, min 1, max 3 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 9 | 5 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 | 224 |
| 16 | 32 | 11 | 7 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 | 448 |

- complete records by handle-axis sign: {'axisP': 756, 'axisN': 145}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 24
  - direct sufficient: 19 (79.2%)
  - route alternative required: 2 (8.3%)
  - no complete route: 3 (12.5%)
- where a route was required, the direct route failed at: INSERTION `predictive_static` ×1; NONE `predictive_static/reach_tracking` ×1
- route wall time in this search 8.852 s over 1560 route rollouts (mean 5.67 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 19 | 11 | 0.818410 | 0.064955 | axisP_side_45deg/direct τ=3.25 | 24 |
| direct + 4 dirs × [40] mm | 5 | 20 | 11 | 0.773665 | 0.020211 | axisP_side_45deg/ring40mm_0of16 τ=3.25 | 120 |
| direct + 8 dirs × [40] mm | 9 | 20 | 11 | 0.771663 | 0.018208 | axisP_side_45deg/ring40mm_14of16 τ=3.25 | 216 |
| direct + 16 dirs × [40] mm | 17 | 20 | 11 | 0.770135 | 0.016680 | axisP_side_45deg/ring40mm_15of16 τ=3.25 | 408 |
| direct + 4 dirs × [80] mm | 5 | 20 | 11 | 0.770343 | 0.016888 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 120 |
| direct + 8 dirs × [80] mm | 9 | 20 | 11 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 216 |
| direct + 16 dirs × [80] mm | 17 | 20 | 11 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 408 |
| direct + 4 dirs × [140] mm | 5 | 20 | 11 | 0.818363 | 0.064909 | axisP_side_45deg/ring140mm_0of16 τ=3.70 | 120 |
| direct + 8 dirs × [140] mm | 9 | 20 | 11 | 0.793113 | 0.039658 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 216 |
| direct + 16 dirs × [140] mm | 17 | 20 | 11 | 0.793113 | 0.039658 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 408 |
| direct + 4 dirs × [200] mm | 5 | 20 | 11 | 0.818410 | 0.064955 | axisP_side_45deg/direct τ=3.25 | 120 |
| direct + 8 dirs × [200] mm | 9 | 20 | 11 | 0.818410 | 0.064955 | axisP_side_45deg/direct τ=3.25 | 216 |
| direct + 16 dirs × [200] mm | 17 | 21 | 11 | 0.818410 | 0.064955 | axisP_side_45deg/direct τ=3.25 | 408 |
| direct + 4 dirs × [80, 140] mm | 9 | 20 | 11 | 0.770343 | 0.016888 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 216 |
| direct + 8 dirs × [80, 140] mm | 17 | 20 | 11 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 408 |
| direct + 16 dirs × [80, 140] mm | 33 | 20 | 11 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 792 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 20 | 11 | 0.770343 | 0.016888 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 408 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 20 | 11 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 792 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 21 | 11 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_14of16 τ=3.25 | 1560 |
