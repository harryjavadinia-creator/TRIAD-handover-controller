# T/G/R characterization — near-ground.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 12.428453s; hypotheses 14, memo reuses 0, static records 1792, route records 1479
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_45deg/ring80mm_7of8 τ=3.250 (timing-admissible plans 267, cost-valid 738)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_45deg/ring80mm_7of8 τ=3.25 J=0.753454 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 13 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [8.00, 8.00]
- timing-admissible and complete: 9 leads, intervals [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 13 | 9 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 4.0 |
| 0.10 | 0.5 | 8 | 8 | 5 | 3.70 | 0.793113 | 0.039658 | axisP_side_45deg/ring140mm_7of8 τ=3.70 | 8.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.793113 | 0.039658 | axisP_side_45deg/ring140mm_7of8 τ=3.70 | 16.0 |
| 0.45 | 3.25 | 12 | 11 | 8 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 36.0 |
| 0.90 | 3.25 | 6 | 5 | 4 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 72.0 |
| 1.80 | 3.25 | 3 | 2 | 2 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 144.0 |
| V1 14 | V1 bank | 14 | 13 | 9 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 64 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 9/64 angles; arcs (deg) samples 39.4..67.5 (6 samples, spacing 5.625), samples 241.9..253.1 (3 samples, spacing 5.625)
- axisN: complete at any evaluated lead for 4/64 angles; arcs (deg) samples 292.5..309.4 (4 samples, spacing 5.625)
- complete grasps per complete lead: median 4, min 2, max 10 (of 128)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 9 | 5 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 | 224 |
| 16 | 32 | 11 | 7 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 | 448 |
| 32 | 64 | 13 | 9 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 | 896 |
| 64 | 128 | 13 | 9 | 3.25 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 | 1792 |

- complete records by handle-axis sign: {'axisP': 604, 'axisN': 134}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 87
  - direct sufficient: 53 (60.9%)
  - route alternative required: 11 (12.6%)
  - no complete route: 23 (26.4%)
- where a route was required, the direct route failed at: NONE `predictive_static/robust_transit_clearance_reserve` ×8; INSERTION `predictive_static` ×3
- route wall time in this search 8.828 s over 1479 route rollouts (mean 5.97 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 53 | 12 | 0.818410 | 0.064955 | axisP_side_45deg/direct τ=3.25 | 87 |
| direct + 4 dirs × [80] mm | 5 | 64 | 13 | 0.770343 | 0.016888 | axisP_side_45deg/ring80mm_0of8 τ=3.25 | 435 |
| direct + 8 dirs × [80] mm | 9 | 64 | 13 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 783 |
| direct + 4 dirs × [140] mm | 5 | 64 | 13 | 0.818363 | 0.064909 | axisP_side_45deg/ring140mm_0of8 τ=3.70 | 435 |
| direct + 8 dirs × [140] mm | 9 | 64 | 13 | 0.793113 | 0.039658 | axisP_side_45deg/ring140mm_7of8 τ=3.70 | 783 |
| direct + 4 dirs × [80, 140] mm | 9 | 64 | 13 | 0.770343 | 0.016888 | axisP_side_45deg/ring80mm_0of8 τ=3.25 | 783 |
| direct + 8 dirs × [80, 140] mm | 17 | 64 | 13 | 0.753454 | 0.000000 | axisP_side_45deg/ring80mm_7of8 τ=3.25 | 1479 |
