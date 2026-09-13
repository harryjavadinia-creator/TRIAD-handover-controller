# T/G/R characterization — near-ground.log, rest epoch search (generation 2)

- search epoch t=18.736 s; object estimate speed at epoch 0.0000 m/s; worker wall 0.494790s; hypotheses 14, memo reuses 13, static records 32, route records 65
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_45deg/ring80mm_15of16 τ=3.250 (timing-admissible plans 477, cost-valid 714)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_45deg/ring80mm_15of16 τ=3.25 J=0.739416 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 10 leads, intervals [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 10 | 3.25 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 0.0 |
| 0.10 | 0.5 | 8 | 8 | 5 | 3.70 | 0.761672 | 0.022256 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 0.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.761672 | 0.022256 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 0.0 |
| 0.45 | 3.25 | 12 | 12 | 9 | 3.25 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 0.0 |
| 0.90 | 3.25 | 6 | 6 | 5 | 3.25 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 0.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 10 | 3.25 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 1/16 angles; arcs (deg) samples 45.0..45.0 (1 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 0/16 angles; arcs (deg) none
- complete grasps per complete lead: median 1, min 1, max 1 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 14 | 10 | 3.25 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 | 224 |
| 16 | 32 | 14 | 10 | 3.25 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 | 448 |

- complete records by handle-axis sign: {'axisP': 714}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 14
  - direct sufficient: 14 (100.0%)
  - route alternative required: 0 (0.0%)
  - no complete route: 0 (0.0%)
- route wall time in this search 0.415 s over 65 route rollouts (mean 6.39 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 14 | 14 | 0.808226 | 0.068810 | axisP_side_45deg/direct τ=3.25 | 14 |
| direct + 4 dirs × [40] mm | 5 | 14 | 14 | 0.748546 | 0.009130 | axisP_side_45deg/ring40mm_0of16 τ=3.25 | 70 |
| direct + 8 dirs × [40] mm | 9 | 14 | 14 | 0.744618 | 0.005201 | axisP_side_45deg/ring40mm_2of16 τ=3.25 | 126 |
| direct + 16 dirs × [40] mm | 17 | 14 | 14 | 0.743692 | 0.004276 | axisP_side_45deg/ring40mm_1of16 τ=3.25 | 238 |
| direct + 4 dirs × [80] mm | 5 | 14 | 14 | 0.748412 | 0.008996 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 70 |
| direct + 8 dirs × [80] mm | 9 | 14 | 14 | 0.748412 | 0.008996 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 126 |
| direct + 16 dirs × [80] mm | 17 | 14 | 14 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 238 |
| direct + 4 dirs × [140] mm | 5 | 14 | 14 | 0.798472 | 0.059056 | axisP_side_45deg/ring140mm_0of16 τ=3.70 | 70 |
| direct + 8 dirs × [140] mm | 9 | 14 | 14 | 0.761672 | 0.022256 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 126 |
| direct + 16 dirs × [140] mm | 17 | 14 | 14 | 0.761672 | 0.022256 | axisP_side_45deg/ring140mm_14of16 τ=3.70 | 238 |
| direct + 4 dirs × [200] mm | 5 | 14 | 14 | 0.808226 | 0.068810 | axisP_side_45deg/direct τ=3.25 | 70 |
| direct + 8 dirs × [200] mm | 9 | 14 | 14 | 0.801620 | 0.062204 | axisP_side_45deg/ring200mm_14of16 τ=4.15 | 126 |
| direct + 16 dirs × [200] mm | 17 | 14 | 14 | 0.801620 | 0.062204 | axisP_side_45deg/ring200mm_14of16 τ=4.15 | 238 |
| direct + 4 dirs × [80, 140] mm | 9 | 14 | 14 | 0.748412 | 0.008996 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 126 |
| direct + 8 dirs × [80, 140] mm | 17 | 14 | 14 | 0.748412 | 0.008996 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 238 |
| direct + 16 dirs × [80, 140] mm | 33 | 14 | 14 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 462 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 14 | 14 | 0.748412 | 0.008996 | axisP_side_45deg/ring80mm_0of16 τ=3.25 | 238 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 14 | 14 | 0.744618 | 0.005201 | axisP_side_45deg/ring40mm_2of16 τ=3.25 | 462 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 14 | 14 | 0.739416 | 0.000000 | axisP_side_45deg/ring80mm_15of16 τ=3.25 | 910 |
