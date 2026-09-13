# T/G/R characterization — diagonal.log, rest epoch search (generation 2)

- search epoch t=17.547 s; object estimate speed at epoch 0.0000 m/s; worker wall 0.327905s; hypotheses 14, memo reuses 13, static records 32, route records 65
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring80mm_3of16 τ=2.800 (timing-admissible plans 520, cost-valid 658)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring80mm_3of16 τ=2.80 J=0.600182 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 12 | 2.35 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 τ=2.80 | 0.0 |
| 0.10 | 0.5 | 8 | 8 | 6 | 2.80 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 τ=2.80 | 0.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.635510 | 0.035328 | axisP_side_337deg/ring140mm_3of16 τ=3.70 | 0.0 |
| 0.45 | 3.25 | 12 | 12 | 11 | 2.35 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 τ=2.80 | 0.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.603148 | 0.002966 | axisP_side_337deg/ring40mm_1of16 τ=2.35 | 0.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.611826 | 0.011644 | axisP_side_337deg/ring140mm_3of16 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 τ=2.80 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 1/16 angles; arcs (deg) samples 337.5..337.5 (1 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 0/16 angles; arcs (deg) none
- complete grasps per complete lead: median 1, min 1, max 1 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 0 | 0 | n/a | n/a | n/a | none | 224 |
| 16 | 32 | 14 | 12 | 2.35 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 | 448 |

- complete records by handle-axis sign: {'axisP': 658}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 14
  - direct sufficient: 14 (100.0%)
  - route alternative required: 0 (0.0%)
  - no complete route: 0 (0.0%)
- route wall time in this search 0.268 s over 65 route rollouts (mean 4.13 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 14 | 14 | 0.607315 | 0.007133 | axisP_side_337deg/direct τ=2.35 | 14 |
| direct + 4 dirs × [40] mm | 5 | 14 | 14 | 0.604464 | 0.004282 | axisP_side_337deg/ring40mm_0of16 τ=2.35 | 70 |
| direct + 8 dirs × [40] mm | 9 | 14 | 14 | 0.603170 | 0.002988 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 126 |
| direct + 16 dirs × [40] mm | 17 | 14 | 14 | 0.603148 | 0.002966 | axisP_side_337deg/ring40mm_1of16 τ=2.35 | 238 |
| direct + 4 dirs × [80] mm | 5 | 14 | 14 | 0.603710 | 0.003528 | axisP_side_337deg/ring80mm_4of16 τ=2.80 | 70 |
| direct + 8 dirs × [80] mm | 9 | 14 | 14 | 0.603538 | 0.003356 | axisP_side_337deg/ring80mm_2of16 τ=2.80 | 126 |
| direct + 16 dirs × [80] mm | 17 | 14 | 14 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 τ=2.80 | 238 |
| direct + 4 dirs × [140] mm | 5 | 14 | 14 | 0.607315 | 0.007133 | axisP_side_337deg/direct τ=2.35 | 70 |
| direct + 8 dirs × [140] mm | 9 | 14 | 14 | 0.607315 | 0.007133 | axisP_side_337deg/direct τ=2.35 | 126 |
| direct + 16 dirs × [140] mm | 17 | 14 | 14 | 0.607315 | 0.007133 | axisP_side_337deg/direct τ=2.35 | 238 |
| direct + 4 dirs × [200] mm | 5 | 14 | 14 | 0.607315 | 0.007133 | axisP_side_337deg/direct τ=2.35 | 70 |
| direct + 8 dirs × [200] mm | 9 | 14 | 14 | 0.607315 | 0.007133 | axisP_side_337deg/direct τ=2.35 | 126 |
| direct + 16 dirs × [200] mm | 17 | 14 | 14 | 0.607315 | 0.007133 | axisP_side_337deg/direct τ=2.35 | 238 |
| direct + 4 dirs × [80, 140] mm | 9 | 14 | 14 | 0.603710 | 0.003528 | axisP_side_337deg/ring80mm_4of16 τ=2.80 | 126 |
| direct + 8 dirs × [80, 140] mm | 17 | 14 | 14 | 0.603538 | 0.003356 | axisP_side_337deg/ring80mm_2of16 τ=2.80 | 238 |
| direct + 16 dirs × [80, 140] mm | 33 | 14 | 14 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 τ=2.80 | 462 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 14 | 14 | 0.603710 | 0.003528 | axisP_side_337deg/ring80mm_4of16 τ=2.80 | 238 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 14 | 14 | 0.603170 | 0.002988 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 462 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 14 | 14 | 0.600182 | 0.000000 | axisP_side_337deg/ring80mm_3of16 τ=2.80 | 910 |
