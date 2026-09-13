# T/G/R characterization — longitudinal.log, rest epoch search (generation 2)

- search epoch t=54.575 s; object estimate speed at epoch 0.0000 m/s; worker wall 0.662991s; hypotheses 191, memo reuses 190, static records 32, route records 102
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/direct τ=2.150 (timing-admissible plans 6220, cost-valid 8213)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/direct τ=2.15 J=0.615685 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 191 on [0.50, 10.00] s, finest step 0.050 s
- geometrically complete (any cost-valid complete action): 191 leads, intervals [0.50, 10.00]
- timing-admissible and complete: 158 leads, intervals [2.15, 10.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 191 | 191 | 158 | 2.15 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 0.0 |
| 0.10 | 0.5 | 96 | 96 | 79 | 2.20 | 0.618316 | 0.002632 | axisP_side_337deg/direct τ=2.20 | 0.0 |
| 0.20 | 0.5 | 48 | 48 | 39 | 2.30 | 0.623579 | 0.007895 | axisP_side_337deg/direct τ=2.30 | 0.0 |
| 0.45 | 3.25 | 22 | 22 | 18 | 2.35 | 0.626211 | 0.010526 | axisP_side_337deg/direct τ=2.35 | 0.0 |
| 0.90 | 3.25 | 11 | 11 | 9 | 2.35 | 0.626211 | 0.010526 | axisP_side_337deg/direct τ=2.35 | 0.0 |
| 1.80 | 3.25 | 5 | 5 | 4 | 3.25 | 0.643458 | 0.027773 | axisP_side_337deg/ring140mm_1of8 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.626211 | 0.010526 | axisP_side_337deg/direct τ=2.35 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 4/16 angles; arcs (deg) samples 45.0..67.5 (2 samples, spacing 22.500), samples 315.0..337.5 (2 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 0/16 angles; arcs (deg) none
- complete grasps per complete lead: median 4, min 4, max 4 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 1528 |
| 8 | 16 | 191 | 157 | 2.20 | 0.648841 | 0.033157 | axisP_side_315deg/direct | 3056 |
| 16 | 32 | 191 | 158 | 2.15 | 0.615685 | 0.000000 | axisP_side_337deg/direct | 6112 |

- complete records by handle-axis sign: {'axisP': 8213}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 1146
  - direct sufficient: 573 (50.0%)
  - route alternative required: 191 (16.7%)
  - no complete route: 382 (33.3%)
- where a route was required, the direct route failed at: NONE `predictive_static/runtime_clearance_reserve` ×191
- route wall time in this search 0.524 s over 102 route rollouts (mean 5.13 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 573 | 191 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 1146 |
| direct + 4 dirs × [80] mm | 5 | 764 | 191 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 5730 |
| direct + 8 dirs × [80] mm | 9 | 764 | 191 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 10314 |
| direct + 4 dirs × [140] mm | 5 | 764 | 191 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 5730 |
| direct + 8 dirs × [140] mm | 9 | 764 | 191 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 10314 |
| direct + 4 dirs × [80, 140] mm | 9 | 764 | 191 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 10314 |
| direct + 8 dirs × [80, 140] mm | 17 | 764 | 191 | 0.615685 | 0.000000 | axisP_side_337deg/direct τ=2.15 | 19482 |
