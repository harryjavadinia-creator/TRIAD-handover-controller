# T/G/R characterization — longitudinal.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 45.550110s; hypotheses 191, memo reuses 0, static records 6112, route records 5933
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring80mm_1of8 τ=2.850 (timing-admissible plans 2011, cost-valid 2317)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring80mm_1of8 τ=2.85 J=0.611263 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 191 on [0.50, 10.00] s, finest step 0.050 s
- geometrically complete (any cost-valid complete action): 92 leads, intervals [0.80, 1.30], [2.10, 2.35], [2.60, 6.00], [7.20, 7.45]
- timing-admissible and complete: 76 leads, intervals [2.10, 2.35], [2.85, 6.00], [7.20, 7.45]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 191 | 92 | 76 | 2.10 | 0.611263 | 0.000000 | axisP_side_337deg/ring80mm_1of8 τ=2.85 | 4.0 |
| 0.10 | 0.5 | 96 | 47 | 38 | 2.10 | 0.613869 | 0.002607 | axisP_side_337deg/ring140mm_2of8 τ=3.10 | 8.0 |
| 0.20 | 0.5 | 48 | 23 | 19 | 2.10 | 0.613869 | 0.002607 | axisP_side_337deg/ring140mm_2of8 τ=3.10 | 16.0 |
| 0.45 | 3.25 | 22 | 11 | 9 | 2.35 | 0.621368 | 0.010106 | axisP_side_337deg/direct τ=2.35 | 36.0 |
| 0.90 | 3.25 | 11 | 5 | 5 | 2.35 | 0.621368 | 0.010106 | axisP_side_337deg/direct τ=2.35 | 72.0 |
| 1.80 | 3.25 | 5 | 2 | 2 | 3.25 | 0.625383 | 0.014120 | axisP_side_337deg/ring140mm_2of8 τ=3.25 | 144.0 |
| V1 14 | V1 bank | 14 | 9 | 8 | 2.35 | 0.621368 | 0.010106 | axisP_side_337deg/direct τ=2.35 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 7/16 angles; arcs (deg) samples -45.0..90.0 (7 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 3/16 angles; arcs (deg) samples 247.5..292.5 (3 samples, spacing 22.500)
- complete grasps per complete lead: median 2, min 1, max 4 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 29 | 18 | 4.20 | 0.713861 | 0.102598 | axisP_side_0deg/ring140mm_2of8 | 1528 |
| 8 | 16 | 63 | 43 | 3.65 | 0.713861 | 0.102598 | axisP_side_0deg/ring140mm_2of8 | 3056 |
| 16 | 32 | 92 | 76 | 2.10 | 0.611263 | 0.000000 | axisP_side_337deg/ring80mm_1of8 | 6112 |

- complete records by handle-axis sign: {'axisP': 2281, 'axisN': 36}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 349
  - direct sufficient: 159 (45.6%)
  - route alternative required: 36 (10.3%)
  - no complete route: 154 (44.1%)
- where a route was required, the direct route failed at: NONE `predictive_static/runtime_clearance_reserve` ×16; NONE `predictive_static/reach_tracking` ×16; NONE `predictive_static/robust_transit_clearance_reserve` ×3; INSERTION `predictive_static` ×1
- route wall time in this search 31.041 s over 5933 route rollouts (mean 5.23 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 159 | 86 | 0.621368 | 0.010106 | axisP_side_337deg/direct τ=2.35 | 349 |
| direct + 4 dirs × [80] mm | 5 | 174 | 87 | 0.612310 | 0.001048 | axisP_side_337deg/ring80mm_2of8 τ=2.85 | 1745 |
| direct + 8 dirs × [80] mm | 9 | 174 | 87 | 0.611263 | 0.000000 | axisP_side_337deg/ring80mm_1of8 τ=2.85 | 3141 |
| direct + 4 dirs × [140] mm | 5 | 191 | 92 | 0.613869 | 0.002607 | axisP_side_337deg/ring140mm_2of8 τ=3.10 | 1745 |
| direct + 8 dirs × [140] mm | 9 | 195 | 92 | 0.613869 | 0.002607 | axisP_side_337deg/ring140mm_2of8 τ=3.10 | 3141 |
| direct + 4 dirs × [80, 140] mm | 9 | 191 | 92 | 0.612310 | 0.001048 | axisP_side_337deg/ring80mm_2of8 τ=2.85 | 3141 |
| direct + 8 dirs × [80, 140] mm | 17 | 195 | 92 | 0.611263 | 0.000000 | axisP_side_337deg/ring80mm_1of8 τ=2.85 | 5933 |
