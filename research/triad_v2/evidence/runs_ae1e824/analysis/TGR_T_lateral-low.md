# T/G/R characterization — lateral-low.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 78.437108s; hypotheses 191, memo reuses 0, static records 6112, route records 11594
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisN_side_337deg/direct τ=3.450 (timing-admissible plans 5131, cost-valid 6976)
- re-implemented argmin globalJ on admissible cost-valid records: axisN_side_337deg/direct τ=3.45 J=0.669654 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 191 on [0.50, 10.00] s, finest step 0.050 s
- geometrically complete (any cost-valid complete action): 191 leads, intervals [0.50, 10.00]
- timing-admissible and complete: 139 leads, intervals [3.10, 10.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 191 | 191 | 139 | 3.10 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 4.0 |
| 0.10 | 0.5 | 96 | 96 | 70 | 3.10 | 0.672435 | 0.002781 | axisN_side_337deg/direct τ=3.50 | 8.0 |
| 0.20 | 0.5 | 48 | 48 | 35 | 3.10 | 0.672435 | 0.002781 | axisN_side_337deg/direct τ=3.50 | 16.0 |
| 0.45 | 3.25 | 22 | 22 | 16 | 3.25 | 0.677720 | 0.008066 | axisN_side_337deg/direct τ=3.70 | 36.0 |
| 0.90 | 3.25 | 11 | 11 | 8 | 3.25 | 0.692601 | 0.022946 | axisN_side_337deg/direct τ=4.15 | 72.0 |
| 1.80 | 3.25 | 5 | 5 | 4 | 3.25 | 0.741496 | 0.071842 | axisN_side_337deg/ring140mm_3of8 τ=5.05 | 144.0 |
| V1 14 | V1 bank | 14 | 14 | 10 | 3.25 | 0.677720 | 0.008066 | axisN_side_337deg/direct τ=3.70 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 6/16 angles; arcs (deg) samples -90.0..22.5 (6 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 8/16 angles; arcs (deg) samples -45.0..112.5 (8 samples, spacing 22.500)
- complete grasps per complete lead: median 4, min 1, max 7 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 110 | 110 | 4.30 | 0.858271 | 0.188617 | axisN_side_90deg/ring140mm_7of8 | 1528 |
| 8 | 16 | 151 | 129 | 3.60 | 0.714950 | 0.045296 | axisN_side_45deg/ring140mm_6of8 | 3056 |
| 16 | 32 | 191 | 139 | 3.10 | 0.669654 | 0.000000 | axisN_side_337deg/direct | 6112 |

- complete records by handle-axis sign: {'axisP': 1720, 'axisN': 5256}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 682
  - direct sufficient: 209 (30.6%)
  - route alternative required: 462 (67.7%)
  - no complete route: 11 (1.6%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×402; NONE `predictive_static/robust_transit_clearance_reserve` ×35; NONE `predictive_static/runtime_clearance_reserve` ×19; NONE `predictive_static` ×5; INSERTION `predictive_static` ×1
- route wall time in this search 70.351 s over 11594 route rollouts (mean 6.07 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 209 | 120 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 682 |
| direct + 4 dirs × [80] mm | 5 | 573 | 161 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 3410 |
| direct + 8 dirs × [80] mm | 9 | 641 | 180 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 6138 |
| direct + 4 dirs × [140] mm | 5 | 666 | 191 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 3410 |
| direct + 8 dirs × [140] mm | 9 | 671 | 191 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 6138 |
| direct + 4 dirs × [80, 140] mm | 9 | 666 | 191 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 6138 |
| direct + 8 dirs × [80, 140] mm | 17 | 671 | 191 | 0.669654 | 0.000000 | axisN_side_337deg/direct τ=3.45 | 11594 |
