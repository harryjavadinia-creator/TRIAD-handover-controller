# T/G/R characterization — longitudinal.log, rest epoch search (generation 2)

- search epoch t=25.459 s; object estimate speed at epoch 0.0000 m/s; worker wall 2.284394s; hypotheses 14, memo reuses 13, static records 128, route records 408
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_354deg/direct τ=2.350 (timing-admissible plans 1737, cost-valid 2366)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_354deg/direct τ=2.35 J=0.623574 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 12 | 2.35 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 0.0 |
| 0.10 | 0.5 | 8 | 8 | 6 | 2.80 | 0.633717 | 0.010143 | axisP_side_354deg/ring80mm_1of8 τ=2.80 | 0.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.663841 | 0.040267 | axisP_side_343deg/ring140mm_1of8 τ=3.70 | 0.0 |
| 0.45 | 3.25 | 12 | 12 | 11 | 2.35 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 0.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 0.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.640156 | 0.016583 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 64 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 15/64 angles; arcs (deg) samples 33.8..67.5 (7 samples, spacing 5.625), samples 315.0..354.4 (8 samples, spacing 5.625)
- axisN: complete at any evaluated lead for 1/64 angles; arcs (deg) samples 241.9..241.9 (1 samples, spacing 5.625)
- complete grasps per complete lead: median 16, min 16, max 16 (of 128)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 14 | 12 | 2.35 | 0.656736 | 0.033162 | axisP_side_315deg/direct | 224 |
| 16 | 32 | 14 | 12 | 2.35 | 0.626211 | 0.002637 | axisP_side_337deg/direct | 448 |
| 32 | 64 | 14 | 12 | 2.35 | 0.624471 | 0.000897 | axisP_side_349deg/direct | 896 |
| 64 | 128 | 14 | 12 | 2.35 | 0.623574 | 0.000000 | axisP_side_354deg/direct | 1792 |

- complete records by handle-axis sign: {'axisP': 2352, 'axisN': 14}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 336
  - direct sufficient: 126 (37.5%)
  - route alternative required: 98 (29.2%)
  - no complete route: 112 (33.3%)
- where a route was required, the direct route failed at: NONE `predictive_static/runtime_clearance_reserve` ×84; NONE `predictive_static/robust_transit_clearance_reserve` ×14
- route wall time in this search 1.995 s over 408 route rollouts (mean 4.89 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 126 | 14 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 336 |
| direct + 4 dirs × [80] mm | 5 | 210 | 14 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 1680 |
| direct + 8 dirs × [80] mm | 9 | 210 | 14 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 3024 |
| direct + 4 dirs × [140] mm | 5 | 224 | 14 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 1680 |
| direct + 8 dirs × [140] mm | 9 | 224 | 14 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 3024 |
| direct + 4 dirs × [80, 140] mm | 9 | 224 | 14 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 3024 |
| direct + 8 dirs × [80, 140] mm | 17 | 224 | 14 | 0.623574 | 0.000000 | axisP_side_354deg/direct τ=2.35 | 5712 |
