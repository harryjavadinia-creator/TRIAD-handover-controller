# T/G/R characterization — lateral-low.log, rest epoch search (generation 2)

- search epoch t=28.081 s; object estimate speed at epoch 0.0000 m/s; worker wall 1.425804s; hypotheses 14, memo reuses 13, static records 128, route records 204
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisN_side_337deg/direct τ=2.350 (timing-admissible plans 1062, cost-valid 1512)
- re-implemented argmin globalJ on admissible cost-valid records: axisN_side_337deg/direct τ=2.35 J=0.597838 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 12 | 2.35 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 0.0 |
| 0.10 | 0.5 | 8 | 8 | 6 | 2.80 | 0.621522 | 0.023684 | axisN_side_337deg/direct τ=2.80 | 0.0 |
| 0.20 | 0.5 | 3 | 3 | 2 | 3.70 | 0.668891 | 0.071053 | axisN_side_337deg/direct τ=3.70 | 0.0 |
| 0.45 | 3.25 | 12 | 12 | 11 | 2.35 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 0.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 0.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.645206 | 0.047368 | axisN_side_337deg/direct τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 64 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 1/64 angles; arcs (deg) samples 292.5..292.5 (1 samples, spacing 5.625)
- axisN: complete at any evaluated lead for 11/64 angles; arcs (deg) samples 45.0..84.4 (8 samples, spacing 5.625), samples 326.2..337.5 (3 samples, spacing 5.625)
- complete grasps per complete lead: median 12, min 12, max 12 (of 128)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 14 | 10 | 3.25 | 0.690792 | 0.092954 | axisN_side_45deg/ring140mm_6of8 | 224 |
| 16 | 32 | 14 | 12 | 2.35 | 0.597838 | 0.000000 | axisN_side_337deg/direct | 448 |
| 32 | 64 | 14 | 12 | 2.35 | 0.597838 | 0.000000 | axisN_side_337deg/direct | 896 |
| 64 | 128 | 14 | 12 | 2.35 | 0.597838 | 0.000000 | axisN_side_337deg/direct | 1792 |

- complete records by handle-axis sign: {'axisP': 168, 'axisN': 1344}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 168
  - direct sufficient: 56 (33.3%)
  - route alternative required: 112 (66.7%)
  - no complete route: 0 (0.0%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×112
- route wall time in this search 1.211 s over 204 route rollouts (mean 5.93 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 56 | 14 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 168 |
| direct + 4 dirs × [80] mm | 5 | 140 | 14 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 840 |
| direct + 8 dirs × [80] mm | 9 | 168 | 14 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 1512 |
| direct + 4 dirs × [140] mm | 5 | 168 | 14 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 840 |
| direct + 8 dirs × [140] mm | 9 | 168 | 14 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 1512 |
| direct + 4 dirs × [80, 140] mm | 9 | 168 | 14 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 1512 |
| direct + 8 dirs × [80, 140] mm | 17 | 168 | 14 | 0.597838 | 0.000000 | axisN_side_337deg/direct τ=2.35 | 2856 |
