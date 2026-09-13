# T/G/R characterization — diagonal.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 8.522715s; hypotheses 14, memo reuses 0, static records 448, route records 1560
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring140mm_4of16 τ=4.150 (timing-admissible plans 976, cost-valid 976)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring140mm_4of16 τ=4.15 J=0.681985 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 8 leads, intervals [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 8 leads, intervals [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 8 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 4.0 |
| 0.10 | 0.5 | 8 | 4 | 4 | 4.60 | 0.704844 | 0.022859 | axisP_side_337deg/ring140mm_4of16 τ=4.60 | 8.0 |
| 0.20 | 0.5 | 3 | 1 | 1 | 5.50 | 0.768808 | 0.086823 | axisP_side_337deg/ring140mm_4of16 τ=5.50 | 16.0 |
| 0.45 | 3.25 | 12 | 7 | 7 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 36.0 |
| 0.90 | 3.25 | 6 | 4 | 4 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 72.0 |
| 1.80 | 3.25 | 3 | 2 | 2 | 5.05 | 0.734594 | 0.052609 | axisP_side_337deg/ring140mm_4of16 τ=5.05 | 144.0 |
| V1 14 | V1 bank | 14 | 8 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 5/16 angles; arcs (deg) samples -45.0..45.0 (5 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 1/16 angles; arcs (deg) samples 315.0..315.0 (1 samples, spacing 22.500)
- complete grasps per complete lead: median 3, min 1, max 5 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 1 | 1 | 5.95 | 0.847008 | 0.165023 | axisP_side_0deg/ring80mm_5of16 | 112 |
| 8 | 16 | 7 | 7 | 4.60 | 0.765422 | 0.083437 | axisP_side_315deg/ring140mm_4of16 | 224 |
| 16 | 32 | 8 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 | 448 |

- complete records by handle-axis sign: {'axisP': 964, 'axisN': 12}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 24
  - direct sufficient: 16 (66.7%)
  - route alternative required: 8 (33.3%)
  - no complete route: 0 (0.0%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×6; NONE `predictive_static/runtime_clearance_reserve` ×2
- route wall time in this search 7.741 s over 1560 route rollouts (mean 4.96 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 16 | 8 | 0.730966 | 0.048981 | axisP_side_337deg/direct τ=4.15 | 24 |
| direct + 4 dirs × [40] mm | 5 | 21 | 8 | 0.715654 | 0.033669 | axisP_side_337deg/ring40mm_4of16 τ=4.15 | 120 |
| direct + 8 dirs × [40] mm | 9 | 21 | 8 | 0.715654 | 0.033669 | axisP_side_337deg/ring40mm_4of16 τ=4.15 | 216 |
| direct + 16 dirs × [40] mm | 17 | 21 | 8 | 0.715538 | 0.033553 | axisP_side_337deg/ring40mm_5of16 τ=4.15 | 408 |
| direct + 4 dirs × [80] mm | 5 | 20 | 8 | 0.691373 | 0.009388 | axisP_side_337deg/ring80mm_4of16 τ=4.15 | 120 |
| direct + 8 dirs × [80] mm | 9 | 21 | 8 | 0.691373 | 0.009388 | axisP_side_337deg/ring80mm_4of16 τ=4.15 | 216 |
| direct + 16 dirs × [80] mm | 17 | 22 | 8 | 0.691373 | 0.009388 | axisP_side_337deg/ring80mm_4of16 τ=4.15 | 408 |
| direct + 4 dirs × [140] mm | 5 | 20 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 120 |
| direct + 8 dirs × [140] mm | 9 | 21 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 216 |
| direct + 16 dirs × [140] mm | 17 | 23 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 408 |
| direct + 4 dirs × [200] mm | 5 | 19 | 8 | 0.727740 | 0.045754 | axisP_side_337deg/ring200mm_4of16 τ=4.60 | 120 |
| direct + 8 dirs × [200] mm | 9 | 23 | 8 | 0.727740 | 0.045754 | axisP_side_337deg/ring200mm_4of16 τ=4.60 | 216 |
| direct + 16 dirs × [200] mm | 17 | 23 | 8 | 0.727740 | 0.045754 | axisP_side_337deg/ring200mm_4of16 τ=4.60 | 408 |
| direct + 4 dirs × [80, 140] mm | 9 | 21 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 216 |
| direct + 8 dirs × [80, 140] mm | 17 | 22 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 408 |
| direct + 16 dirs × [80, 140] mm | 33 | 23 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 792 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 21 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 408 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 24 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 792 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 24 | 8 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_4of16 τ=4.15 | 1560 |
