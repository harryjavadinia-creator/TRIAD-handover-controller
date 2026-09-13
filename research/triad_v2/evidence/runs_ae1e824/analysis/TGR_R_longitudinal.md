# T/G/R characterization — longitudinal.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 12.028477s; hypotheses 14, memo reuses 0, static records 448, route records 2275
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring40mm_3of16 τ=2.350 (timing-admissible plans 787, cost-valid 868)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring40mm_3of16 τ=2.35 J=0.615062 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 9 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95]
- timing-admissible and complete: 8 leads, intervals [2.35, 2.35], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 9 | 8 | 2.35 | 0.615062 | 0.000000 | axisP_side_337deg/ring40mm_3of16 τ=2.35 | 4.0 |
| 0.10 | 0.5 | 8 | 4 | 3 | 3.70 | 0.656309 | 0.041247 | axisP_side_337deg/ring140mm_3of16 τ=3.70 | 8.0 |
| 0.20 | 0.5 | 3 | 2 | 2 | 3.70 | 0.656309 | 0.041247 | axisP_side_337deg/ring140mm_3of16 τ=3.70 | 16.0 |
| 0.45 | 3.25 | 12 | 9 | 8 | 2.35 | 0.615062 | 0.000000 | axisP_side_337deg/ring40mm_3of16 τ=2.35 | 36.0 |
| 0.90 | 3.25 | 6 | 5 | 5 | 2.35 | 0.615062 | 0.000000 | axisP_side_337deg/ring40mm_3of16 τ=2.35 | 72.0 |
| 1.80 | 3.25 | 3 | 2 | 2 | 3.25 | 0.624182 | 0.009120 | axisP_side_337deg/ring140mm_3of16 τ=3.25 | 144.0 |
| V1 14 | V1 bank | 14 | 9 | 8 | 2.35 | 0.615062 | 0.000000 | axisP_side_337deg/ring40mm_3of16 τ=2.35 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 6/16 angles; arcs (deg) samples -45.0..0.0 (3 samples, spacing 22.500), samples 45.0..90.0 (3 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 3/16 angles; arcs (deg) samples 247.5..292.5 (3 samples, spacing 22.500)
- complete grasps per complete lead: median 3, min 1, max 6 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 6 | 3 | 4.15 | 0.746522 | 0.131460 | axisP_side_0deg/ring140mm_4of16 | 112 |
| 8 | 16 | 6 | 4 | 3.70 | 0.736585 | 0.121523 | axisP_side_315deg/ring140mm_1of16 | 224 |
| 16 | 32 | 9 | 8 | 2.35 | 0.615062 | 0.000000 | axisP_side_337deg/ring40mm_3of16 | 448 |

- complete records by handle-axis sign: {'axisP': 827, 'axisN': 41}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 35
  - direct sufficient: 15 (42.9%)
  - route alternative required: 13 (37.1%)
  - no complete route: 7 (20.0%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×9; NONE `predictive_static/robust_transit_clearance_reserve` ×2; NONE `predictive_static/runtime_clearance_reserve` ×2
- route wall time in this search 11.085 s over 2275 route rollouts (mean 4.87 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 15 | 8 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 35 |
| direct + 4 dirs × [40] mm | 5 | 16 | 8 | 0.615275 | 0.000213 | axisP_side_337deg/ring40mm_4of16 τ=2.35 | 175 |
| direct + 8 dirs × [40] mm | 9 | 16 | 8 | 0.615181 | 0.000119 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 315 |
| direct + 16 dirs × [40] mm | 17 | 16 | 8 | 0.615062 | 0.000000 | axisP_side_337deg/ring40mm_3of16 τ=2.35 | 595 |
| direct + 4 dirs × [80] mm | 5 | 17 | 8 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 175 |
| direct + 8 dirs × [80] mm | 9 | 17 | 8 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 315 |
| direct + 16 dirs × [80] mm | 17 | 17 | 8 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 595 |
| direct + 4 dirs × [140] mm | 5 | 19 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 175 |
| direct + 8 dirs × [140] mm | 9 | 20 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 315 |
| direct + 16 dirs × [140] mm | 17 | 20 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 595 |
| direct + 4 dirs × [200] mm | 5 | 26 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 175 |
| direct + 8 dirs × [200] mm | 9 | 27 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 315 |
| direct + 16 dirs × [200] mm | 17 | 28 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 595 |
| direct + 4 dirs × [80, 140] mm | 9 | 19 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 315 |
| direct + 8 dirs × [80, 140] mm | 17 | 20 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 595 |
| direct + 16 dirs × [80, 140] mm | 33 | 20 | 9 | 0.621368 | 0.006306 | axisP_side_337deg/direct τ=2.35 | 1155 |
| direct + 4 dirs × [40, 80, 140, 200] mm | 17 | 26 | 9 | 0.615275 | 0.000213 | axisP_side_337deg/ring40mm_4of16 τ=2.35 | 595 |
| direct + 8 dirs × [40, 80, 140, 200] mm | 33 | 27 | 9 | 0.615181 | 0.000119 | axisP_side_337deg/ring40mm_2of16 τ=2.35 | 1155 |
| direct + 16 dirs × [40, 80, 140, 200] mm | 65 | 28 | 9 | 0.615062 | 0.000000 | axisP_side_337deg/ring40mm_3of16 τ=2.35 | 2275 |
