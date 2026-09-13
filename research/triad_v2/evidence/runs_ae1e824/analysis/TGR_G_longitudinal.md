# T/G/R characterization — longitudinal.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 16.432468s; hypotheses 14, memo reuses 0, static records 1792, route records 2601
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_343deg/ring140mm_1of8 τ=3.250 (timing-admissible plans 962, cost-valid 1007)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_343deg/ring140mm_1of8 τ=3.25 J=0.615856 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 12 leads, intervals [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 12 | 12 | 2.35 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 4.0 |
| 0.10 | 0.5 | 8 | 6 | 6 | 2.80 | 0.650341 | 0.034485 | axisP_side_343deg/ring140mm_1of8 τ=3.70 | 8.0 |
| 0.20 | 0.5 | 3 | 2 | 2 | 3.70 | 0.650341 | 0.034485 | axisP_side_343deg/ring140mm_1of8 τ=3.70 | 16.0 |
| 0.45 | 3.25 | 12 | 11 | 11 | 2.35 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 36.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 72.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 144.0 |
| V1 14 | V1 bank | 14 | 12 | 12 | 2.35 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 64 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 23/64 angles; arcs (deg) samples -61.9..0.0 (12 samples, spacing 5.625), samples 33.8..90.0 (11 samples, spacing 5.625)
- axisN: complete at any evaluated lead for 7/64 angles; arcs (deg) samples 247.5..247.5 (1 samples, spacing 5.625), samples 258.8..270.0 (3 samples, spacing 5.625), samples 281.2..292.5 (3 samples, spacing 5.625)
- complete grasps per complete lead: median 6, min 1, max 18 (of 128)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 3 | 2 | 4.60 | 0.746522 | 0.130666 | axisP_side_0deg/ring140mm_2of8 | 112 |
| 8 | 16 | 5 | 4 | 3.70 | 0.739612 | 0.123756 | axisP_side_315deg/ring140mm_1of8 | 224 |
| 16 | 32 | 9 | 8 | 2.35 | 0.621368 | 0.005512 | axisP_side_337deg/direct | 448 |
| 32 | 64 | 11 | 11 | 2.35 | 0.621368 | 0.005512 | axisP_side_337deg/direct | 896 |
| 64 | 128 | 12 | 12 | 2.35 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 | 1792 |

- complete records by handle-axis sign: {'axisP': 988, 'axisN': 19}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 153
  - direct sufficient: 62 (40.5%)
  - route alternative required: 30 (19.6%)
  - no complete route: 61 (39.9%)
- where a route was required, the direct route failed at: NONE `predictive_static/runtime_clearance_reserve` ×15; NONE `predictive_static/reach_tracking` ×13; NONE `predictive_static/robust_transit_clearance_reserve` ×1; INSERTION `predictive_static` ×1
- route wall time in this search 12.689 s over 2601 route rollouts (mean 4.88 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 62 | 12 | 0.621368 | 0.005512 | axisP_side_337deg/direct τ=2.35 | 153 |
| direct + 4 dirs × [80] mm | 5 | 79 | 12 | 0.621368 | 0.005512 | axisP_side_337deg/direct τ=2.35 | 765 |
| direct + 8 dirs × [80] mm | 9 | 79 | 12 | 0.621368 | 0.005512 | axisP_side_337deg/direct τ=2.35 | 1377 |
| direct + 4 dirs × [140] mm | 5 | 89 | 12 | 0.616621 | 0.000764 | axisP_side_343deg/ring140mm_2of8 τ=3.25 | 765 |
| direct + 8 dirs × [140] mm | 9 | 91 | 12 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 1377 |
| direct + 4 dirs × [80, 140] mm | 9 | 90 | 12 | 0.616621 | 0.000764 | axisP_side_343deg/ring140mm_2of8 τ=3.25 | 1377 |
| direct + 8 dirs × [80, 140] mm | 17 | 92 | 12 | 0.615856 | 0.000000 | axisP_side_343deg/ring140mm_1of8 τ=3.25 | 2601 |
