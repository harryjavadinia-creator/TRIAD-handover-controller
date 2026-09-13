# T/G/R characterization — diagonal.log, rest epoch search (generation 2)

- search epoch t=20.296 s; object estimate speed at epoch 0.0000 m/s; worker wall 0.658754s; hypotheses 14, memo reuses 13, static records 128, route records 68
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_332deg/ring80mm_2of8 τ=2.350 (timing-admissible plans 717, cost-valid 924)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_332deg/ring80mm_2of8 τ=2.35 J=0.590726 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 14 leads, intervals [1.80, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 13 leads, intervals [1.90, 1.90], [2.35, 2.35], [2.80, 2.80], [3.25, 3.25], [3.70, 3.70], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 14 | 13 | 1.90 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 0.0 |
| 0.10 | 0.5 | 8 | 8 | 7 | 1.90 | 0.596933 | 0.006207 | axisP_side_343deg/ring80mm_1of8 τ=2.80 | 0.0 |
| 0.20 | 0.5 | 3 | 3 | 3 | 1.90 | 0.605032 | 0.014306 | axisP_side_326deg/direct τ=1.90 | 0.0 |
| 0.45 | 3.25 | 12 | 12 | 12 | 1.90 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 0.0 |
| 0.90 | 3.25 | 6 | 6 | 6 | 2.35 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 0.0 |
| 1.80 | 3.25 | 3 | 3 | 3 | 3.25 | 0.605601 | 0.014875 | axisP_side_343deg/ring140mm_2of8 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 13 | 1.90 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 64 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 4/64 angles; arcs (deg) samples 326.2..343.1 (4 samples, spacing 5.625)
- axisN: complete at any evaluated lead for 0/64 angles; arcs (deg) none
- complete grasps per complete lead: median 4, min 4, max 4 (of 128)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 112 |
| 8 | 16 | 0 | 0 | n/a | n/a | n/a | none | 224 |
| 16 | 32 | 14 | 12 | 2.35 | 0.603538 | 0.012812 | axisP_side_337deg/ring80mm_1of8 | 448 |
| 32 | 64 | 14 | 13 | 1.90 | 0.603538 | 0.012812 | axisP_side_337deg/ring80mm_1of8 | 896 |
| 64 | 128 | 14 | 13 | 1.90 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 | 1792 |

- complete records by handle-axis sign: {'axisP': 924}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 56
  - direct sufficient: 56 (100.0%)
  - route alternative required: 0 (0.0%)
  - no complete route: 0 (0.0%)
- route wall time in this search 0.423 s over 68 route rollouts (mean 6.22 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 56 | 14 | 0.603343 | 0.012617 | axisP_side_343deg/direct τ=2.35 | 56 |
| direct + 4 dirs × [80] mm | 5 | 56 | 14 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 280 |
| direct + 8 dirs × [80] mm | 9 | 56 | 14 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 504 |
| direct + 4 dirs × [140] mm | 5 | 56 | 14 | 0.603343 | 0.012617 | axisP_side_343deg/direct τ=2.35 | 280 |
| direct + 8 dirs × [140] mm | 9 | 56 | 14 | 0.603343 | 0.012617 | axisP_side_343deg/direct τ=2.35 | 504 |
| direct + 4 dirs × [80, 140] mm | 9 | 56 | 14 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 504 |
| direct + 8 dirs × [80, 140] mm | 17 | 56 | 14 | 0.590726 | 0.000000 | axisP_side_332deg/ring80mm_2of8 τ=2.35 | 952 |
