# T/G/R characterization — diagonal.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 11.270499s; hypotheses 14, memo reuses 0, static records 1792, route records 1649
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring140mm_2of8 τ=4.150 (timing-admissible plans 995, cost-valid 998)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring140mm_2of8 τ=4.15 J=0.681985 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 14 on [1.80, 8.00] s, finest step 0.100 s
- geometrically complete (any cost-valid complete action): 10 leads, intervals [2.35, 2.35], [3.25, 3.25], [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- timing-admissible and complete: 8 leads, intervals [4.15, 4.15], [4.60, 4.60], [5.05, 5.05], [5.50, 5.50], [5.95, 5.95], [6.40, 6.40], [6.85, 6.85], [8.00, 8.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 14 | 10 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 4.0 |
| 0.10 | 0.5 | 8 | 4 | 4 | 4.60 | 0.704844 | 0.022859 | axisP_side_337deg/ring140mm_2of8 τ=4.60 | 8.0 |
| 0.20 | 0.5 | 3 | 1 | 1 | 5.50 | 0.768559 | 0.086574 | axisP_side_332deg/ring140mm_2of8 τ=5.50 | 16.0 |
| 0.45 | 3.25 | 12 | 9 | 7 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 36.0 |
| 0.90 | 3.25 | 6 | 6 | 4 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 72.0 |
| 1.80 | 3.25 | 3 | 3 | 2 | 5.05 | 0.734594 | 0.052609 | axisP_side_337deg/ring140mm_2of8 τ=5.05 | 144.0 |
| V1 14 | V1 bank | 14 | 10 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 64 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 20/64 angles; arcs (deg) samples -56.2..0.0 (11 samples, spacing 5.625), samples 16.9..61.9 (9 samples, spacing 5.625)
- axisN: complete at any evaluated lead for 5/64 angles; arcs (deg) samples 275.6..281.2 (2 samples, spacing 5.625), samples 309.4..320.6 (3 samples, spacing 5.625)
- complete grasps per complete lead: median 8, min 1, max 18 (of 128)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 1 | 1 | 5.95 | 0.847669 | 0.165684 | axisP_side_0deg/ring80mm_3of8 | 112 |
| 8 | 16 | 7 | 7 | 4.60 | 0.765422 | 0.083437 | axisP_side_315deg/ring140mm_2of8 | 224 |
| 16 | 32 | 8 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 | 448 |
| 32 | 64 | 10 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 | 896 |
| 64 | 128 | 10 | 8 | 4.15 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 | 1792 |

- complete records by handle-axis sign: {'axisN': 9, 'axisP': 989}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 97
  - direct sufficient: 65 (67.0%)
  - route alternative required: 20 (20.6%)
  - no complete route: 12 (12.4%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×14; NONE `predictive_static/runtime_clearance_reserve` ×5; NONE `predictive_static` ×1
- route wall time in this search 8.162 s over 1649 route rollouts (mean 4.95 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 65 | 8 | 0.730966 | 0.048981 | axisP_side_337deg/direct τ=4.15 | 97 |
| direct + 4 dirs × [80] mm | 5 | 76 | 8 | 0.688824 | 0.006838 | axisP_side_343deg/ring80mm_2of8 τ=4.15 | 485 |
| direct + 8 dirs × [80] mm | 9 | 79 | 8 | 0.688824 | 0.006838 | axisP_side_343deg/ring80mm_2of8 τ=4.15 | 873 |
| direct + 4 dirs × [140] mm | 5 | 81 | 10 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 485 |
| direct + 8 dirs × [140] mm | 9 | 84 | 10 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 873 |
| direct + 4 dirs × [80, 140] mm | 9 | 82 | 10 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 873 |
| direct + 8 dirs × [80, 140] mm | 17 | 85 | 10 | 0.681985 | 0.000000 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 1649 |
