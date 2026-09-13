# T/G/R characterization — diagonal.log, rest epoch search (generation 2)

- search epoch t=53.594 s; object estimate speed at epoch 0.0000 m/s; worker wall 0.186270s; hypotheses 191, memo reuses 190, static records 32, route records 17
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring80mm_1of8 τ=2.400 (timing-admissible plans 2505, cost-valid 3247)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring80mm_1of8 τ=2.40 J=0.582485 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 191 on [0.50, 10.00] s, finest step 0.050 s
- geometrically complete (any cost-valid complete action): 191 leads, intervals [0.50, 10.00]
- timing-admissible and complete: 161 leads, intervals [2.00, 10.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > inf s at 0.0000 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 191 | 191 | 161 | 2.00 | 0.582485 | 0.000000 | axisP_side_337deg/ring80mm_1of8 τ=2.40 | 0.0 |
| 0.10 | 0.5 | 96 | 96 | 81 | 2.00 | 0.582485 | 0.000000 | axisP_side_337deg/ring80mm_1of8 τ=2.40 | 0.0 |
| 0.20 | 0.5 | 48 | 48 | 40 | 2.10 | 0.587748 | 0.005263 | axisP_side_337deg/ring80mm_1of8 τ=2.50 | 0.0 |
| 0.45 | 3.25 | 22 | 22 | 18 | 2.35 | 0.603538 | 0.021053 | axisP_side_337deg/ring80mm_1of8 τ=2.80 | 0.0 |
| 0.90 | 3.25 | 11 | 11 | 9 | 2.35 | 0.607315 | 0.024830 | axisP_side_337deg/direct τ=2.35 | 0.0 |
| 1.80 | 3.25 | 5 | 5 | 4 | 3.25 | 0.612048 | 0.029563 | axisP_side_337deg/ring140mm_2of8 τ=3.25 | 0.0 |
| V1 14 | V1 bank | 14 | 14 | 12 | 2.35 | 0.603538 | 0.021053 | axisP_side_337deg/ring80mm_1of8 τ=2.80 | 0.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 1/16 angles; arcs (deg) samples 337.5..337.5 (1 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 0/16 angles; arcs (deg) none
- complete grasps per complete lead: median 1, min 1, max 1 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 0 | 0 | n/a | n/a | n/a | none | 1528 |
| 8 | 16 | 0 | 0 | n/a | n/a | n/a | none | 3056 |
| 16 | 32 | 191 | 161 | 2.00 | 0.582485 | 0.000000 | axisP_side_337deg/ring80mm_1of8 | 6112 |

- complete records by handle-axis sign: {'axisP': 3247}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 191
  - direct sufficient: 191 (100.0%)
  - route alternative required: 0 (0.0%)
  - no complete route: 0 (0.0%)
- route wall time in this search 0.105 s over 17 route rollouts (mean 6.17 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 191 | 191 | 0.588894 | 0.006409 | axisP_side_337deg/direct τ=2.00 | 191 |
| direct + 4 dirs × [80] mm | 5 | 191 | 191 | 0.582657 | 0.000172 | axisP_side_337deg/ring80mm_2of8 τ=2.40 | 955 |
| direct + 8 dirs × [80] mm | 9 | 191 | 191 | 0.582485 | 0.000000 | axisP_side_337deg/ring80mm_1of8 τ=2.40 | 1719 |
| direct + 4 dirs × [140] mm | 5 | 191 | 191 | 0.588894 | 0.006409 | axisP_side_337deg/direct τ=2.00 | 955 |
| direct + 8 dirs × [140] mm | 9 | 191 | 191 | 0.588894 | 0.006409 | axisP_side_337deg/direct τ=2.00 | 1719 |
| direct + 4 dirs × [80, 140] mm | 9 | 191 | 191 | 0.582657 | 0.000172 | axisP_side_337deg/ring80mm_2of8 τ=2.40 | 1719 |
| direct + 8 dirs × [80, 140] mm | 17 | 191 | 191 | 0.582485 | 0.000000 | axisP_side_337deg/ring80mm_1of8 τ=2.40 | 3247 |
