# T/G/R characterization — diagonal.log, moving epoch search (generation 1)

- search epoch t=9.024 s; object estimate speed at epoch 0.0800 m/s; worker wall 44.569056s; hypotheses 191, memo reuses 0, static records 6112, route records 5984
- timing admission applied here at the epoch: lead >= 1.6 s and presentationDuration + 0.05 s <= lead
- unchanged selector on the full grid at the epoch: axisP_side_337deg/ring80mm_2of8 τ=3.900 (timing-admissible plans 3338, cost-valid 3344)
- re-implemented argmin globalJ on admissible cost-valid records: axisP_side_337deg/ring80mm_2of8 τ=3.90 J=0.675810 — matches the logged selector (tie-break differences possible)

## T — temporal interception interval and resolution

- evaluated leads: 191 on [0.50, 10.00] s, finest step 0.050 s
- geometrically complete (any cost-valid complete action): 129 leads, intervals [2.45, 2.60], [2.75, 2.75], [2.95, 2.95], [3.90, 10.00]
- timing-admissible and complete: 123 leads, intervals [3.90, 10.00]
- constant-twist prediction: displacement per lead step = speed × Δτ; the commit-freshness tube (15 mm) is crossed per step when Δτ > 0.188 s at 0.0800 m/s

| Δτ (s) | grid origin | leads | complete leads | admissible leads | earliest admissible τ (s) | best J | ΔJ vs finest | chosen action | displacement per step (mm) |
|---:|---|---:|---:|---:|---:|---:|---:|---|---:|
| 0.05 | 0.5 | 191 | 129 | 123 | 3.90 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 τ=3.90 | 4.0 |
| 0.10 | 0.5 | 96 | 64 | 62 | 3.90 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 τ=3.90 | 8.0 |
| 0.20 | 0.5 | 48 | 32 | 31 | 3.90 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 τ=3.90 | 16.0 |
| 0.45 | 3.25 | 22 | 14 | 14 | 4.15 | 0.681985 | 0.006175 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 36.0 |
| 0.90 | 3.25 | 11 | 7 | 7 | 4.15 | 0.681985 | 0.006175 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 72.0 |
| 1.80 | 3.25 | 5 | 3 | 3 | 5.05 | 0.734594 | 0.058784 | axisP_side_337deg/ring140mm_2of8 τ=5.05 | 144.0 |
| V1 14 | V1 bank | 14 | 8 | 8 | 4.15 | 0.681985 | 0.006175 | axisP_side_337deg/ring140mm_2of8 τ=4.15 | 36.0 |

## G — receiver approach family and angular resolution

- grasp ring: 16 angles × 2 handle-axis signs; angle 0 = robot-relative outward direction (see beginCapturePlanningCore)
- axisP: complete at any evaluated lead for 6/16 angles; arcs (deg) samples -45.0..67.5 (6 samples, spacing 22.500)
- axisN: complete at any evaluated lead for 3/16 angles; arcs (deg) samples 45.0..45.0 (1 samples, spacing 22.500), samples 270.0..270.0 (1 samples, spacing 22.500), samples 315.0..315.0 (1 samples, spacing 22.500)
- complete grasps per complete lead: median 2, min 1, max 5 (of 32)

| N_θ | grasps | leads with ≥1 complete grasp | admissible leads | earliest admissible τ | best J | ΔJ vs finest | chosen | static records (count) |
|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 4 | 8 | 19 | 13 | 5.70 | 0.829380 | 0.153570 | axisP_side_0deg/ring80mm_3of8 | 1528 |
| 8 | 16 | 116 | 110 | 4.55 | 0.764815 | 0.089005 | axisP_side_315deg/ring140mm_2of8 | 3056 |
| 16 | 32 | 129 | 123 | 3.90 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 | 6112 |

- complete records by handle-axis sign: {'axisN': 176, 'axisP': 3168}

## R — direct motion versus route alternatives

- (event, grasp) pairs that passed the static screen and entered route certification: 352
  - direct sufficient: 220 (62.5%)
  - route alternative required: 93 (26.4%)
  - no complete route: 39 (11.1%)
- where a route was required, the direct route failed at: NONE `predictive_static/reach_tracking` ×69; NONE `predictive_static/runtime_clearance_reserve` ×18; NONE `predictive_static/robust_transit_clearance_reserve` ×4; INSERTION `predictive_static` ×2
- route wall time in this search 33.161 s over 5984 route rollouts (mean 5.54 ms per rollout; memoized hypotheses cost nothing)

| route set | routes | (event, grasp) with ≥1 complete route | leads with ≥1 complete action | best J | ΔJ vs finest | chosen | rollouts in this search |
|---|---:|---:|---:|---:|---:|---|---:|
| direct only | 1 | 220 | 123 | 0.719698 | 0.043888 | axisP_side_337deg/direct τ=3.90 | 352 |
| direct + 4 dirs × [80] mm | 5 | 286 | 123 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 τ=3.90 | 1760 |
| direct + 8 dirs × [80] mm | 9 | 295 | 123 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 τ=3.90 | 3168 |
| direct + 4 dirs × [140] mm | 5 | 284 | 129 | 0.680111 | 0.004300 | axisP_side_337deg/ring140mm_2of8 τ=4.20 | 1760 |
| direct + 8 dirs × [140] mm | 9 | 299 | 129 | 0.680111 | 0.004300 | axisP_side_337deg/ring140mm_2of8 τ=4.20 | 3168 |
| direct + 4 dirs × [80, 140] mm | 9 | 296 | 129 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 τ=3.90 | 3168 |
| direct + 8 dirs × [80, 140] mm | 17 | 313 | 129 | 0.675810 | 0.000000 | axisP_side_337deg/ring80mm_2of8 τ=3.90 | 5984 |
