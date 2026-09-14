
### logs/char_R/diagonal

- moving: direct outcomes {'success': 16, 'predictive_static/runtime_clearance_reserve': 2, 'predictive_static/reach_tracking': 6}
  - after direct `predictive_static/runtime_clearance_reserve`: anyRing=True,anyStretch=False 2
    success/attempts by alternative: ring140 1/32, ring200 3/32, ring40 0/32, ring80 1/32
  - after direct `predictive_static/reach_tracking`: anyRing=True,anyStretch=False 6
    success/attempts by alternative: ring140 21/96, ring200 19/96, ring40 31/96, ring80 29/96
- rest: direct outcomes {'success': 1}

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 24 (0.02) | 16/24 | (4.15, 'axisP_side_337deg', 'direct') | +0.0490 | 4.15 |
| moving | ALL | 1560 (1.00) | 24/24 | (4.15, 'axisP_side_337deg', 'ring140mm_4of16') | +0.0000 | 4.15 |
| moving | V1_17 (8 dirs x 80,140) | 408 (0.26) | 22/24 | (4.15, 'axisP_side_337deg', 'ring140mm_4of16') | +0.0000 | 4.15 |
| moving | 9 (4 dirs x 80,140) | 216 (0.14) | 21/24 | (4.15, 'axisP_side_337deg', 'ring140mm_4of16') | +0.0000 | 4.15 |
| moving | 5 (4 dirs x 140) | 120 (0.08) | 20/24 | (4.15, 'axisP_side_337deg', 'ring140mm_4of16') | +0.0000 | 4.15 |
| moving | LAZY: direct, then 8 dirs x 80,140 if direct fails | 152 (0.10) | 22/24 | (4.15, 'axisP_side_337deg', 'direct') | +0.0490 | 4.15 |
| moving | LAZY: direct, then all 64 if direct fails | 536 (0.34) | 24/24 | (4.15, 'axisP_side_337deg', 'direct') | +0.0490 | 4.15 |
| rest | DIRECT_ONLY | 1 (0.02) | 1/1 | (1.8, 'axisP_side_337deg', 'direct') | +0.0094 | 1.98 |
| rest | ALL | 65 (1.00) | 1/1 | (1.8, 'axisP_side_337deg', 'ring80mm_3of16') | +0.0000 | 2.38 |
| rest | V1_17 (8 dirs x 80,140) | 17 (0.26) | 1/1 | (1.8, 'axisP_side_337deg', 'ring80mm_2of16') | +0.0034 | 2.38 |
| rest | 9 (4 dirs x 80,140) | 9 (0.14) | 1/1 | (1.8, 'axisP_side_337deg', 'ring80mm_4of16') | +0.0035 | 2.38 |
| rest | 5 (4 dirs x 140) | 5 (0.08) | 1/1 | (1.8, 'axisP_side_337deg', 'direct') | +0.0094 | 1.98 |
| rest | LAZY: direct, then 8 dirs x 80,140 if direct fails | 1 (0.02) | 1/1 | (1.8, 'axisP_side_337deg', 'direct') | +0.0094 | 1.98 |
| rest | LAZY: direct, then all 64 if direct fails | 1 (0.02) | 1/1 | (1.8, 'axisP_side_337deg', 'direct') | +0.0094 | 1.98 |

### logs/char_R/lateral-low

- moving: direct outcomes {'predictive_static/reach_tracking': 34, 'success': 8, 'predictive_static/robust_transit_clearance_reserve': 1}
  - after direct `predictive_static/reach_tracking`: anyRing=True,anyStretch=False 34
    success/attempts by alternative: ring140 311/544, ring200 312/544, ring40 10/544, ring80 181/544
  - after direct `predictive_static/robust_transit_clearance_reserve`: anyRing=True,anyStretch=False 1
    success/attempts by alternative: ring140 2/16, ring200 3/16, ring40 0/16, ring80 0/16
- rest: direct outcomes {'success': 2, 'predictive_static/reach_tracking': 2}
  - after direct `predictive_static/reach_tracking`: anyRing=True,anyStretch=False 2
    success/attempts by alternative: ring140 18/32, ring200 17/32, ring40 0/32, ring80 10/32

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 43 (0.02) | 8/43 | (3.7, 'axisN_side_337deg', 'direct') | +0.0022 | 3.70 |
| moving | ALL | 2795 (1.00) | 43/43 | (3.7, 'axisN_side_337deg', 'ring40mm_7of16') | +0.0000 | 3.25 |
| moving | V1_17 (8 dirs x 80,140) | 731 (0.26) | 42/43 | (3.7, 'axisN_side_337deg', 'direct') | +0.0022 | 3.25 |
| moving | 9 (4 dirs x 80,140) | 387 (0.14) | 41/43 | (3.7, 'axisN_side_337deg', 'direct') | +0.0022 | 3.70 |
| moving | 5 (4 dirs x 140) | 215 (0.08) | 41/43 | (3.7, 'axisN_side_337deg', 'direct') | +0.0022 | 3.70 |
| moving | LAZY: direct, then 8 dirs x 80,140 if direct fails | 603 (0.22) | 42/43 | (3.7, 'axisN_side_337deg', 'direct') | +0.0022 | 3.25 |
| moving | LAZY: direct, then all 64 if direct fails | 2283 (0.82) | 43/43 | (3.7, 'axisN_side_337deg', 'direct') | +0.0022 | 3.25 |
| rest | DIRECT_ONLY | 4 (0.02) | 2/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.22 |
| rest | ALL | 260 (1.00) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.22 |
| rest | V1_17 (8 dirs x 80,140) | 68 (0.26) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.22 |
| rest | 9 (4 dirs x 80,140) | 36 (0.14) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.22 |
| rest | 5 (4 dirs x 140) | 20 (0.08) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.22 |
| rest | LAZY: direct, then 8 dirs x 80,140 if direct fails | 36 (0.14) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.22 |
| rest | LAZY: direct, then all 64 if direct fails | 132 (0.51) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.22 |

### logs/char_R/longitudinal

- moving: direct outcomes {'success': 15, 'predictive_static/robust_transit_clearance_reserve': 6, 'predictive_static/reach_tracking': 10, 'predictive_static/runtime_clearance_reserve': 3, 'predictive_static/static_acquire': 1}
  - after direct `predictive_static/robust_transit_clearance_reserve`: anyRing=False,anyStretch=False 4; anyRing=True,anyStretch=False 2
    success/attempts by alternative: ring140 3/96, ring200 7/96, ring40 0/96, ring80 0/96
  - after direct `predictive_static/reach_tracking`: anyRing=False,anyStretch=False 1; anyRing=True,anyStretch=False 9
    success/attempts by alternative: ring140 5/160, ring200 26/160, ring40 0/160, ring80 0/160
  - after direct `predictive_static/runtime_clearance_reserve`: anyRing=False,anyStretch=False 1; anyRing=True,anyStretch=False 2
    success/attempts by alternative: ring140 14/48, ring200 10/48, ring40 6/48, ring80 15/48
  - after direct `predictive_static/static_acquire`: anyRing=False,anyStretch=False 1
    success/attempts by alternative: ring140 0/16, ring200 0/16, ring40 0/16, ring80 0/16
- rest: direct outcomes {'predictive_static/runtime_clearance_reserve': 1, 'success': 3, 'predictive_static/reach_tracking': 2}
  - after direct `predictive_static/runtime_clearance_reserve`: anyRing=True,anyStretch=False 1
    success/attempts by alternative: ring140 7/16, ring200 5/16, ring40 0/16, ring80 7/16
  - after direct `predictive_static/reach_tracking`: anyRing=True,anyStretch=False 2
    success/attempts by alternative: ring140 0/32, ring200 7/32, ring40 0/32, ring80 0/32

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 35 (0.02) | 15/28 | (2.35, 'axisP_side_337deg', 'direct') | +0.0063 | 2.35 |
| moving | ALL | 2275 (1.00) | 28/28 | (2.35, 'axisP_side_337deg', 'ring40mm_3of16') | +0.0000 | 2.35 |
| moving | V1_17 (8 dirs x 80,140) | 595 (0.26) | 20/28 | (2.35, 'axisP_side_337deg', 'direct') | +0.0063 | 2.35 |
| moving | 9 (4 dirs x 80,140) | 315 (0.14) | 19/28 | (2.35, 'axisP_side_337deg', 'direct') | +0.0063 | 2.35 |
| moving | 5 (4 dirs x 140) | 175 (0.08) | 19/28 | (2.35, 'axisP_side_337deg', 'direct') | +0.0063 | 2.35 |
| moving | LAZY: direct, then 8 dirs x 80,140 if direct fails | 355 (0.16) | 20/28 | (2.35, 'axisP_side_337deg', 'direct') | +0.0063 | 2.35 |
| moving | LAZY: direct, then all 64 if direct fails | 1315 (0.58) | 28/28 | (2.35, 'axisP_side_337deg', 'direct') | +0.0063 | 2.35 |
| rest | DIRECT_ONLY | 6 (0.02) | 3/6 | (1.8, 'axisP_side_337deg', 'direct') | +0.0000 | 2.15 |
| rest | ALL | 390 (1.00) | 6/6 | (1.8, 'axisP_side_337deg', 'direct') | +0.0000 | 2.15 |
| rest | V1_17 (8 dirs x 80,140) | 102 (0.26) | 4/6 | (1.8, 'axisP_side_337deg', 'direct') | +0.0000 | 2.15 |
| rest | 9 (4 dirs x 80,140) | 54 (0.14) | 4/6 | (1.8, 'axisP_side_337deg', 'direct') | +0.0000 | 2.15 |
| rest | 5 (4 dirs x 140) | 30 (0.08) | 4/6 | (1.8, 'axisP_side_337deg', 'direct') | +0.0000 | 2.15 |
| rest | LAZY: direct, then 8 dirs x 80,140 if direct fails | 54 (0.14) | 4/6 | (1.8, 'axisP_side_337deg', 'direct') | +0.0000 | 2.15 |
| rest | LAZY: direct, then all 64 if direct fails | 198 (0.51) | 6/6 | (1.8, 'axisP_side_337deg', 'direct') | +0.0000 | 2.15 |

### logs/char_R/near-ground

- moving: direct outcomes {'success': 19, 'predictive_static/static_acquire': 1, 'predictive_static/robust_transit_clearance_reserve': 2, 'predictive_static/reach_tracking': 2}
  - after direct `predictive_static/static_acquire`: anyRing=True,anyStretch=False 1
    success/attempts by alternative: ring140 5/16, ring200 5/16, ring40 8/16, ring80 6/16
  - after direct `predictive_static/robust_transit_clearance_reserve`: anyRing=False,anyStretch=False 2
    success/attempts by alternative: ring140 0/32, ring200 0/32, ring40 0/32, ring80 0/32
  - after direct `predictive_static/reach_tracking`: anyRing=False,anyStretch=False 1; anyRing=True,anyStretch=False 1
    success/attempts by alternative: ring140 0/32, ring200 1/32, ring40 0/32, ring80 0/32
- rest: direct outcomes {'success': 1}

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 24 (0.02) | 19/21 | (3.25, 'axisP_side_45deg', 'direct') | +0.0650 | 3.25 |
| moving | ALL | 1560 (1.00) | 21/21 | (3.25, 'axisP_side_45deg', 'ring80mm_14of16') | +0.0000 | 3.25 |
| moving | V1_17 (8 dirs x 80,140) | 408 (0.26) | 20/21 | (3.25, 'axisP_side_45deg', 'ring80mm_14of16') | +0.0000 | 3.25 |
| moving | 9 (4 dirs x 80,140) | 216 (0.14) | 20/21 | (3.25, 'axisP_side_45deg', 'ring80mm_0of16') | +0.0169 | 3.25 |
| moving | 5 (4 dirs x 140) | 120 (0.08) | 20/21 | (3.7, 'axisP_side_45deg', 'ring140mm_0of16') | +0.0649 | 3.25 |
| moving | LAZY: direct, then 8 dirs x 80,140 if direct fails | 104 (0.07) | 20/21 | (3.25, 'axisP_side_45deg', 'direct') | +0.0650 | 3.25 |
| moving | LAZY: direct, then all 64 if direct fails | 344 (0.22) | 21/21 | (3.25, 'axisP_side_45deg', 'direct') | +0.0650 | 3.25 |
| rest | DIRECT_ONLY | 1 (0.02) | 1/1 | (1.8, 'axisP_side_45deg', 'direct') | +0.0627 | 3.00 |
| rest | ALL | 65 (1.00) | 1/1 | (1.8, 'axisP_side_45deg', 'ring40mm_1of16') | +0.0000 | 3.03 |
| rest | V1_17 (8 dirs x 80,140) | 17 (0.26) | 1/1 | (1.8, 'axisP_side_45deg', 'ring80mm_0of16') | +0.0099 | 3.13 |
| rest | 9 (4 dirs x 80,140) | 9 (0.14) | 1/1 | (1.8, 'axisP_side_45deg', 'ring80mm_0of16') | +0.0099 | 3.13 |
| rest | 5 (4 dirs x 140) | 5 (0.08) | 1/1 | (1.8, 'axisP_side_45deg', 'ring140mm_0of16') | +0.0493 | 3.38 |
| rest | LAZY: direct, then 8 dirs x 80,140 if direct fails | 1 (0.02) | 1/1 | (1.8, 'axisP_side_45deg', 'direct') | +0.0627 | 3.00 |
| rest | LAZY: direct, then all 64 if direct fails | 1 (0.02) | 1/1 | (1.8, 'axisP_side_45deg', 'direct') | +0.0627 | 3.00 |
