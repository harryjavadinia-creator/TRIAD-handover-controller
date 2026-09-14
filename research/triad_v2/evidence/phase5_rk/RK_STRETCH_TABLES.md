
### logs/char_stretch/diagonal

- moving: direct outcomes {'success': 16, 'predictive_static/runtime_clearance_reserve': 2, 'predictive_static/reach_tracking': 6}
  - after direct `predictive_static/runtime_clearance_reserve`: anyRing=False,anyStretch=False 1; anyRing=True,anyStretch=False 1
    success/attempts by alternative: ring140 1/16, ring80 0/16, x1.05 0/2, x1.10 0/2, x1.20 0/2, x1.35 0/2, x1.50 0/2, x1.70 0/2
  - after direct `predictive_static/reach_tracking`: anyRing=False,anyStretch=False 1; anyRing=True,anyStretch=True 5
    success/attempts by alternative: ring140 9/48, ring80 15/48, x1.05 4/6, x1.10 4/6, x1.20 4/6, x1.35 4/6, x1.50 3/6, x1.70 3/6
- rest: direct outcomes {'success': 1}

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 24 (0.04) | 16/22 | (4.15, 'axisP_side_337deg', 'direct') | +0.0490 | 4.15 |
| moving | ALL | 552 (1.00) | 22/22 | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | +0.0000 | 4.15 |
| moving | DIRECT + stretch ladder (no rings) | 168 (0.30) | 21/22 | (4.15, 'axisP_side_337deg', 'directx150') | +0.0068 | 4.15 |
| moving | RINGS only (17, current) | 408 (0.74) | 22/22 | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | +0.0000 | 4.15 |
| moving | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 85 (0.15) | 22/22 | (4.15, 'axisP_side_337deg', 'direct') | +0.0490 | 4.15 |
| rest | DIRECT_ONLY | 1 (0.04) | 1/1 | (1.8, 'axisP_side_337deg', 'direct') | +0.0183 | 1.97 |
| rest | ALL | 23 (1.00) | 1/1 | (1.8, 'axisP_side_337deg', 'directx150') | +0.0000 | 2.94 |
| rest | DIRECT + stretch ladder (no rings) | 7 (0.30) | 1/1 | (1.8, 'axisP_side_337deg', 'directx150') | +0.0000 | 2.94 |
| rest | RINGS only (17, current) | 17 (0.74) | 1/1 | (1.8, 'axisP_side_337deg', 'ring80mm_1of8') | +0.0122 | 2.38 |
| rest | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 1 (0.04) | 1/1 | (1.8, 'axisP_side_337deg', 'direct') | +0.0183 | 1.97 |

### logs/char_stretch/lateral-low

- moving: direct outcomes {'predictive_static/reach_tracking': 34, 'success': 8, 'predictive_static/robust_transit_clearance_reserve': 1}
  - after direct `predictive_static/reach_tracking`: anyRing=False,anyStretch=False 1; anyRing=True,anyStretch=False 5; anyRing=True,anyStretch=True 28
    success/attempts by alternative: ring140 156/272, ring80 89/272, x1.05 1/34, x1.10 23/34, x1.20 28/34, x1.35 26/34, x1.50 26/34, x1.70 25/34
  - after direct `predictive_static/robust_transit_clearance_reserve`: anyRing=True,anyStretch=False 1
    success/attempts by alternative: ring140 1/8, ring80 0/8, x1.05 0/1, x1.10 0/1, x1.20 0/1, x1.35 0/1, x1.50 0/1, x1.70 0/1
- rest: direct outcomes {'success': 2, 'predictive_static/reach_tracking': 2}
  - after direct `predictive_static/reach_tracking`: anyRing=True,anyStretch=True 2
    success/attempts by alternative: ring140 8/16, ring80 5/16, x1.05 0/2, x1.10 2/2, x1.20 2/2, x1.35 2/2, x1.50 2/2, x1.70 2/2

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 43 (0.04) | 8/42 | (3.7, 'axisN_side_337deg', 'direct') | +0.0478 | 3.70 |
| moving | ALL | 989 (1.00) | 42/42 | (3.7, 'axisN_side_337deg', 'directx170') | +0.0000 | 3.25 |
| moving | DIRECT + stretch ladder (no rings) | 301 (0.30) | 36/42 | (3.7, 'axisN_side_337deg', 'directx170') | +0.0000 | 3.25 |
| moving | RINGS only (17, current) | 731 (0.74) | 42/42 | (3.7, 'axisN_side_337deg', 'direct') | +0.0478 | 3.25 |
| moving | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 251 (0.25) | 42/42 | (3.7, 'axisN_side_337deg', 'direct') | +0.0478 | 3.25 |
| rest | DIRECT_ONLY | 4 (0.04) | 2/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.23 |
| rest | ALL | 92 (1.00) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.23 |
| rest | DIRECT + stretch ladder (no rings) | 28 (0.30) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.23 |
| rest | RINGS only (17, current) | 68 (0.74) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.23 |
| rest | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 8 (0.09) | 4/4 | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 2.23 |

### logs/char_stretch/longitudinal

- moving: direct outcomes {'success': 15, 'predictive_static/robust_transit_clearance_reserve': 6, 'predictive_static/reach_tracking': 10, 'predictive_static/runtime_clearance_reserve': 3, 'predictive_static/static_acquire': 1}
  - after direct `predictive_static/robust_transit_clearance_reserve`: anyRing=False,anyStretch=False 5; anyRing=True,anyStretch=True 1
    success/attempts by alternative: ring140 2/48, ring80 0/48, x1.05 0/6, x1.10 0/6, x1.20 0/6, x1.35 0/6, x1.50 1/6, x1.70 1/6
  - after direct `predictive_static/reach_tracking`: anyRing=False,anyStretch=False 8; anyRing=True,anyStretch=False 1; anyRing=True,anyStretch=True 1
    success/attempts by alternative: ring140 3/80, ring80 0/80, x1.05 0/10, x1.10 0/10, x1.20 0/10, x1.35 0/10, x1.50 0/10, x1.70 1/10
  - after direct `predictive_static/runtime_clearance_reserve`: anyRing=False,anyStretch=False 1; anyRing=True,anyStretch=True 2
    success/attempts by alternative: ring140 8/24, ring80 7/24, x1.05 0/3, x1.10 0/3, x1.20 2/3, x1.35 2/3, x1.50 2/3, x1.70 2/3
  - after direct `predictive_static/static_acquire`: anyRing=False,anyStretch=False 1
    success/attempts by alternative: ring140 0/8, ring80 0/8, x1.05 0/1, x1.10 0/1, x1.20 0/1, x1.35 0/1, x1.50 0/1, x1.70 0/1
- rest: direct outcomes {'predictive_static/runtime_clearance_reserve': 1, 'success': 3, 'predictive_static/reach_tracking': 2}
  - after direct `predictive_static/runtime_clearance_reserve`: anyRing=True,anyStretch=True 1
    success/attempts by alternative: ring140 4/8, ring80 3/8, x1.05 0/1, x1.10 0/1, x1.20 1/1, x1.35 1/1, x1.50 1/1, x1.70 1/1
  - after direct `predictive_static/reach_tracking`: anyRing=False,anyStretch=False 2
    success/attempts by alternative: ring140 0/16, ring80 0/16, x1.05 0/2, x1.10 0/2, x1.20 0/2, x1.35 0/2, x1.50 0/2, x1.70 0/2

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 35 (0.04) | 15/20 | (2.35, 'axisP_side_337deg', 'direct') | +0.0259 | 2.35 |
| moving | ALL | 805 (1.00) | 20/20 | (2.35, 'axisP_side_337deg', 'directx120') | +0.0000 | 2.35 |
| moving | DIRECT + stretch ladder (no rings) | 245 (0.30) | 19/20 | (2.35, 'axisP_side_337deg', 'directx120') | +0.0000 | 2.35 |
| moving | RINGS only (17, current) | 595 (0.74) | 20/20 | (2.35, 'axisP_side_337deg', 'direct') | +0.0259 | 2.35 |
| moving | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 399 (0.50) | 20/20 | (2.35, 'axisP_side_337deg', 'direct') | +0.0259 | 2.35 |
| rest | DIRECT_ONLY | 6 (0.04) | 3/4 | (1.8, 'axisP_side_337deg', 'direct') | +0.0135 | 2.15 |
| rest | ALL | 138 (1.00) | 4/4 | (1.8, 'axisP_side_337deg', 'directx150') | +0.0000 | 3.20 |
| rest | DIRECT + stretch ladder (no rings) | 42 (0.30) | 4/4 | (1.8, 'axisP_side_337deg', 'directx150') | +0.0000 | 3.20 |
| rest | RINGS only (17, current) | 102 (0.74) | 4/4 | (1.8, 'axisP_side_337deg', 'direct') | +0.0135 | 2.15 |
| rest | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 66 (0.48) | 4/4 | (1.8, 'axisP_side_337deg', 'direct') | +0.0135 | 2.15 |

### logs/char_stretch/near-ground

- moving: direct outcomes {'success': 19, 'predictive_static/static_acquire': 1, 'predictive_static/robust_transit_clearance_reserve': 2, 'predictive_static/reach_tracking': 2}
  - after direct `predictive_static/static_acquire`: anyRing=True,anyStretch=True 1
    success/attempts by alternative: ring140 2/8, ring80 3/8, x1.05 1/1, x1.10 1/1, x1.20 1/1, x1.35 1/1, x1.50 1/1, x1.70 1/1
  - after direct `predictive_static/robust_transit_clearance_reserve`: anyRing=False,anyStretch=False 2
    success/attempts by alternative: ring140 0/16, ring80 0/16, x1.05 0/2, x1.10 0/2, x1.20 0/2, x1.35 0/2, x1.50 0/2, x1.70 0/2
  - after direct `predictive_static/reach_tracking`: anyRing=False,anyStretch=False 1; anyRing=False,anyStretch=True 1
    success/attempts by alternative: ring140 0/16, ring80 0/16, x1.05 0/2, x1.10 0/2, x1.20 0/2, x1.35 1/2, x1.50 1/2, x1.70 1/2
- rest: direct outcomes {'success': 1}

| epoch | law | route rollouts (fraction) | (τ,g) pairs with a complete action | selected | ΔJ | earliest admissible lead (rest: τ − t) |
|---|---|---|---|---|---:|---|
| moving | DIRECT_ONLY | 24 (0.04) | 19/21 | (3.25, 'axisP_side_45deg', 'direct') | +0.0650 | 3.25 |
| moving | ALL | 552 (1.00) | 21/21 | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 |
| moving | DIRECT + stretch ladder (no rings) | 168 (0.30) | 21/21 | (3.25, 'axisP_side_45deg', 'directx105') | +0.0542 | 3.25 |
| moving | RINGS only (17, current) | 408 (0.74) | 20/21 | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 |
| moving | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 98 (0.18) | 21/21 | (3.25, 'axisP_side_45deg', 'direct') | +0.0650 | 3.25 |
| rest | DIRECT_ONLY | 1 (0.04) | 1/1 | (1.8, 'axisP_side_45deg', 'direct') | +0.0528 | 3.00 |
| rest | ALL | 23 (1.00) | 1/1 | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 3.13 |
| rest | DIRECT + stretch ladder (no rings) | 7 (0.30) | 1/1 | (1.8, 'axisP_side_45deg', 'direct') | +0.0528 | 3.00 |
| rest | RINGS only (17, current) | 17 (0.74) | 1/1 | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 3.13 |
| rest | FAILURE-CONDITIONED: direct; reach_tracking -> stretch ladder (then rings); other -> rings | 1 (0.04) | 1/1 | (1.8, 'axisP_side_45deg', 'direct') | +0.0528 | 3.00 |
