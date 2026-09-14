### Feasible arc widths (samples of 5.625 deg; per lead and axis sign)

| scenario / epoch | static-feasible arcs: count, min, median, singletons | complete-action arcs: count, min, median, singletons |
|---|---|---|
| diagonal / moving | 22, 1, 3.0, 3 | 19, 1, 2, 6 |
| diagonal / rest | 1, 4, 4, 0 | 1, 4, 4, 0 |
| lateral-low / moving | 31, 1, 4, 5 | 31, 1, 4, 5 |
| lateral-low / rest | 3, 1, 3, 1 | 3, 1, 3, 1 |
| longitudinal / moving | 31, 1, 5, 5 | 26, 1, 2.5, 8 |
| longitudinal / rest | 3, 7, 8, 0 | 3, 1, 7, 1 |
| near-ground / moving | 21, 1, 5, 1 | 17, 2, 4, 0 |
| near-ground / rest | 1, 5, 5, 0 | 1, 4, 4, 0 |

### Laws (zero-latency selection at the search epoch; reference = 64 angles per sign)

| scenario / epoch | law | static screens (fraction) | route rollouts (fraction) | selected (lead, grasp, route) | ΔJ | earliest admissible lead (Δ) |
|---|---|---|---|---|---:|---|
| diagonal / moving | UNIFORM(4) | 112 (0.06) | 17 (0.01) | (5.95, 'axisP_side_0deg', 'ring80mm_3of8') | +0.1657 | 5.95 (+1.80) |
| diagonal / moving | UNIFORM(8) | 224 (0.12) | 238 (0.14) | (5.05, 'axisP_side_315deg', 'ring140mm_2of8') | +0.0834 | 4.6 (+0.45) |
| diagonal / moving | UNIFORM(16) | 448 (0.25) | 408 (0.25) | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | +0.0000 | 4.15 (+0.00) |
| diagonal / moving | UNIFORM(32) | 896 (0.50) | 782 (0.47) | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | +0.0000 | 4.15 (+0.00) |
| diagonal / moving | UNIFORM(64) | 1792 (1.00) | 1649 (1.00) | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | +0.0000 | 4.15 (+0.00) |
| diagonal / moving | ADAPT(4) | 133 (0.07) | 221 (0.13) | (5.95, 'axisP_side_332deg', 'ring140mm_2of8') | +0.1182 | 5.95 (+1.80) |
| diagonal / moving | ADAPT(8) | 336 (0.19) | 1326 (0.80) | (4.6, 'axisP_side_337deg', 'ring140mm_2of8') | +0.0229 | 4.6 (+0.45) |
| diagonal / moving | ADAPT(16) | 543 (0.30) | 1428 (0.87) | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | +0.0000 | 4.15 (+0.00) |
| diagonal / moving | ARCREP(1) | 1792 (1.00) | 374 (0.23) | (4.15, 'axisP_side_332deg', 'ring140mm_2of8') | +0.0016 | 4.15 (+0.00) |
| diagonal / moving | ARCREP(3) | 1792 (1.00) | 901 (0.55) | (4.15, 'axisP_side_332deg', 'ring140mm_2of8') | +0.0016 | 4.15 (+0.00) |
| diagonal / rest | UNIFORM(4) | 8 (0.06) | 0 (0.00) | none |  | None () |
| diagonal / rest | UNIFORM(8) | 16 (0.12) | 0 (0.00) | none |  | None () |
| diagonal / rest | UNIFORM(16) | 32 (0.25) | 17 (0.25) | (1.8, 'axisP_side_337deg', 'ring80mm_1of8') | +0.0028 | 1.8 (+0.00) |
| diagonal / rest | UNIFORM(32) | 64 (0.50) | 34 (0.50) | (1.8, 'axisP_side_337deg', 'ring80mm_1of8') | +0.0028 | 1.8 (+0.00) |
| diagonal / rest | UNIFORM(64) | 128 (1.00) | 68 (1.00) | (1.8, 'axisP_side_343deg', 'ring80mm_1of8') | +0.0000 | 1.8 (+0.00) |
| diagonal / rest | ADAPT(4) | 8 (0.06) | 0 (0.00) | none |  | None () |
| diagonal / rest | ADAPT(8) | 16 (0.12) | 0 (0.00) | none |  | None () |
| diagonal / rest | ADAPT(16) | 37 (0.29) | 68 (1.00) | (1.8, 'axisP_side_343deg', 'ring80mm_1of8') | +0.0000 | 1.8 (+0.00) |
| diagonal / rest | ARCREP(1) | 128 (1.00) | 17 (0.25) | (1.8, 'axisP_side_337deg', 'ring80mm_1of8') | +0.0028 | 1.8 (+0.00) |
| diagonal / rest | ARCREP(3) | 128 (1.00) | 51 (0.75) | (1.8, 'axisP_side_343deg', 'ring80mm_1of8') | +0.0000 | 1.8 (+0.00) |
| lateral-low / moving | UNIFORM(4) | 112 (0.06) | 153 (0.05) | (5.05, 'axisN_side_90deg', 'ring140mm_7of8') | +0.1941 | 4.6 (+2.25) |
| lateral-low / moving | UNIFORM(8) | 224 (0.12) | 306 (0.11) | (3.7, 'axisN_side_45deg', 'ring140mm_6of8') | +0.0414 | 3.7 (+1.35) |
| lateral-low / moving | UNIFORM(16) | 448 (0.25) | 731 (0.25) | (3.7, 'axisN_side_337deg', 'direct') | +0.0000 | 3.25 (+0.90) |
| lateral-low / moving | UNIFORM(32) | 896 (0.50) | 1496 (0.51) | (3.7, 'axisN_side_337deg', 'direct') | +0.0000 | 2.35 (+0.00) |
| lateral-low / moving | UNIFORM(64) | 1792 (1.00) | 2907 (1.00) | (3.7, 'axisN_side_337deg', 'direct') | +0.0000 | 2.35 (+0.00) |
| lateral-low / moving | ADAPT(4) | 248 (0.14) | 1683 (0.58) | (4.6, 'axisN_side_45deg', 'ring80mm_6of8') | +0.0845 | 4.6 (+2.25) |
| lateral-low / moving | ADAPT(8) | 363 (0.20) | 1955 (0.67) | (3.7, 'axisN_side_45deg', 'ring140mm_6of8') | +0.0414 | 3.7 (+1.35) |
| lateral-low / moving | ADAPT(16) | 622 (0.35) | 2720 (0.94) | (3.7, 'axisN_side_337deg', 'direct') | +0.0000 | 2.8 (+0.45) |
| lateral-low / moving | ARCREP(1) | 1792 (1.00) | 527 (0.18) | (3.7, 'axisN_side_332deg', 'direct') | +0.0234 | 2.35 (+0.00) |
| lateral-low / moving | ARCREP(3) | 1792 (1.00) | 1343 (0.46) | (3.7, 'axisN_side_337deg', 'direct') | +0.0000 | 2.35 (+0.00) |
| lateral-low / rest | UNIFORM(4) | 8 (0.06) | 0 (0.00) | none |  | None () |
| lateral-low / rest | UNIFORM(8) | 16 (0.12) | 17 (0.08) | (1.8, 'axisN_side_45deg', 'ring80mm_6of8') | +0.0784 | 1.8 (+0.00) |
| lateral-low / rest | UNIFORM(16) | 32 (0.25) | 68 (0.33) | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| lateral-low / rest | UNIFORM(32) | 64 (0.50) | 119 (0.58) | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| lateral-low / rest | UNIFORM(64) | 128 (1.00) | 204 (1.00) | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| lateral-low / rest | ADAPT(4) | 8 (0.06) | 0 (0.00) | none |  | None () |
| lateral-low / rest | ADAPT(8) | 26 (0.20) | 136 (0.67) | (1.8, 'axisN_side_45deg', 'ring80mm_6of8') | +0.0784 | 1.8 (+0.00) |
| lateral-low / rest | ADAPT(16) | 49 (0.38) | 204 (1.00) | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| lateral-low / rest | ARCREP(1) | 128 (1.00) | 51 (0.25) | (1.8, 'axisN_side_332deg', 'direct') | +0.0197 | 1.8 (+0.00) |
| lateral-low / rest | ARCREP(3) | 128 (1.00) | 119 (0.58) | (1.8, 'axisN_side_337deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| longitudinal / moving | UNIFORM(4) | 112 (0.06) | 170 (0.07) | (4.6, 'axisP_side_0deg', 'ring140mm_2of8') | +0.1307 | 4.6 (+2.25) |
| longitudinal / moving | UNIFORM(8) | 224 (0.12) | 289 (0.11) | (4.15, 'axisP_side_315deg', 'ring140mm_1of8') | +0.1238 | 3.7 (+1.35) |
| longitudinal / moving | UNIFORM(16) | 448 (0.25) | 595 (0.23) | (2.35, 'axisP_side_337deg', 'direct') | +0.0055 | 2.35 (+0.00) |
| longitudinal / moving | UNIFORM(32) | 896 (0.50) | 1343 (0.52) | (2.35, 'axisP_side_337deg', 'direct') | +0.0055 | 2.35 (+0.00) |
| longitudinal / moving | UNIFORM(64) | 1792 (1.00) | 2601 (1.00) | (3.25, 'axisP_side_343deg', 'ring140mm_1of8') | +0.0000 | 2.35 (+0.00) |
| longitudinal / moving | ADAPT(4) | 230 (0.13) | 1258 (0.48) | (4.6, 'axisP_side_332deg', 'ring140mm_1of8') | +0.1141 | 3.25 (+0.90) |
| longitudinal / moving | ADAPT(8) | 377 (0.21) | 1972 (0.76) | (3.7, 'axisP_side_343deg', 'ring140mm_1of8') | +0.0345 | 3.25 (+0.90) |
| longitudinal / moving | ADAPT(16) | 598 (0.33) | 2346 (0.90) | (3.25, 'axisP_side_343deg', 'ring140mm_1of8') | +0.0000 | 2.35 (+0.00) |
| longitudinal / moving | ARCREP(1) | 1792 (1.00) | 527 (0.20) | (2.35, 'axisP_side_337deg', 'direct') | +0.0055 | 2.35 (+0.00) |
| longitudinal / moving | ARCREP(3) | 1792 (1.00) | 1326 (0.51) | (3.25, 'axisP_side_343deg', 'ring140mm_1of8') | +0.0000 | 2.35 (+0.00) |
| longitudinal / rest | UNIFORM(4) | 8 (0.06) | 17 (0.04) | none |  | None () |
| longitudinal / rest | UNIFORM(8) | 16 (0.12) | 51 (0.12) | (1.8, 'axisP_side_315deg', 'direct') | +0.0323 | 1.8 (+0.00) |
| longitudinal / rest | UNIFORM(16) | 32 (0.25) | 102 (0.25) | (1.8, 'axisP_side_337deg', 'direct') | +0.0004 | 1.8 (+0.00) |
| longitudinal / rest | UNIFORM(32) | 64 (0.50) | 204 (0.50) | (1.8, 'axisP_side_337deg', 'direct') | +0.0004 | 1.8 (+0.00) |
| longitudinal / rest | UNIFORM(64) | 128 (1.00) | 408 (1.00) | (1.8, 'axisP_side_343deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| longitudinal / rest | ADAPT(4) | 20 (0.16) | 153 (0.38) | (1.8, 'axisN_side_242deg', 'ring140mm_6of8') | +0.5216 | 1.8 (+0.00) |
| longitudinal / rest | ADAPT(8) | 46 (0.36) | 408 (1.00) | (1.8, 'axisP_side_343deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| longitudinal / rest | ADAPT(16) | 56 (0.44) | 408 (1.00) | (1.8, 'axisP_side_343deg', 'direct') | +0.0000 | 1.8 (+0.00) |
| longitudinal / rest | ARCREP(1) | 128 (1.00) | 51 (0.12) | (1.8, 'axisP_side_337deg', 'direct') | +0.0004 | 1.8 (+0.00) |
| longitudinal / rest | ARCREP(3) | 128 (1.00) | 153 (0.38) | (1.8, 'axisP_side_337deg', 'direct') | +0.0004 | 1.8 (+0.00) |
| near-ground / moving | UNIFORM(4) | 112 (0.06) | 0 (0.00) | none |  | None () |
| near-ground / moving | UNIFORM(8) | 224 (0.12) | 204 (0.14) | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 (+0.00) |
| near-ground / moving | UNIFORM(16) | 448 (0.25) | 408 (0.28) | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 (+0.00) |
| near-ground / moving | UNIFORM(32) | 896 (0.50) | 731 (0.49) | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 (+0.00) |
| near-ground / moving | UNIFORM(64) | 1792 (1.00) | 1479 (1.00) | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 (+0.00) |
| near-ground / moving | ADAPT(4) | 112 (0.06) | 0 (0.00) | none |  | None () |
| near-ground / moving | ADAPT(8) | 319 (0.18) | 1054 (0.71) | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 (+0.00) |
| near-ground / moving | ADAPT(16) | 551 (0.31) | 1411 (0.95) | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | +0.0000 | 3.25 (+0.00) |
| near-ground / moving | ARCREP(1) | 1792 (1.00) | 357 (0.24) | (3.25, 'axisP_side_56deg', 'ring80mm_0of8') | +0.0523 | 3.25 (+0.00) |
| near-ground / moving | ARCREP(3) | 1792 (1.00) | 952 (0.64) | (3.25, 'axisP_side_56deg', 'ring80mm_0of8') | +0.0523 | 3.25 (+0.00) |
| near-ground / rest | UNIFORM(4) | 8 (0.06) | 0 (0.00) | none |  | None () |
| near-ground / rest | UNIFORM(8) | 16 (0.12) | 17 (0.20) | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 1.8 (+0.00) |
| near-ground / rest | UNIFORM(16) | 32 (0.25) | 17 (0.20) | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 1.8 (+0.00) |
| near-ground / rest | UNIFORM(32) | 64 (0.50) | 34 (0.40) | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 1.8 (+0.00) |
| near-ground / rest | UNIFORM(64) | 128 (1.00) | 85 (1.00) | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 1.8 (+0.00) |
| near-ground / rest | ADAPT(4) | 8 (0.06) | 0 (0.00) | none |  | None () |
| near-ground / rest | ADAPT(8) | 23 (0.18) | 85 (1.00) | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 1.8 (+0.00) |
| near-ground / rest | ADAPT(16) | 37 (0.29) | 85 (1.00) | (1.8, 'axisP_side_45deg', 'ring80mm_0of8') | +0.0000 | 1.8 (+0.00) |
| near-ground / rest | ARCREP(1) | 128 (1.00) | 17 (0.20) | (1.8, 'axisP_side_51deg', 'ring80mm_0of8') | +0.0070 | 1.8 (+0.00) |
| near-ground / rest | ARCREP(3) | 128 (1.00) | 51 (0.60) | (1.8, 'axisP_side_51deg', 'ring80mm_0of8') | +0.0070 | 1.8 (+0.00) |
