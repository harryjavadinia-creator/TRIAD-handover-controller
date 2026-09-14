### Per search

| dataset / scenario / epoch | admissible | ranker | tuple | completion (s) | min clearance (mm) | regret completion (s) | clearance deficit vs best within +0.5 s (mm) | J regret |
|---|---:|---|---|---:|---:|---:|---:|---:|
| logs/char/diagonal/moving | 267 | J7 | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char/diagonal/moving | 267 | EARLIEST_TAU | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char/diagonal/moving | 267 | EARLIEST_DONE | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char/diagonal/moving | 267 | LEX(done+0.2s -> clearance -> effort) | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char/diagonal/moving | 267 | LEX(done+0.5s -> clearance -> effort) | (4.6, 'axisP_side_337deg', 'ring140mm_3of8') | 9.78 | 75.7 | 0.45 | 0.0 | 0.0604 |
| logs/char/diagonal/moving | 267 | LEX(done+1.0s -> clearance -> effort) | (5.05, 'axisP_side_337deg', 'ring140mm_3of8') | 10.20 | 75.8 | 0.87 | -0.1 | 0.0956 |
| logs/char/diagonal/moving | 267 | SAFE_EARLIEST (clear >= 80 mm) | (8.0, 'axisP_side_23deg', 'ring140mm_3of8') | 14.37 | 79.0 | 5.04 | -3.3 | 0.5278 |
| logs/char/diagonal/moving | 267 | J_T+C | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char/diagonal/moving | 267 | J7 - V | (4.15, 'axisP_side_337deg', 'ring80mm_2of8') | 9.33 | 74.5 | 0.00 | 1.2 | 0.0094 |
| logs/char/diagonal/moving | 267 | J7 - E,L | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char/diagonal/moving | 267 | J7 - Q,K | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char/diagonal/rest | 17 | J7 | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char/diagonal/rest | 17 | EARLIEST_TAU | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char/diagonal/rest | 17 | EARLIEST_DONE | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char/diagonal/rest | 17 | LEX(done+0.2s -> clearance -> effort) | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char/diagonal/rest | 17 | LEX(done+0.5s -> clearance -> effort) | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char/diagonal/rest | 17 | LEX(done+1.0s -> clearance -> effort) | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char/diagonal/rest | 17 | SAFE_EARLIEST (clear >= 80 mm) | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char/diagonal/rest | 17 | J_T+C | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char/diagonal/rest | 17 | J7 - V | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char/diagonal/rest | 17 | J7 - E,L | (2.381, 'axisP_side_337deg', 'ring80mm_2of8') | 7.56 | 80.5 | 0.38 | 0.3 | 0.0002 |
| logs/char/diagonal/rest | 17 | J7 - Q,K | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char/lateral-low/moving | 306 | J7 | (3.7, 'axisN_side_337deg', 'direct') | 8.88 | 77.3 | 0.41 | 2.9 | 0.0000 |
| logs/char/lateral-low/moving | 306 | EARLIEST_TAU | (3.25, 'axisN_side_68deg', 'ring80mm_5of8') | 8.47 | 47.9 | 0.00 | 32.3 | 0.1309 |
| logs/char/lateral-low/moving | 306 | EARLIEST_DONE | (3.25, 'axisN_side_68deg', 'ring80mm_5of8') | 8.47 | 47.9 | 0.00 | 32.3 | 0.1309 |
| logs/char/lateral-low/moving | 306 | LEX(done+0.2s -> clearance -> effort) | (3.25, 'axisN_side_68deg', 'ring80mm_5of8') | 8.47 | 47.9 | 0.00 | 32.3 | 0.1309 |
| logs/char/lateral-low/moving | 306 | LEX(done+0.5s -> clearance -> effort) | (3.7, 'axisN_side_45deg', 'ring140mm_6of8') | 8.87 | 80.2 | 0.40 | 0.0 | 0.0414 |
| logs/char/lateral-low/moving | 306 | LEX(done+1.0s -> clearance -> effort) | (4.15, 'axisN_side_337deg', 'ring80mm_5of8') | 9.33 | 81.3 | 0.86 | -1.1 | 0.0188 |
| logs/char/lateral-low/moving | 306 | SAFE_EARLIEST (clear >= 80 mm) | (3.7, 'axisN_side_45deg', 'ring140mm_6of8') | 8.87 | 80.2 | 0.40 | 0.0 | 0.0414 |
| logs/char/lateral-low/moving | 306 | J_T+C | (3.7, 'axisN_side_45deg', 'ring140mm_1of8') | 8.85 | 79.9 | 0.38 | 0.3 | 0.3485 |
| logs/char/lateral-low/moving | 306 | J7 - V | (3.7, 'axisN_side_337deg', 'direct') | 8.88 | 77.3 | 0.41 | 2.9 | 0.0000 |
| logs/char/lateral-low/moving | 306 | J7 - E,L | (3.7, 'axisN_side_337deg', 'ring80mm_3of8') | 8.88 | 77.3 | 0.40 | 2.9 | 0.0007 |
| logs/char/lateral-low/moving | 306 | J7 - Q,K | (3.7, 'axisN_side_337deg', 'direct') | 8.88 | 77.3 | 0.41 | 2.9 | 0.0000 |
| logs/char/lateral-low/rest | 42 | J7 | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | EARLIEST_TAU | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | EARLIEST_DONE | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | LEX(done+0.2s -> clearance -> effort) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | LEX(done+0.5s -> clearance -> effort) | (2.458, 'axisN_side_337deg', 'ring80mm_3of8') | 7.64 | 80.8 | 0.23 | 0.0 | 0.0126 |
| logs/char/lateral-low/rest | 42 | LEX(done+1.0s -> clearance -> effort) | (2.815, 'axisN_side_45deg', 'ring80mm_6of8') | 7.99 | 82.1 | 0.58 | -1.3 | 0.0784 |
| logs/char/lateral-low/rest | 42 | SAFE_EARLIEST (clear >= 80 mm) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | J_T+C | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | J7 - V | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | J7 - E,L | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/lateral-low/rest | 42 | J7 - Q,K | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char/longitudinal/moving | 212 | J7 | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/moving | 212 | EARLIEST_TAU | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/moving | 212 | EARLIEST_DONE | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/moving | 212 | LEX(done+0.2s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/moving | 212 | LEX(done+0.5s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/moving | 212 | LEX(done+1.0s -> clearance -> effort) | (3.25, 'axisP_side_337deg', 'ring140mm_4of8') | 8.48 | 80.9 | 0.75 | -1.5 | 0.0555 |
| logs/char/longitudinal/moving | 212 | SAFE_EARLIEST (clear >= 80 mm) | (3.25, 'axisP_side_337deg', 'ring80mm_1of8') | 8.48 | 80.8 | 0.74 | -1.3 | 0.0224 |
| logs/char/longitudinal/moving | 212 | J_T+C | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/moving | 212 | J7 - V | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/moving | 212 | J7 - E,L | (3.25, 'axisP_side_337deg', 'ring140mm_2of8') | 8.48 | 80.6 | 0.75 | -1.2 | 0.0040 |
| logs/char/longitudinal/moving | 212 | J7 - Q,K | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0000 |
| logs/char/longitudinal/rest | 43 | J7 | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | EARLIEST_TAU | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | EARLIEST_DONE | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | LEX(done+0.2s -> clearance -> effort) | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | LEX(done+0.5s -> clearance -> effort) | (2.435, 'axisP_side_337deg', 'ring80mm_1of8') | 7.64 | 80.9 | 0.29 | 0.0 | 0.0062 |
| logs/char/longitudinal/rest | 43 | LEX(done+1.0s -> clearance -> effort) | (2.959, 'axisP_side_68deg', 'ring80mm_6of8') | 8.14 | 81.8 | 0.78 | -0.8 | 0.0254 |
| logs/char/longitudinal/rest | 43 | SAFE_EARLIEST (clear >= 80 mm) | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | J_T+C | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | J7 - V | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | J7 - E,L | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/longitudinal/rest | 43 | J7 - Q,K | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char/near-ground/moving | 81 | J7 | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char/near-ground/moving | 81 | EARLIEST_TAU | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char/near-ground/moving | 81 | EARLIEST_DONE | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char/near-ground/moving | 81 | LEX(done+0.2s -> clearance -> effort) | (3.25, 'axisP_side_45deg', 'ring80mm_0of8') | 8.44 | 74.2 | 0.02 | 0.7 | 0.0169 |
| logs/char/near-ground/moving | 81 | LEX(done+0.5s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of8') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0923 |
| logs/char/near-ground/moving | 81 | LEX(done+1.0s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of8') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0923 |
| logs/char/near-ground/moving | 81 | SAFE_EARLIEST (clear >= 80 mm) | (8.0, 'axisP_side_247deg', 'direct') | 13.46 | 82.5 | 5.04 | -7.6 | 0.3260 |
| logs/char/near-ground/moving | 81 | J_T+C | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char/near-ground/moving | 81 | J7 - V | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char/near-ground/moving | 81 | J7 - E,L | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char/near-ground/moving | 81 | J7 - Q,K | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char/near-ground/rest | 13 | J7 | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char/near-ground/rest | 13 | EARLIEST_TAU | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0529 |
| logs/char/near-ground/rest | 13 | EARLIEST_DONE | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0529 |
| logs/char/near-ground/rest | 13 | LEX(done+0.2s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char/near-ground/rest | 13 | LEX(done+0.5s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char/near-ground/rest | 13 | LEX(done+1.0s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char/near-ground/rest | 13 | SAFE_EARLIEST (clear >= 80 mm) | (3.133, 'axisP_side_45deg', 'ring80mm_1of8') | 8.33 | 82.8 | 0.11 | 0.1 | 0.0104 |
| logs/char/near-ground/rest | 13 | J_T+C | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char/near-ground/rest | 13 | J7 - V | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char/near-ground/rest | 13 | J7 - E,L | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char/near-ground/rest | 13 | J7 - Q,K | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_G/diagonal/moving | 995 | J7 | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 9.35 | 72.4 | 0.05 | 4.2 | 0.0000 |
| logs/char_G/diagonal/moving | 995 | EARLIEST_TAU | (4.15, 'axisP_side_343deg', 'ring80mm_2of8') | 9.31 | 73.2 | 0.00 | 3.4 | 0.0069 |
| logs/char_G/diagonal/moving | 995 | EARLIEST_DONE | (4.15, 'axisP_side_343deg', 'ring80mm_2of8') | 9.31 | 73.2 | 0.00 | 3.4 | 0.0069 |
| logs/char_G/diagonal/moving | 995 | LEX(done+0.2s -> clearance -> effort) | (4.15, 'axisP_side_326deg', 'ring140mm_3of8') | 9.48 | 76.5 | 0.18 | 0.1 | 0.0498 |
| logs/char_G/diagonal/moving | 995 | LEX(done+0.5s -> clearance -> effort) | (4.15, 'axisP_side_326deg', 'ring140mm_3of8') | 9.48 | 76.5 | 0.18 | 0.1 | 0.0498 |
| logs/char_G/diagonal/moving | 995 | LEX(done+1.0s -> clearance -> effort) | (5.05, 'axisP_side_321deg', 'ring140mm_3of8') | 10.28 | 76.9 | 0.98 | -0.4 | 0.1091 |
| logs/char_G/diagonal/moving | 995 | SAFE_EARLIEST (clear >= 80 mm) | (6.4, 'axisP_side_51deg', 'direct') | 11.57 | 82.7 | 2.27 | -6.2 | 0.2074 |
| logs/char_G/diagonal/moving | 995 | J_T+C | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.03 | 0.9 | 0.0335 |
| logs/char_G/diagonal/moving | 995 | J7 - V | (4.15, 'axisP_side_343deg', 'ring80mm_2of8') | 9.31 | 73.2 | 0.00 | 3.4 | 0.0069 |
| logs/char_G/diagonal/moving | 995 | J7 - E,L | (4.15, 'axisP_side_332deg', 'ring140mm_2of8') | 9.38 | 75.4 | 0.08 | 1.1 | 0.0016 |
| logs/char_G/diagonal/moving | 995 | J7 - Q,K | (4.15, 'axisP_side_332deg', 'ring140mm_2of8') | 9.38 | 75.4 | 0.08 | 1.1 | 0.0016 |
| logs/char_G/diagonal/rest | 66 | J7 | (2.454, 'axisP_side_343deg', 'ring80mm_1of8') | 7.66 | 80.9 | 0.50 | 0.0 | 0.0000 |
| logs/char_G/diagonal/rest | 66 | EARLIEST_TAU | (1.9, 'axisP_side_326deg', 'direct') | 7.26 | 79.8 | 0.10 | 1.2 | 0.0263 |
| logs/char_G/diagonal/rest | 66 | EARLIEST_DONE | (1.925, 'axisP_side_332deg', 'direct') | 7.16 | 79.9 | 0.00 | 1.0 | 0.0131 |
| logs/char_G/diagonal/rest | 66 | LEX(done+0.2s -> clearance -> effort) | (2.025, 'axisP_side_343deg', 'direct') | 7.21 | 80.3 | 0.05 | 0.6 | 0.0075 |
| logs/char_G/diagonal/rest | 66 | LEX(done+0.5s -> clearance -> effort) | (2.454, 'axisP_side_343deg', 'ring80mm_1of8') | 7.66 | 80.9 | 0.50 | 0.0 | 0.0000 |
| logs/char_G/diagonal/rest | 66 | LEX(done+1.0s -> clearance -> effort) | (2.454, 'axisP_side_343deg', 'ring80mm_1of8') | 7.66 | 80.9 | 0.50 | 0.0 | 0.0000 |
| logs/char_G/diagonal/rest | 66 | SAFE_EARLIEST (clear >= 80 mm) | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.03 | 0.8 | 0.0089 |
| logs/char_G/diagonal/rest | 66 | J_T+C | (1.925, 'axisP_side_332deg', 'direct') | 7.16 | 79.9 | 0.00 | 1.0 | 0.0131 |
| logs/char_G/diagonal/rest | 66 | J7 - V | (2.025, 'axisP_side_343deg', 'direct') | 7.21 | 80.3 | 0.05 | 0.6 | 0.0075 |
| logs/char_G/diagonal/rest | 66 | J7 - E,L | (2.381, 'axisP_side_337deg', 'ring80mm_2of8') | 7.56 | 80.5 | 0.41 | 0.5 | 0.0030 |
| logs/char_G/diagonal/rest | 66 | J7 - Q,K | (2.454, 'axisP_side_343deg', 'ring80mm_1of8') | 7.66 | 80.9 | 0.50 | 0.0 | 0.0000 |
| logs/char_G/lateral-low/moving | 1200 | J7 | (3.7, 'axisN_side_337deg', 'direct') | 8.88 | 77.3 | 1.38 | -4.2 | 0.0000 |
| logs/char_G/lateral-low/moving | 1200 | EARLIEST_TAU | (2.35, 'axisN_side_332deg', 'ring80mm_3of8') | 7.50 | 51.1 | 0.00 | 21.9 | 0.0361 |
| logs/char_G/lateral-low/moving | 1200 | EARLIEST_DONE | (2.35, 'axisN_side_332deg', 'ring80mm_3of8') | 7.50 | 51.1 | 0.00 | 21.9 | 0.0361 |
| logs/char_G/lateral-low/moving | 1200 | LEX(done+0.2s -> clearance -> effort) | (2.35, 'axisN_side_332deg', 'ring80mm_4of8') | 7.50 | 51.1 | 0.00 | 22.0 | 0.0351 |
| logs/char_G/lateral-low/moving | 1200 | LEX(done+0.5s -> clearance -> effort) | (2.8, 'axisN_side_51deg', 'ring80mm_5of8') | 7.97 | 73.1 | 0.47 | 0.0 | 0.0121 |
| logs/char_G/lateral-low/moving | 1200 | LEX(done+1.0s -> clearance -> effort) | (3.25, 'axisN_side_51deg', 'ring140mm_6of8') | 8.40 | 79.3 | 0.90 | -6.2 | 0.0225 |
| logs/char_G/lateral-low/moving | 1200 | SAFE_EARLIEST (clear >= 80 mm) | (3.7, 'axisN_side_45deg', 'ring140mm_6of8') | 8.87 | 80.2 | 1.37 | -7.1 | 0.0414 |
| logs/char_G/lateral-low/moving | 1200 | J_T+C | (2.8, 'axisN_side_51deg', 'ring80mm_5of8') | 7.97 | 73.1 | 0.47 | 0.0 | 0.0121 |
| logs/char_G/lateral-low/moving | 1200 | J7 - V | (3.7, 'axisN_side_337deg', 'direct') | 8.88 | 77.3 | 1.38 | -4.2 | 0.0000 |
| logs/char_G/lateral-low/moving | 1200 | J7 - E,L | (3.7, 'axisN_side_337deg', 'ring80mm_3of8') | 8.88 | 77.3 | 1.37 | -4.3 | 0.0007 |
| logs/char_G/lateral-low/moving | 1200 | J7 - Q,K | (2.8, 'axisN_side_51deg', 'ring80mm_5of8') | 7.97 | 73.1 | 0.47 | 0.0 | 0.0121 |
| logs/char_G/lateral-low/rest | 108 | J7 | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.02 | 0.1 | 0.0000 |
| logs/char_G/lateral-low/rest | 108 | EARLIEST_TAU | (2.175, 'axisN_side_332deg', 'direct') | 7.38 | 73.6 | 0.00 | 7.2 | 0.0198 |
| logs/char_G/lateral-low/rest | 108 | EARLIEST_DONE | (2.175, 'axisN_side_332deg', 'direct') | 7.38 | 73.6 | 0.00 | 7.2 | 0.0198 |
| logs/char_G/lateral-low/rest | 108 | LEX(done+0.2s -> clearance -> effort) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.02 | 0.1 | 0.0000 |
| logs/char_G/lateral-low/rest | 108 | LEX(done+0.5s -> clearance -> effort) | (2.458, 'axisN_side_337deg', 'ring80mm_3of8') | 7.64 | 80.8 | 0.26 | 0.0 | 0.0126 |
| logs/char_G/lateral-low/rest | 108 | LEX(done+1.0s -> clearance -> effort) | (2.815, 'axisN_side_45deg', 'ring80mm_6of8') | 7.99 | 82.1 | 0.61 | -1.3 | 0.0784 |
| logs/char_G/lateral-low/rest | 108 | SAFE_EARLIEST (clear >= 80 mm) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.02 | 0.1 | 0.0000 |
| logs/char_G/lateral-low/rest | 108 | J_T+C | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.02 | 0.1 | 0.0000 |
| logs/char_G/lateral-low/rest | 108 | J7 - V | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.02 | 0.1 | 0.0000 |
| logs/char_G/lateral-low/rest | 108 | J7 - E,L | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.02 | 0.1 | 0.0000 |
| logs/char_G/lateral-low/rest | 108 | J7 - Q,K | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.02 | 0.1 | 0.0000 |
| logs/char_G/longitudinal/moving | 962 | J7 | (3.25, 'axisP_side_343deg', 'ring140mm_1of8') | 8.45 | 81.1 | 0.72 | -1.6 | 0.0000 |
| logs/char_G/longitudinal/moving | 962 | EARLIEST_TAU | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0055 |
| logs/char_G/longitudinal/moving | 962 | EARLIEST_DONE | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0055 |
| logs/char_G/longitudinal/moving | 962 | LEX(done+0.2s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0055 |
| logs/char_G/longitudinal/moving | 962 | LEX(done+0.5s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0055 |
| logs/char_G/longitudinal/moving | 962 | LEX(done+1.0s -> clearance -> effort) | (3.25, 'axisP_side_343deg', 'ring140mm_1of8') | 8.45 | 81.1 | 0.72 | -1.6 | 0.0000 |
| logs/char_G/longitudinal/moving | 962 | SAFE_EARLIEST (clear >= 80 mm) | (3.25, 'axisP_side_343deg', 'ring140mm_1of8') | 8.45 | 81.1 | 0.72 | -1.6 | 0.0000 |
| logs/char_G/longitudinal/moving | 962 | J_T+C | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0055 |
| logs/char_G/longitudinal/moving | 962 | J7 - V | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0055 |
| logs/char_G/longitudinal/moving | 962 | J7 - E,L | (3.25, 'axisP_side_343deg', 'ring140mm_2of8') | 8.45 | 80.8 | 0.72 | -1.4 | 0.0008 |
| logs/char_G/longitudinal/moving | 962 | J7 - Q,K | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 0.0 | 0.0055 |
| logs/char_G/longitudinal/rest | 169 | J7 | (2.175, 'axisP_side_343deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0000 |
| logs/char_G/longitudinal/rest | 169 | EARLIEST_TAU | (2.125, 'axisP_side_332deg', 'direct') | 7.35 | 80.2 | 0.00 | 1.3 | 0.0027 |
| logs/char_G/longitudinal/rest | 169 | EARLIEST_DONE | (2.175, 'axisP_side_343deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0000 |
| logs/char_G/longitudinal/rest | 169 | LEX(done+0.2s -> clearance -> effort) | (2.3, 'axisP_side_354deg', 'direct') | 7.45 | 80.9 | 0.10 | 0.6 | 0.0057 |
| logs/char_G/longitudinal/rest | 169 | LEX(done+0.5s -> clearance -> effort) | (2.616, 'axisP_side_354deg', 'ring80mm_5of8') | 7.77 | 81.5 | 0.42 | 0.0 | 0.0319 |
| logs/char_G/longitudinal/rest | 169 | LEX(done+1.0s -> clearance -> effort) | (2.959, 'axisP_side_68deg', 'ring80mm_6of8') | 8.14 | 81.8 | 0.78 | -0.3 | 0.0258 |
| logs/char_G/longitudinal/rest | 169 | SAFE_EARLIEST (clear >= 80 mm) | (2.175, 'axisP_side_343deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0000 |
| logs/char_G/longitudinal/rest | 169 | J_T+C | (2.125, 'axisP_side_332deg', 'direct') | 7.35 | 80.2 | 0.00 | 1.3 | 0.0027 |
| logs/char_G/longitudinal/rest | 169 | J7 - V | (2.175, 'axisP_side_343deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0000 |
| logs/char_G/longitudinal/rest | 169 | J7 - E,L | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.2 | 0.0004 |
| logs/char_G/longitudinal/rest | 169 | J7 - Q,K | (2.175, 'axisP_side_343deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0000 |
| logs/char_G/near-ground/moving | 267 | J7 | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_G/near-ground/moving | 267 | EARLIEST_TAU | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_G/near-ground/moving | 267 | EARLIEST_DONE | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_G/near-ground/moving | 267 | LEX(done+0.2s -> clearance -> effort) | (3.25, 'axisP_side_45deg', 'ring80mm_0of8') | 8.44 | 74.2 | 0.02 | 0.7 | 0.0169 |
| logs/char_G/near-ground/moving | 267 | LEX(done+0.5s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of8') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0923 |
| logs/char_G/near-ground/moving | 267 | LEX(done+1.0s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of8') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0923 |
| logs/char_G/near-ground/moving | 267 | SAFE_EARLIEST (clear >= 80 mm) | (8.0, 'axisP_side_253deg', 'direct') | 13.40 | 82.7 | 4.98 | -7.7 | 0.3231 |
| logs/char_G/near-ground/moving | 267 | J_T+C | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_G/near-ground/moving | 267 | J7 - V | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_G/near-ground/moving | 267 | J7 - E,L | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_G/near-ground/moving | 267 | J7 - Q,K | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_G/near-ground/rest | 51 | J7 | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.13 | 0.3 | 0.0000 |
| logs/char_G/near-ground/rest | 51 | EARLIEST_TAU | (2.975, 'axisP_side_51deg', 'direct') | 8.20 | 56.3 | 0.00 | 26.6 | 0.0584 |
| logs/char_G/near-ground/rest | 51 | EARLIEST_DONE | (2.975, 'axisP_side_51deg', 'direct') | 8.20 | 56.3 | 0.00 | 26.6 | 0.0584 |
| logs/char_G/near-ground/rest | 51 | LEX(done+0.2s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.15 | 0.0 | 0.0022 |
| logs/char_G/near-ground/rest | 51 | LEX(done+0.5s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.15 | 0.0 | 0.0022 |
| logs/char_G/near-ground/rest | 51 | LEX(done+1.0s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.15 | 0.0 | 0.0022 |
| logs/char_G/near-ground/rest | 51 | SAFE_EARLIEST (clear >= 80 mm) | (3.133, 'axisP_side_45deg', 'ring80mm_1of8') | 8.33 | 82.8 | 0.13 | 0.1 | 0.0104 |
| logs/char_G/near-ground/rest | 51 | J_T+C | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.13 | 0.3 | 0.0000 |
| logs/char_G/near-ground/rest | 51 | J7 - V | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.13 | 0.3 | 0.0000 |
| logs/char_G/near-ground/rest | 51 | J7 - E,L | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.15 | 0.0 | 0.0022 |
| logs/char_G/near-ground/rest | 51 | J7 - Q,K | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.13 | 0.3 | 0.0000 |
| logs/char_R/diagonal/moving | 976 | J7 | (4.15, 'axisP_side_337deg', 'ring140mm_4of16') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char_R/diagonal/moving | 976 | EARLIEST_TAU | (4.15, 'axisP_side_337deg', 'ring80mm_6of16') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_R/diagonal/moving | 976 | EARLIEST_DONE | (4.15, 'axisP_side_337deg', 'ring80mm_6of16') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_R/diagonal/moving | 976 | LEX(done+0.2s -> clearance -> effort) | (4.15, 'axisP_side_337deg', 'ring80mm_6of16') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_R/diagonal/moving | 976 | LEX(done+0.5s -> clearance -> effort) | (4.6, 'axisP_side_337deg', 'ring140mm_6of16') | 9.78 | 75.7 | 0.45 | 0.0 | 0.0604 |
| logs/char_R/diagonal/moving | 976 | LEX(done+1.0s -> clearance -> effort) | (5.05, 'axisP_side_337deg', 'ring140mm_7of16') | 10.20 | 75.9 | 0.87 | -0.1 | 0.1086 |
| logs/char_R/diagonal/moving | 976 | SAFE_EARLIEST (clear >= 80 mm) | (5.95, 'axisP_side_45deg', 'ring40mm_10of16') | 11.14 | 86.5 | 1.81 | -10.8 | 0.1870 |
| logs/char_R/diagonal/moving | 976 | J_T+C | (4.15, 'axisP_side_337deg', 'ring80mm_6of16') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_R/diagonal/moving | 976 | J7 - V | (4.15, 'axisP_side_337deg', 'ring80mm_4of16') | 9.33 | 74.5 | 0.00 | 1.2 | 0.0094 |
| logs/char_R/diagonal/moving | 976 | J7 - E,L | (4.15, 'axisP_side_337deg', 'ring140mm_4of16') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char_R/diagonal/moving | 976 | J7 - Q,K | (4.15, 'axisP_side_337deg', 'ring140mm_3of16') | 9.35 | 71.9 | 0.02 | 3.8 | 0.0010 |
| logs/char_R/diagonal/rest | 47 | J7 | (2.381, 'axisP_side_337deg', 'ring80mm_3of16') | 7.56 | 80.6 | 0.38 | 0.4 | 0.0000 |
| logs/char_R/diagonal/rest | 47 | EARLIEST_TAU | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.9 | 0.0094 |
| logs/char_R/diagonal/rest | 47 | EARLIEST_DONE | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.9 | 0.0094 |
| logs/char_R/diagonal/rest | 47 | LEX(done+0.2s -> clearance -> effort) | (2.089, 'axisP_side_337deg', 'ring40mm_1of16') | 7.30 | 81.0 | 0.11 | 0.0 | 0.0112 |
| logs/char_R/diagonal/rest | 47 | LEX(done+0.5s -> clearance -> effort) | (2.089, 'axisP_side_337deg', 'ring40mm_1of16') | 7.30 | 81.0 | 0.11 | 0.0 | 0.0112 |
| logs/char_R/diagonal/rest | 47 | LEX(done+1.0s -> clearance -> effort) | (2.089, 'axisP_side_337deg', 'ring40mm_1of16') | 7.30 | 81.0 | 0.11 | 0.0 | 0.0112 |
| logs/char_R/diagonal/rest | 47 | SAFE_EARLIEST (clear >= 80 mm) | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.9 | 0.0094 |
| logs/char_R/diagonal/rest | 47 | J_T+C | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.9 | 0.0094 |
| logs/char_R/diagonal/rest | 47 | J7 - V | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.9 | 0.0094 |
| logs/char_R/diagonal/rest | 47 | J7 - E,L | (2.381, 'axisP_side_337deg', 'ring80mm_3of16') | 7.56 | 80.6 | 0.38 | 0.4 | 0.0000 |
| logs/char_R/diagonal/rest | 47 | J7 - Q,K | (2.381, 'axisP_side_337deg', 'ring80mm_3of16') | 7.56 | 80.6 | 0.38 | 0.4 | 0.0000 |
| logs/char_R/lateral-low/moving | 1029 | J7 | (3.7, 'axisN_side_337deg', 'ring40mm_7of16') | 8.88 | 77.3 | 0.41 | 4.9 | 0.0000 |
| logs/char_R/lateral-low/moving | 1029 | EARLIEST_TAU | (3.25, 'axisN_side_68deg', 'ring80mm_10of16') | 8.47 | 47.9 | 0.00 | 34.3 | 0.1330 |
| logs/char_R/lateral-low/moving | 1029 | EARLIEST_DONE | (3.25, 'axisN_side_68deg', 'ring80mm_10of16') | 8.47 | 47.9 | 0.00 | 34.3 | 0.1330 |
| logs/char_R/lateral-low/moving | 1029 | LEX(done+0.2s -> clearance -> effort) | (3.25, 'axisN_side_68deg', 'ring80mm_10of16') | 8.47 | 47.9 | 0.00 | 34.3 | 0.1330 |
| logs/char_R/lateral-low/moving | 1029 | LEX(done+0.5s -> clearance -> effort) | (3.7, 'axisN_side_45deg', 'ring200mm_1of16') | 8.87 | 82.2 | 0.40 | 0.0 | 0.3884 |
| logs/char_R/lateral-low/moving | 1029 | LEX(done+1.0s -> clearance -> effort) | (3.7, 'axisN_side_45deg', 'ring200mm_1of16') | 8.87 | 82.2 | 0.40 | 0.0 | 0.3884 |
| logs/char_R/lateral-low/moving | 1029 | SAFE_EARLIEST (clear >= 80 mm) | (3.7, 'axisN_side_45deg', 'ring140mm_13of16') | 8.85 | 80.9 | 0.38 | 1.2 | 0.0549 |
| logs/char_R/lateral-low/moving | 1029 | J_T+C | (3.7, 'axisN_side_45deg', 'ring140mm_13of16') | 8.85 | 80.9 | 0.38 | 1.2 | 0.0549 |
| logs/char_R/lateral-low/moving | 1029 | J7 - V | (3.7, 'axisN_side_337deg', 'ring40mm_7of16') | 8.88 | 77.3 | 0.41 | 4.9 | 0.0000 |
| logs/char_R/lateral-low/moving | 1029 | J7 - E,L | (3.7, 'axisN_side_337deg', 'ring200mm_5of16') | 8.88 | 75.1 | 0.41 | 7.1 | 0.0191 |
| logs/char_R/lateral-low/moving | 1029 | J7 - Q,K | (3.7, 'axisN_side_45deg', 'ring80mm_11of16') | 8.87 | 78.2 | 0.40 | 3.9 | 0.0439 |
| logs/char_R/lateral-low/rest | 152 | J7 | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/lateral-low/rest | 152 | EARLIEST_TAU | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/lateral-low/rest | 152 | EARLIEST_DONE | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/lateral-low/rest | 152 | LEX(done+0.2s -> clearance -> effort) | (2.287, 'axisN_side_337deg', 'ring40mm_7of16') | 7.46 | 80.8 | 0.06 | 0.1 | 0.0009 |
| logs/char_R/lateral-low/rest | 152 | LEX(done+0.5s -> clearance -> effort) | (2.458, 'axisN_side_337deg', 'ring80mm_7of16') | 7.64 | 80.8 | 0.23 | 0.0 | 0.0121 |
| logs/char_R/lateral-low/rest | 152 | LEX(done+1.0s -> clearance -> effort) | (3.228, 'axisN_side_45deg', 'ring140mm_13of16') | 8.38 | 82.2 | 0.98 | -1.3 | 0.1097 |
| logs/char_R/lateral-low/rest | 152 | SAFE_EARLIEST (clear >= 80 mm) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/lateral-low/rest | 152 | J_T+C | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/lateral-low/rest | 152 | J7 - V | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/lateral-low/rest | 152 | J7 - E,L | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/lateral-low/rest | 152 | J7 - Q,K | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_R/longitudinal/moving | 787 | J7 | (2.35, 'axisP_side_337deg', 'ring40mm_3of16') | 7.73 | 80.4 | 0.00 | 0.3 | 0.0000 |
| logs/char_R/longitudinal/moving | 787 | EARLIEST_TAU | (2.35, 'axisP_side_337deg', 'ring40mm_1of16') | 7.73 | 80.7 | 0.00 | 0.0 | 0.0004 |
| logs/char_R/longitudinal/moving | 787 | EARLIEST_DONE | (2.35, 'axisP_side_337deg', 'ring40mm_1of16') | 7.73 | 80.7 | 0.00 | 0.0 | 0.0004 |
| logs/char_R/longitudinal/moving | 787 | LEX(done+0.2s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'ring40mm_1of16') | 7.73 | 80.7 | 0.00 | 0.0 | 0.0004 |
| logs/char_R/longitudinal/moving | 787 | LEX(done+0.5s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'ring40mm_1of16') | 7.73 | 80.7 | 0.00 | 0.0 | 0.0004 |
| logs/char_R/longitudinal/moving | 787 | LEX(done+1.0s -> clearance -> effort) | (3.25, 'axisP_side_337deg', 'ring40mm_2of16') | 8.48 | 80.9 | 0.75 | -0.2 | 0.0431 |
| logs/char_R/longitudinal/moving | 787 | SAFE_EARLIEST (clear >= 80 mm) | (2.35, 'axisP_side_337deg', 'ring40mm_1of16') | 7.73 | 80.7 | 0.00 | 0.0 | 0.0004 |
| logs/char_R/longitudinal/moving | 787 | J_T+C | (2.35, 'axisP_side_337deg', 'ring40mm_10of16') | 7.73 | 80.2 | 0.00 | 0.5 | 0.0043 |
| logs/char_R/longitudinal/moving | 787 | J7 - V | (2.35, 'axisP_side_337deg', 'ring40mm_3of16') | 7.73 | 80.4 | 0.00 | 0.3 | 0.0000 |
| logs/char_R/longitudinal/moving | 787 | J7 - E,L | (3.25, 'axisP_side_337deg', 'ring140mm_4of16') | 8.48 | 80.6 | 0.75 | 0.1 | 0.0103 |
| logs/char_R/longitudinal/moving | 787 | J7 - Q,K | (2.35, 'axisP_side_337deg', 'ring40mm_1of16') | 7.73 | 80.7 | 0.00 | 0.0 | 0.0004 |
| logs/char_R/longitudinal/rest | 166 | J7 | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.8 | 0.0000 |
| logs/char_R/longitudinal/rest | 166 | EARLIEST_TAU | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.8 | 0.0000 |
| logs/char_R/longitudinal/rest | 166 | EARLIEST_DONE | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.8 | 0.0000 |
| logs/char_R/longitudinal/rest | 166 | LEX(done+0.2s -> clearance -> effort) | (2.227, 'axisP_side_337deg', 'ring40mm_2of16') | 7.43 | 81.1 | 0.08 | 0.0 | 0.0011 |
| logs/char_R/longitudinal/rest | 166 | LEX(done+0.5s -> clearance -> effort) | (2.227, 'axisP_side_337deg', 'ring40mm_2of16') | 7.43 | 81.1 | 0.08 | 0.0 | 0.0011 |
| logs/char_R/longitudinal/rest | 166 | LEX(done+1.0s -> clearance -> effort) | (2.959, 'axisP_side_68deg', 'ring80mm_11of16') | 8.14 | 81.9 | 0.78 | -0.8 | 0.0308 |
| logs/char_R/longitudinal/rest | 166 | SAFE_EARLIEST (clear >= 80 mm) | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.8 | 0.0000 |
| logs/char_R/longitudinal/rest | 166 | J_T+C | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.8 | 0.0000 |
| logs/char_R/longitudinal/rest | 166 | J7 - V | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.8 | 0.0000 |
| logs/char_R/longitudinal/rest | 166 | J7 - E,L | (2.902, 'axisP_side_337deg', 'ring140mm_3of16') | 8.10 | 80.8 | 0.75 | 0.3 | 0.0093 |
| logs/char_R/longitudinal/rest | 166 | J7 - Q,K | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.8 | 0.0000 |
| logs/char_R/near-ground/moving | 290 | J7 | (3.25, 'axisP_side_45deg', 'ring80mm_14of16') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_R/near-ground/moving | 290 | EARLIEST_TAU | (3.25, 'axisP_side_45deg', 'ring80mm_2of16') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_R/near-ground/moving | 290 | EARLIEST_DONE | (3.25, 'axisP_side_45deg', 'ring80mm_2of16') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_R/near-ground/moving | 290 | LEX(done+0.2s -> clearance -> effort) | (3.25, 'axisP_side_45deg', 'ring80mm_1of16') | 8.44 | 74.6 | 0.02 | 0.3 | 0.0241 |
| logs/char_R/near-ground/moving | 290 | LEX(done+0.5s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of16') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0810 |
| logs/char_R/near-ground/moving | 290 | LEX(done+1.0s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of16') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0810 |
| logs/char_R/near-ground/moving | 290 | SAFE_EARLIEST (clear >= 80 mm) | (8.0, 'axisP_side_247deg', 'ring200mm_13of16') | 13.44 | 81.9 | 5.02 | -7.0 | 0.3181 |
| logs/char_R/near-ground/moving | 290 | J_T+C | (3.25, 'axisP_side_45deg', 'ring80mm_1of16') | 8.44 | 74.6 | 0.02 | 0.3 | 0.0241 |
| logs/char_R/near-ground/moving | 290 | J7 - V | (3.25, 'axisP_side_45deg', 'ring80mm_14of16') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_R/near-ground/moving | 290 | J7 - E,L | (3.25, 'axisP_side_45deg', 'ring80mm_14of16') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_R/near-ground/moving | 290 | J7 - Q,K | (3.25, 'axisP_side_45deg', 'ring80mm_15of16') | 8.44 | 72.9 | 0.02 | 2.0 | 0.0086 |
| logs/char_R/near-ground/rest | 51 | J7 | (3.034, 'axisP_side_45deg', 'ring40mm_1of16') | 8.25 | 79.0 | 0.03 | 3.9 | 0.0000 |
| logs/char_R/near-ground/rest | 51 | EARLIEST_TAU | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0628 |
| logs/char_R/near-ground/rest | 51 | EARLIEST_DONE | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0628 |
| logs/char_R/near-ground/rest | 51 | LEX(done+0.2s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_4of16') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0121 |
| logs/char_R/near-ground/rest | 51 | LEX(done+0.5s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_4of16') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0121 |
| logs/char_R/near-ground/rest | 51 | LEX(done+1.0s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_4of16') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0121 |
| logs/char_R/near-ground/rest | 51 | SAFE_EARLIEST (clear >= 80 mm) | (3.133, 'axisP_side_45deg', 'ring80mm_3of16') | 8.33 | 82.9 | 0.11 | 0.0 | 0.0182 |
| logs/char_R/near-ground/rest | 51 | J_T+C | (3.034, 'axisP_side_45deg', 'ring40mm_2of16') | 8.25 | 79.2 | 0.03 | 3.7 | 0.0009 |
| logs/char_R/near-ground/rest | 51 | J7 - V | (3.034, 'axisP_side_45deg', 'ring40mm_1of16') | 8.25 | 79.0 | 0.03 | 3.9 | 0.0000 |
| logs/char_R/near-ground/rest | 51 | J7 - E,L | (3.034, 'axisP_side_45deg', 'ring40mm_2of16') | 8.25 | 79.2 | 0.03 | 3.7 | 0.0009 |
| logs/char_R/near-ground/rest | 51 | J7 - Q,K | (3.034, 'axisP_side_45deg', 'ring40mm_1of16') | 8.25 | 79.0 | 0.03 | 3.9 | 0.0000 |
| logs/char_stretch/diagonal/moving | 385 | J7 | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char_stretch/diagonal/moving | 385 | EARLIEST_TAU | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_stretch/diagonal/moving | 385 | EARLIEST_DONE | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_stretch/diagonal/moving | 385 | LEX(done+0.2s -> clearance -> effort) | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_stretch/diagonal/moving | 385 | LEX(done+0.5s -> clearance -> effort) | (4.6, 'axisP_side_337deg', 'ring140mm_3of8') | 9.78 | 75.7 | 0.45 | 0.0 | 0.0604 |
| logs/char_stretch/diagonal/moving | 385 | LEX(done+1.0s -> clearance -> effort) | (5.05, 'axisP_side_337deg', 'ring140mm_3of8') | 10.20 | 75.8 | 0.87 | -0.1 | 0.0956 |
| logs/char_stretch/diagonal/moving | 385 | SAFE_EARLIEST (clear >= 80 mm) | (6.85, 'axisP_side_45deg', 'directx105') | 12.02 | 83.6 | 2.69 | -7.9 | 0.2424 |
| logs/char_stretch/diagonal/moving | 385 | J_T+C | (4.15, 'axisP_side_337deg', 'ring80mm_3of8') | 9.33 | 75.6 | 0.00 | 0.1 | 0.0335 |
| logs/char_stretch/diagonal/moving | 385 | J7 - V | (4.15, 'axisP_side_337deg', 'ring80mm_2of8') | 9.33 | 74.5 | 0.00 | 1.2 | 0.0094 |
| logs/char_stretch/diagonal/moving | 385 | J7 - E,L | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char_stretch/diagonal/moving | 385 | J7 - Q,K | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 9.35 | 72.4 | 0.02 | 3.4 | 0.0000 |
| logs/char_stretch/diagonal/rest | 23 | J7 | (2.938, 'axisP_side_337deg', 'directx150') | 8.14 | 81.4 | 0.96 | -0.2 | 0.0000 |
| logs/char_stretch/diagonal/rest | 23 | EARLIEST_TAU | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 1.0 | 0.0183 |
| logs/char_stretch/diagonal/rest | 23 | EARLIEST_DONE | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 1.0 | 0.0183 |
| logs/char_stretch/diagonal/rest | 23 | LEX(done+0.2s -> clearance -> effort) | (2.167, 'axisP_side_337deg', 'directx110') | 7.38 | 81.0 | 0.19 | 0.2 | 0.0194 |
| logs/char_stretch/diagonal/rest | 23 | LEX(done+0.5s -> clearance -> effort) | (2.36, 'axisP_side_337deg', 'directx120') | 7.54 | 81.2 | 0.36 | 0.0 | 0.0099 |
| logs/char_stretch/diagonal/rest | 23 | LEX(done+1.0s -> clearance -> effort) | (2.938, 'axisP_side_337deg', 'directx150') | 8.14 | 81.4 | 0.96 | -0.2 | 0.0000 |
| logs/char_stretch/diagonal/rest | 23 | SAFE_EARLIEST (clear >= 80 mm) | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 1.0 | 0.0183 |
| logs/char_stretch/diagonal/rest | 23 | J_T+C | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 1.0 | 0.0183 |
| logs/char_stretch/diagonal/rest | 23 | J7 - V | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 1.0 | 0.0183 |
| logs/char_stretch/diagonal/rest | 23 | J7 - E,L | (2.381, 'axisP_side_337deg', 'ring80mm_2of8') | 7.56 | 80.5 | 0.38 | 0.7 | 0.0124 |
| logs/char_stretch/diagonal/rest | 23 | J7 - Q,K | (2.649, 'axisP_side_337deg', 'directx135') | 7.85 | 81.3 | 0.67 | -0.1 | 0.0015 |
| logs/char_stretch/lateral-low/moving | 431 | J7 | (3.7, 'axisN_side_337deg', 'directx170') | 8.88 | 77.3 | 0.41 | 2.9 | 0.0000 |
| logs/char_stretch/lateral-low/moving | 431 | EARLIEST_TAU | (3.25, 'axisN_side_68deg', 'directx110') | 8.47 | 68.8 | 0.00 | 11.3 | 0.1087 |
| logs/char_stretch/lateral-low/moving | 431 | EARLIEST_DONE | (3.25, 'axisN_side_68deg', 'directx110') | 8.47 | 68.8 | 0.00 | 11.3 | 0.1087 |
| logs/char_stretch/lateral-low/moving | 431 | LEX(done+0.2s -> clearance -> effort) | (3.25, 'axisN_side_68deg', 'directx110') | 8.47 | 68.8 | 0.00 | 11.3 | 0.1087 |
| logs/char_stretch/lateral-low/moving | 431 | LEX(done+0.5s -> clearance -> effort) | (3.7, 'axisN_side_45deg', 'ring140mm_6of8') | 8.87 | 80.2 | 0.40 | 0.0 | 0.0892 |
| logs/char_stretch/lateral-low/moving | 431 | LEX(done+1.0s -> clearance -> effort) | (4.15, 'axisN_side_337deg', 'directx170') | 9.33 | 81.3 | 0.86 | -1.1 | 0.0093 |
| logs/char_stretch/lateral-low/moving | 431 | SAFE_EARLIEST (clear >= 80 mm) | (3.7, 'axisN_side_45deg', 'ring140mm_6of8') | 8.87 | 80.2 | 0.40 | 0.0 | 0.0892 |
| logs/char_stretch/lateral-low/moving | 431 | J_T+C | (3.7, 'axisN_side_45deg', 'ring140mm_1of8') | 8.85 | 79.9 | 0.38 | 0.3 | 0.3963 |
| logs/char_stretch/lateral-low/moving | 431 | J7 - V | (3.7, 'axisN_side_337deg', 'directx170') | 8.88 | 77.3 | 0.41 | 2.9 | 0.0000 |
| logs/char_stretch/lateral-low/moving | 431 | J7 - E,L | (3.7, 'axisN_side_337deg', 'directx170') | 8.88 | 77.3 | 0.41 | 2.9 | 0.0000 |
| logs/char_stretch/lateral-low/moving | 431 | J7 - Q,K | (3.7, 'axisN_side_337deg', 'directx170') | 8.88 | 77.3 | 0.41 | 2.9 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | J7 | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | EARLIEST_TAU | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | EARLIEST_DONE | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | LEX(done+0.2s -> clearance -> effort) | (2.334, 'axisN_side_337deg', 'directx105') | 7.51 | 80.8 | 0.11 | 0.1 | 0.0029 |
| logs/char_stretch/lateral-low/rest | 64 | LEX(done+0.5s -> clearance -> effort) | (2.458, 'axisN_side_337deg', 'ring80mm_3of8') | 7.64 | 80.8 | 0.23 | 0.0 | 0.0126 |
| logs/char_stretch/lateral-low/rest | 64 | LEX(done+1.0s -> clearance -> effort) | (2.815, 'axisN_side_45deg', 'ring80mm_6of8') | 7.99 | 82.1 | 0.58 | -1.3 | 0.0784 |
| logs/char_stretch/lateral-low/rest | 64 | SAFE_EARLIEST (clear >= 80 mm) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | J_T+C | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | J7 - V | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | J7 - E,L | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/lateral-low/rest | 64 | J7 - Q,K | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | J7 | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | EARLIEST_TAU | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | EARLIEST_DONE | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | LEX(done+0.2s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | LEX(done+0.5s -> clearance -> effort) | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | LEX(done+1.0s -> clearance -> effort) | (3.25, 'axisP_side_337deg', 'directx150') | 8.48 | 81.4 | 0.74 | -0.6 | 0.0001 |
| logs/char_stretch/longitudinal/moving | 302 | SAFE_EARLIEST (clear >= 80 mm) | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | J_T+C | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | J7 - V | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/moving | 302 | J7 - E,L | (3.25, 'axisP_side_337deg', 'ring140mm_2of8') | 8.48 | 80.6 | 0.75 | 0.2 | 0.0298 |
| logs/char_stretch/longitudinal/moving | 302 | J7 - Q,K | (2.35, 'axisP_side_337deg', 'directx120') | 7.73 | 80.8 | 0.00 | 0.0 | 0.0000 |
| logs/char_stretch/longitudinal/rest | 65 | J7 | (3.2, 'axisP_side_337deg', 'directx150') | 8.40 | 81.6 | 1.05 | -0.2 | 0.0000 |
| logs/char_stretch/longitudinal/rest | 65 | EARLIEST_TAU | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0135 |
| logs/char_stretch/longitudinal/rest | 65 | EARLIEST_DONE | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0135 |
| logs/char_stretch/longitudinal/rest | 65 | LEX(done+0.2s -> clearance -> effort) | (2.255, 'axisP_side_337deg', 'directx105') | 7.46 | 81.0 | 0.11 | 0.4 | 0.0139 |
| logs/char_stretch/longitudinal/rest | 65 | LEX(done+0.5s -> clearance -> effort) | (2.57, 'axisP_side_337deg', 'directx120') | 7.77 | 81.4 | 0.42 | 0.0 | 0.0165 |
| logs/char_stretch/longitudinal/rest | 65 | LEX(done+1.0s -> clearance -> effort) | (2.959, 'axisP_side_68deg', 'ring80mm_6of8') | 8.14 | 81.8 | 0.78 | -0.4 | 0.0389 |
| logs/char_stretch/longitudinal/rest | 65 | SAFE_EARLIEST (clear >= 80 mm) | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0135 |
| logs/char_stretch/longitudinal/rest | 65 | J_T+C | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0135 |
| logs/char_stretch/longitudinal/rest | 65 | J7 - V | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0135 |
| logs/char_stretch/longitudinal/rest | 65 | J7 - E,L | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0135 |
| logs/char_stretch/longitudinal/rest | 65 | J7 - Q,K | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 1.1 | 0.0135 |
| logs/char_stretch/near-ground/moving | 104 | J7 | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_stretch/near-ground/moving | 104 | EARLIEST_TAU | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_stretch/near-ground/moving | 104 | EARLIEST_DONE | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_stretch/near-ground/moving | 104 | LEX(done+0.2s -> clearance -> effort) | (3.25, 'axisP_side_45deg', 'ring80mm_0of8') | 8.44 | 74.2 | 0.02 | 0.7 | 0.0169 |
| logs/char_stretch/near-ground/moving | 104 | LEX(done+0.5s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of8') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0923 |
| logs/char_stretch/near-ground/moving | 104 | LEX(done+1.0s -> clearance -> effort) | (3.7, 'axisP_side_45deg', 'ring140mm_1of8') | 8.91 | 75.0 | 0.49 | 0.0 | 0.0923 |
| logs/char_stretch/near-ground/moving | 104 | SAFE_EARLIEST (clear >= 80 mm) | (8.0, 'axisP_side_247deg', 'direct') | 13.46 | 82.5 | 5.04 | -7.6 | 0.3260 |
| logs/char_stretch/near-ground/moving | 104 | J_T+C | (3.25, 'axisP_side_45deg', 'ring80mm_1of8') | 8.42 | 74.0 | 0.00 | 0.9 | 0.0279 |
| logs/char_stretch/near-ground/moving | 104 | J7 - V | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_stretch/near-ground/moving | 104 | J7 - E,L | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_stretch/near-ground/moving | 104 | J7 - Q,K | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 8.42 | 70.6 | 0.00 | 4.3 | 0.0000 |
| logs/char_stretch/near-ground/rest | 19 | J7 | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_stretch/near-ground/rest | 19 | EARLIEST_TAU | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0529 |
| logs/char_stretch/near-ground/rest | 19 | EARLIEST_DONE | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0529 |
| logs/char_stretch/near-ground/rest | 19 | LEX(done+0.2s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_stretch/near-ground/rest | 19 | LEX(done+0.5s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_stretch/near-ground/rest | 19 | LEX(done+1.0s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_stretch/near-ground/rest | 19 | SAFE_EARLIEST (clear >= 80 mm) | (3.133, 'axisP_side_45deg', 'ring80mm_1of8') | 8.33 | 82.8 | 0.11 | 0.1 | 0.0104 |
| logs/char_stretch/near-ground/rest | 19 | J_T+C | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_stretch/near-ground/rest | 19 | J7 - V | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_stretch/near-ground/rest | 19 | J7 - E,L | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_stretch/near-ground/rest | 19 | J7 - Q,K | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_T/diagonal/moving | 3338 | J7 | (3.9, 'axisP_side_337deg', 'ring80mm_2of8') | 9.11 | 74.5 | 0.00 | 1.2 | 0.0000 |
| logs/char_T/diagonal/moving | 3338 | EARLIEST_TAU | (3.9, 'axisP_side_337deg', 'direct') | 9.11 | 69.1 | 0.00 | 6.6 | 0.0439 |
| logs/char_T/diagonal/moving | 3338 | EARLIEST_DONE | (3.9, 'axisP_side_337deg', 'direct') | 9.11 | 69.1 | 0.00 | 6.6 | 0.0439 |
| logs/char_T/diagonal/moving | 3338 | LEX(done+0.2s -> clearance -> effort) | (4.1, 'axisP_side_337deg', 'ring80mm_3of8') | 9.28 | 75.6 | 0.18 | 0.1 | 0.0361 |
| logs/char_T/diagonal/moving | 3338 | LEX(done+0.5s -> clearance -> effort) | (4.25, 'axisP_side_337deg', 'ring80mm_3of8') | 9.43 | 75.7 | 0.33 | 0.0 | 0.0468 |
| logs/char_T/diagonal/moving | 3338 | LEX(done+1.0s -> clearance -> effort) | (4.8, 'axisP_side_337deg', 'ring140mm_3of8') | 9.98 | 75.8 | 0.87 | -0.1 | 0.0826 |
| logs/char_T/diagonal/moving | 3338 | SAFE_EARLIEST (clear >= 80 mm) | (7.55, 'axisP_side_23deg', 'direct') | 12.76 | 80.3 | 3.65 | -4.6 | 0.3341 |
| logs/char_T/diagonal/moving | 3338 | J_T+C | (3.9, 'axisP_side_337deg', 'ring80mm_3of8') | 9.11 | 75.6 | 0.00 | 0.1 | 0.0246 |
| logs/char_T/diagonal/moving | 3338 | J7 - V | (3.9, 'axisP_side_337deg', 'ring80mm_2of8') | 9.11 | 74.5 | 0.00 | 1.2 | 0.0000 |
| logs/char_T/diagonal/moving | 3338 | J7 - E,L | (4.2, 'axisP_side_337deg', 'ring140mm_2of8') | 9.40 | 74.2 | 0.30 | 1.4 | 0.0043 |
| logs/char_T/diagonal/moving | 3338 | J7 - Q,K | (3.9, 'axisP_side_337deg', 'ring80mm_2of8') | 9.11 | 74.5 | 0.00 | 1.2 | 0.0000 |
| logs/char_T/diagonal/rest | 17 | J7 | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char_T/diagonal/rest | 17 | EARLIEST_TAU | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char_T/diagonal/rest | 17 | EARLIEST_DONE | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char_T/diagonal/rest | 17 | LEX(done+0.2s -> clearance -> effort) | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char_T/diagonal/rest | 17 | LEX(done+0.5s -> clearance -> effort) | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char_T/diagonal/rest | 17 | LEX(done+1.0s -> clearance -> effort) | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char_T/diagonal/rest | 17 | SAFE_EARLIEST (clear >= 80 mm) | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char_T/diagonal/rest | 17 | J_T+C | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char_T/diagonal/rest | 17 | J7 - V | (1.975, 'axisP_side_337deg', 'direct') | 7.18 | 80.2 | 0.00 | 0.6 | 0.0061 |
| logs/char_T/diagonal/rest | 17 | J7 - E,L | (2.381, 'axisP_side_337deg', 'ring80mm_2of8') | 7.56 | 80.5 | 0.38 | 0.3 | 0.0002 |
| logs/char_T/diagonal/rest | 17 | J7 - Q,K | (2.381, 'axisP_side_337deg', 'ring80mm_1of8') | 7.59 | 80.8 | 0.41 | 0.0 | 0.0000 |
| logs/char_T/lateral-low/moving | 5131 | J7 | (3.45, 'axisN_side_337deg', 'direct') | 8.60 | 75.4 | 0.28 | 5.0 | 0.0000 |
| logs/char_T/lateral-low/moving | 5131 | EARLIEST_TAU | (3.1, 'axisN_side_68deg', 'ring80mm_5of8') | 8.32 | 49.5 | 0.00 | 30.9 | 0.1245 |
| logs/char_T/lateral-low/moving | 5131 | EARLIEST_DONE | (3.1, 'axisN_side_68deg', 'ring80mm_5of8') | 8.32 | 49.5 | 0.00 | 30.9 | 0.1245 |
| logs/char_T/lateral-low/moving | 5131 | LEX(done+0.2s -> clearance -> effort) | (3.1, 'axisN_side_68deg', 'ring80mm_5of8') | 8.32 | 49.5 | 0.00 | 30.9 | 0.1245 |
| logs/char_T/lateral-low/moving | 5131 | LEX(done+0.5s -> clearance -> effort) | (3.6, 'axisN_side_45deg', 'ring140mm_6of8') | 8.77 | 80.4 | 0.45 | 0.0 | 0.0453 |
| logs/char_T/lateral-low/moving | 5131 | LEX(done+1.0s -> clearance -> effort) | (4.1, 'axisN_side_337deg', 'ring140mm_2of8') | 9.28 | 80.8 | 0.96 | -0.4 | 0.0334 |
| logs/char_T/lateral-low/moving | 5131 | SAFE_EARLIEST (clear >= 80 mm) | (3.6, 'axisN_side_45deg', 'ring140mm_6of8') | 8.77 | 80.4 | 0.45 | 0.0 | 0.0453 |
| logs/char_T/lateral-low/moving | 5131 | J_T+C | (3.6, 'axisN_side_45deg', 'ring140mm_1of8') | 8.77 | 80.2 | 0.45 | 0.2 | 0.3456 |
| logs/char_T/lateral-low/moving | 5131 | J7 - V | (3.45, 'axisN_side_337deg', 'direct') | 8.60 | 75.4 | 0.28 | 5.0 | 0.0000 |
| logs/char_T/lateral-low/moving | 5131 | J7 - E,L | (3.45, 'axisN_side_337deg', 'direct') | 8.60 | 75.4 | 0.28 | 5.0 | 0.0000 |
| logs/char_T/lateral-low/moving | 5131 | J7 - Q,K | (3.45, 'axisN_side_337deg', 'direct') | 8.60 | 75.4 | 0.28 | 5.0 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | J7 | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | EARLIEST_TAU | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | EARLIEST_DONE | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | LEX(done+0.2s -> clearance -> effort) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | LEX(done+0.5s -> clearance -> effort) | (2.458, 'axisN_side_337deg', 'ring80mm_3of8') | 7.64 | 80.8 | 0.23 | 0.0 | 0.0126 |
| logs/char_T/lateral-low/rest | 42 | LEX(done+1.0s -> clearance -> effort) | (2.815, 'axisN_side_45deg', 'ring80mm_6of8') | 7.99 | 82.1 | 0.58 | -1.3 | 0.0784 |
| logs/char_T/lateral-low/rest | 42 | SAFE_EARLIEST (clear >= 80 mm) | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | J_T+C | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | J7 - V | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | J7 - E,L | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/lateral-low/rest | 42 | J7 - Q,K | (2.225, 'axisN_side_337deg', 'direct') | 7.40 | 80.7 | 0.00 | 0.1 | 0.0000 |
| logs/char_T/longitudinal/moving | 2011 | J7 | (2.85, 'axisP_side_337deg', 'ring80mm_1of8') | 8.08 | 80.7 | 0.35 | 0.1 | 0.0000 |
| logs/char_T/longitudinal/moving | 2011 | EARLIEST_TAU | (2.1, 'axisP_side_337deg', 'direct') | 8.16 | 79.6 | 0.43 | 1.1 | 0.0537 |
| logs/char_T/longitudinal/moving | 2011 | EARLIEST_DONE | (2.3, 'axisP_side_337deg', 'direct') | 7.73 | 79.6 | 0.00 | 1.1 | 0.0125 |
| logs/char_T/longitudinal/moving | 2011 | LEX(done+0.2s -> clearance -> effort) | (2.3, 'axisP_side_337deg', 'direct') | 7.73 | 79.6 | 0.00 | 1.1 | 0.0125 |
| logs/char_T/longitudinal/moving | 2011 | LEX(done+0.5s -> clearance -> effort) | (2.85, 'axisP_side_337deg', 'ring80mm_1of8') | 8.08 | 80.7 | 0.35 | 0.1 | 0.0000 |
| logs/char_T/longitudinal/moving | 2011 | LEX(done+1.0s -> clearance -> effort) | (3.3, 'axisP_side_337deg', 'ring140mm_1of8') | 8.53 | 80.9 | 0.79 | -0.2 | 0.0166 |
| logs/char_T/longitudinal/moving | 2011 | SAFE_EARLIEST (clear >= 80 mm) | (2.85, 'axisP_side_337deg', 'ring80mm_1of8') | 8.08 | 80.7 | 0.35 | 0.1 | 0.0000 |
| logs/char_T/longitudinal/moving | 2011 | J_T+C | (2.3, 'axisP_side_337deg', 'direct') | 7.73 | 79.6 | 0.00 | 1.1 | 0.0125 |
| logs/char_T/longitudinal/moving | 2011 | J7 - V | (2.35, 'axisP_side_337deg', 'direct') | 7.73 | 79.4 | 0.00 | 1.3 | 0.0101 |
| logs/char_T/longitudinal/moving | 2011 | J7 - E,L | (3.1, 'axisP_side_337deg', 'ring140mm_2of8') | 8.33 | 80.5 | 0.60 | 0.2 | 0.0026 |
| logs/char_T/longitudinal/moving | 2011 | J7 - Q,K | (2.3, 'axisP_side_337deg', 'direct') | 7.73 | 79.6 | 0.00 | 1.1 | 0.0125 |
| logs/char_T/longitudinal/rest | 43 | J7 | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | EARLIEST_TAU | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | EARLIEST_DONE | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | LEX(done+0.2s -> clearance -> effort) | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | LEX(done+0.5s -> clearance -> effort) | (2.435, 'axisP_side_337deg', 'ring80mm_1of8') | 7.64 | 80.9 | 0.29 | 0.0 | 0.0062 |
| logs/char_T/longitudinal/rest | 43 | LEX(done+1.0s -> clearance -> effort) | (2.959, 'axisP_side_68deg', 'ring80mm_6of8') | 8.14 | 81.8 | 0.78 | -0.8 | 0.0254 |
| logs/char_T/longitudinal/rest | 43 | SAFE_EARLIEST (clear >= 80 mm) | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | J_T+C | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | J7 - V | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | J7 - E,L | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/longitudinal/rest | 43 | J7 - Q,K | (2.15, 'axisP_side_337deg', 'direct') | 7.35 | 80.3 | 0.00 | 0.7 | 0.0000 |
| logs/char_T/near-ground/moving | 1286 | J7 | (3.2, 'axisP_side_45deg', 'ring80mm_7of8') | 8.37 | 71.1 | 0.18 | 4.9 | 0.0000 |
| logs/char_T/near-ground/moving | 1286 | EARLIEST_TAU | (2.9, 'axisN_side_293deg', 'ring80mm_1of8') | 8.78 | 68.0 | 0.58 | 8.0 | 0.0795 |
| logs/char_T/near-ground/moving | 1286 | EARLIEST_DONE | (3.05, 'axisP_side_45deg', 'direct') | 8.20 | 56.7 | 0.00 | 19.3 | 0.0434 |
| logs/char_T/near-ground/moving | 1286 | LEX(done+0.2s -> clearance -> effort) | (3.2, 'axisP_side_45deg', 'ring80mm_0of8') | 8.37 | 74.6 | 0.17 | 1.5 | 0.0164 |
| logs/char_T/near-ground/moving | 1286 | LEX(done+0.5s -> clearance -> effort) | (3.45, 'axisP_side_45deg', 'ring140mm_1of8') | 8.64 | 76.0 | 0.45 | 0.0 | 0.0825 |
| logs/char_T/near-ground/moving | 1286 | LEX(done+1.0s -> clearance -> effort) | (3.45, 'axisP_side_45deg', 'ring140mm_1of8') | 8.64 | 76.0 | 0.45 | 0.0 | 0.0825 |
| logs/char_T/near-ground/moving | 1286 | SAFE_EARLIEST (clear >= 80 mm) | (7.7, 'axisP_side_247deg', 'ring80mm_7of8') | 13.15 | 82.3 | 4.95 | -6.3 | 0.2985 |
| logs/char_T/near-ground/moving | 1286 | J_T+C | (3.2, 'axisP_side_45deg', 'ring80mm_1of8') | 8.35 | 74.3 | 0.15 | 1.7 | 0.0275 |
| logs/char_T/near-ground/moving | 1286 | J7 - V | (3.2, 'axisP_side_45deg', 'ring80mm_7of8') | 8.37 | 71.1 | 0.18 | 4.9 | 0.0000 |
| logs/char_T/near-ground/moving | 1286 | J7 - E,L | (3.2, 'axisP_side_45deg', 'ring80mm_7of8') | 8.37 | 71.1 | 0.18 | 4.9 | 0.0000 |
| logs/char_T/near-ground/moving | 1286 | J7 - Q,K | (3.2, 'axisP_side_45deg', 'ring80mm_7of8') | 8.37 | 71.1 | 0.18 | 4.9 | 0.0000 |
| logs/char_T/near-ground/rest | 13 | J7 | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_T/near-ground/rest | 13 | EARLIEST_TAU | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0529 |
| logs/char_T/near-ground/rest | 13 | EARLIEST_DONE | (3.0, 'axisP_side_45deg', 'direct') | 8.22 | 55.7 | 0.00 | 27.3 | 0.0529 |
| logs/char_T/near-ground/rest | 13 | LEX(done+0.2s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_T/near-ground/rest | 13 | LEX(done+0.5s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_T/near-ground/rest | 13 | LEX(done+1.0s -> clearance -> effort) | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_T/near-ground/rest | 13 | SAFE_EARLIEST (clear >= 80 mm) | (3.133, 'axisP_side_45deg', 'ring80mm_1of8') | 8.33 | 82.8 | 0.11 | 0.1 | 0.0104 |
| logs/char_T/near-ground/rest | 13 | J_T+C | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_T/near-ground/rest | 13 | J7 - V | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |
| logs/char_T/near-ground/rest | 13 | J7 - E,L | (3.133, 'axisP_side_45deg', 'ring80mm_2of8') | 8.35 | 82.9 | 0.13 | 0.0 | 0.0022 |
| logs/char_T/near-ground/rest | 13 | J7 - Q,K | (3.133, 'axisP_side_45deg', 'ring80mm_0of8') | 8.33 | 82.6 | 0.11 | 0.3 | 0.0000 |

### Pooled over all searches

| ranker | searches | same tuple as J7 | completion regret mean / max (s) | min clearance min / median (mm) | clearance deficit vs best within +0.5 s: mean / max (mm) | J regret mean / max |
|---|---:|---:|---|---|---|---|
| J7 | 40 | 40 | 0.21 / 1.38 | 70.6 / 80.3 | 1.4 / 5.0 | 0.0000 / 0.0000 |
| EARLIEST_TAU | 40 | 9 | 0.03 / 0.58 | 47.9 / 75.6 | 7.6 / 34.3 | 0.0329 / 0.1330 |
| EARLIEST_DONE | 40 | 10 | 0.00 / 0.00 | 47.9 / 75.6 | 7.9 / 34.3 | 0.0306 / 0.1330 |
| LEX(done+0.2s -> clearance -> effort) | 40 | 7 | 0.05 / 0.19 | 47.9 / 80.2 | 3.5 / 34.3 | 0.0231 / 0.1330 |
| LEX(done+0.5s -> clearance -> effort) | 40 | 6 | 0.30 / 0.50 | 73.1 / 80.8 | 0.0 / 0.1 | 0.0367 / 0.3884 |
| LEX(done+1.0s -> clearance -> effort) | 40 | 5 | 0.62 / 0.98 | 75.0 / 81.2 | -0.6 / 0.0 | 0.0530 / 0.3884 |
| SAFE_EARLIEST (clear >= 80 mm) | 40 | 12 | 1.15 / 5.04 | 79.0 / 80.7 | -1.7 / 1.2 | 0.0877 / 0.5278 |
| J_T+C | 40 | 14 | 0.07 / 0.47 | 73.1 / 80.2 | 0.6 / 3.7 | 0.0386 / 0.3963 |
| J7 - V | 40 | 28 | 0.09 / 1.38 | 70.6 / 80.2 | 1.4 / 5.0 | 0.0028 / 0.0183 |
| J7 - E,L | 40 | 18 | 0.26 / 1.37 | 70.6 / 80.4 | 1.4 / 7.1 | 0.0031 / 0.0298 |
| J7 - Q,K | 40 | 30 | 0.13 / 0.67 | 70.6 / 80.3 | 1.5 / 5.0 | 0.0025 / 0.0439 |

### J7 weight sensitivity (selection changes when one weight is halved or doubled and weights renormalized)

| perturbation | searches changed | completion change when changed (s) | min-clearance change when changed (mm) |
|---|---:|---|---|
| Cx0.5 | 8/40 | -1.38 … +0.02 | -26.22 … -3.45 |
| Cx2.0 | 2/40 | +0.03 … +0.10 | +3.01 … +3.14 |
| Ex0.5 | 7/40 | -0.72 … +0.03 | -1.65 … +3.01 |
| Ex2.0 | 21/40 | -0.00 … +0.77 | -3.58 … +3.14 |
| Kx0.5 | 1/40 | -0.91 … -0.91 | -4.22 … -4.22 |
| Kx2.0 | 1/40 | +0.74 … +0.74 | +0.58 … +0.58 |
| Lx0.5 | 19/40 | -0.00 … +0.77 | -3.58 … +3.14 |
| Lx2.0 | 11/40 | -0.72 … +0.00 | -10.05 … +2.13 |
| Qx0.5 | 5/40 | -0.72 … +0.03 | -1.65 … +3.01 |
| Qx2.0 | 15/40 | -0.03 … +0.77 | -4.47 … +1.16 |
| Tx0.5 | 19/40 | +0.06 … +1.52 | -3.58 … +3.95 |
| Tx2.0 | 9/40 | -1.38 … -0.35 | -26.22 … -0.41 |
| Vx0.5 | 11/40 | -1.05 … -0.02 | -1.65 … +2.13 |
| Vx2.0 | 15/40 | +0.25 … +1.52 | -0.27 … +3.17 |
