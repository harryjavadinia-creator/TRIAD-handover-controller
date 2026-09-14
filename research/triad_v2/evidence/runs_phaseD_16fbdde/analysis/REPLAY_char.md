
### Policy comparison — moving-epoch searches that completed (8 searches)

| policy | plan found | admissible & before reach-start deadline | median wall to plan (s) | median work units | median fraction of full work avoided | median t_plan − t_rest (s) | τ = FULL_ARGMIN τ | tuple = FULL_ARGMIN | median / max ΔJ vs FULL_ARGMIN | τ = zero-latency argmin τ | tuple = zero-latency argmin | median / max ΔJ vs zero-latency argmin | median Δ reach clearance (m) | median Δ retreat clearance (m) |
|---|---|---|---:|---:|---:|---:|---|---|---|---|---|---|---:|---:|
| FULL_ARGMIN | 6/8 | 6/6 | 3.210 | 242457 | 0.00 | -1.31 | 100% (6) | 100% (6) | 0.0000 / 0.0000 | 0% (6) | 0% (6) | 0.2544 / 0.2832 | 0.0000 | 0.0000 |
| FIRST_COMPLETE_FEASIBLE | 8/8 | 4/8 | 0.116 | 14002 | 0.95 | -4.41 | 0% (6) | 0% (6) | -0.1047 / -0.0437 | 50% (8) | 25% (8) | 0.0245 / 0.1497 | -0.0045 | 0.0102 |
| FIRST_ADMISSIBLE_COMPLETE | 8/8 | 8/8 | 0.611 | 61739 | 0.74 | -3.91 | 0% (6) | 0% (6) | 0.0420 / 0.2186 | 75% (8) | 25% (8) | 0.1727 / 0.5018 | -0.0045 | -0.0061 |
| EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 8/8 | 8/8 | 0.813 | 74030 | 0.69 | -3.71 | 0% (6) | 0% (6) | -0.0927 / 0.0657 | 75% (8) | 75% (8) | 0.0000 / 0.3489 | -0.0019 | -0.0219 |
| ANYTIME_INCUMBENT | 8/8 | 8/8 | 0.611 | 61739 | 0.74 | -3.91 | 0% (6) | 0% (6) | 0.0420 / 0.2186 | 75% (8) | 25% (8) | 0.1727 / 0.5018 | -0.0045 | -0.0061 |

### Policy comparison — rest-epoch searches that completed (8 searches)

| policy | plan found | admissible & before reach-start deadline | median wall to plan (s) | median work units | median fraction of full work avoided | median t_plan − t_rest (s) | τ = FULL_ARGMIN τ | tuple = FULL_ARGMIN | median / max ΔJ vs FULL_ARGMIN | τ = zero-latency argmin τ | tuple = zero-latency argmin | median / max ΔJ vs zero-latency argmin | median Δ reach clearance (m) | median Δ retreat clearance (m) |
|---|---|---|---:|---:|---:|---:|---|---|---|---|---|---|---:|---:|
| FULL_ARGMIN | 8/8 | 8/8 | 0.322 | 20412 | 0.00 | 1.31 | 100% (8) | 100% (8) | 0.0000 / 0.0000 | 25% (8) | 25% (8) | 0.0185 / 0.0237 | 0.0000 | 0.0000 |
| FIRST_COMPLETE_FEASIBLE | 8/8 | 0/8 | 0.022 | 2146 | 0.89 | 1.01 | 0% (8) | 0% (8) | 0.0228 / 0.5156 | 0% (8) | 0% (8) | 0.0390 / 0.5393 | -0.0245 | -0.0000 |
| FIRST_ADMISSIBLE_COMPLETE | 8/8 | 8/8 | 0.320 | 20412 | 0.00 | 1.31 | 50% (8) | 25% (8) | 0.0172 / 0.0466 | 25% (8) | 0% (8) | 0.0389 / 0.0598 | -0.0003 | -0.0001 |
| EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 8/8 | 8/8 | 0.320 | 20412 | 0.00 | 1.31 | 50% (8) | 50% (8) | 0.0019 / 0.0466 | 25% (8) | 0% (8) | 0.0237 / 0.0598 | -0.0003 | 0.0000 |
| ANYTIME_INCUMBENT | 8/8 | 8/8 | 0.320 | 20412 | 0.00 | 1.31 | 50% (8) | 25% (8) | 0.0172 / 0.0466 | 25% (8) | 0% (8) | 0.0389 / 0.0598 | -0.0003 | -0.0001 |

### Per search: plan availability, tuple, cost and clearance

ΔJ: vs FULL_ARGMIN at its own receipt / vs the zero-latency exhaustive argmin (all records admitted at the search epoch). H/G/R: temporal hypotheses touched / grasp screens / route rollouts up to the stop. Deadline: available with ≥1.6 s to τ and reach duration + 0.05 s ≤ τ − t.

| search | kind | outcome | policy | wall to plan (s) | work units | t − t_rest (s) | deadline ok | τ lead (s) | g | r | J | ΔJ vs FULL / vs zero-latency | same τ,g,r as FULL | same τ,g,r as zero-latency | reach clr (m) | retreat clr (m) | H/G/R | full work avoided |
|---|---|---|---|---:|---:|---:|---|---:|---|---|---:|---|---|---|---:|---:|---|---:|
| char_a/diagonal g1 | moving | accepted | FULL_ARGMIN | 2.816 | 222159 | -1.70 | yes | 5.50 | axisP_side_337deg | ring80mm_2of8 | 0.7746 | 0.0000 / 0.0927 | τgr | ·g· | 0.074 | 0.223 | 14/448/408 | 0.00 |
| char_a/diagonal g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.369 | 59919 | -4.15 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_a/diagonal g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.369 | 59919 | -4.15 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_a/diagonal g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.472 | 67595 | -4.05 | yes | 4.15 | axisP_side_337deg | ring140mm_2of8 | 0.6820 | -0.0927 / 0.0000 | ·g· | τgr | 0.072 | 0.218 | 7/224/17 | 0.70 |
| char_a/diagonal g1 | moving | accepted | ANYTIME_INCUMBENT | 0.369 | 59919 | -4.15 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_a/diagonal g2 | rest | accepted | FULL_ARGMIN | 0.157 | 13401 | +1.15 | yes | 2.80 | axisP_side_337deg | ring80mm_1of8 | 0.6035 | 0.0000 / 0.0000 | τgr | τgr | 0.081 | 0.217 | 14/32/17 | 0.00 |
| char_a/diagonal g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.038 | 5022 | +1.03 | no | 1.80 | axisP_side_337deg | direct | 0.5784 | -0.0252 / -0.0252 | ·g· | ·g· | 0.080 | 0.217 | 1/16/1 | 0.63 |
| char_a/diagonal g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.155 | 13401 | +1.15 | yes | 2.35 | axisP_side_337deg | direct | 0.6073 | 0.0038 / 0.0038 | ·g· | ·g· | 0.080 | 0.217 | 3/32/17 | 0.00 |
| char_a/diagonal g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.155 | 13401 | +1.15 | yes | 2.35 | axisP_side_337deg | direct | 0.6073 | 0.0038 / 0.0038 | ·g· | ·g· | 0.080 | 0.217 | 3/32/17 | 0.00 |
| char_a/diagonal g2 | rest | accepted | ANYTIME_INCUMBENT | 0.155 | 13401 | +1.15 | yes | 2.35 | axisP_side_337deg | direct | 0.6073 | 0.0038 / 0.0038 | ·g· | ·g· | 0.080 | 0.217 | 3/32/17 | 0.00 |
| char_b/diagonal g1 | moving | accepted | FULL_ARGMIN | 2.880 | 222159 | -1.64 | yes | 5.50 | axisP_side_337deg | ring80mm_2of8 | 0.7746 | 0.0000 / 0.0927 | τgr | ·g· | 0.074 | 0.223 | 14/448/408 | 0.00 |
| char_b/diagonal g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.384 | 59919 | -4.14 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_b/diagonal g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.384 | 59919 | -4.14 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_b/diagonal g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.496 | 67595 | -4.03 | yes | 4.15 | axisP_side_337deg | ring140mm_2of8 | 0.6820 | -0.0927 / 0.0000 | ·g· | τgr | 0.072 | 0.218 | 7/224/17 | 0.70 |
| char_b/diagonal g1 | moving | accepted | ANYTIME_INCUMBENT | 0.384 | 59919 | -4.14 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_b/diagonal g2 | rest | accepted | FULL_ARGMIN | 0.157 | 13401 | +1.15 | yes | 2.80 | axisP_side_337deg | ring80mm_1of8 | 0.6035 | 0.0000 / 0.0000 | τgr | τgr | 0.081 | 0.217 | 14/32/17 | 0.00 |
| char_b/diagonal g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.039 | 5022 | +1.03 | no | 1.80 | axisP_side_337deg | direct | 0.5784 | -0.0252 / -0.0252 | ·g· | ·g· | 0.080 | 0.217 | 1/16/1 | 0.63 |
| char_b/diagonal g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.155 | 13401 | +1.15 | yes | 2.35 | axisP_side_337deg | direct | 0.6073 | 0.0038 / 0.0038 | ·g· | ·g· | 0.080 | 0.217 | 3/32/17 | 0.00 |
| char_b/diagonal g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.155 | 13401 | +1.15 | yes | 2.35 | axisP_side_337deg | direct | 0.6073 | 0.0038 / 0.0038 | ·g· | ·g· | 0.080 | 0.217 | 3/32/17 | 0.00 |
| char_b/diagonal g2 | rest | accepted | ANYTIME_INCUMBENT | 0.155 | 13401 | +1.15 | yes | 2.35 | axisP_side_337deg | direct | 0.6073 | 0.0038 / 0.0038 | ·g· | ·g· | 0.080 | 0.217 | 3/32/17 | 0.00 |
| char_a/lateral-low g1 | moving | accepted | FULL_ARGMIN | 4.638 | 263056 | +0.12 | yes | 8.00 | axisN_side_23deg | ring80mm_6of8 | 0.9321 | 0.0000 / 0.2544 | τgr | ··· | 0.069 | 0.099 | 14/448/731 | 0.00 |
| char_a/lateral-low g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.081 | 5645 | -4.44 | no | 1.80 | axisN_side_68deg | ring140mm_2of8 | 0.8274 | -0.1047 / 0.1497 | ··· | ··· | 0.067 | 0.109 | 1/20/12 | 0.98 |
| char_a/lateral-low g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.838 | 63559 | -3.69 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0420 / 0.2964 | ··· | τ·· | 0.078 | 0.105 | 6/179/122 | 0.76 |
| char_a/lateral-low g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 1.131 | 80465 | -3.39 | yes | 3.70 | axisN_side_337deg | direct | 0.6777 | -0.2544 / 0.0000 | ··· | τgr | 0.082 | 0.077 | 6/192/170 | 0.69 |
| char_a/lateral-low g1 | moving | accepted | ANYTIME_INCUMBENT | 0.838 | 63559 | -3.69 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0420 / 0.2964 | ··· | τ·· | 0.078 | 0.105 | 6/179/122 | 0.76 |
| char_a/lateral-low g2 | rest | accepted | FULL_ARGMIN | 0.473 | 25514 | +1.47 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 14/32/68 | 0.00 |
| char_a/lateral-low g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.026 | 2664 | +1.02 | no | 1.80 | axisP_side_293deg | direct | 0.6922 | 0.0707 / 0.0944 | ··r | ··r | 0.056 | 0.104 | 1/14/1 | 0.90 |
| char_a/lateral-low g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.469 | 25514 | +1.46 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| char_a/lateral-low g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.469 | 25514 | +1.46 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| char_a/lateral-low g2 | rest | accepted | ANYTIME_INCUMBENT | 0.469 | 25514 | +1.46 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| char_b/lateral-low g1 | moving | accepted | FULL_ARGMIN | 4.545 | 263056 | +0.03 | yes | 8.00 | axisN_side_23deg | ring80mm_6of8 | 0.9321 | 0.0000 / 0.2544 | τgr | ··· | 0.069 | 0.099 | 14/448/731 | 0.00 |
| char_b/lateral-low g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.081 | 5645 | -4.44 | no | 1.80 | axisN_side_68deg | ring140mm_2of8 | 0.8274 | -0.1047 / 0.1497 | ··· | ··· | 0.067 | 0.109 | 1/20/12 | 0.98 |
| char_b/lateral-low g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.850 | 63559 | -3.67 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0420 / 0.2964 | ··· | τ·· | 0.078 | 0.105 | 6/179/122 | 0.76 |
| char_b/lateral-low g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 1.146 | 80465 | -3.38 | yes | 3.70 | axisN_side_337deg | direct | 0.6777 | -0.2544 / 0.0000 | ··· | τgr | 0.082 | 0.077 | 6/192/170 | 0.69 |
| char_b/lateral-low g1 | moving | accepted | ANYTIME_INCUMBENT | 0.850 | 63559 | -3.67 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0420 / 0.2964 | ··· | τ·· | 0.078 | 0.105 | 6/179/122 | 0.76 |
| char_b/lateral-low g2 | rest | accepted | FULL_ARGMIN | 0.514 | 25514 | +1.52 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 14/32/68 | 0.00 |
| char_b/lateral-low g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.027 | 2664 | +1.02 | no | 1.80 | axisP_side_293deg | direct | 0.6922 | 0.0707 / 0.0944 | ··r | ··r | 0.056 | 0.104 | 1/14/1 | 0.90 |
| char_b/lateral-low g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.510 | 25514 | +1.50 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| char_b/lateral-low g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.510 | 25514 | +1.50 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| char_b/lateral-low g2 | rest | accepted | ANYTIME_INCUMBENT | 0.510 | 25514 | +1.50 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| char_a/longitudinal g1 | moving | accepted | FULL_ARGMIN | none | | | | | | | | | | | | | | |
| char_a/longitudinal g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.168 | 22359 | -4.36 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_a/longitudinal g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.168 | 22359 | -4.36 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_a/longitudinal g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.385 | 34443 | -4.14 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/96/34 | 0.87 |
| char_a/longitudinal g1 | moving | accepted | ANYTIME_INCUMBENT | 0.168 | 22359 | -4.36 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_a/longitudinal g2 | rest | accepted | FULL_ARGMIN | 0.538 | 28770 | +1.54 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 14/32/102 | 0.00 |
| char_a/longitudinal g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.019 | 1152 | +1.01 | no | 1.80 | axisP_side_45deg | ring80mm_0of8 | 1.1655 | 0.5156 / 0.5393 | ··· | ··· | 0.022 | 0.037 | 1/3/2 | 0.96 |
| char_a/longitudinal g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.534 | 28770 | +1.53 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| char_a/longitudinal g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.534 | 28770 | +1.53 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 4/32/102 | 0.00 |
| char_a/longitudinal g2 | rest | accepted | ANYTIME_INCUMBENT | 0.534 | 28770 | +1.53 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| char_b/longitudinal g1 | moving | accepted | FULL_ARGMIN | none | | | | | | | | | | | | | | |
| char_b/longitudinal g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.151 | 22359 | -4.37 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_b/longitudinal g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.151 | 22359 | -4.37 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_b/longitudinal g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.351 | 34443 | -4.17 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/96/34 | 0.87 |
| char_b/longitudinal g1 | moving | accepted | ANYTIME_INCUMBENT | 0.151 | 22359 | -4.37 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_b/longitudinal g2 | rest | accepted | FULL_ARGMIN | 0.520 | 28770 | +1.52 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 14/32/102 | 0.00 |
| char_b/longitudinal g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.015 | 1152 | +1.01 | no | 1.80 | axisP_side_45deg | ring80mm_0of8 | 1.1655 | 0.5156 / 0.5393 | ··· | ··· | 0.022 | 0.037 | 1/3/2 | 0.96 |
| char_b/longitudinal g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.516 | 28770 | +1.51 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| char_b/longitudinal g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.516 | 28770 | +1.51 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 4/32/102 | 0.00 |
| char_b/longitudinal g2 | rest | accepted | ANYTIME_INCUMBENT | 0.516 | 28770 | +1.51 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| char_a/near-ground g1 | moving | accepted | FULL_ARGMIN | 3.204 | 242457 | -1.32 | yes | 8.00 | axisP_side_247deg | ring140mm_5of8 | 1.0367 | 0.0000 / 0.2832 | τgr | ··· | 0.082 | 0.088 | 14/448/408 | 0.00 |
| char_a/near-ground g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.015 | 1503 | -4.51 | no | 1.80 | axisP_side_45deg | direct | 0.6811 | -0.3556 / -0.0723 | ··· | ·g· | 0.074 | 0.108 | 1/3/1 | 0.99 |
| char_a/near-ground g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 2.650 | 197175 | -1.87 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |
| char_a/near-ground g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 2.779 | 208454 | -1.74 | yes | 6.40 | axisP_side_68deg | ring80mm_2of8 | 1.1024 | 0.0657 / 0.3489 | ··· | ··· | 0.047 | 0.049 | 12/384/357 | 0.14 |
| char_a/near-ground g1 | moving | accepted | ANYTIME_INCUMBENT | 2.650 | 197175 | -1.87 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |
| char_a/near-ground g2 | rest | accepted | FULL_ARGMIN | 0.172 | 15309 | +1.15 | yes | 3.70 | axisP_side_45deg | ring140mm_7of8 | 0.7617 | 0.0000 / 0.0133 | τgr | ·g· | 0.079 | 0.098 | 14/32/17 | 0.00 |
| char_a/near-ground g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.016 | 1629 | +1.01 | no | 1.80 | axisP_side_45deg | direct | 0.7319 | -0.0298 / -0.0165 | ·g· | ·g· | 0.056 | 0.098 | 1/3/1 | 0.89 |
| char_a/near-ground g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.171 | 15309 | +1.16 | yes | 3.25 | axisP_side_45deg | direct | 0.8082 | 0.0466 / 0.0598 | ·g· | τg· | 0.056 | 0.098 | 5/32/17 | 0.00 |
| char_a/near-ground g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.171 | 15309 | +1.16 | yes | 3.25 | axisP_side_45deg | direct | 0.8082 | 0.0466 / 0.0598 | ·g· | τg· | 0.056 | 0.098 | 5/32/17 | 0.00 |
| char_a/near-ground g2 | rest | accepted | ANYTIME_INCUMBENT | 0.171 | 15309 | +1.16 | yes | 3.25 | axisP_side_45deg | direct | 0.8082 | 0.0466 / 0.0598 | ·g· | τg· | 0.056 | 0.098 | 5/32/17 | 0.00 |
| char_b/near-ground g1 | moving | accepted | FULL_ARGMIN | 3.215 | 242457 | -1.31 | yes | 8.00 | axisP_side_247deg | ring140mm_5of8 | 1.0367 | 0.0000 / 0.2832 | τgr | ··· | 0.082 | 0.088 | 14/448/408 | 0.00 |
| char_b/near-ground g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.015 | 1503 | -4.51 | no | 1.80 | axisP_side_45deg | direct | 0.6811 | -0.3556 / -0.0723 | ··· | ·g· | 0.074 | 0.108 | 1/3/1 | 0.99 |
| char_b/near-ground g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 2.642 | 197175 | -1.88 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |
| char_b/near-ground g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 2.780 | 208454 | -1.74 | yes | 6.40 | axisP_side_68deg | ring80mm_2of8 | 1.1024 | 0.0657 / 0.3489 | ··· | ··· | 0.047 | 0.049 | 12/384/357 | 0.14 |
| char_b/near-ground g1 | moving | accepted | ANYTIME_INCUMBENT | 2.642 | 197175 | -1.88 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |
| char_b/near-ground g2 | rest | accepted | FULL_ARGMIN | 0.158 | 15309 | +1.14 | yes | 3.70 | axisP_side_45deg | ring140mm_7of8 | 0.7617 | 0.0000 / 0.0133 | τgr | ·g· | 0.079 | 0.098 | 14/32/17 | 0.00 |
| char_b/near-ground g2 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.015 | 1629 | +1.01 | no | 1.80 | axisP_side_45deg | direct | 0.7319 | -0.0298 / -0.0165 | ·g· | ·g· | 0.056 | 0.098 | 1/3/1 | 0.89 |
| char_b/near-ground g2 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.157 | 15309 | +1.15 | yes | 3.25 | axisP_side_45deg | direct | 0.8082 | 0.0466 / 0.0598 | ·g· | τg· | 0.056 | 0.098 | 5/32/17 | 0.00 |
| char_b/near-ground g2 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.157 | 15309 | +1.15 | yes | 3.25 | axisP_side_45deg | direct | 0.8082 | 0.0466 / 0.0598 | ·g· | τg· | 0.056 | 0.098 | 5/32/17 | 0.00 |
| char_b/near-ground g2 | rest | accepted | ANYTIME_INCUMBENT | 0.157 | 15309 | +1.15 | yes | 3.25 | axisP_side_45deg | direct | 0.8082 | 0.0466 / 0.0598 | ·g· | τg· | 0.056 | 0.098 | 5/32/17 | 0.00 |

### Does exhaustive argmin behave like 'earliest timing-admissible τ, then best (g,r)'?

- zero latency (both evaluated with every record admitted at the search epoch), moving: n=8; P(τ equal)=6/8, P(g equal)=6/8, P(r equal)=6/8, P(tuple equal)=6/8
- zero latency (both evaluated with every record admitted at the search epoch), rest: n=8; P(τ equal)=6/8, P(g equal)=8/8, P(r equal)=6/8, P(tuple equal)=6/8
- zero latency (both evaluated with every record admitted at the search epoch), all: n=16; P(τ equal)=12/16, P(g equal)=14/16, P(r equal)=12/16, P(tuple equal)=12/16
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), moving: n=6; P(τ equal)=0/6, P(g equal)=2/6, P(r equal)=0/6, P(tuple equal)=0/6
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), rest: n=8; P(τ equal)=4/8, P(g equal)=8/8, P(r equal)=4/8, P(tuple equal)=4/8
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), all: n=14; P(τ equal)=4/14, P(g equal)=10/14, P(r equal)=4/14, P(tuple equal)=4/14
- completed searches in which FULL_ARGMIN had no admissible plan at receipt: 2 (char_a/longitudinal g1, char_b/longitudinal g1); bounded policies with an admissible plan in those: FIRST_COMPLETE_FEASIBLE=2, FIRST_ADMISSIBLE_COMPLETE=2, EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR=2, ANYTIME_INCUMBENT=2

### ANYTIME_INCUMBENT evolution (moving-epoch searches)

| search | first incumbent wall (s) | incumbent changes | incumbent lost to expiry | incumbent J (ΔJ vs zero-latency argmin) at 10% / 25% / 50% / 75% / 100% of the full search wall |
|---|---:|---:|---:|---|
| char_a/diagonal g1 (accepted) | 0.369 | 9 | 0 | none / 0.682 (0.000) τ=4.15 / 0.691 (0.009) τ=4.15 / 0.720 (0.038) τ=4.60 / 0.775 (0.093) τ=5.50 |
| char_a/lateral-low g1 (accepted) | 0.838 | 27 | 6 | none / 0.678 (0.000) τ=3.70 / 0.722 (0.044) τ=4.60 / none / 0.932 (0.254) τ=8.00 |
| char_a/longitudinal g1 (accepted) | 0.168 | 21 | 2 | 0.621 (0.000) τ=2.35 / 0.663 (0.041) τ=3.25 / 0.722 (0.101) τ=4.15 / 0.817 (0.195) τ=5.50 / none |
| char_a/near-ground g1 (accepted) | 2.650 | 6 | 0 | none / none / none / none / 1.037 (0.283) τ=8.00 |
| char_b/diagonal g1 (accepted) | 0.384 | 9 | 0 | none / 0.682 (0.000) τ=4.15 / 0.691 (0.009) τ=4.15 / 0.720 (0.038) τ=4.60 / 0.775 (0.093) τ=5.50 |
| char_b/lateral-low g1 (accepted) | 0.850 | 28 | 6 | none / 0.678 (0.000) τ=3.70 / 0.722 (0.044) τ=4.60 / 0.868 (0.190) τ=6.40 / 0.932 (0.254) τ=8.00 |
| char_b/longitudinal g1 (accepted) | 0.151 | 16 | 0 | 0.621 (0.000) τ=2.35 / 0.663 (0.041) τ=3.25 / 0.722 (0.101) τ=4.15 / 0.786 (0.165) τ=5.05 / none |
| char_b/near-ground g1 (accepted) | 2.642 | 6 | 0 | none / none / none / none / 1.037 (0.283) τ=8.00 |

### Computation before the stop (moving-epoch searches; worker wall in s, bounded work units in parentheses)

| search | policy | temporal hypothesis setup | grasp/static reach screening | insertion/closure certification | route (transit reach) certification | carried-retreat certification | cost / terminal timing audit | nested: IK steps | nested: swept-volume queries | nested: configuration safety | nested: closure/contact safety |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| char_a/diagonal g1 | FULL_ARGMIN | 0.000 (14) | 0.533 (89105) | 0.342 (56528) | 1.573 (50614) | 0.180 (25631) | 0.175 (267) | 1.220 (195256) | 1.058 (49798) | 0.318 (220512) | 0.263 (53070) |
| char_a/diagonal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (7) | 0.266 (44564) | 0.053 (8847) | 0.003 (99) | 0.044 (6401) | 0.001 (1) | 0.324 (53874) | 0.002 (97) | 0.067 (47527) | 0.030 (6206) |
| char_a/diagonal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (7) | 0.266 (44564) | 0.053 (8847) | 0.003 (99) | 0.044 (6401) | 0.001 (1) | 0.324 (53874) | 0.002 (97) | 0.067 (47527) | 0.030 (6206) |
| char_a/diagonal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (7) | 0.278 (46577) | 0.067 (11239) | 0.062 (2058) | 0.053 (7700) | 0.009 (14) | 0.365 (60531) | 0.042 (2024) | 0.077 (54865) | 0.041 (8508) |
| char_a/diagonal g1 | ANYTIME_INCUMBENT | 0.000 (7) | 0.266 (44564) | 0.053 (8847) | 0.003 (99) | 0.044 (6401) | 0.001 (1) | 0.324 (53874) | 0.002 (97) | 0.067 (47527) | 0.030 (6206) |
| char_a/lateral-low g1 | FULL_ARGMIN | 0.001 (14) | 0.318 (50616) | 0.475 (74623) | 3.365 (111001) | 0.197 (26420) | 0.266 (382) | 1.495 (227759) | 2.220 (109551) | 0.453 (312528) | 0.385 (73015) |
| char_a/lateral-low g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.018 (2558) | 0.009 (1206) | 0.053 (1785) | 0.001 (94) | 0.001 (1) | 0.032 (4822) | 0.035 (1761) | 0.010 (6495) | 0.006 (906) |
| char_a/lateral-low g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.160 (25428) | 0.101 (16068) | 0.499 (17454) | 0.032 (4541) | 0.041 (62) | 0.347 (55093) | 0.329 (17213) | 0.095 (68011) | 0.076 (14666) |
| char_a/lateral-low g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.176 (28101) | 0.132 (20997) | 0.703 (24403) | 0.049 (6862) | 0.065 (96) | 0.444 (70087) | 0.463 (24067) | 0.122 (87673) | 0.104 (19945) |
| char_a/lateral-low g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.160 (25428) | 0.101 (16068) | 0.499 (17454) | 0.032 (4541) | 0.041 (62) | 0.347 (55093) | 0.329 (17213) | 0.095 (68011) | 0.076 (14666) |
| char_a/longitudinal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (3) | 0.128 (17601) | 0.029 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.142 (19764) | 0.002 (92) | 0.032 (19033) | 0.016 (2723) |
| char_a/longitudinal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (3) | 0.128 (17601) | 0.029 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.142 (19764) | 0.002 (92) | 0.032 (19033) | 0.016 (2723) |
| char_a/longitudinal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (3) | 0.148 (20664) | 0.047 (6672) | 0.154 (4631) | 0.020 (2455) | 0.014 (18) | 0.222 (30636) | 0.099 (4564) | 0.053 (32805) | 0.032 (5567) |
| char_a/longitudinal g1 | ANYTIME_INCUMBENT | 0.000 (3) | 0.128 (17601) | 0.029 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.142 (19764) | 0.002 (92) | 0.032 (19033) | 0.016 (2723) |
| char_a/near-ground g1 | FULL_ARGMIN | 0.000 (14) | 0.526 (84783) | 0.493 (77104) | 1.835 (62800) | 0.130 (17500) | 0.206 (256) | 1.450 (226353) | 1.205 (61988) | 0.382 (271847) | 0.241 (45959) |
| char_a/near-ground g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.007 (965) | 0.002 (297) | 0.005 (153) | 0.001 (86) | 0.001 (1) | 0.010 (1367) | 0.003 (151) | 0.002 (1435) | 0.001 (237) |
| char_a/near-ground g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (12) | 0.476 (76434) | 0.353 (54667) | 1.508 (51181) | 0.110 (14647) | 0.190 (234) | 1.182 (183096) | 0.992 (50501) | 0.311 (219890) | 0.219 (41679) |
| char_a/near-ground g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (12) | 0.492 (79024) | 0.392 (60776) | 1.575 (53365) | 0.113 (15037) | 0.195 (240) | 1.250 (193916) | 1.034 (52655) | 0.329 (232495) | 0.225 (42772) |
| char_a/near-ground g1 | ANYTIME_INCUMBENT | 0.000 (12) | 0.476 (76434) | 0.353 (54667) | 1.508 (51181) | 0.110 (14647) | 0.190 (234) | 1.182 (183096) | 0.992 (50501) | 0.311 (219890) | 0.219 (41679) |
| char_b/diagonal g1 | FULL_ARGMIN | 0.000 (14) | 0.551 (89105) | 0.350 (56528) | 1.597 (50614) | 0.184 (25631) | 0.185 (267) | 1.257 (195256) | 1.075 (49798) | 0.333 (220512) | 0.270 (53070) |
| char_b/diagonal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (7) | 0.276 (44564) | 0.056 (8847) | 0.003 (99) | 0.046 (6401) | 0.001 (1) | 0.337 (53874) | 0.002 (97) | 0.069 (47527) | 0.032 (6206) |
| char_b/diagonal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (7) | 0.276 (44564) | 0.056 (8847) | 0.003 (99) | 0.046 (6401) | 0.001 (1) | 0.337 (53874) | 0.002 (97) | 0.069 (47527) | 0.032 (6206) |
| char_b/diagonal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (7) | 0.289 (46577) | 0.071 (11239) | 0.068 (2058) | 0.055 (7700) | 0.010 (14) | 0.381 (60531) | 0.046 (2024) | 0.081 (54865) | 0.044 (8508) |
| char_b/diagonal g1 | ANYTIME_INCUMBENT | 0.000 (7) | 0.276 (44564) | 0.056 (8847) | 0.003 (99) | 0.046 (6401) | 0.001 (1) | 0.337 (53874) | 0.002 (97) | 0.069 (47527) | 0.032 (6206) |
| char_b/lateral-low g1 | FULL_ARGMIN | 0.001 (14) | 0.315 (50616) | 0.464 (74623) | 3.294 (111001) | 0.193 (26420) | 0.260 (382) | 1.465 (227759) | 2.179 (109551) | 0.444 (312528) | 0.378 (73015) |
| char_b/lateral-low g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.018 (2558) | 0.009 (1206) | 0.052 (1785) | 0.001 (94) | 0.001 (1) | 0.033 (4822) | 0.034 (1761) | 0.010 (6495) | 0.006 (906) |
| char_b/lateral-low g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.160 (25428) | 0.102 (16068) | 0.508 (17454) | 0.034 (4541) | 0.042 (62) | 0.351 (55093) | 0.335 (17213) | 0.096 (68011) | 0.076 (14666) |
| char_b/lateral-low g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.176 (28101) | 0.133 (20997) | 0.715 (24403) | 0.051 (6862) | 0.066 (96) | 0.450 (70087) | 0.471 (24067) | 0.124 (87673) | 0.104 (19945) |
| char_b/lateral-low g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.160 (25428) | 0.102 (16068) | 0.508 (17454) | 0.034 (4541) | 0.042 (62) | 0.351 (55093) | 0.335 (17213) | 0.096 (68011) | 0.076 (14666) |
| char_b/longitudinal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (3) | 0.114 (17601) | 0.026 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.128 (19764) | 0.002 (92) | 0.029 (19033) | 0.015 (2723) |
| char_b/longitudinal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (3) | 0.114 (17601) | 0.026 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.128 (19764) | 0.002 (92) | 0.029 (19033) | 0.015 (2723) |
| char_b/longitudinal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (3) | 0.134 (20664) | 0.044 (6672) | 0.141 (4631) | 0.018 (2455) | 0.013 (18) | 0.199 (30636) | 0.094 (4564) | 0.049 (32805) | 0.030 (5567) |
| char_b/longitudinal g1 | ANYTIME_INCUMBENT | 0.000 (3) | 0.114 (17601) | 0.026 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.128 (19764) | 0.002 (92) | 0.029 (19033) | 0.015 (2723) |
| char_b/near-ground g1 | FULL_ARGMIN | 0.000 (14) | 0.530 (84783) | 0.492 (77104) | 1.842 (62800) | 0.131 (17500) | 0.206 (256) | 1.460 (226353) | 1.199 (61988) | 0.386 (271847) | 0.241 (45959) |
| char_b/near-ground g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.007 (965) | 0.002 (297) | 0.005 (153) | 0.001 (86) | 0.001 (1) | 0.010 (1367) | 0.003 (151) | 0.002 (1435) | 0.001 (237) |
| char_b/near-ground g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (12) | 0.477 (76434) | 0.347 (54667) | 1.507 (51181) | 0.110 (14647) | 0.190 (234) | 1.181 (183096) | 0.980 (50501) | 0.312 (219890) | 0.219 (41679) |
| char_b/near-ground g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (12) | 0.494 (79024) | 0.388 (60776) | 1.577 (53365) | 0.113 (15037) | 0.194 (240) | 1.256 (193916) | 1.024 (52655) | 0.331 (232495) | 0.225 (42772) |
| char_b/near-ground g1 | ANYTIME_INCUMBENT | 0.000 (12) | 0.477 (76434) | 0.347 (54667) | 1.507 (51181) | 0.110 (14647) | 0.190 (234) | 1.181 (183096) | 0.980 (50501) | 0.312 (219890) | 0.219 (41679) |

### Per-candidate certification cost and work before the first complete certified record

| search | kind | full wall (s) | static screens rejected / passed (median ms) | route rollouts rejected / complete (median ms) | before first complete: rejected screens, passed screens, rejected routes, hypotheses | wall of rejected screens / passed screens / rejected routes / the successful route (s) |
|---|---|---:|---|---|---|---|
| char_a/diagonal g1 | moving | 2.817 | 424 (1.19) / 24 (1.67) | 141 (3.36) / 267 (5.82) | 207, 1, 0, 7 | 0.362 / 0.002 / 0.000 / 0.005 |
| char_a/diagonal g2 | rest | 0.157 | 31 (1.30) / 1 (2.17) | 0 (nan) / 17 (5.38) | 15, 1, 0, 1 | 0.030 / 0.002 / 0.000 / 0.005 |
| char_b/diagonal g1 | moving | 2.881 | 424 (1.30) / 24 (1.74) | 141 (3.41) / 267 (5.91) | 207, 1, 0, 7 | 0.377 / 0.002 / 0.000 / 0.005 |
| char_b/diagonal g2 | rest | 0.157 | 31 (1.21) / 1 (2.34) | 0 (nan) / 17 (5.73) | 15, 1, 0, 1 | 0.032 / 0.002 / 0.000 / 0.005 |
| char_a/lateral-low g1 | moving | 4.641 | 405 (0.43) / 43 (1.83) | 349 (4.49) / 382 (6.53) | 19, 1, 11, 1 | 0.025 / 0.002 / 0.049 / 0.006 |
| char_a/lateral-low g2 | rest | 0.474 | 28 (0.94) / 4 (1.71) | 26 (4.58) / 42 (6.56) | 13, 1, 0, 1 | 0.014 / 0.002 / 0.000 / 0.009 |
| char_b/lateral-low g1 | moving | 4.547 | 405 (0.42) / 43 (1.82) | 349 (4.38) / 382 (6.46) | 19, 1, 11, 1 | 0.025 / 0.002 / 0.047 / 0.006 |
| char_b/lateral-low g2 | rest | 0.515 | 28 (1.03) / 4 (2.21) | 26 (5.12) / 42 (6.91) | 13, 1, 0, 1 | 0.015 / 0.002 / 0.000 / 0.010 |
| char_a/longitudinal g1 | moving | 3.791 | 413 (1.44) / 35 (1.82) | 363 (3.62) / 232 (6.22) | 79, 1, 0, 3 | 0.160 / 0.002 / 0.000 / 0.006 |
| char_a/longitudinal g2 | rest | 0.540 | 26 (0.96) / 6 (1.81) | 59 (3.25) / 43 (6.08) | 2, 1, 1, 1 | 0.005 / 0.003 / 0.003 / 0.008 |
| char_b/longitudinal g1 | moving | 3.665 | 413 (1.37) / 35 (1.76) | 363 (3.51) / 232 (6.05) | 79, 1, 0, 3 | 0.144 / 0.002 / 0.000 / 0.005 |
| char_b/longitudinal g2 | rest | 0.522 | 26 (1.00) / 6 (1.80) | 59 (3.06) / 43 (6.12) | 2, 1, 1, 1 | 0.003 / 0.002 / 0.003 / 0.007 |
| char_a/near-ground g1 | moving | 3.205 | 424 (0.51) / 24 (1.97) | 152 (4.49) / 256 (6.64) | 2, 1, 0, 1 | 0.006 / 0.002 / 0.000 / 0.007 |
| char_a/near-ground g2 | rest | 0.172 | 31 (1.00) / 1 (2.18) | 4 (2.95) / 13 (6.92) | 2, 1, 0, 1 | 0.007 / 0.002 / 0.000 / 0.007 |
| char_b/near-ground g1 | moving | 3.216 | 424 (0.50) / 24 (1.98) | 152 (4.47) / 256 (6.77) | 2, 1, 0, 1 | 0.006 / 0.002 / 0.000 / 0.007 |
| char_b/near-ground g2 | rest | 0.158 | 31 (1.11) / 1 (2.07) | 4 (2.52) / 13 (6.43) | 2, 1, 0, 1 | 0.007 / 0.002 / 0.000 / 0.007 |
