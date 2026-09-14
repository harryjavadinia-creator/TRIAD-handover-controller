
### Policy comparison — moving-epoch searches that completed (5 searches)

| policy | plan found | admissible & before reach-start deadline | median wall to plan (s) | median work units | median fraction of full work avoided | median t_plan − t_rest (s) | τ = FULL_ARGMIN τ | tuple = FULL_ARGMIN | median / max ΔJ vs FULL_ARGMIN | τ = zero-latency argmin τ | tuple = zero-latency argmin | median / max ΔJ vs zero-latency argmin | median Δ reach clearance (m) | median Δ retreat clearance (m) |
|---|---|---|---:|---:|---:|---:|---|---|---|---|---|---|---:|---:|
| FULL_ARGMIN | 4/5 | 4/4 | 3.025 | 232308 | 0.00 | -1.50 | 100% (4) | 100% (4) | 0.0000 / 0.0000 | 0% (4) | 0% (4) | 0.1879 / 0.2832 | 0.0000 | 0.0000 |
| FIRST_COMPLETE_FEASIBLE | 5/5 | 3/5 | 0.139 | 22359 | 0.91 | -4.38 | 0% (4) | 0% (4) | -0.1996 / -0.0437 | 60% (5) | 20% (5) | 0.0000 / 0.0490 | -0.0063 | 0.0071 |
| FIRST_ADMISSIBLE_COMPLETE | 5/5 | 5/5 | 0.398 | 59919 | 0.73 | -4.13 | 0% (4) | 0% (4) | 0.0875 / 0.2186 | 60% (5) | 20% (5) | 0.0490 / 0.5018 | -0.0283 | -0.0225 |
| EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 5/5 | 5/5 | 0.509 | 67595 | 0.70 | -4.01 | 0% (4) | 0% (4) | -0.0135 / 0.0657 | 60% (5) | 60% (5) | 0.0000 / 0.3489 | -0.0187 | -0.0221 |
| ANYTIME_INCUMBENT | 5/5 | 5/5 | 0.398 | 59919 | 0.73 | -4.13 | 0% (4) | 0% (4) | 0.0875 / 0.2186 | 60% (5) | 20% (5) | 0.0490 / 0.5018 | -0.0283 | -0.0225 |

### Policy comparison — rest-epoch searches that completed (4 searches)

| policy | plan found | admissible & before reach-start deadline | median wall to plan (s) | median work units | median fraction of full work avoided | median t_plan − t_rest (s) | τ = FULL_ARGMIN τ | tuple = FULL_ARGMIN | median / max ΔJ vs FULL_ARGMIN | τ = zero-latency argmin τ | tuple = zero-latency argmin | median / max ΔJ vs zero-latency argmin | median Δ reach clearance (m) | median Δ retreat clearance (m) |
|---|---|---|---:|---:|---:|---:|---|---|---|---|---|---|---:|---:|
| FULL_ARGMIN | 4/4 | 4/4 | 0.516 | 27146 | 0.00 | 0.51 | 100% (4) | 100% (4) | 0.0000 / 0.0000 | 0% (4) | 0% (4) | 0.0237 / 0.0237 | 0.0000 | 0.0000 |
| FIRST_COMPLETE_FEASIBLE | 4/4 | 0/4 | 0.021 | 1908 | 0.93 | 0.01 | 0% (4) | 0% (4) | 0.2936 / 0.5156 | 0% (4) | 0% (4) | 0.3173 / 0.5393 | -0.0421 | -0.0798 |
| FIRST_ADMISSIBLE_COMPLETE | 4/4 | 4/4 | 0.512 | 27146 | 0.00 | 0.50 | 100% (4) | 50% (4) | 0.0153 / 0.0305 | 0% (4) | 0% (4) | 0.0389 / 0.0542 | 0.0000 | -0.0056 |
| EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 4/4 | 4/4 | 0.512 | 27146 | 0.00 | 0.50 | 100% (4) | 100% (4) | 0.0000 / 0.0000 | 0% (4) | 0% (4) | 0.0237 / 0.0237 | 0.0000 | 0.0000 |
| ANYTIME_INCUMBENT | 4/4 | 4/4 | 0.512 | 27146 | 0.00 | 0.50 | 100% (4) | 50% (4) | 0.0153 / 0.0305 | 0% (4) | 0% (4) | 0.0389 / 0.0542 | 0.0000 | -0.0056 |

### Per search: plan availability, tuple, cost and clearance

ΔJ: vs FULL_ARGMIN at its own receipt / vs the zero-latency exhaustive argmin (all records admitted at the search epoch). H/G/R: temporal hypotheses touched / grasp screens / route rollouts up to the stop. Deadline: available with ≥1.6 s to τ and reach duration + 0.05 s ≤ τ − t.

| search | kind | outcome | policy | wall to plan (s) | work units | t − t_rest (s) | deadline ok | τ lead (s) | g | r | J | ΔJ vs FULL / vs zero-latency | same τ,g,r as FULL | same τ,g,r as zero-latency | reach clr (m) | retreat clr (m) | H/G/R | full work avoided |
|---|---|---|---|---:|---:|---:|---|---:|---|---|---:|---|---|---|---:|---:|---|---:|
| insitu_a/diagonal g1 | moving | accepted | FULL_ARGMIN | 2.960 | 222159 | -1.56 | yes | 5.50 | axisP_side_337deg | ring80mm_2of8 | 0.7746 | 0.0000 / 0.0927 | τgr | ·g· | 0.074 | 0.223 | 14/448/408 | 0.00 |
| insitu_a/diagonal g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.398 | 59919 | -4.13 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| insitu_a/diagonal g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.398 | 59919 | -4.13 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| insitu_a/diagonal g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.509 | 67595 | -4.01 | yes | 4.15 | axisP_side_337deg | ring140mm_2of8 | 0.6820 | -0.0927 / 0.0000 | ·g· | τgr | 0.072 | 0.218 | 7/224/17 | 0.70 |
| insitu_a/diagonal g1 | moving | accepted | ANYTIME_INCUMBENT | 0.398 | 59919 | -4.13 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| insitu_b/diagonal g1 | moving | accepted | FULL_ARGMIN | 2.847 | 222159 | -1.67 | yes | 5.50 | axisP_side_337deg | ring80mm_2of8 | 0.7746 | 0.0000 / 0.0927 | τgr | ·g· | 0.074 | 0.223 | 14/448/408 | 0.00 |
| insitu_b/diagonal g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.393 | 59919 | -4.13 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| insitu_b/diagonal g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.393 | 59919 | -4.13 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| insitu_b/diagonal g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.502 | 67595 | -4.02 | yes | 4.15 | axisP_side_337deg | ring140mm_2of8 | 0.6820 | -0.0927 / 0.0000 | ·g· | τgr | 0.072 | 0.218 | 7/224/17 | 0.70 |
| insitu_b/diagonal g1 | moving | accepted | ANYTIME_INCUMBENT | 0.393 | 59919 | -4.13 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| insitu_a/lateral-low g1 | moving | cancelled | FIRST_COMPLETE_FEASIBLE | 0.083 | 5645 | -4.44 | no | 1.80 | axisN_side_68deg | ring140mm_2of8 | 0.8274 | n/a / n/a | n/a | n/a | 0.067 | 0.109 | 1/20/12 | 0.97 |
| insitu_a/lateral-low g1 | moving | cancelled | FIRST_ADMISSIBLE_COMPLETE | 0.863 | 63559 | -3.66 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | n/a / n/a | n/a | n/a | 0.078 | 0.105 | 6/179/122 | 0.72 |
| insitu_a/lateral-low g1 | moving | cancelled | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 1.173 | 80465 | -3.35 | yes | 3.70 | axisN_side_337deg | direct | 0.6777 | n/a / n/a | n/a | n/a | 0.082 | 0.077 | 6/192/170 | 0.64 |
| insitu_a/lateral-low g1 | moving | cancelled | ANYTIME_INCUMBENT | 0.863 | 63559 | -3.66 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | n/a / n/a | n/a | n/a | 0.078 | 0.105 | 6/179/122 | 0.72 |
| insitu_a/lateral-low g38 | rest | accepted | FULL_ARGMIN | 0.489 | 25523 | +0.48 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 14/32/68 | 0.00 |
| insitu_a/lateral-low g38 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.029 | 2664 | +0.02 | no | 1.80 | axisP_side_293deg | direct | 0.6931 | 0.0716 / 0.0953 | ··r | ··r | 0.056 | 0.104 | 1/14/1 | 0.90 |
| insitu_a/lateral-low g38 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.485 | 25523 | +0.48 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| insitu_a/lateral-low g38 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.485 | 25523 | +0.48 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| insitu_a/lateral-low g38 | rest | accepted | ANYTIME_INCUMBENT | 0.485 | 25523 | +0.48 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| insitu_b/lateral-low g1 | moving | cancelled | FIRST_COMPLETE_FEASIBLE | 0.079 | 5645 | -4.44 | no | 1.80 | axisN_side_68deg | ring140mm_2of8 | 0.8274 | n/a / n/a | n/a | n/a | 0.067 | 0.109 | 1/20/12 | 0.98 |
| insitu_b/lateral-low g1 | moving | cancelled | FIRST_ADMISSIBLE_COMPLETE | 0.828 | 63559 | -3.70 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | n/a / n/a | n/a | n/a | 0.078 | 0.105 | 6/179/122 | 0.73 |
| insitu_b/lateral-low g1 | moving | cancelled | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 1.108 | 80465 | -3.42 | yes | 3.70 | axisN_side_337deg | direct | 0.6777 | n/a / n/a | n/a | n/a | 0.082 | 0.077 | 6/192/170 | 0.65 |
| insitu_b/lateral-low g1 | moving | cancelled | ANYTIME_INCUMBENT | 0.828 | 63559 | -3.70 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | n/a / n/a | n/a | n/a | 0.078 | 0.105 | 6/179/122 | 0.73 |
| insitu_b/lateral-low g36 | rest | accepted | FULL_ARGMIN | 0.474 | 25523 | +0.47 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 14/32/68 | 0.00 |
| insitu_b/lateral-low g36 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.025 | 2664 | +0.02 | no | 1.80 | axisP_side_293deg | direct | 0.6931 | 0.0716 / 0.0953 | ··r | ··r | 0.056 | 0.104 | 1/14/1 | 0.90 |
| insitu_b/lateral-low g36 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.466 | 25523 | +0.46 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| insitu_b/lateral-low g36 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.466 | 25523 | +0.46 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| insitu_b/lateral-low g36 | rest | accepted | ANYTIME_INCUMBENT | 0.466 | 25523 | +0.46 | yes | 2.80 | axisN_side_337deg | direct | 0.6215 | 0.0000 / 0.0237 | τgr | ·gr | 0.082 | 0.081 | 4/32/68 | 0.00 |
| insitu_a/longitudinal g1 | moving | cancelled | FIRST_COMPLETE_FEASIBLE | 0.170 | 22359 | -4.35 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / n/a | n/a | n/a | 0.079 | 0.210 | 3/80/1 | 0.91 |
| insitu_a/longitudinal g1 | moving | cancelled | FIRST_ADMISSIBLE_COMPLETE | 0.170 | 22359 | -4.35 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / n/a | n/a | n/a | 0.079 | 0.210 | 3/80/1 | 0.91 |
| insitu_a/longitudinal g1 | moving | cancelled | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.407 | 34443 | -4.12 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / n/a | n/a | n/a | 0.079 | 0.210 | 3/96/34 | 0.86 |
| insitu_a/longitudinal g1 | moving | cancelled | ANYTIME_INCUMBENT | 0.170 | 22359 | -4.35 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / n/a | n/a | n/a | 0.079 | 0.210 | 3/80/1 | 0.91 |
| insitu_a/longitudinal g37 | rest | accepted | FULL_ARGMIN | 0.543 | 28770 | +0.54 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 14/32/102 | 0.00 |
| insitu_a/longitudinal g37 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.016 | 1152 | +0.01 | no | 1.80 | axisP_side_45deg | ring80mm_0of8 | 1.1655 | 0.5156 / 0.5393 | ··· | ··· | 0.022 | 0.037 | 1/3/2 | 0.96 |
| insitu_a/longitudinal g37 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.538 | 28770 | +0.53 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| insitu_a/longitudinal g37 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.538 | 28770 | +0.53 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 4/32/102 | 0.00 |
| insitu_a/longitudinal g37 | rest | accepted | ANYTIME_INCUMBENT | 0.538 | 28770 | +0.53 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| insitu_b/longitudinal g1 | moving | accepted | FULL_ARGMIN | none | | | | | | | | | | | | | | |
| insitu_b/longitudinal g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.139 | 22359 | -4.38 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| insitu_b/longitudinal g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.139 | 22359 | -4.38 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| insitu_b/longitudinal g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.329 | 34443 | -4.19 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/96/34 | 0.87 |
| insitu_b/longitudinal g1 | moving | accepted | ANYTIME_INCUMBENT | 0.139 | 22359 | -4.38 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | n/a / 0.0000 | n/a | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| insitu_b/longitudinal g38 | rest | accepted | FULL_ARGMIN | 0.568 | 28770 | +0.56 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 14/32/102 | 0.00 |
| insitu_b/longitudinal g38 | rest | accepted | FIRST_COMPLETE_FEASIBLE | 0.017 | 1152 | +0.01 | no | 1.80 | axisP_side_45deg | ring80mm_0of8 | 1.1655 | 0.5156 / 0.5393 | ··· | ··· | 0.022 | 0.037 | 1/3/2 | 0.96 |
| insitu_b/longitudinal g38 | rest | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.564 | 28770 | +0.56 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| insitu_b/longitudinal g38 | rest | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.564 | 28770 | +0.56 | yes | 2.80 | axisP_side_337deg | direct | 0.6499 | 0.0000 / 0.0237 | τgr | ·gr | 0.080 | 0.221 | 4/32/102 | 0.00 |
| insitu_b/longitudinal g38 | rest | accepted | ANYTIME_INCUMBENT | 0.564 | 28770 | +0.56 | yes | 2.80 | axisP_side_315deg | direct | 0.6804 | 0.0305 / 0.0542 | τ·r | ··r | 0.080 | 0.209 | 4/32/102 | 0.00 |
| insitu_a/near-ground g1 | moving | accepted | FULL_ARGMIN | 3.238 | 242457 | -1.28 | yes | 8.00 | axisP_side_247deg | ring140mm_5of8 | 1.0367 | 0.0000 / 0.2832 | τgr | ··· | 0.082 | 0.088 | 14/448/408 | 0.00 |
| insitu_a/near-ground g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.016 | 1503 | -4.51 | no | 1.80 | axisP_side_45deg | direct | 0.6811 | -0.3556 / -0.0723 | ··· | ·g· | 0.074 | 0.108 | 1/3/1 | 0.99 |
| insitu_a/near-ground g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 2.660 | 197175 | -1.86 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |
| insitu_a/near-ground g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 2.788 | 208454 | -1.73 | yes | 6.40 | axisP_side_68deg | ring80mm_2of8 | 1.1024 | 0.0657 / 0.3489 | ··· | ··· | 0.047 | 0.049 | 12/384/357 | 0.14 |
| insitu_a/near-ground g1 | moving | accepted | ANYTIME_INCUMBENT | 2.660 | 197175 | -1.86 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |
| insitu_b/near-ground g1 | moving | accepted | FULL_ARGMIN | 3.091 | 242457 | -1.43 | yes | 8.00 | axisP_side_247deg | ring140mm_5of8 | 1.0367 | 0.0000 / 0.2832 | τgr | ··· | 0.082 | 0.088 | 14/448/408 | 0.00 |
| insitu_b/near-ground g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.015 | 1503 | -4.51 | no | 1.80 | axisP_side_45deg | direct | 0.6811 | -0.3556 / -0.0723 | ··· | ·g· | 0.074 | 0.108 | 1/3/1 | 0.99 |
| insitu_b/near-ground g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 2.541 | 197175 | -1.98 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |
| insitu_b/near-ground g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 2.666 | 208454 | -1.86 | yes | 6.40 | axisP_side_68deg | ring80mm_2of8 | 1.1024 | 0.0657 / 0.3489 | ··· | ··· | 0.047 | 0.049 | 12/384/357 | 0.14 |
| insitu_b/near-ground g1 | moving | accepted | ANYTIME_INCUMBENT | 2.541 | 197175 | -1.98 | yes | 6.40 | axisP_side_68deg | direct | 1.2553 | 0.2186 / 0.5018 | ··· | ··· | 0.030 | 0.049 | 12/356/341 | 0.19 |

### Does exhaustive argmin behave like 'earliest timing-admissible τ, then best (g,r)'?

- zero latency (both evaluated with every record admitted at the search epoch), moving: n=5; P(τ equal)=5/5, P(g equal)=5/5, P(r equal)=5/5, P(tuple equal)=5/5
- zero latency (both evaluated with every record admitted at the search epoch), rest: n=4; P(τ equal)=4/4, P(g equal)=4/4, P(r equal)=4/4, P(tuple equal)=4/4
- zero latency (both evaluated with every record admitted at the search epoch), all: n=9; P(τ equal)=9/9, P(g equal)=9/9, P(r equal)=9/9, P(tuple equal)=9/9
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), moving: n=4; P(τ equal)=0/4, P(g equal)=2/4, P(r equal)=0/4, P(tuple equal)=0/4
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), rest: n=4; P(τ equal)=4/4, P(g equal)=4/4, P(r equal)=4/4, P(tuple equal)=4/4
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), all: n=8; P(τ equal)=4/8, P(g equal)=6/8, P(r equal)=4/8, P(tuple equal)=4/8
- completed searches in which FULL_ARGMIN had no admissible plan at receipt: 1 (insitu_b/longitudinal g1); bounded policies with an admissible plan in those: FIRST_COMPLETE_FEASIBLE=1, FIRST_ADMISSIBLE_COMPLETE=1, EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR=1, ANYTIME_INCUMBENT=1

### ANYTIME_INCUMBENT evolution (moving-epoch searches)

| search | first incumbent wall (s) | incumbent changes | incumbent lost to expiry | incumbent J (ΔJ vs zero-latency argmin) at 10% / 25% / 50% / 75% / 100% of the full search wall |
|---|---:|---:|---:|---|
| insitu_a/diagonal g1 (accepted) | 0.398 | 9 | 0 | none / 0.682 (0.000) τ=4.15 / 0.691 (0.009) τ=4.15 / 0.720 (0.038) τ=4.60 / 0.775 (0.093) τ=5.50 |
| insitu_a/lateral-low g1 (cancelled) | 0.863 | 13 | 7 | none / none / none / 0.783 (n/a) τ=5.50 / none |
| insitu_a/longitudinal g1 (cancelled) | 0.170 | 17 | 4 | 0.621 (n/a) τ=2.35 / 0.663 (n/a) τ=3.25 / 0.722 (n/a) τ=4.15 / none / none |
| insitu_a/near-ground g1 (accepted) | 2.660 | 6 | 0 | none / none / none / none / 1.037 (0.283) τ=8.00 |
| insitu_b/diagonal g1 (accepted) | 0.393 | 9 | 0 | none / 0.682 (0.000) τ=4.15 / 0.691 (0.009) τ=4.15 / 0.720 (0.038) τ=4.60 / 0.775 (0.093) τ=5.50 |
| insitu_b/lateral-low g1 (cancelled) | 0.828 | 23 | 3 | none / none / 0.720 (n/a) τ=4.60 / 0.783 (n/a) τ=5.50 / none |
| insitu_b/longitudinal g1 (accepted) | 0.139 | 15 | 0 | 0.621 (0.000) τ=2.35 / 0.663 (0.041) τ=3.25 / 0.722 (0.101) τ=4.15 / 0.786 (0.165) τ=5.05 / none |
| insitu_b/near-ground g1 (accepted) | 2.541 | 6 | 0 | none / none / none / none / 1.037 (0.283) τ=8.00 |

### Computation before the stop (moving-epoch searches; worker wall in s, bounded work units in parentheses)

| search | policy | temporal hypothesis setup | grasp/static reach screening | insertion/closure certification | route (transit reach) certification | carried-retreat certification | cost / terminal timing audit | nested: IK steps | nested: swept-volume queries | nested: configuration safety | nested: closure/contact safety |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| insitu_a/diagonal g1 | FULL_ARGMIN | 0.000 (14) | 0.568 (89105) | 0.364 (56528) | 1.635 (50614) | 0.193 (25631) | 0.185 (267) | 1.296 (195256) | 1.099 (49798) | 0.337 (220512) | 0.279 (53070) |
| insitu_a/diagonal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (7) | 0.285 (44564) | 0.058 (8847) | 0.003 (99) | 0.048 (6401) | 0.001 (1) | 0.349 (53874) | 0.002 (97) | 0.071 (47527) | 0.033 (6206) |
| insitu_a/diagonal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (7) | 0.285 (44564) | 0.058 (8847) | 0.003 (99) | 0.048 (6401) | 0.001 (1) | 0.349 (53874) | 0.002 (97) | 0.071 (47527) | 0.033 (6206) |
| insitu_a/diagonal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (7) | 0.298 (46577) | 0.073 (11239) | 0.068 (2058) | 0.058 (7700) | 0.010 (14) | 0.394 (60531) | 0.046 (2024) | 0.083 (54865) | 0.045 (8508) |
| insitu_a/diagonal g1 | ANYTIME_INCUMBENT | 0.000 (7) | 0.285 (44564) | 0.058 (8847) | 0.003 (99) | 0.048 (6401) | 0.001 (1) | 0.349 (53874) | 0.002 (97) | 0.071 (47527) | 0.033 (6206) |
| insitu_a/lateral-low g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.018 (2558) | 0.009 (1206) | 0.054 (1785) | 0.001 (94) | 0.001 (1) | 0.033 (4822) | 0.035 (1761) | 0.010 (6495) | 0.006 (906) |
| insitu_a/lateral-low g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.163 (25428) | 0.103 (16068) | 0.515 (17454) | 0.034 (4541) | 0.043 (62) | 0.357 (55093) | 0.339 (17213) | 0.097 (68011) | 0.077 (14666) |
| insitu_a/lateral-low g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.180 (28101) | 0.136 (20997) | 0.732 (24403) | 0.053 (6862) | 0.068 (96) | 0.460 (70087) | 0.482 (24067) | 0.126 (87673) | 0.107 (19945) |
| insitu_a/lateral-low g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.163 (25428) | 0.103 (16068) | 0.515 (17454) | 0.034 (4541) | 0.043 (62) | 0.357 (55093) | 0.339 (17213) | 0.097 (68011) | 0.077 (14666) |
| insitu_a/longitudinal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (3) | 0.127 (17601) | 0.030 (3851) | 0.004 (94) | 0.007 (809) | 0.001 (1) | 0.143 (19764) | 0.002 (92) | 0.032 (19033) | 0.017 (2723) |
| insitu_a/longitudinal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (3) | 0.127 (17601) | 0.030 (3851) | 0.004 (94) | 0.007 (809) | 0.001 (1) | 0.143 (19764) | 0.002 (92) | 0.032 (19033) | 0.017 (2723) |
| insitu_a/longitudinal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (3) | 0.149 (20664) | 0.051 (6672) | 0.169 (4631) | 0.021 (2455) | 0.015 (18) | 0.230 (30636) | 0.107 (4564) | 0.055 (32805) | 0.034 (5567) |
| insitu_a/longitudinal g1 | ANYTIME_INCUMBENT | 0.000 (3) | 0.127 (17601) | 0.030 (3851) | 0.004 (94) | 0.007 (809) | 0.001 (1) | 0.143 (19764) | 0.002 (92) | 0.032 (19033) | 0.017 (2723) |
| insitu_a/near-ground g1 | FULL_ARGMIN | 0.000 (14) | 0.536 (84783) | 0.488 (77104) | 1.863 (62800) | 0.130 (17500) | 0.206 (256) | 1.458 (226353) | 1.228 (61988) | 0.387 (271847) | 0.241 (45959) |
| insitu_a/near-ground g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.007 (965) | 0.002 (297) | 0.005 (153) | 0.001 (86) | 0.001 (1) | 0.010 (1367) | 0.003 (151) | 0.002 (1435) | 0.002 (237) |
| insitu_a/near-ground g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (12) | 0.483 (76434) | 0.343 (54667) | 1.523 (51181) | 0.109 (14647) | 0.189 (234) | 1.179 (183096) | 1.004 (50501) | 0.313 (219890) | 0.218 (41679) |
| insitu_a/near-ground g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (12) | 0.500 (79024) | 0.383 (60776) | 1.587 (53365) | 0.112 (15037) | 0.194 (240) | 1.248 (193916) | 1.047 (52655) | 0.331 (232495) | 0.224 (42772) |
| insitu_a/near-ground g1 | ANYTIME_INCUMBENT | 0.000 (12) | 0.483 (76434) | 0.343 (54667) | 1.523 (51181) | 0.109 (14647) | 0.189 (234) | 1.179 (183096) | 1.004 (50501) | 0.313 (219890) | 0.218 (41679) |
| insitu_b/diagonal g1 | FULL_ARGMIN | 0.000 (14) | 0.557 (89105) | 0.347 (56528) | 1.569 (50614) | 0.185 (25631) | 0.176 (267) | 1.250 (195256) | 1.057 (49798) | 0.325 (220512) | 0.267 (53070) |
| insitu_b/diagonal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (7) | 0.282 (44564) | 0.057 (8847) | 0.003 (99) | 0.047 (6401) | 0.001 (1) | 0.344 (53874) | 0.002 (97) | 0.071 (47527) | 0.033 (6206) |
| insitu_b/diagonal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (7) | 0.282 (44564) | 0.057 (8847) | 0.003 (99) | 0.047 (6401) | 0.001 (1) | 0.344 (53874) | 0.002 (97) | 0.071 (47527) | 0.033 (6206) |
| insitu_b/diagonal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (7) | 0.295 (46577) | 0.072 (11239) | 0.066 (2058) | 0.056 (7700) | 0.010 (14) | 0.388 (60531) | 0.045 (2024) | 0.082 (54865) | 0.044 (8508) |
| insitu_b/diagonal g1 | ANYTIME_INCUMBENT | 0.000 (7) | 0.282 (44564) | 0.057 (8847) | 0.003 (99) | 0.047 (6401) | 0.001 (1) | 0.344 (53874) | 0.002 (97) | 0.071 (47527) | 0.033 (6206) |
| insitu_b/lateral-low g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.017 (2558) | 0.009 (1206) | 0.051 (1785) | 0.001 (94) | 0.001 (1) | 0.032 (4822) | 0.033 (1761) | 0.010 (6495) | 0.006 (906) |
| insitu_b/lateral-low g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.156 (25428) | 0.100 (16068) | 0.494 (17454) | 0.032 (4541) | 0.042 (62) | 0.342 (55093) | 0.325 (17213) | 0.094 (68011) | 0.075 (14666) |
| insitu_b/lateral-low g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.172 (28101) | 0.129 (20997) | 0.688 (24403) | 0.049 (6862) | 0.064 (96) | 0.437 (70087) | 0.452 (24067) | 0.121 (87673) | 0.101 (19945) |
| insitu_b/lateral-low g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.156 (25428) | 0.100 (16068) | 0.494 (17454) | 0.032 (4541) | 0.042 (62) | 0.342 (55093) | 0.325 (17213) | 0.094 (68011) | 0.075 (14666) |
| insitu_b/longitudinal g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (3) | 0.105 (17601) | 0.024 (3851) | 0.003 (94) | 0.005 (809) | 0.001 (1) | 0.118 (19764) | 0.002 (92) | 0.027 (19033) | 0.014 (2723) |
| insitu_b/longitudinal g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (3) | 0.105 (17601) | 0.024 (3851) | 0.003 (94) | 0.005 (809) | 0.001 (1) | 0.118 (19764) | 0.002 (92) | 0.027 (19033) | 0.014 (2723) |
| insitu_b/longitudinal g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (3) | 0.123 (20664) | 0.040 (6672) | 0.136 (4631) | 0.016 (2455) | 0.011 (18) | 0.184 (30636) | 0.091 (4564) | 0.045 (32805) | 0.027 (5567) |
| insitu_b/longitudinal g1 | ANYTIME_INCUMBENT | 0.000 (3) | 0.105 (17601) | 0.024 (3851) | 0.003 (94) | 0.005 (809) | 0.001 (1) | 0.118 (19764) | 0.002 (92) | 0.027 (19033) | 0.014 (2723) |
| insitu_b/near-ground g1 | FULL_ARGMIN | 0.000 (14) | 0.509 (84783) | 0.469 (77104) | 1.780 (62800) | 0.124 (17500) | 0.195 (256) | 1.387 (226353) | 1.176 (61988) | 0.369 (271847) | 0.232 (45959) |
| insitu_b/near-ground g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (1) | 0.007 (965) | 0.002 (297) | 0.005 (153) | 0.001 (86) | 0.001 (1) | 0.010 (1367) | 0.003 (151) | 0.002 (1435) | 0.001 (237) |
| insitu_b/near-ground g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (12) | 0.459 (76434) | 0.332 (54667) | 1.454 (51181) | 0.104 (14647) | 0.180 (234) | 1.123 (183096) | 0.961 (50501) | 0.299 (219890) | 0.211 (41679) |
| insitu_b/near-ground g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (12) | 0.475 (79024) | 0.370 (60776) | 1.518 (53365) | 0.107 (15037) | 0.185 (240) | 1.190 (193916) | 1.004 (52655) | 0.316 (232495) | 0.216 (42772) |
| insitu_b/near-ground g1 | ANYTIME_INCUMBENT | 0.000 (12) | 0.459 (76434) | 0.332 (54667) | 1.454 (51181) | 0.104 (14647) | 0.180 (234) | 1.123 (183096) | 0.961 (50501) | 0.299 (219890) | 0.211 (41679) |

### Per-candidate certification cost and work before the first complete certified record

| search | kind | full wall (s) | static screens rejected / passed (median ms) | route rollouts rejected / complete (median ms) | before first complete: rejected screens, passed screens, rejected routes, hypotheses | wall of rejected screens / passed screens / rejected routes / the successful route (s) |
|---|---|---:|---|---|---|---|
| insitu_a/diagonal g1 | moving | 2.961 | 424 (1.33) / 24 (1.77) | 141 (3.41) / 267 (6.02) | 207, 1, 0, 7 | 0.390 / 0.002 / 0.000 / 0.005 |
| insitu_b/diagonal g1 | moving | 2.848 | 424 (1.29) / 24 (1.72) | 141 (3.34) / 267 (5.76) | 207, 1, 0, 7 | 0.386 / 0.002 / 0.000 / 0.005 |
| insitu_a/lateral-low g1 | moving | 3.893 | 366 (0.57) / 35 (1.88) | 275 (4.62) / 318 (6.59) | 19, 1, 11, 1 | 0.025 / 0.002 / 0.049 / 0.006 |
| insitu_a/lateral-low g38 | rest | 0.491 | 28 (0.93) / 4 (1.86) | 26 (4.87) / 42 (6.91) | 13, 1, 0, 1 | 0.016 / 0.002 / 0.000 / 0.010 |
| insitu_b/lateral-low g1 | moving | 3.894 | 367 (0.53) / 37 (1.78) | 291 (4.45) / 334 (6.38) | 19, 1, 11, 1 | 0.024 / 0.002 / 0.047 / 0.007 |
| insitu_b/lateral-low g36 | rest | 0.475 | 28 (0.87) / 4 (1.84) | 26 (4.72) / 42 (6.44) | 13, 1, 0, 1 | 0.013 / 0.002 / 0.000 / 0.009 |
| insitu_a/longitudinal g1 | moving | 3.893 | 396 (1.51) / 35 (1.95) | 357 (3.87) / 232 (6.57) | 79, 1, 0, 3 | 0.162 / 0.002 / 0.000 / 0.006 |
| insitu_a/longitudinal g37 | rest | 0.545 | 26 (1.03) / 6 (1.90) | 59 (3.22) / 43 (6.19) | 2, 1, 1, 1 | 0.004 / 0.002 / 0.003 / 0.007 |
| insitu_b/longitudinal g1 | moving | 3.617 | 413 (1.32) / 35 (1.77) | 363 (3.36) / 232 (6.00) | 79, 1, 0, 3 | 0.133 / 0.001 / 0.000 / 0.005 |
| insitu_b/longitudinal g38 | rest | 0.570 | 26 (0.99) / 6 (2.00) | 59 (3.17) / 43 (6.78) | 2, 1, 1, 1 | 0.004 / 0.002 / 0.003 / 0.008 |
| insitu_a/near-ground g1 | moving | 3.239 | 424 (0.56) / 24 (2.02) | 152 (4.49) / 256 (6.71) | 2, 1, 0, 1 | 0.007 / 0.002 / 0.000 / 0.007 |
| insitu_b/near-ground g1 | moving | 3.092 | 424 (0.53) / 24 (1.88) | 152 (4.32) / 256 (6.45) | 2, 1, 0, 1 | 0.006 / 0.002 / 0.000 / 0.007 |
