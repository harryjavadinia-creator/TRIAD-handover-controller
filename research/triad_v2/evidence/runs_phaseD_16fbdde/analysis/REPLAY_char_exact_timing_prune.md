
### Policy comparison — moving-epoch searches that completed (8 searches)

| policy | plan found | admissible & before reach-start deadline | median wall to plan (s) | median work units | median fraction of full work avoided | median t_plan − t_rest (s) | τ = FULL_ARGMIN τ | tuple = FULL_ARGMIN | median / max ΔJ vs FULL_ARGMIN | τ = zero-latency argmin τ | tuple = zero-latency argmin | median / max ΔJ vs zero-latency argmin | median Δ reach clearance (m) | median Δ retreat clearance (m) |
|---|---|---|---:|---:|---:|---:|---|---|---|---|---|---|---:|---:|
| FULL_ARGMIN | 8/8 | 8/8 | 3.072 | 227094 | 0.00 | -1.45 | 100% (8) | 100% (8) | 0.0000 / 0.0000 | 0% (8) | 0% (8) | 0.2068 / 0.2544 | 0.0000 | 0.0000 |
| FIRST_COMPLETE_FEASIBLE | 8/8 | 6/8 | 0.251 | 36720 | 0.81 | -4.27 | 0% (8) | 0% (8) | -0.0738 / -0.0437 | 50% (8) | 25% (8) | 0.0899 / 0.1350 | -0.0065 | 0.0102 |
| FIRST_ADMISSIBLE_COMPLETE | 8/8 | 8/8 | 0.312 | 41968 | 0.79 | -4.21 | 0% (8) | 0% (8) | -0.0519 / 0.0781 | 75% (8) | 25% (8) | 0.0920 / 0.2964 | -0.0032 | 0.0089 |
| EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 8/8 | 8/8 | 0.458 | 56910 | 0.73 | -4.07 | 0% (8) | 0% (8) | -0.1857 / -0.0927 | 75% (8) | 75% (8) | 0.0000 / 0.0422 | 0.0049 | 0.0032 |
| ANYTIME_INCUMBENT | 8/8 | 8/8 | 0.312 | 41968 | 0.79 | -4.21 | 0% (8) | 0% (8) | -0.0519 / 0.0781 | 75% (8) | 25% (8) | 0.0920 / 0.2964 | -0.0032 | 0.0089 |

### Per search: plan availability, tuple, cost and clearance

ΔJ: vs FULL_ARGMIN at its own receipt / vs the zero-latency exhaustive argmin (all records admitted at the search epoch). H/G/R: temporal hypotheses touched / grasp screens / route rollouts up to the stop. Deadline: available with ≥1.6 s to τ and reach duration + 0.05 s ≤ τ − t.

| search | kind | outcome | policy | wall to plan (s) | work units | t − t_rest (s) | deadline ok | τ lead (s) | g | r | J | ΔJ vs FULL / vs zero-latency | same τ,g,r as FULL | same τ,g,r as zero-latency | reach clr (m) | retreat clr (m) | H/G/R | full work avoided |
|---|---|---|---|---:|---:|---:|---|---:|---|---|---:|---|---|---|---:|---:|---|---:|
| char_a/diagonal [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 2.816 | 222159 | -1.71 | yes | 5.50 | axisP_side_337deg | ring80mm_2of8 | 0.7746 | 0.0000 / 0.0927 | τgr | ·g· | 0.074 | 0.223 | 14/448/408 | 0.00 |
| char_a/diagonal [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.369 | 59919 | -4.15 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_a/diagonal [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.369 | 59919 | -4.15 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_a/diagonal [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.472 | 67595 | -4.05 | yes | 4.15 | axisP_side_337deg | ring140mm_2of8 | 0.6820 | -0.0927 / 0.0000 | ·g· | τgr | 0.072 | 0.218 | 7/224/17 | 0.70 |
| char_a/diagonal [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.369 | 59919 | -4.15 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_b/diagonal [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 2.880 | 222159 | -1.64 | yes | 5.50 | axisP_side_337deg | ring80mm_2of8 | 0.7746 | 0.0000 / 0.0927 | τgr | ·g· | 0.074 | 0.223 | 14/448/408 | 0.00 |
| char_b/diagonal [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.384 | 59919 | -4.14 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_b/diagonal [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.384 | 59919 | -4.14 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_b/diagonal [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.496 | 67595 | -4.03 | yes | 4.15 | axisP_side_337deg | ring140mm_2of8 | 0.6820 | -0.0927 / 0.0000 | ·g· | τgr | 0.072 | 0.218 | 7/224/17 | 0.70 |
| char_b/diagonal [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.384 | 59919 | -4.14 | yes | 4.15 | axisP_side_337deg | direct | 0.7310 | -0.0437 / 0.0490 | ·g· | τg· | 0.070 | 0.217 | 7/208/1 | 0.73 |
| char_a/lateral-low [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 3.983 | 232028 | -0.54 | yes | 8.00 | axisN_side_23deg | ring80mm_6of8 | 0.9321 | 0.0000 / 0.2544 | τgr | ··· | 0.069 | 0.099 | 14/448/612 | 0.00 |
| char_a/lateral-low [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.214 | 28977 | -4.31 | no | 3.25 | axisN_side_68deg | ring80mm_5of8 | 0.8086 | -0.1235 / 0.1308 | ··· | ··· | 0.048 | 0.108 | 5/148/7 | 0.88 |
| char_a/lateral-low [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.331 | 39472 | -4.19 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0420 / 0.2964 | ··· | τ·· | 0.078 | 0.105 | 6/179/20 | 0.83 |
| char_a/lateral-low [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.624 | 56378 | -3.90 | yes | 3.70 | axisN_side_337deg | direct | 0.6777 | -0.2544 / 0.0000 | ··· | τgr | 0.082 | 0.077 | 6/192/68 | 0.76 |
| char_a/lateral-low [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.331 | 39472 | -4.19 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0420 / 0.2964 | ··· | τ·· | 0.078 | 0.105 | 6/179/20 | 0.83 |
| char_b/lateral-low [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 3.884 | 232028 | -0.64 | yes | 6.85 | axisN_side_45deg | ring80mm_6of8 | 0.8961 | 0.0000 / 0.2183 | τgr | ··· | 0.064 | 0.100 | 14/448/612 | 0.00 |
| char_b/lateral-low [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.214 | 28977 | -4.31 | no | 3.25 | axisN_side_68deg | ring80mm_5of8 | 0.8086 | -0.0875 / 0.1308 | ··· | ··· | 0.048 | 0.108 | 5/148/7 | 0.88 |
| char_b/lateral-low [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.332 | 39472 | -4.19 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0781 / 0.2964 | ·g· | τ·· | 0.078 | 0.105 | 6/179/20 | 0.83 |
| char_b/lateral-low [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.628 | 56378 | -3.90 | yes | 3.70 | axisN_side_337deg | direct | 0.6777 | -0.2183 / 0.0000 | ··· | τgr | 0.082 | 0.077 | 6/192/68 | 0.76 |
| char_b/lateral-low [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.332 | 39472 | -4.19 | yes | 3.70 | axisN_side_45deg | ring80mm_1of8 | 0.9741 | 0.0781 / 0.2964 | ·g· | τ·· | 0.078 | 0.105 | 6/179/20 | 0.83 |
| char_a/longitudinal [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 3.306 | 240904 | -1.22 | yes | 5.95 | axisP_side_337deg | ring80mm_1of8 | 0.8515 | 0.0000 / 0.2302 | τgr | ·g· | 0.081 | 0.177 | 14/448/476 | 0.00 |
| char_a/longitudinal [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.168 | 22359 | -4.36 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_a/longitudinal [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.168 | 22359 | -4.36 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_a/longitudinal [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.307 | 31809 | -4.22 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/96/17 | 0.87 |
| char_a/longitudinal [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.168 | 22359 | -4.36 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_b/longitudinal [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 3.265 | 242818 | -1.26 | yes | 5.95 | axisP_side_337deg | ring80mm_1of8 | 0.8515 | 0.0000 / 0.2302 | τgr | ·g· | 0.081 | 0.177 | 14/448/493 | 0.00 |
| char_b/longitudinal [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.151 | 22359 | -4.37 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_b/longitudinal [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.151 | 22359 | -4.37 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_b/longitudinal [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.281 | 31809 | -4.24 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/96/17 | 0.87 |
| char_b/longitudinal [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.151 | 22359 | -4.37 | yes | 2.35 | axisP_side_337deg | direct | 0.6214 | -0.2302 / 0.0000 | ·g· | τgr | 0.079 | 0.210 | 3/80/1 | 0.91 |
| char_a/near-ground [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 1.798 | 171989 | -2.72 | yes | 5.05 | axisP_side_45deg | ring80mm_1of8 | 0.9487 | 0.0000 / 0.1952 | τgr | ·g· | 0.053 | 0.090 | 14/448/170 | 0.00 |
| char_a/near-ground [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.288 | 44463 | -4.23 | yes | 3.70 | axisP_side_45deg | direct | 0.8885 | -0.0602 / 0.1350 | ·g· | ·g· | 0.045 | 0.102 | 6/163/1 | 0.74 |
| char_a/near-ground [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.288 | 44463 | -4.23 | yes | 3.70 | axisP_side_45deg | direct | 0.8885 | -0.0602 / 0.1350 | ·g· | ·g· | 0.045 | 0.102 | 6/163/1 | 0.74 |
| char_a/near-ground [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.440 | 57443 | -4.08 | yes | 3.70 | axisP_side_45deg | ring80mm_7of8 | 0.7956 | -0.1531 / 0.0422 | ·g· | ·gr | 0.065 | 0.101 | 6/192/17 | 0.67 |
| char_a/near-ground [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.288 | 44463 | -4.23 | yes | 3.70 | axisP_side_45deg | direct | 0.8885 | -0.0602 / 0.1350 | ·g· | ·g· | 0.045 | 0.102 | 6/163/1 | 0.74 |
| char_b/near-ground [exact timing prune] g1 | moving | accepted | FULL_ARGMIN | 1.790 | 171989 | -2.73 | yes | 5.05 | axisP_side_45deg | ring80mm_1of8 | 0.9487 | 0.0000 / 0.1952 | τgr | ·g· | 0.053 | 0.090 | 14/448/170 | 0.00 |
| char_b/near-ground [exact timing prune] g1 | moving | accepted | FIRST_COMPLETE_FEASIBLE | 0.293 | 44463 | -4.23 | yes | 3.70 | axisP_side_45deg | direct | 0.8885 | -0.0602 / 0.1350 | ·g· | ·g· | 0.045 | 0.102 | 6/163/1 | 0.74 |
| char_b/near-ground [exact timing prune] g1 | moving | accepted | FIRST_ADMISSIBLE_COMPLETE | 0.293 | 44463 | -4.23 | yes | 3.70 | axisP_side_45deg | direct | 0.8885 | -0.0602 / 0.1350 | ·g· | ·g· | 0.045 | 0.102 | 6/163/1 | 0.74 |
| char_b/near-ground [exact timing prune] g1 | moving | accepted | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.444 | 57443 | -4.08 | yes | 3.70 | axisP_side_45deg | ring80mm_7of8 | 0.7956 | -0.1531 / 0.0422 | ·g· | ·gr | 0.065 | 0.101 | 6/192/17 | 0.67 |
| char_b/near-ground [exact timing prune] g1 | moving | accepted | ANYTIME_INCUMBENT | 0.293 | 44463 | -4.23 | yes | 3.70 | axisP_side_45deg | direct | 0.8885 | -0.0602 / 0.1350 | ·g· | ·g· | 0.045 | 0.102 | 6/163/1 | 0.74 |

### Does exhaustive argmin behave like 'earliest timing-admissible τ, then best (g,r)'?

- zero latency (both evaluated with every record admitted at the search epoch), moving: n=8; P(τ equal)=4/8, P(g equal)=6/8, P(r equal)=4/8, P(tuple equal)=4/8
- zero latency (both evaluated with every record admitted at the search epoch), all: n=8; P(τ equal)=4/8, P(g equal)=6/8, P(r equal)=4/8, P(tuple equal)=4/8
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), moving: n=8; P(τ equal)=0/8, P(g equal)=6/8, P(r equal)=0/8, P(tuple equal)=0/8
- as executed (each policy decides at its own time; FULL_ARGMIN at receipt), all: n=8; P(τ equal)=0/8, P(g equal)=6/8, P(r equal)=0/8, P(tuple equal)=0/8
- completed searches in which FULL_ARGMIN had no admissible plan at receipt: 0 (none); bounded policies with an admissible plan in those: FIRST_COMPLETE_FEASIBLE=0, FIRST_ADMISSIBLE_COMPLETE=0, EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR=0, ANYTIME_INCUMBENT=0

### ANYTIME_INCUMBENT evolution (moving-epoch searches)

| search | first incumbent wall (s) | incumbent changes | incumbent lost to expiry | incumbent J (ΔJ vs zero-latency argmin) at 10% / 25% / 50% / 75% / 100% of the full search wall |
|---|---:|---:|---:|---|
| char_a/diagonal [exact timing prune] g1 (accepted) | 0.369 | 9 | 0 | none / 0.682 (0.000) τ=4.15 / 0.691 (0.009) τ=4.15 / 0.720 (0.038) τ=4.60 / 0.775 (0.093) τ=5.50 |
| char_a/lateral-low [exact timing prune] g1 (accepted) | 0.331 | 16 | 0 | 0.738 (0.060) τ=3.70 / 0.678 (0.000) τ=3.70 / 0.720 (0.042) τ=4.60 / 0.783 (0.105) τ=5.50 / 0.932 (0.254) τ=8.00 |
| char_a/longitudinal [exact timing prune] g1 (accepted) | 0.168 | 11 | 0 | 0.621 (0.000) τ=2.35 / 0.644 (0.022) τ=3.25 / 0.715 (0.093) τ=4.15 / 0.782 (0.161) τ=5.05 / 0.852 (0.230) τ=5.95 |
| char_a/near-ground [exact timing prune] g1 (accepted) | 0.288 | 7 | 0 | none / 0.796 (0.042) τ=3.70 / 0.839 (0.085) τ=4.15 / 0.896 (0.143) τ=4.60 / 0.949 (0.195) τ=5.05 |
| char_b/diagonal [exact timing prune] g1 (accepted) | 0.384 | 9 | 0 | none / 0.682 (0.000) τ=4.15 / 0.691 (0.009) τ=4.15 / 0.720 (0.038) τ=4.60 / 0.775 (0.093) τ=5.50 |
| char_b/lateral-low [exact timing prune] g1 (accepted) | 0.332 | 15 | 0 | 0.738 (0.060) τ=3.70 / 0.678 (0.000) τ=3.70 / 0.720 (0.042) τ=4.60 / 0.783 (0.105) τ=5.50 / 0.896 (0.218) τ=6.85 |
| char_b/longitudinal [exact timing prune] g1 (accepted) | 0.151 | 11 | 0 | 0.621 (0.000) τ=2.35 / 0.644 (0.022) τ=3.25 / 0.715 (0.093) τ=4.15 / 0.782 (0.161) τ=5.05 / 0.852 (0.230) τ=5.95 |
| char_b/near-ground [exact timing prune] g1 (accepted) | 0.293 | 7 | 0 | none / 0.796 (0.042) τ=3.70 / 0.839 (0.085) τ=4.15 / 0.896 (0.143) τ=4.60 / 0.949 (0.195) τ=5.05 |

### Computation before the stop (moving-epoch searches; worker wall in s, bounded work units in parentheses)

| search | policy | temporal hypothesis setup | grasp/static reach screening | insertion/closure certification | route (transit reach) certification | carried-retreat certification | cost / terminal timing audit | nested: IK steps | nested: swept-volume queries | nested: configuration safety | nested: closure/contact safety |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| char_a/diagonal [exact timing prune] g1 | FULL_ARGMIN | 0.000 (14) | 0.533 (89105) | 0.342 (56528) | 1.573 (50614) | 0.180 (25631) | 0.175 (267) | 1.220 (195256) | 1.058 (49798) | 0.318 (220512) | 0.263 (53070) |
| char_a/diagonal [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (7) | 0.266 (44564) | 0.053 (8847) | 0.003 (99) | 0.044 (6401) | 0.001 (1) | 0.324 (53874) | 0.002 (97) | 0.067 (47527) | 0.030 (6206) |
| char_a/diagonal [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (7) | 0.266 (44564) | 0.053 (8847) | 0.003 (99) | 0.044 (6401) | 0.001 (1) | 0.324 (53874) | 0.002 (97) | 0.067 (47527) | 0.030 (6206) |
| char_a/diagonal [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (7) | 0.278 (46577) | 0.067 (11239) | 0.062 (2058) | 0.053 (7700) | 0.009 (14) | 0.365 (60531) | 0.042 (2024) | 0.077 (54865) | 0.041 (8508) |
| char_a/diagonal [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (7) | 0.266 (44564) | 0.053 (8847) | 0.003 (99) | 0.044 (6401) | 0.001 (1) | 0.324 (53874) | 0.002 (97) | 0.067 (47527) | 0.030 (6206) |
| char_a/lateral-low [exact timing prune] g1 | FULL_ARGMIN | 0.001 (14) | 0.318 (50616) | 0.475 (74623) | 3.365 (111001) | 0.197 (26420) | 0.266 (382) | 1.495 (227759) | 2.220 (109551) | 0.453 (312528) | 0.385 (73015) |
| char_a/lateral-low [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (5) | 0.129 (20375) | 0.086 (13711) | 0.440 (15408) | 0.024 (3509) | 0.037 (56) | 0.289 (45945) | 0.290 (15192) | 0.080 (57851) | 0.065 (12631) |
| char_a/lateral-low [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.160 (25428) | 0.101 (16068) | 0.499 (17454) | 0.032 (4541) | 0.041 (62) | 0.347 (55093) | 0.329 (17213) | 0.095 (68011) | 0.076 (14666) |
| char_a/lateral-low [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.176 (28101) | 0.132 (20997) | 0.703 (24403) | 0.049 (6862) | 0.065 (96) | 0.444 (70087) | 0.463 (24067) | 0.122 (87673) | 0.104 (19945) |
| char_a/lateral-low [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.160 (25428) | 0.101 (16068) | 0.499 (17454) | 0.032 (4541) | 0.041 (62) | 0.347 (55093) | 0.329 (17213) | 0.095 (68011) | 0.076 (14666) |
| char_a/longitudinal [exact timing prune] g1 | FULL_ARGMIN | 0.000 (14) | 0.640 (99852) | 0.390 (60867) | 2.429 (74270) | 0.155 (20907) | 0.161 (232) | 1.509 (230244) | 1.651 (73110) | 0.421 (283492) | 0.254 (48311) |
| char_a/longitudinal [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (3) | 0.128 (17601) | 0.029 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.142 (19764) | 0.002 (92) | 0.032 (19033) | 0.016 (2723) |
| char_a/longitudinal [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (3) | 0.128 (17601) | 0.029 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.142 (19764) | 0.002 (92) | 0.032 (19033) | 0.016 (2723) |
| char_a/longitudinal [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (3) | 0.148 (20664) | 0.047 (6672) | 0.154 (4631) | 0.020 (2455) | 0.014 (18) | 0.222 (30636) | 0.099 (4564) | 0.053 (32805) | 0.032 (5567) |
| char_a/longitudinal [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (3) | 0.128 (17601) | 0.029 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.142 (19764) | 0.002 (92) | 0.032 (19033) | 0.016 (2723) |
| char_a/near-ground [exact timing prune] g1 | FULL_ARGMIN | 0.000 (14) | 0.526 (84783) | 0.493 (77104) | 1.835 (62800) | 0.130 (17500) | 0.206 (256) | 1.450 (226353) | 1.205 (61988) | 0.382 (271847) | 0.241 (45959) |
| char_a/near-ground [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (6) | 0.251 (40197) | 0.176 (28303) | 1.069 (36681) | 0.070 (9567) | 0.121 (177) | 0.662 (103786) | 0.706 (36203) | 0.182 (131164) | 0.149 (28829) |
| char_a/near-ground [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.251 (40197) | 0.176 (28303) | 1.069 (36681) | 0.070 (9567) | 0.121 (177) | 0.662 (103786) | 0.706 (36203) | 0.182 (131164) | 0.149 (28829) |
| char_a/near-ground [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.299 (48170) | 0.189 (30298) | 1.146 (39153) | 0.075 (10095) | 0.130 (189) | 0.738 (115982) | 0.757 (38643) | 0.202 (145312) | 0.160 (30848) |
| char_a/near-ground [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.251 (40197) | 0.176 (28303) | 1.069 (36681) | 0.070 (9567) | 0.121 (177) | 0.662 (103786) | 0.706 (36203) | 0.182 (131164) | 0.149 (28829) |
| char_b/diagonal [exact timing prune] g1 | FULL_ARGMIN | 0.000 (14) | 0.551 (89105) | 0.350 (56528) | 1.597 (50614) | 0.184 (25631) | 0.185 (267) | 1.257 (195256) | 1.075 (49798) | 0.333 (220512) | 0.270 (53070) |
| char_b/diagonal [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (7) | 0.276 (44564) | 0.056 (8847) | 0.003 (99) | 0.046 (6401) | 0.001 (1) | 0.337 (53874) | 0.002 (97) | 0.069 (47527) | 0.032 (6206) |
| char_b/diagonal [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (7) | 0.276 (44564) | 0.056 (8847) | 0.003 (99) | 0.046 (6401) | 0.001 (1) | 0.337 (53874) | 0.002 (97) | 0.069 (47527) | 0.032 (6206) |
| char_b/diagonal [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (7) | 0.289 (46577) | 0.071 (11239) | 0.068 (2058) | 0.055 (7700) | 0.010 (14) | 0.381 (60531) | 0.046 (2024) | 0.081 (54865) | 0.044 (8508) |
| char_b/diagonal [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (7) | 0.276 (44564) | 0.056 (8847) | 0.003 (99) | 0.046 (6401) | 0.001 (1) | 0.337 (53874) | 0.002 (97) | 0.069 (47527) | 0.032 (6206) |
| char_b/lateral-low [exact timing prune] g1 | FULL_ARGMIN | 0.001 (14) | 0.315 (50616) | 0.464 (74623) | 3.294 (111001) | 0.193 (26420) | 0.260 (382) | 1.465 (227759) | 2.179 (109551) | 0.444 (312528) | 0.378 (73015) |
| char_b/lateral-low [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (5) | 0.129 (20375) | 0.087 (13711) | 0.448 (15408) | 0.026 (3509) | 0.038 (56) | 0.294 (45945) | 0.296 (15192) | 0.081 (57851) | 0.066 (12631) |
| char_b/lateral-low [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.160 (25428) | 0.102 (16068) | 0.508 (17454) | 0.034 (4541) | 0.042 (62) | 0.351 (55093) | 0.335 (17213) | 0.096 (68011) | 0.076 (14666) |
| char_b/lateral-low [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.176 (28101) | 0.133 (20997) | 0.715 (24403) | 0.051 (6862) | 0.066 (96) | 0.450 (70087) | 0.471 (24067) | 0.124 (87673) | 0.104 (19945) |
| char_b/lateral-low [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.160 (25428) | 0.102 (16068) | 0.508 (17454) | 0.034 (4541) | 0.042 (62) | 0.351 (55093) | 0.335 (17213) | 0.096 (68011) | 0.076 (14666) |
| char_b/longitudinal [exact timing prune] g1 | FULL_ARGMIN | 0.000 (14) | 0.619 (99852) | 0.381 (60867) | 2.340 (74270) | 0.152 (20907) | 0.157 (232) | 1.462 (230244) | 1.592 (73110) | 0.410 (283492) | 0.248 (48311) |
| char_b/longitudinal [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (3) | 0.114 (17601) | 0.026 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.128 (19764) | 0.002 (92) | 0.029 (19033) | 0.015 (2723) |
| char_b/longitudinal [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (3) | 0.114 (17601) | 0.026 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.128 (19764) | 0.002 (92) | 0.029 (19033) | 0.015 (2723) |
| char_b/longitudinal [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (3) | 0.134 (20664) | 0.044 (6672) | 0.141 (4631) | 0.018 (2455) | 0.013 (18) | 0.199 (30636) | 0.094 (4564) | 0.049 (32805) | 0.030 (5567) |
| char_b/longitudinal [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (3) | 0.114 (17601) | 0.026 (3851) | 0.003 (94) | 0.006 (809) | 0.001 (1) | 0.128 (19764) | 0.002 (92) | 0.029 (19033) | 0.015 (2723) |
| char_b/near-ground [exact timing prune] g1 | FULL_ARGMIN | 0.000 (14) | 0.530 (84783) | 0.492 (77104) | 1.842 (62800) | 0.131 (17500) | 0.206 (256) | 1.460 (226353) | 1.199 (61988) | 0.386 (271847) | 0.241 (45959) |
| char_b/near-ground [exact timing prune] g1 | FIRST_COMPLETE_FEASIBLE | 0.000 (6) | 0.255 (40197) | 0.179 (28303) | 1.079 (36681) | 0.072 (9567) | 0.124 (177) | 0.682 (103786) | 0.697 (36203) | 0.186 (131164) | 0.151 (28829) |
| char_b/near-ground [exact timing prune] g1 | FIRST_ADMISSIBLE_COMPLETE | 0.000 (6) | 0.255 (40197) | 0.179 (28303) | 1.079 (36681) | 0.072 (9567) | 0.124 (177) | 0.682 (103786) | 0.697 (36203) | 0.186 (131164) | 0.151 (28829) |
| char_b/near-ground [exact timing prune] g1 | EARLIEST_ADMISSIBLE_TAU_THEN_BEST_GR | 0.000 (6) | 0.305 (48170) | 0.192 (30298) | 1.154 (39153) | 0.076 (10095) | 0.133 (189) | 0.759 (115982) | 0.747 (38643) | 0.206 (145312) | 0.162 (30848) |
| char_b/near-ground [exact timing prune] g1 | ANYTIME_INCUMBENT | 0.000 (6) | 0.255 (40197) | 0.179 (28303) | 1.079 (36681) | 0.072 (9567) | 0.124 (177) | 0.682 (103786) | 0.697 (36203) | 0.186 (131164) | 0.151 (28829) |

### Per-candidate certification cost and work before the first complete certified record

| search | kind | full wall (s) | static screens rejected / passed (median ms) | route rollouts rejected / complete (median ms) | before first complete: rejected screens, passed screens, rejected routes, hypotheses | wall of rejected screens / passed screens / rejected routes / the successful route (s) |
|---|---|---:|---|---|---|---|
| char_a/diagonal [exact timing prune] g1 | moving | 2.817 | 424 (1.19) / 24 (1.67) | 141 (3.36) / 267 (5.82) | 207, 1, 0, 7 | 0.362 / 0.002 / 0.000 / 0.005 |
| char_b/diagonal [exact timing prune] g1 | moving | 2.881 | 424 (1.30) / 24 (1.74) | 141 (3.41) / 267 (5.91) | 207, 1, 0, 7 | 0.377 / 0.002 / 0.000 / 0.005 |
| char_a/lateral-low [exact timing prune] g1 | moving | 3.985 | 405 (0.43) / 43 (1.83) | 302 (4.50) / 310 (6.64) | 141, 7, 6, 5 | 0.168 / 0.013 / 0.027 / 0.007 |
| char_b/lateral-low [exact timing prune] g1 | moving | 3.886 | 405 (0.42) / 43 (1.82) | 302 (4.37) / 310 (6.52) | 141, 7, 6, 5 | 0.169 / 0.013 / 0.026 / 0.006 |
| char_a/longitudinal [exact timing prune] g1 | moving | 3.307 | 413 (1.44) / 35 (1.82) | 249 (4.15) / 227 (6.20) | 79, 1, 0, 3 | 0.160 / 0.002 / 0.000 / 0.006 |
| char_b/longitudinal [exact timing prune] g1 | moving | 3.266 | 413 (1.37) / 35 (1.76) | 266 (3.82) / 227 (6.05) | 79, 1, 0, 3 | 0.144 / 0.002 / 0.000 / 0.005 |
| char_a/near-ground [exact timing prune] g1 | moving | 1.799 | 424 (0.51) / 24 (1.97) | 90 (4.55) / 80 (7.02) | 148, 15, 0, 6 | 0.252 / 0.029 / 0.000 / 0.007 |
| char_b/near-ground [exact timing prune] g1 | moving | 1.791 | 424 (0.50) / 24 (1.98) | 90 (4.48) / 80 (7.07) | 148, 15, 0, 6 | 0.256 / 0.030 / 0.000 / 0.006 |
