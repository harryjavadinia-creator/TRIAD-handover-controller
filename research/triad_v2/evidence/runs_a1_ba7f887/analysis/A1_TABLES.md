
### MOVING-epoch completed searches: 4 unique (33 occurrences across runs)

**Candidate survival and conditional full rejection**

| level | STATIC (τ,g) surviving | % | P(full rejection \| STATIC level accepted) | ROUTE (τ,g,r) surviving | % | P(full rejection \| ROUTE level accepted) |
|---|---:|---:|---:|---:|---:|---:|
| F1 | 763 | 42.6% | 659/763 = 86.4% | 1176 | 54.9% | 53/1176 = 4.5% |
| F1+F2 | 702 | 39.2% | 598/702 = 85.2% | 1176 | 54.9% | 53/1176 = 4.5% |
| F1+F2+F3 | 170 | 9.5% | 66/170 = 38.8% | 1137 | 53.1% | 14/1137 = 1.2% |
| F1+F2+F3+F5 | 126 | 7.0% | 22/126 = 17.5% | 1137 | 53.1% | 14/1137 = 1.2% |
| full (C5) | 104 (with ≥1 complete route) | 5.8% | 0 | 1123 | 52.4% | 0 |

Totals evaluated: STATIC 1792, ROUTE 2142 (route rollouts exist only for STATIC F5 passes).

**Newly rejected by each stage and top reasons**

| population | stage | newly rejected | % of population | top reasons (count) |
|---|---|---:|---:|---|
| STATIC | F1 reach | 1029 | 57.4% | `ik_preview_no_convergence` 305; `right_tip_body_2/ground_plane` 171; `left_tip_body_2/ground_plane` 46; `base_rear/human_grey_handle` 34 |
| STATIC | F2 insertion | 61 | 3.4% | `ik_preview_no_convergence` 57; `mouth_corridor/blue_handle_axial_offset` 3; `mouth_corridor/blue_handle_lateral` 1 |
| STATIC | F3 closure/contact | 532 | 29.7% | `closure/pad_pair/blue_handle_acquisition_tube` 462; `closure/right_pad_shoulder_low/robot_blue_handle` 36; `closure/left_pad_shoulder_low/robot_blue_handle` 32; `closure/right_inner_pad/robot_blue_handle` 1 |
| STATIC | F5 carried retreat | 44 | 2.5% | `ik_preview_no_convergence` 43; `gen3_spherical_wrist_2_link/ground_plane` 1 |
| ROUTE | F1 reach | 966 | 45.1% | `runtime_clearance_reserve` 419; `reach_tracking` 349; `robust_transit_clearance_reserve` 151; `reach/left_finger_15/robot_blue_handle` 5 |
| ROUTE | F2 insertion | 0 | 0.0% | — |
| ROUTE | F3 closure/contact | 39 | 1.8% | `static_acquire/closure/pad_pair` 22; `static_acquire/closure/right_pad_shoulder_low` 17 |
| ROUTE | F5 carried retreat | 0 | 0.0% | — |
| ROUTE | terminal timing audit / cost validity | 14 | 0.7% | `audit:timeout` 14 |

**Worker wall by stage (sum over these searches)**

| static F1 | static F2 | static F3 | static F5 | route F1 | route F2 | route F3 | route F5 | audit+cost | total |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.10s (14%) | 0.44s (3%) | 0.33s (2%) | 0.27s (2%) | 9.44s (63%) | 0.29s (2%) | 0.68s (5%) | 0.41s (3%) | 0.84s (6%) | 14.88s |

### REST-epoch completed searches: 23 unique (38 occurrences across runs)

**Candidate survival and conditional full rejection**

| level | STATIC (τ,g) surviving | % | P(full rejection \| STATIC level accepted) | ROUTE (τ,g,r) surviving | % | P(full rejection \| ROUTE level accepted) |
|---|---:|---:|---:|---:|---:|---:|
| F1 | 312 | 42.4% | 248/312 = 79.5% | 588 | 43.2% | 0/588 = 0.0% |
| F1+F2 | 298 | 40.5% | 234/298 = 78.5% | 588 | 43.2% | 0/588 = 0.0% |
| F1+F2+F3 | 105 | 14.3% | 41/105 = 39.0% | 588 | 43.2% | 0/588 = 0.0% |
| F1+F2+F3+F5 | 80 | 10.9% | 16/80 = 20.0% | 588 | 43.2% | 0/588 = 0.0% |
| full (C5) | 64 (with ≥1 complete route) | 8.7% | 0 | 588 | 43.2% | 0 |

Totals evaluated: STATIC 736, ROUTE 1360 (route rollouts exist only for STATIC F5 passes).

**Newly rejected by each stage and top reasons**

| population | stage | newly rejected | % of population | top reasons (count) |
|---|---|---:|---:|---|
| STATIC | F1 reach | 424 | 57.6% | `ik_preview_no_convergence` 107; `right_tip_body_2/ground_plane` 88; `left_tip_body_2/ground_plane` 25; `left_finger_14/ground_plane` 17 |
| STATIC | F2 insertion | 14 | 1.9% | `ik_preview_no_convergence` 14 |
| STATIC | F3 closure/contact | 193 | 26.2% | `closure/pad_pair/blue_handle_acquisition_tube` 169; `closure/right_pad_shoulder_low/robot_blue_handle` 22; `closure/left_pad_shoulder_low/robot_blue_handle` 1; `closure/left_inner_pad/robot_blue_handle` 1 |
| STATIC | F5 carried retreat | 25 | 3.4% | `ik_preview_no_convergence` 25 |
| ROUTE | F1 reach | 772 | 56.8% | `transit_route/path_stretch_limit` 256; `runtime_clearance_reserve` 255; `reach_tracking` 162; `robust_transit_clearance_reserve` 82 |
| ROUTE | F2 insertion | 0 | 0.0% | — |
| ROUTE | F3 closure/contact | 0 | 0.0% | — |
| ROUTE | F5 carried retreat | 0 | 0.0% | — |
| ROUTE | terminal timing audit / cost validity | 0 | 0.0% | — |

**Worker wall by stage (sum over these searches)**

| static F1 | static F2 | static F3 | static F5 | route F1 | route F2 | route F3 | route F5 | audit+cost | total |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.73s (10%) | 0.13s (2%) | 0.15s (2%) | 0.16s (2%) | 4.82s (66%) | 0.15s (2%) | 0.35s (5%) | 0.21s (3%) | 0.42s (6%) | 7.28s |

### Selection-level counterfactual per unique completed search

Status: DETERMINED_SAME = no shallow-only candidate can be admitted at the decision time with J lower bound ≤ winner J, so the shallower policy provably selects the same tuple; NOT_DETERMINED = at least one admissible shallow-only candidate whose J is undefined could outrank the full winner (count, stages). PROXY (C4 only) = selector over full records plus audit-rejected records ranked by the logged pre-audit objective; not a policy.

| scenario | epoch | searches | decision at | C5 winner (lead, g, r) | J | reach / retreat clr (m) | C4 | C3 | C2 | C1 | C4 PROXY winner |
|---|---|---:|---|---|---:|---|---|---|---|---|---|
| diagonal | moving | 1 unique / 10 occ. | receipt | (5.5, 'axisP_side_337deg', 'ring80mm_2of8') | 0.7746 | 0.074 / 0.223 | same | NOT DET. (3: static CLOSURE_CONTACT 3) | NOT DET. (56: INSERTION 5, static CLOSURE_CONTACT 3, static INSERTION 48) | NOT DET. (56: INSERTION 5, static CLOSURE_CONTACT 3, static INSERTION 48) | n/a (no audit-rejected record) |
| diagonal | moving | 1 unique / 10 occ. | epoch | (4.15, 'axisP_side_337deg', 'ring140mm_2of8') | 0.6820 | 0.072 / 0.218 | same | NOT DET. (13: static CLOSURE_CONTACT 13) | NOT DET. (152: INSERTION 5, static CLOSURE_CONTACT 13, static INSERTION 134) | NOT DET. (152: INSERTION 5, static CLOSURE_CONTACT 13, static INSERTION 134) | n/a (no audit-rejected record) |
| diagonal | rest | 1 unique / 4 occ. | receipt | (2.8, 'axisP_side_337deg', 'ring80mm_1of8') | 0.6035 | 0.081 / 0.217 | same | NOT DET. (21: static CLOSURE_CONTACT 21) | NOT DET. (153: static CLOSURE_CONTACT 21, static INSERTION 132) | NOT DET. (153: static CLOSURE_CONTACT 21, static INSERTION 132) | n/a (no audit-rejected record) |
| diagonal | rest | 1 unique / 4 occ. | epoch | (2.8, 'axisP_side_337deg', 'ring80mm_1of8') | 0.6035 | 0.081 / 0.217 | same | NOT DET. (23: static CLOSURE_CONTACT 23) | NOT DET. (161: static CLOSURE_CONTACT 23, static INSERTION 138) | NOT DET. (161: static CLOSURE_CONTACT 23, static INSERTION 138) | n/a (no audit-rejected record) |
| lateral-low | moving | 1 unique / 4 occ. | receipt | (8.0, 'axisN_side_23deg', 'ring80mm_6of8') | 0.9321 | 0.069 / 0.099 | same | same | NOT DET. (6: static INSERTION 6) | NOT DET. (6: static INSERTION 6) | n/a (no audit-rejected record) |
| lateral-low | moving | 1 unique / 4 occ. | epoch | (3.7, 'axisN_side_337deg', 'direct') | 0.6777 | 0.082 / 0.077 | same | NOT DET. (10: static CLOSURE_CONTACT 10) | NOT DET. (135: INSERTION 4, static CLOSURE_CONTACT 10, static INSERTION 121) | NOT DET. (135: INSERTION 4, static CLOSURE_CONTACT 10, static INSERTION 121) | n/a (no audit-rejected record) |
| lateral-low | rest | 4 unique / 9 occ. | receipt | (2.8, 'axisN_side_337deg', 'direct') | 0.6215 | 0.082 / 0.081 | same | NOT DET. (17: static CLOSURE_CONTACT 17) | NOT DET. (117: static CLOSURE_CONTACT 17, static INSERTION 100) | NOT DET. (117: static CLOSURE_CONTACT 17, static INSERTION 100) | n/a (no audit-rejected record) |
| lateral-low | rest | 5 unique / 10 occ. | epoch | (2.35, 'axisN_side_337deg', 'direct') | 0.5978 | 0.082 / 0.081 | same | NOT DET. (19: static CLOSURE_CONTACT 19) | NOT DET. (129: static CLOSURE_CONTACT 19, static INSERTION 110) | NOT DET. (129: static CLOSURE_CONTACT 19, static INSERTION 110) | n/a (no audit-rejected record) |
| lateral-low | rest | 1 unique / 1 occ. | receipt | (1.8, 'axisN_side_337deg', 'direct') | 0.4709 | 0.067 / 0.077 | same | same | NOT DET. (107: static INSERTION 107) | NOT DET. (107: static INSERTION 107) | n/a (no audit-rejected record) |
| lateral-low | rest | 4 unique / 4 occ. | epoch | (1.8, 'axisN_side_337deg', 'direct') | 0.4709 | 0.067 / 0.077 | same | same | NOT DET. (108: static INSERTION 108) | NOT DET. (108: static INSERTION 108) | n/a (no audit-rejected record) |
| lateral-low | rest | 3 unique / 3 occ. | receipt | (1.8, 'axisN_side_337deg', 'direct') | 0.4709 | 0.067 / 0.077 | same | same | NOT DET. (108: static INSERTION 108) | NOT DET. (108: static INSERTION 108) | n/a (no audit-rejected record) |
| lateral-low | rest | 1 unique / 1 occ. | receipt | (2.8, 'axisN_side_337deg', 'direct') | 0.6215 | 0.082 / 0.081 | same | NOT DET. (17: static CLOSURE_CONTACT 17) | NOT DET. (116: static CLOSURE_CONTACT 17, static INSERTION 99) | NOT DET. (116: static CLOSURE_CONTACT 17, static INSERTION 99) | n/a (no audit-rejected record) |
| longitudinal | moving | 1 unique / 9 occ. | receipt | none | nan | nan / nan | none | none | **INVALID** (15 admissible shallow-only records: INSERTION 15, static INSERTION 10) | **INVALID** (15 admissible shallow-only records: INSERTION 15, static INSERTION 10, static REACH 5) | n/a (no audit-rejected record) |
| longitudinal | moving | 1 unique / 9 occ. | epoch | (2.35, 'axisP_side_337deg', 'direct') | 0.6214 | 0.079 / 0.210 | same | NOT DET. (10: static CLOSURE_CONTACT 10) | NOT DET. (152: INSERTION 15, static CLOSURE_CONTACT 10, static INSERTION 127) | NOT DET. (163: INSERTION 15, static CLOSURE_CONTACT 10, static INSERTION 127, static REACH 11) | n/a (no audit-rejected record) |
| longitudinal | rest | 5 unique / 9 occ. | receipt | (2.8, 'axisP_side_337deg', 'direct') | 0.6499 | 0.080 / 0.221 | same | NOT DET. (10: static CLOSURE_CONTACT 10) | NOT DET. (111: static CLOSURE_CONTACT 10, static INSERTION 101) | NOT DET. (111: static CLOSURE_CONTACT 10, static INSERTION 101) | n/a (no audit-rejected record) |
| longitudinal | rest | 6 unique / 10 occ. | epoch | (2.35, 'axisP_side_337deg', 'direct') | 0.6262 | 0.080 / 0.221 | same | NOT DET. (12: static CLOSURE_CONTACT 12) | NOT DET. (127: static CLOSURE_CONTACT 12, static INSERTION 115) | NOT DET. (127: static CLOSURE_CONTACT 12, static INSERTION 115) | n/a (no audit-rejected record) |
| longitudinal | rest | 1 unique / 1 occ. | receipt | (2.8, 'axisP_side_337deg', 'direct') | 0.6499 | 0.080 / 0.221 | same | NOT DET. (10: static CLOSURE_CONTACT 10) | NOT DET. (110: static CLOSURE_CONTACT 10, static INSERTION 100) | NOT DET. (110: static CLOSURE_CONTACT 10, static INSERTION 100) | n/a (no audit-rejected record) |
| near-ground | moving | 1 unique / 10 occ. | receipt | (8.0, 'axisP_side_247deg', 'ring140mm_5of8') | 1.0367 | 0.082 / 0.088 | same | NOT DET. (2: static CLOSURE_CONTACT 2) | NOT DET. (15: static CLOSURE_CONTACT 2, static INSERTION 13) | NOT DET. (37: static CLOSURE_CONTACT 2, static INSERTION 13, static REACH 22) | same |
| near-ground | moving | 1 unique / 10 occ. | epoch | (3.25, 'axisP_side_45deg', 'ring80mm_7of8') | 0.7535 | 0.071 / 0.106 | NOT DET. (14: CARRIED_RETREAT/audit_or_cost 14) | NOT DET. (19: CARRIED_RETREAT/audit_or_cost 14, static CLOSURE_CONTACT 5) | NOT DET. (68: CARRIED_RETREAT/audit_or_cost 14, INSERTION 1, static CLOSURE_CONTACT 5, static INSERTION 48) | NOT DET. (117: CARRIED_RETREAT/audit_or_cost 14, INSERTION 1, static CLOSURE_CONTACT 5, static INSERTION 48, static REACH 49) | same |
| near-ground | rest | 1 unique / 4 occ. | receipt | (3.7, 'axisP_side_45deg', 'ring140mm_7of8') | 0.7617 | 0.079 / 0.098 | same | NOT DET. (10: static CLOSURE_CONTACT 10) | NOT DET. (48: static CLOSURE_CONTACT 10, static INSERTION 38) | NOT DET. (67: static CLOSURE_CONTACT 10, static INSERTION 38, static REACH 19) | n/a (no audit-rejected record) |
| near-ground | rest | 7 unique / 10 occ. | epoch | (3.25, 'axisP_side_45deg', 'ring80mm_0of8') | 0.7484 | 0.083 / 0.098 | same | NOT DET. (11: static CLOSURE_CONTACT 11) | NOT DET. (51: static CLOSURE_CONTACT 11, static INSERTION 40) | NOT DET. (71: static CLOSURE_CONTACT 11, static INSERTION 40, static REACH 20) | n/a (no audit-rejected record) |
| near-ground | rest | 4 unique / 4 occ. | receipt | (3.7, 'axisP_side_45deg', 'ring140mm_7of8') | 0.7644 | 0.078 / 0.098 | same | NOT DET. (10: static CLOSURE_CONTACT 10) | NOT DET. (48: static CLOSURE_CONTACT 10, static INSERTION 38) | NOT DET. (67: static CLOSURE_CONTACT 10, static INSERTION 38, static REACH 19) | n/a (no audit-rejected record) |
| near-ground | rest | 2 unique / 2 occ. | receipt | (3.7, 'axisP_side_45deg', 'ring140mm_7of8') | 0.7643 | 0.078 / 0.098 | same | NOT DET. (10: static CLOSURE_CONTACT 10) | NOT DET. (48: static CLOSURE_CONTACT 10, static INSERTION 38) | NOT DET. (67: static CLOSURE_CONTACT 10, static INSERTION 38, static REACH 19) | n/a (no audit-rejected record) |

**Determinacy counts (unique completed searches)**

| decision at | epoch | level | DETERMINED_SAME | DETERMINED_INVALID | POTENTIALLY_INVALID_UNOBSERVED_ROUTES | DETERMINED_NONE | NOT_DETERMINED |
|---|---|---|---:|---:|---:|---:|---:|
| epoch | moving | C1 | 0 | 0 | 0 | 0 | 4 |
| epoch | moving | C2 | 0 | 0 | 0 | 0 | 4 |
| epoch | moving | C3 | 0 | 0 | 0 | 0 | 4 |
| epoch | moving | C4 | 3 | 0 | 0 | 0 | 1 |
| epoch | rest | C1 | 0 | 0 | 0 | 0 | 23 |
| epoch | rest | C2 | 0 | 0 | 0 | 0 | 23 |
| epoch | rest | C3 | 4 | 0 | 0 | 0 | 19 |
| epoch | rest | C4 | 23 | 0 | 0 | 0 | 0 |
| receipt | moving | C1 | 0 | 1 | 0 | 0 | 3 |
| receipt | moving | C2 | 0 | 1 | 0 | 0 | 3 |
| receipt | moving | C3 | 1 | 0 | 0 | 1 | 2 |
| receipt | moving | C4 | 3 | 0 | 0 | 1 | 0 |
| receipt | rest | C1 | 0 | 0 | 0 | 0 | 23 |
| receipt | rest | C2 | 0 | 0 | 0 | 0 | 23 |
| receipt | rest | C3 | 4 | 0 | 0 | 0 | 19 |
| receipt | rest | C4 | 23 | 0 | 0 | 0 | 0 |

### Runtime connection: executed (full-certificate) actions in situ

| run | adoptions (kind: lead, g, r) | runtime invalidations (phase: stage/reason) | commit | completed |
|---|---|---|---|---|
| insitu/diagonal | INITIAL: 5.500s, axisP_side_337deg, ring80mm_2of8 |  | yes | yes |
| insitu/lateral-low | INITIAL: 2.800s, axisN_side_337deg, direct; REPLACEMENT: 1.800s, axisN_side_337deg, direct | PROVISIONAL_REACH: recertification_infeasible/predictive_static/static_acquire/closure; TERMINAL_TRACK: terminal_certification_infeasible/predictive_static/static_acquire/closure | no | no |
| insitu/longitudinal | INITIAL: 2.800s, axisP_side_337deg, direct |  | yes | yes |
| insitu/near-ground | INITIAL: 8.000s, axisP_side_247deg, ring140mm_5of8; REPLACEMENT: 3.700s, axisP_side_45deg, ring140mm_7of8 | PROVISIONAL_REACH: recertification_infeasible/cost_invalid/timeout | yes | yes |
| insitu_a/diagonal | INITIAL: 5.500s, axisP_side_337deg, ring80mm_2of8 |  | yes | yes |
| insitu_a/lateral-low | INITIAL: 2.800s, axisN_side_337deg, direct; REPLACEMENT: 1.800s, axisN_side_337deg, direct | PROVISIONAL_REACH: recertification_infeasible/predictive_static/static_acquire/closure; TERMINAL_TRACK: terminal_certification_infeasible/predictive_static/static_acquire/closure | no | no |
| insitu_a/longitudinal | INITIAL: 2.800s, axisP_side_337deg, direct |  | yes | yes |
| insitu_a/near-ground | INITIAL: 8.000s, axisP_side_247deg, ring140mm_5of8; REPLACEMENT: 3.700s, axisP_side_45deg, ring140mm_7of8 | PROVISIONAL_REACH: recertification_infeasible/cost_invalid/timeout | yes | yes |
| insitu_b/diagonal | INITIAL: 5.500s, axisP_side_337deg, ring80mm_2of8 |  | yes | yes |
| insitu_b/lateral-low | INITIAL: 2.800s, axisN_side_337deg, direct; REPLACEMENT: 1.800s, axisN_side_337deg, direct | PROVISIONAL_REACH: recertification_infeasible/predictive_static/static_acquire/closure; TERMINAL_TRACK: terminal_certification_infeasible/predictive_static/static_acquire/closure | no | no |
| insitu_b/longitudinal | INITIAL: 2.800s, axisP_side_337deg, direct |  | yes | yes |
| insitu_b/near-ground | INITIAL: 8.000s, axisP_side_247deg, ring140mm_5of8; REPLACEMENT: 3.700s, axisP_side_45deg, ring140mm_7of8 | PROVISIONAL_REACH: recertification_infeasible/cost_invalid/timeout | yes | yes |
| off_a/diagonal | INITIAL: 5.500s, axisP_side_337deg, ring80mm_2of8 |  | yes | yes |
| off_a/lateral-low | INITIAL: 2.800s, axisN_side_337deg, direct |  | yes | yes |
| off_a/longitudinal | INITIAL: 2.800s, axisP_side_337deg, direct |  | yes | yes |
| off_a/near-ground | INITIAL: 8.000s, axisP_side_247deg, ring140mm_5of8; REPLACEMENT: 3.700s, axisP_side_45deg, ring140mm_7of8 | PROVISIONAL_REACH: recertification_infeasible/cost_invalid/timeout | yes | yes |
| off_b/diagonal | INITIAL: 5.500s, axisP_side_337deg, ring80mm_2of8 |  | yes | yes |
| off_b/lateral-low | INITIAL: 2.800s, axisN_side_337deg, direct |  | yes | yes |
| off_b/longitudinal | INITIAL: 2.800s, axisP_side_337deg, direct |  | yes | yes |
| off_b/near-ground | INITIAL: 8.000s, axisP_side_247deg, ring140mm_5of8; REPLACEMENT: 3.700s, axisP_side_45deg, ring140mm_7of8 | PROVISIONAL_REACH: recertification_infeasible/cost_invalid/timeout | yes | yes |
| off_c/diagonal | INITIAL: 5.500s, axisP_side_337deg, ring80mm_2of8 |  | yes | yes |
| off_c/lateral-low | INITIAL: 2.800s, axisN_side_337deg, direct; REPLACEMENT: 1.800s, axisN_side_337deg, direct | PROVISIONAL_REACH: recertification_infeasible/predictive_static/static_acquire/closure; TERMINAL_TRACK: terminal_certification_infeasible/predictive_static/static_acquire/closure | no | no |
| off_c/longitudinal | INITIAL: 2.800s, axisP_side_337deg, direct |  | yes | yes |
| off_c/near-ground | INITIAL: 8.000s, axisP_side_247deg, ring140mm_5of8; REPLACEMENT: 3.700s, axisP_side_45deg, ring140mm_7of8 | PROVISIONAL_REACH: recertification_infeasible/cost_invalid/timeout | yes | yes |
