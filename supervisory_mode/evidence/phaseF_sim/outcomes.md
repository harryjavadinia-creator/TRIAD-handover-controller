| scenario | variant | completed | first plan while moving | meeting (t − stop) s | meeting error mm / rad | freeze − giver stop s | switches | aborts | patched / retained | refused / inconclusive | failures |
|---|---|---|---|---|---|---|---|---|---|---|---|
| longitudinal | reactive | 0/3 | 3/3 | — | — | — | 7 | 2 | 0 / 0 | 0 / 0 | control_aware_track_clearance_reserve,no_certified_provisional_plan_within_presentation_window |
| longitudinal | predictive | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.6/0.022; 1.6/0.023; 1.6/0.022 | 4.16; 4.16; 4.16 | 0 | 6 | 0 / 76 | 0 / 0 | — |
| longitudinal | predictive_capability | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.8/0.022; 1.9/0.023; 1.8/0.022 | 3.87; 3.87; 3.87 | 0 | 6 | 0 / 67 | 3 / 0 | — |
| longitudinal | full | 2/3 | 3/3 | -1.62; -1.62; -1.62 | 1.6/0.022; 1.6/0.022; 1.6/0.022 | 6.03; 6.06 | 1 | 140 | 0 / 47 | 4 / 2 | no_certified_provisional_plan_within_presentation_window |
| near-ground | reactive | 2/3 | 3/3 | -1.48; -1.63; -1.48 | 6.7/0.016; 3.7/0.044; 5.2/0.043 | 1.85; 2.25 | 49 | 4 | 0 / 0 | 0 / 0 | no_certified_provisional_plan_within_presentation_window |
| near-ground | predictive | 2/3 | 3/3 | — | — | 2.29; 2.50 | 17 | 1 | 3 / 44 | 6 / 0 | control_aware_no_admission_within_presentation_window |
| near-ground | predictive_capability | 1/3 | 3/3 | — | — | 2.01 | 13 | 3 | 3 / 57 | 5 / 0 | control_aware_no_admission_within_presentation_window |
| near-ground | full | 1/3 | 3/3 | 0.32 | 5.0/0.020 | 0.36 | 2 | 4 | 0 / 39 | 11 / 1 | no_certified_provisional_plan_within_presentation_window |
| lateral-low | reactive | 3/3 | 3/3 | -2.63; -2.63; -2.63 | 11.3/0.020; 11.4/0.020; 11.2/0.020 | 0.08; 0.08; 0.08 | 18 | 0 | 0 / 0 | 0 / 0 | — |
| lateral-low | predictive | 3/3 | 3/3 | -0.63; -0.60; -0.65 | 2.9/0.024; 2.6/0.024; 3.1/0.025 | 0.84; 0.83; 0.83 | 3 | 3 | 0 / 33 | 0 / 6 | — |
| lateral-low | predictive_capability | 3/3 | 3/3 | -0.62; -0.59; -0.64 | 2.7/0.024; 2.6/0.024; 2.9/0.024 | 0.83; 0.83; 0.84 | 3 | 3 | 0 / 32 | 0 / 6 | — |
| lateral-low | full | 0/3 | 3/3 | -1.05; -1.05; -1.05 | 4.8/0.023; 4.9/0.023; 4.8/0.023 | — | 0 | 46 | 0 / 0 | 3 / 6 | control_aware_no_admission_within_presentation_window,no_certified_provisional_plan_within_presentation_window |
| diagonal | reactive | 0/3 | 3/3 | — | — | — | 15 | 1 | 0 / 0 | 0 / 0 | control_aware_track_clearance_reserve,no_certified_provisional_plan_within_presentation_window |
| diagonal | predictive | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.5/0.023; 1.5/0.023; 1.5/0.023 | 0.08; 0.08; 0.08 | 0 | 0 | 0 / 6 | 0 / 3 | — |
| diagonal | predictive_capability | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.8/0.023; 1.8/0.023; 1.8/0.023 | 1.37; 1.37; 1.37 | 0 | 3 | 0 / 39 | 0 / 3 | — |
| diagonal | full | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.6/0.024; 1.5/0.023; 1.5/0.023 | 0.08; 0.08; 0.08 | 0 | 0 | 0 / 0 | 3 / 6 | — |

completions per variant: reactive 5/12, predictive 11/12, predictive_capability 10/12, full 6/12
selection-job latency reactive: median=0.064s p90=0.113s max=0.249s n=779
selection-job latency predictive: median=0.139s p90=0.609s max=1.069s n=220
selection-job latency predictive_capability: median=0.136s p90=0.515s max=1.105s n=261
selection-job latency full: median=0.052s p90=0.167s max=1.278s n=504
regression bank_search: diagonal completed=False, lateral-low completed=True, longitudinal completed=False, near-ground completed=True
