| scenario | variant | completed | first plan while moving | meeting (t − stop) s | meeting error mm / rad | freeze − giver stop s | switches | aborts | patched / retained | refused / inconclusive | failures |
|---|---|---|---|---|---|---|---|---|---|---|---|
| longitudinal | reactive | 0/3 | 3/3 | — | — | — | 7 | 2 | 0 / 0 | 0 / 0 | control_aware_track_clearance_reserve,no_certified_provisional_plan_within_presentation_window |
| longitudinal | predictive | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.8/0.024; 1.9/0.024; 1.9/0.025 | 4.79; 4.82; 4.82 | 3 | 6 | 0 / 79 | 2 / 0 | — |
| longitudinal | predictive_capability | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 2.2/0.025; 2.1/0.024; 2.1/0.024 | 1.61; 1.65; 1.67 | 3 | 3 | 0 / 51 | 1 / 0 | — |
| longitudinal | full | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.9/0.024; 1.9/0.024; 1.9/0.024 | 4.79; 4.78; 4.83 | 3 | 6 | 0 / 75 | 6 / 0 | — |
| near-ground | reactive | 2/3 | 3/3 | -1.48; -1.63; -1.48 | 6.7/0.016; 3.7/0.044; 5.2/0.043 | 1.85; 2.25 | 49 | 4 | 0 / 0 | 0 / 0 | no_certified_provisional_plan_within_presentation_window |
| near-ground | predictive | 0/3 | 3/3 | — | — | — | 9 | 4 | 0 / 39 | 9 / 0 | control_aware_no_admission_within_presentation_window,no_certified_provisional_plan_within_presentation_window |
| near-ground | predictive_capability | 0/3 | 3/3 | — | — | — | 9 | 7 | 0 / 28 | 9 / 0 | control_aware_no_admission_within_presentation_window,no_certified_provisional_plan_within_presentation_window |
| near-ground | full | 3/3 | 3/3 | 0.65 | 5.5/0.021 | 0.52; 1.15; 3.69 | 0 | 1 | 0 / 2 | 10 / 0 | — |
| lateral-low | reactive | 3/3 | 3/3 | -2.63; -2.63; -2.63 | 11.3/0.020; 11.4/0.020; 11.2/0.020 | 0.08; 0.08; 0.08 | 18 | 0 | 0 / 0 | 0 / 0 | — |
| lateral-low | predictive | 3/3 | 3/3 | -0.16; -0.23 | 1.5/0.025; 1.4/0.025 | 0.28; 0.91; 1.02 | 6 | 2 | 0 / 15 | 0 / 18 | — |
| lateral-low | predictive_capability | 3/3 | 3/3 | -0.07; -0.18; -0.21 | 1.4/0.016; 1.6/0.018; 1.6/0.019 | 0.74; 0.76; 0.75 | 3 | 3 | 0 / 14 | 1 / 28 | — |
| lateral-low | full | 3/3 | 3/3 | -1.05; -1.05; -1.05 | 5.3/0.025; 5.4/0.026; 5.3/0.026 | 0.87; 0.87; 0.87 | 0 | 3 | 0 / 12 | 5 / 2 | — |
| diagonal | reactive | 0/3 | 3/3 | — | — | — | 15 | 1 | 0 / 0 | 0 / 0 | control_aware_track_clearance_reserve,no_certified_provisional_plan_within_presentation_window |
| diagonal | predictive | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.7/0.025; 1.9/0.026; 1.5/0.023 | 0.08; 0.08; 0.08 | 0 | 0 | 0 / 5 | 2 / 0 | — |
| diagonal | predictive_capability | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 2.1/0.025; 2.1/0.026; 1.8/0.023 | 1.38; 1.38; 1.37 | 0 | 3 | 0 / 32 | 2 / 0 | — |
| diagonal | full | 3/3 | 3/3 | -1.62; -1.62; -1.62 | 1.8/0.026; 1.8/0.025; 1.6/0.023 | 0.08; 0.08; 0.08 | 0 | 0 | 0 / 0 | 5 / 1 | — |

completions per variant: reactive 5/12, predictive 9/12, predictive_capability 9/12, full 12/12
selection-job latency reactive: median=0.064s p90=0.113s max=0.249s n=779
selection-job latency predictive: median=0.176s p90=0.730s max=1.368s n=214
selection-job latency predictive_capability: median=0.123s p90=0.701s max=1.356s n=218
selection-job latency full: median=0.062s p90=1.125s max=1.561s n=143
regression bank_search: diagonal completed=False, lateral-low completed=True, longitudinal completed=False, near-ground completed=True
