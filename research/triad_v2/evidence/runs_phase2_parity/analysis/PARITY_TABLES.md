### Reach: copied-state preview vs runtime (per executed plan)

| run | plan | candidate / route | ended | executed fraction | time-aligned max / RMS path dev (mm) | spatial max dev (mm) | max ori dev (rad) | standoff err preview / runtime (mm) | arrival preview / runtime (s after reach start) | min clearance preview / runtime (mm) | max presentation update while executing (mm) |
|---|---:|---|---|---:|---|---:|---:|---|---|---|---:|
| parity_a/diagonal | 1 | axisP_side_337deg / ring80mm_2of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.52 | 12.3 / 7.5 | 5.8 | 0.040 | 2.0 / — | 2.02 / — | 74.5 / 136.1 | 0.5 |
| parity_a/diagonal | 2 | axisP_side_113deg / ring140mm_6of8 | invalidated:recertification_infeasible/predictive_static/sta | 0.87 | 25.6 / 11.6 | 16.3 | 0.024 | 1.2 / — | 3.24 / — | 39.6 / 35.7 | 0.0 |
| parity_a/lateral-low | 1 | axisN_side_45deg / ring80mm_6of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.06 | 1.4 / 0.8 | 0.2 | 0.006 | 0.9 / — | 2.58 / — | 64.3 / 229.8 | 2.9 |
| parity_a/lateral-low | 2 | axisN_side_337deg / direct | invalidated:recertification_infeasible/predictive_static/sta | 0.62 | 174.8 / 88.1 | 58.2 | 0.243 | 1.8 / — | 1.94 / — | 81.6 / 57.6 | 0.0 |
| parity_a/lateral-low | 3 | axisN_side_337deg / direct | invalidated:terminal_certification_infeasible/predictive_sta | 1.00 | 5.2 / 3.4 | 2.3 | 0.061 | 2.0 / 3.7 | 0.76 / 0.76 | 57.0 / 57.0 | 0.0 |
| parity_a/longitudinal | 1 | axisP_side_337deg / ring80mm_1of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.48 | 76.1 / 31.3 | 23.9 | 0.038 | 1.5 / — | 2.28 / — | 81.4 / 125.5 | 5.8 |
| parity_a/longitudinal | 2 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.21 | 4.6 / 2.6 | 1.5 | 0.028 | 1.6 / — | 1.28 / — | 81.3 / 120.5 | 0.0 |
| parity_a/longitudinal | 3 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.19 | 3.9 / 2.1 | 1.7 | 0.024 | 1.5 / — | 1.28 / — | 81.4 / 116.8 | 0.0 |
| parity_a/near-ground | 1 | axisP_side_247deg / ring140mm_5of8 | invalidated:recertification_infeasible/cost_invalid/timeout | 0.00 | — / — | — | — | 0.8 / — | 3.76 / — | 82.3 / — | 0.0 |
| parity_a/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 11.0 / 6.8 | 10.9 | 0.052 | 1.2 / 5.5 | 3.04 / 3.01 | 79.0 / 79.5 | 0.0 |
| parity_b/diagonal | 1 | axisP_side_337deg / ring80mm_2of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.51 | 11.3 / 7.4 | 5.7 | 0.040 | 2.0 / — | 2.02 / — | 74.5 / 136.1 | 0.3 |
| parity_b/diagonal | 2 | axisP_side_113deg / ring140mm_6of8 | invalidated:recertification_infeasible/predictive_static/sta | 0.88 | 28.7 / 12.8 | 18.2 | 0.022 | 1.1 / — | 3.28 / — | 40.6 / 36.8 | 0.0 |
| parity_b/lateral-low | 1 | axisN_side_45deg / ring80mm_6of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.06 | 1.4 / 0.8 | 0.2 | 0.006 | 0.9 / — | 2.58 / — | 64.3 / 229.8 | 2.9 |
| parity_b/lateral-low | 2 | axisN_side_337deg / direct | invalidated:recertification_infeasible/predictive_static/sta | 0.62 | 175.2 / 88.7 | 58.5 | 0.238 | 1.8 / — | 1.94 / — | 81.6 / 56.8 | 0.0 |
| parity_b/lateral-low | 3 | axisN_side_337deg / direct | invalidated:terminal_certification_infeasible/predictive_sta | 1.00 | 5.3 / 3.4 | 2.3 | 0.057 | 2.0 / 3.6 | 0.76 / 0.77 | 56.4 / 56.4 | 0.0 |
| parity_b/longitudinal | 1 | axisP_side_337deg / ring80mm_1of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.48 | 76.1 / 31.3 | 23.9 | 0.038 | 1.5 / — | 2.28 / — | 81.4 / 125.5 | 6.0 |
| parity_b/longitudinal | 2 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.21 | 4.6 / 2.6 | 1.6 | 0.028 | 1.6 / — | 1.28 / — | 81.3 / 120.5 | 0.0 |
| parity_b/longitudinal | 3 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.19 | 3.9 / 2.2 | 1.8 | 0.025 | 1.5 / — | 1.28 / — | 81.4 / 116.5 | 0.0 |
| parity_b/near-ground | 1 | axisP_side_247deg / ring140mm_5of8 | invalidated:recertification_infeasible/cost_invalid/timeout | 0.00 | 0.3 / 0.3 | 0.3 | 0.000 | 0.8 / — | 3.76 / — | 82.3 / 229.8 | 0.0 |
| parity_b/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 11.0 / 6.8 | 10.9 | 0.053 | 1.2 / 5.5 | 3.04 / 3.01 | 79.0 / 79.5 | 0.0 |
| parity_c/diagonal | 1 | axisP_side_337deg / ring80mm_2of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.52 | 11.9 / 7.5 | 5.8 | 0.040 | 2.0 / — | 2.02 / — | 74.5 / 136.1 | 0.4 |
| parity_c/diagonal | 2 | axisP_side_113deg / ring140mm_6of8 | invalidated:recertification_infeasible/predictive_static/sta | 0.88 | 26.2 / 11.8 | 16.7 | 0.024 | 1.2 / — | 3.22 / — | 39.8 / 35.8 | 0.0 |
| parity_c/lateral-low | 1 | axisN_side_337deg / direct | reached_standoff_window | 1.00 | 163.5 / 74.3 | 49.8 | 0.236 | 1.8 / 6.0 | 1.94 / 1.90 | 81.6 / 64.6 | 0.0 |
| parity_c/longitudinal | 1 | axisP_side_337deg / direct | reached_standoff_window | 1.00 | 107.4 / 64.2 | 30.5 | 0.115 | 1.6 / 107.3 | 2.06 / — | 81.2 / 82.4 | 4.0 |
| parity_c/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 11.0 / 6.8 | 10.9 | 0.053 | 1.2 / 5.5 | 3.04 / 3.01 | 79.0 / 79.5 | 0.0 |
| parity_d/diagonal | 1 | axisP_side_337deg / ring80mm_2of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.51 | 11.3 / 7.4 | 5.7 | 0.040 | 2.0 / — | 2.02 / — | 74.5 / 136.1 | 0.4 |
| parity_d/diagonal | 2 | axisP_side_113deg / ring140mm_6of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.78 | 29.3 / 10.8 | 18.7 | 0.025 | 1.2 / — | 3.26 / — | 40.5 / 36.7 | 0.0 |
| parity_d/lateral-low | 1 | axisN_side_337deg / direct | invalidated:recertification_infeasible/predictive_static/sta | 0.64 | 163.5 / 84.4 | 49.8 | 0.236 | 1.8 / — | 1.94 / — | 81.6 / 64.6 | 0.0 |
| parity_d/lateral-low | 2 | axisN_side_337deg / direct | invalidated:terminal_certification_infeasible/predictive_sta | 1.00 | 4.9 / 3.2 | 2.3 | 0.056 | 1.9 / 3.9 | 0.70 / 0.71 | 66.8 / 66.8 | 0.0 |
| parity_d/longitudinal | 1 | axisP_side_337deg / ring80mm_1of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.48 | 76.1 / 31.3 | 23.9 | 0.038 | 1.5 / — | 2.28 / — | 81.4 / 125.5 | 6.0 |
| parity_d/longitudinal | 2 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.21 | 4.7 / 2.6 | 1.6 | 0.027 | 1.6 / — | 1.28 / — | 81.3 / 120.5 | 0.0 |
| parity_d/longitudinal | 3 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/run | 0.19 | 3.9 / 2.2 | 1.8 | 0.022 | 1.5 / — | 1.28 / — | 81.4 / 116.6 | 0.0 |
| parity_d/near-ground | 1 | axisP_side_247deg / ring140mm_5of8 | invalidated:recertification_infeasible/cost_invalid/timeout | 0.00 | 0.3 / 0.3 | 0.3 | 0.000 | 0.8 / — | 3.76 / — | 82.3 / 229.8 | 0.0 |
| parity_d/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 11.0 / 6.8 | 10.9 | 0.052 | 1.2 / 5.5 | 3.04 / 3.01 | 79.0 / 79.5 | 0.0 |
| parity_e/diagonal | 1 | axisP_side_337deg / ring80mm_2of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.51 | 11.3 / 7.4 | 5.7 | 0.040 | 2.0 / — | 2.02 / — | 74.5 / 136.1 | 0.3 |
| parity_e/diagonal | 2 | axisP_side_113deg / ring140mm_6of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.78 | 29.3 / 10.8 | 18.6 | 0.025 | 1.2 / — | 3.26 / — | 40.5 / 36.7 | 0.0 |
| parity_e/lateral-low | 1 | axisN_side_337deg / direct | reached_standoff_window | 1.00 | 163.5 / 74.3 | 49.8 | 0.236 | 1.8 / 6.0 | 1.94 / 1.90 | 81.6 / 64.6 | 0.0 |
| parity_e/longitudinal | 1 | axisP_side_337deg / direct | reached_standoff_window | 1.00 | 107.4 / 64.2 | 30.5 | 0.115 | 1.6 / 107.3 | 2.06 / — | 81.2 / 82.4 | 4.0 |
| parity_e/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 11.0 / 6.8 | 10.9 | 0.052 | 1.2 / 5.5 | 3.04 / 3.01 | 79.0 / 79.5 | 0.0 |
| parity_f/diagonal | 1 | axisP_side_337deg / ring80mm_2of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.51 | 11.3 / 7.4 | 5.7 | 0.040 | 2.0 / — | 2.02 / — | 74.5 / 136.1 | 0.3 |
| parity_f/diagonal | 2 | axisP_side_113deg / ring140mm_6of8 | invalidated:recertification_infeasible/predictive_static/sta | 0.88 | 28.8 / 12.9 | 18.3 | 0.022 | 1.1 / — | 3.28 / — | 40.6 / 36.9 | 0.0 |
| parity_f/lateral-low | 1 | axisN_side_337deg / direct | reached_standoff_window | 1.00 | 163.5 / 74.3 | 49.8 | 0.236 | 1.8 / 6.0 | 1.94 / 1.90 | 81.6 / 64.6 | 0.0 |
| parity_f/longitudinal | 1 | axisP_side_337deg / ring80mm_1of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.48 | 76.1 / 31.3 | 23.9 | 0.038 | 1.5 / — | 2.28 / — | 81.4 / 125.5 | 5.8 |
| parity_f/longitudinal | 2 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.21 | 4.5 / 2.5 | 1.5 | 0.027 | 1.6 / — | 1.28 / — | 81.3 / 120.8 | 0.0 |
| parity_f/longitudinal | 3 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.20 | 4.0 / 2.2 | 1.8 | 0.024 | 1.5 / — | 1.28 / — | 81.4 / 116.5 | 0.0 |
| parity_f/near-ground | 1 | axisP_side_247deg / ring140mm_5of8 | invalidated:recertification_infeasible/cost_invalid/timeout | 0.00 | — / — | — | — | 0.8 / — | 3.76 / — | 82.3 / — | 0.0 |
| parity_f/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 11.0 / 6.8 | 10.9 | 0.053 | 1.2 / 5.5 | 3.04 / 3.01 | 79.0 / 79.5 | 0.0 |
| speed004/longitudinal | 1 | axisP_side_337deg / ring80mm_2of8 | invalidated:recertification_infeasible/predictive_static/sta | 0.60 | 6.4 / 5.1 | 3.3 | 0.033 | 2.5 / — | 2.08 / — | 80.2 / 161.5 | 0.0 |
| speed004/longitudinal | 2 | axisP_side_337deg / ring80mm_1of8 | invalidated:recertification_infeasible/predictive_static/sta | 0.31 | 7.8 / 5.9 | 1.9 | 0.023 | 2.4 / — | 1.68 / — | 80.1 / 156.0 | 0.0 |
| speed004/near-ground | 1 | axisP_side_45deg / ring140mm_7of8 | invalidated:terminal_certification_infeasible/predictive_sta | 1.00 | 15.5 / 8.0 | 15.4 | 0.067 | 1.2 / 5.4 | 3.06 / 3.04 | 75.0 / 78.3 | 0.0 |
| speed004/near-ground | 2 | axisP_side_0deg / direct | reached_standoff_window | 1.00 | 2.4 / 1.5 | 0.7 | 0.046 | 1.5 / 2.2 | 0.48 / 0.51 | 80.6 / 79.3 | 0.0 |
| speed016/longitudinal | 1 | axisP_side_337deg / direct | reached_standoff_window | 1.00 | 25.3 / 13.3 | 25.3 | 0.086 | 2.4 / 5.8 | 1.86 / 1.80 | 80.4 / 82.4 | 0.0 |
| speed016/near-ground | 1 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 11.0 / 6.8 | 10.9 | 0.053 | 1.2 / 5.5 | 3.04 / 3.01 | 79.0 / 79.5 | 0.0 |
| stop0425/longitudinal | 1 | axisP_side_337deg / direct | reached_standoff_window | 1.00 | 107.4 / 64.5 | 30.5 | 0.115 | 1.6 / 107.3 | 2.06 / — | 81.2 / 82.4 | 4.3 |
| stop0425/near-ground | 1 | axisP_side_247deg / ring140mm_5of8 | invalidated:recertification_infeasible/cost_invalid/timeout | 0.03 | 1.5 / 0.9 | 0.3 | 0.003 | 0.8 / — | 3.76 / — | 82.3 / 229.8 | 2.9 |
| stop0425/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 12.5 / 7.0 | 12.4 | 0.062 | 1.1 / 5.4 | 3.06 / 3.05 | 79.4 / 79.5 | 0.0 |
| lat022c/longitudinal | 1 | axisP_side_337deg / direct | reached_standoff_window | 1.00 | 109.1 / 62.6 | 31.7 | 0.117 | 1.6 / 109.0 | 2.06 / — | 81.2 / 82.3 | 3.3 |
| lat022c/near-ground | 1 | axisP_side_247deg / ring140mm_5of8 | invalidated:recertification_infeasible/cost_invalid/timeout | 0.05 | 2.6 / 1.5 | 0.3 | 0.006 | 0.8 / — | 3.76 / — | 82.4 / 229.8 | 1.8 |
| lat022c/near-ground | 2 | axisP_side_68deg / ring140mm_0of8 | reached_standoff_window | 1.00 | 17.4 / 9.8 | 10.7 | 0.038 | 1.4 / 5.4 | 3.08 / 3.06 | 81.2 / 79.1 | 0.0 |
| lat022u/longitudinal | 1 | axisP_side_337deg / ring80mm_1of8 | invalidated:recertification_infeasible/predictive_static/rea | 0.47 | 55.1 / 19.8 | 17.2 | 0.041 | 1.5 / — | 2.24 / — | 81.3 / 127.9 | 1.6 |
| lat022u/longitudinal | 2 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.21 | 4.7 / 2.6 | 1.6 | 0.029 | 1.6 / — | 1.28 / — | 81.3 / 122.6 | 0.0 |
| lat022u/longitudinal | 3 | axisP_side_23deg / direct | invalidated:recertification_infeasible/predictive_static/rea | 0.18 | 3.8 / 2.1 | 1.7 | 0.022 | 1.5 / — | 1.28 / — | 81.4 / 118.9 | 0.0 |
| lat022u/near-ground | 1 | axisP_side_247deg / ring140mm_5of8 | invalidated:recertification_infeasible/cost_invalid/timeout | 0.03 | 1.3 / 0.8 | 0.3 | 0.003 | 0.8 / — | 3.74 / — | 78.2 / 229.8 | 0.7 |
| lat022u/near-ground | 2 | axisP_side_45deg / ring140mm_7of8 | reached_standoff_window | 1.00 | 13.7 / 7.4 | 13.4 | 0.064 | 1.2 / 5.5 | 3.04 / 3.02 | 79.3 / 79.5 | 0.0 |

### Terminal: certificate used at commitment vs post-commit runtime

| run | completed | insertion predicted (audit) / runtime (s) | acquire predicted / runtime (s) | retreat predicted / runtime (s) | total predicted / runtime (s) | insertion max path dev / end err (mm) | retreat max path dev / end err (mm) | retreat clearance predicted / runtime min (mm) |
|---|---|---|---|---|---|---|---|---|
| parity_a/diagonal | False | no commit | | | | | | |
| parity_a/lateral-low | False | no commit | | | | | | |
| parity_a/longitudinal | False | no commit | | | | | | |
| parity_a/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.594 / 5.366 | 14.7 / 12.0 | 12.5 / 3.9 | 97.9 / 91.5 |
| parity_b/diagonal | False | no commit | | | | | | |
| parity_b/lateral-low | False | no commit | | | | | | |
| parity_b/longitudinal | False | no commit | | | | | | |
| parity_b/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.594 / 5.366 | 14.7 / 12.0 | 12.5 / 3.7 | 97.9 / 91.5 |
| parity_c/diagonal | False | no commit | | | | | | |
| parity_c/lateral-low | True | 1.060 / 1.069 | 2.694 / 2.154 | 1.100 / 0.564 | 7.379 / 5.314 | 14.6 / 12.1 | 10.3 / 2.8 | 80.8 / 79.1 |
| parity_c/longitudinal | True | 1.040 / 1.069 | 2.668 / 2.154 | 1.125 / 0.564 | 7.483 / 5.314 | 14.6 / 12.2 | 9.6 / 4.6 | 219.3 / 206.4 |
| parity_c/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.594 / 5.366 | 14.7 / 12.0 | 12.5 / 3.7 | 97.9 / 91.5 |
| parity_d/diagonal | False | no commit | | | | | | |
| parity_d/lateral-low | False | no commit | | | | | | |
| parity_d/longitudinal | False | no commit | | | | | | |
| parity_d/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.594 / 5.366 | 14.7 / 12.0 | 12.5 / 3.9 | 97.9 / 91.5 |
| parity_e/diagonal | False | no commit | | | | | | |
| parity_e/lateral-low | True | 1.060 / 1.069 | 2.694 / 2.154 | 1.100 / 0.564 | 7.379 / 5.314 | 14.6 / 12.1 | 10.3 / 2.8 | 80.8 / 79.1 |
| parity_e/longitudinal | True | 1.040 / 1.069 | 2.668 / 2.154 | 1.125 / 0.564 | 7.483 / 5.314 | 14.6 / 12.2 | 9.6 / 4.6 | 219.3 / 206.4 |
| parity_e/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.594 / 5.366 | 14.7 / 12.0 | 12.5 / 3.9 | 97.9 / 91.5 |
| parity_f/diagonal | False | no commit | | | | | | |
| parity_f/lateral-low | True | 1.060 / 1.069 | 2.694 / 2.154 | 1.100 / 0.564 | 7.379 / 5.314 | 14.6 / 12.1 | 10.3 / 2.8 | 80.8 / 79.1 |
| parity_f/longitudinal | False | no commit | | | | | | |
| parity_f/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.594 / 5.366 | 14.7 / 12.0 | 12.5 / 3.7 | 97.9 / 91.5 |
| speed004/longitudinal | False | no commit | | | | | | |
| speed004/near-ground | True | 1.080 / 1.083 | 2.668 / 2.159 | 1.225 / 0.587 | 5.973 / 5.356 | 14.3 / 11.7 | 8.7 / 5.1 | 93.9 / 88.8 |
| speed016/longitudinal | True | 1.040 / 1.069 | 2.668 / 2.154 | 1.125 / 0.565 | 7.283 / 5.315 | 13.9 / 11.5 | 8.9 / 2.9 | 218.7 / 211.3 |
| speed016/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.594 / 5.366 | 14.7 / 12.0 | 12.5 / 3.9 | 97.9 / 91.5 |
| stop0425/longitudinal | True | 1.040 / 1.069 | 2.668 / 2.154 | 1.125 / 0.564 | 7.483 / 5.314 | 14.6 / 12.2 | 9.6 / 4.6 | 219.3 / 206.2 |
| stop0425/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.585 | 8.624 / 5.365 | 14.8 / 12.0 | 12.0 / 5.1 | 98.6 / 91.9 |
| lat022c/longitudinal | True | 1.040 / 1.069 | 2.668 / 2.154 | 1.125 / 0.564 | 7.483 / 5.314 | 14.6 / 12.2 | 9.6 / 4.2 | 219.3 / 206.4 |
| lat022c/near-ground | True | 1.080 / 1.086 | 2.668 / 2.165 | 2.400 / 0.616 | 9.872 / 5.394 | 14.9 / 12.2 | 9.6 / 8.3 | 90.5 / 87.4 |
| lat022u/longitudinal | False | no commit | | | | | | |
| lat022u/near-ground | True | 1.120 / 1.088 | 2.694 / 2.165 | 1.100 / 0.586 | 8.596 / 5.366 | 14.8 / 12.0 | 12.3 / 4.0 | 98.5 / 91.8 |
