| scenario | variant | runs | completed | first plan before giver rest | first adoption − rest (s) | FULL_SEARCH jobs / cancelled | discarded work units (median) | discarded wall (s, median) | unused worker wall before first plan (s, median) | worker wall to commit (s, median) | concurrent motion (s) | selected-action certificates ok/failed |
|---|---|---:|---:|---:|---|---|---:|---:|---:|---:|---:|---|
| longitudinal | cancel_and_restart (05b5b4c restart_a,b) | 2 | 2 | 0 | 0.57, 0.57 | 74/71 | 179770 | 2.64 | 4.49 | 7.05 | 0.0 | 0/0 |
| longitudinal | select_then_certify, no obsolete-job rule (05b5b4c select_a,b) | 2 | 1 | 0 | 2.05, 2.60 | 10/0 | 0 | 0.00 | 6.21 | 8.74 | 0.0 | 0/0 |
| longitudinal | select_then_certify + obsolete-job rule (73f05a5 final_a,b,c) | 3 | 3 | 0 | 2.15, 0.57, 1.76 | 35/29 | 123469 | 1.97 | 5.69 | 8.30 | 0.0 | 0/0 |
| near-ground | cancel_and_restart (05b5b4c restart_a,b) | 2 | 2 | 2 | -1.38, -1.16 | 71/67 | 54866 | 0.58 | 0.00 | 7.77 | 0.0 | 0/0 |
| near-ground | select_then_certify, no obsolete-job rule (05b5b4c select_a,b) | 2 | 2 | 2 | -1.39, -1.26 | 6/0 | 2361 | 0.05 | 0.00 | 11.16 | 0.0 | 0/10 |
| near-ground | select_then_certify + obsolete-job rule (73f05a5 final_a,b,c) | 3 | 3 | 3 | -1.41, -1.27, -1.44 | 33/27 | 33997 | 0.60 | 0.00 | 7.81 | 0.0 | 0/0 |
| lateral-low | cancel_and_restart (05b5b4c restart_a,b) | 2 | 1 | 0 | 0.48, 0.45 | 76/73 | 297468 | 4.49 | 4.49 | 7.45 | 0.0 | 0/0 |
| lateral-low | select_then_certify, no obsolete-job rule (05b5b4c select_a,b) | 2 | 2 | 1 | -0.01, 1.13 | 3/0 | 918 | 0.04 | 2.48 | 7.80 | 0.0 | 1/13 |
| lateral-low | select_then_certify + obsolete-job rule (73f05a5 final_a,b,c) | 3 | 0 | 0 | 1.18, 0.72, 0.71 | 36/30 | 282889 | 4.76 | 4.76 | 8.08 | 0.0 | 0/0 |
| diagonal | cancel_and_restart (05b5b4c restart_a,b) | 2 | 2 | 2 | -1.55, -1.65 | 2/0 | 0 | 0.00 | 0.00 | 5.19 | 1.2 | 0/0 |
| diagonal | select_then_certify, no obsolete-job rule (05b5b4c select_a,b) | 2 | 2 | 2 | -1.67, -1.49 | 2/0 | 0 | 0.00 | 0.00 | 5.21 | 1.2 | 0/0 |
| diagonal | select_then_certify + obsolete-job rule (73f05a5 final_a,b,c) | 3 | 3 | 3 | -1.57, -1.51, -1.58 | 3/0 | 0 | 0 | 0.00 | 5.21 | 1.2 | 0/0 |
