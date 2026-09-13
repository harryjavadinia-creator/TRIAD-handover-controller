## Worker timing per job type

Wall = worker wall time of the job; latency = controller time from submission to receipt.

| job | outcome | n | wall median | wall p95 | wall max | latency median | latency max | hypotheses (median) | static records (median) | route records (median) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| FULL_SEARCH | accepted | 12 | 0.5501 | 3.3248 | 3.5908 | 0.550 | 3.592 | 14 | 32 | 102 |
| FULL_SEARCH | cancelled | 211 | 0.0148 | 0.0324 | 3.8956 | 0.016 | 3.897 | 1 | 6 | 0 |
| RECERTIFY_ACTIVE | accepted | 4408 | 0.0042 | 0.0082 | 0.0154 | 0.005 | 0.050 | 0 | 0 | 1 |
| RECERTIFY_ACTIVE | cancelled | 4 | 0.0026 | 0.0030 | 0.0031 | 0.003 | 0.004 | 0 | 0 | 1 |
| TERMINAL_CERTIFY | accepted | 210 | 0.0024 | 0.0034 | 0.0044 | 0.003 | 0.005 | 0 | 0 | 1 |

### Stage wall time (sum over jobs, completed or accepted jobs only)

Nested buckets are contained in the phase buckets; phase buckets partition the certification work.

| job | staticReachStandoff | staticReachCapture | staticClosure | staticRetreat | routeSetup | routeReach | routeApproach | routeDwell | routeClosure | routeRetreat | routeFinalize | hypothesisSetup | terminalStandoff | nestedIkStep | nestedSweptQuery | nestedConfigurationSafety | nestedClosureSafety |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| FULL_SEARCH (n=12, wall 18.027s) | 2.966s (16%)/482862 | 0.668s (4%)/107760 | 0.382s (2%)/58967 | 0.374s (2%)/51860 | 0.020s (0%)/2669 | 10.715s (59%)/351145 | 0.353s (2%)/54933 | 0.017s (0%)/7650 | 0.854s (5%)/137674 | 0.524s (3%)/71430 | 1.054s (6%)/1477 | 0.003s (0%)/77 | 0.000s (0%)/0 | 7.624s (42%)/1200512 | 7.165s (40%)/348584 | 2.052s (11%)/1431890 | 1.442s (8%)/281952 |
| RECERTIFY_ACTIVE (n=4408, wall 20.668s) | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.000s (0%)/0 | 10.149s (49%)/287517 | 1.273s (6%)/156956 | 0.063s (0%)/22040 | 3.271s (16%)/397406 | 1.822s (9%)/200461 | 3.937s (19%)/4407 | 0.000s (0%)/0 | 0.000s (0%)/0 | 7.272s (35%)/877388 | 6.480s (31%)/283109 | 2.013s (10%)/978567 | 4.422s (21%)/639631 |
| TERMINAL_CERTIFY (n=210, wall 0.537s) | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.000s (0%)/0 | 0.064s (12%)/7338 | 0.003s (1%)/1050 | 0.168s (31%)/18977 | 0.093s (17%)/9470 | 0.198s (37%)/209 | 0.000s (0%)/0 | 0.002s (0%)/210 | 0.251s (47%)/28065 | 0.000s (0%)/0 | 0.045s (8%)/19500 | 0.226s (42%)/30299 |

### Cancellation

- FULL_SEARCH: 211 cancelled; controller-time cancel latency median 0.001 s, max 0.005 s; worker stopped by cancel in 211, finished before the request was observed in 0
- RECERTIFY_ACTIVE: 4 cancelled; controller-time cancel latency median 0.001 s, max 0.002 s; worker stopped by cancel in 0, finished before the request was observed in 4

### Planning-snapshot age and drift at result receipt

| job | outcome | n | age median s | age max s | robot drift max m | object displacement max m | snapshot prediction error max m |
|---|---|---:|---:|---:|---:|---:|---:|
| FULL_SEARCH | accepted | 12 | 0.550 | 3.592 | 0.0001 | 0.2874 | 0.0000 |
| FULL_SEARCH | cancelled | 211 | 0.016 | 3.897 | 0.0000 | 0.3112 | 0.0005 |
| RECERTIFY_ACTIVE | accepted | 4408 | 0.005 | 0.050 | 0.0148 | 0.0011 | 0.0001 |
| RECERTIFY_ACTIVE | cancelled | 4 | 0.003 | 0.004 | 0.0000 | 0.0000 | 0.0000 |
| TERMINAL_CERTIFY | accepted | 210 | 0.003 | 0.005 | 0.0001 | 0.0000 | 0.0000 |
