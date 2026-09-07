# Low-height degradation audit

TRIAD held-out success by object height: 36 per cent at z 0.18-0.25 (n=11),
60 per cent at 0.30-0.40 (n=42), 78 per cent at 0.45-0.55 (n=9).

Reference measurement: best achievable gripper ground clearance at the handle,
by height band:

| z band | n | min | mean | max | TRIAD success |
|---|---|---|---|---|---|
| 0.18-0.25 | 11 | +0.1288 | +0.2086 | +0.5765 | 36 % |
| 0.30-0.40 | 42 | +0.2095 | +0.4025 | +0.8346 | 60 % |
| 0.45-0.55 | 9  | +0.3678 | +0.5572 | +0.8670 | 78 % |

No scenario has a reference clearance below 0.02 m, so absolute ground
collision is not the binding constraint and gross reachability is not either.
What does vary monotonically with height is the clearance MARGIN, which shrinks
by roughly a factor of three from the high to the low band.

Best supported explanation: at low object height the subset of approach
directions that keeps the whole gripper clear of the ground shrinks, so a
smaller fraction of TRIAD's fixed 32-grasp ring and 17-route bank remains
admissible. The degradation is therefore attributed to the interaction of a
fixed, finite grasp/route bank with a shrinking admissible approach cone,
rather than to physical unreachability.

This remains an attribution, not a proof: the reference cannot enumerate which
specific grasps become inadmissible, because it does not model the gripper
proxy hierarchy or the grasp corridor.
