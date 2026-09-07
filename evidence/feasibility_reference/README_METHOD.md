# Offline reference feasibility analysis — method and limitations

This is NOT ground-truth feasibility. It is an independent, deliberately
permissive upper-bound analysis, named as instructed.

## What it does
Same Gen3 + 2F-85 model, object geometry and ground model as the controller,
loaded through mc_rbdyn/RBDyn python bindings. Per scenario:
- object presentation poses recovered from the run log (14 points), fitted
  linearly in lead and resampled at 0.1 s over [1.8, 8.0] -> 63 leads
- handle centre computed from the object pose and O_T_H
- orientation-free position IK of the gripper mouth onto the handle centre,
  damped least squares, 12 random restarts, 220 iterations, joint-limit clamped
- ground clearance for 7 arm link spheres and 3 gripper bodies, plus a base
  cylinder exclusion
- temporal bound from raw joint velocity limits: t_min = max|dq|/vmax, feasible
  if t_min + 1.4 s fixed acquire <= lead

## How it is broader than TRIAD
| dimension | TRIAD | reference |
|---|---|---|
| event times | 14 discrete leads | 63 leads at 0.1 s |
| grasps | 32 discrete ring grasps | orientation-free (any wrist orientation) |
| routes | 17 | none required (direct IK) |
| IK | one seeded rollout | 12 restarts |
| budget | real-time bounded | offline unbounded |

## Limitations, which are decisive here
It admits ANY wrist orientation, has no dense gripper-versus-object proxy
hierarchy, no self-collision, no grasp corridor, no closure sweep and no
approach-path certification. Those are exactly the constraints that bind in this
task. Consequently it marks all 62 scenarios reachable and temporally feasible
and therefore DISCRIMINATES NOTHING.

## What may and may not be concluded
May: no held-out scenario is rejected because the handle is out of gross
kinematic reach, nor because the arm is too slow in the gross joint-velocity
sense. Both of those hypotheses are excluded.

May not: any statement of the form "TRIAD covers X per cent of feasible
handovers". The reference feasible set is an upper bound of 62/62, so the
resulting coverage figures (61.3 per cent commit, 58.1 per cent end-to-end) are
LOWER BOUNDS ONLY and carry no more information than the raw counts.

A discriminating oracle would need the gripper proxy hierarchy, corridor and
closure constraints and an independent path planner. Implementing those would
re-derive TRIAD's own feasibility model and would be circular, which is why it
was not done.
