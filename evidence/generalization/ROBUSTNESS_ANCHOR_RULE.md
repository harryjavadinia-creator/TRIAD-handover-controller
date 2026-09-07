# Robustness anchor selection rule (fixed before any perturbation was run)

Anchors are chosen from held_out_scenarios.csv by geometry and input values
only. No outcome of any run is consulted.

Rule, applied in order, taking the lowest scenario id where a tie occurs:

1. Two interior anchors: the first two scenarios with region == "interior",
   speed == 0.08, and distinct velocity families.
2. Two near-boundary anchors: the first scenario with region == "low_height"
   and the first with region == "lateral" and y < 0.
3. Two multi-axis / higher-motion anchors: the first scenario with
   family == "diag_xyz", and the first with region == "speed_sweep" and
   speed == 0.11.

Perturbation magnitudes are EXPERIMENTAL values, not system specifications.
They are set from implementation-derived quantities where one exists:
- position steps of +/- 0.020 m and +/- 0.040 m, chosen relative to the
  0.015 m commit-freshness guard and the 0.010 m ground safety margin, so the
  smaller step is of the same order as the tolerances the planner reasons about
- speed steps of +/- 0.02 m/s, one quarter of the development speed
- direction rotation of +/- 10 degrees about the world z axis

All perturbed cases stay inside the declared operating envelope. Orientation is
not perturbed, because the implementation is only exercised at one orientation.
