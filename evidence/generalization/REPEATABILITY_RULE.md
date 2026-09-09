# Repeatability subset selection rule (fixed before repetitions were run)

The specification requires successful cases, so conditioning on outcome A is
sanctioned. Within that constraint no further selection freedom is taken:

1. Interior: the three LOWEST scenario ids with outcome A and region "interior".
2. Boundary/stress: the two LOWEST scenario ids with outcome A and region in
   {low_height, high_height, lateral, multiaxis, speed_sweep}.

Each selected case is repeated 3 times with identical inputs. Checks:
FrozenPlanSet hash determinism, outcome repeatability, and whether any
winner/timing difference is explained by the result-receipt time t_sel.
