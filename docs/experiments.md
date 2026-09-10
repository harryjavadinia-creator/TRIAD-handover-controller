# Simulation experiments

This page summarizes the documented TRIAD simulation experiments. Results were
produced on more than one source state; [`provenance.md`](provenance.md) records
the corresponding source attribution.

All end-to-end handover results reported here are from simulation.

## Canonical TRIAD scenarios

The four canonical moving-object scenarios exercise the complete TRIAD planning
decision. For each scenario, TRIAD selects a future event time, grasp and route
from the bounded finite plan bank.

| Field | Value |
| --- | --- |
| Scenarios | near-ground, longitudinal, lateral-low, diagonal |
| Selection mode | `binding_cost` |
| Event policy | `global_time_plan` |
| Physical execution | disabled; simulation |
| Historical source | `c07368c`, preserved by tag `scientific-baseline` |
| Checker | `tools/check_global_time_plan_log.py` |

Reference winners:

| Scenario | Event lead (s) | Grasp | Route | Global cost |
| --- | ---: | --- | --- | ---: |
| Near-ground | 4.600 | `axisP_side_45deg` | `ring80mm_0of8` | 0.822892544 |
| Longitudinal | 3.700 | `axisP_side_337deg` | `direct` | 0.686806299 |
| Lateral-low | 4.150 | `axisN_side_337deg` | `direct` | 0.700830630 |
| Diagonal | 4.600 | `axisP_side_337deg` | `ring140mm_2of8` | 0.684634405 |

These cases produce different complete `(event time, grasp, route)` decisions
under different object motions and geometries.

See [Simulation](simulation.md) for scenario inputs, labels and expected-output
details.

## Corrected prediction-latency experiment

The corrected nonzero-delay latency sweep is stored under `evidence/latency/`.
It compares compensated and uncompensated prediction across several configured
measurement delays while using the same TRIAD planning logic.

The experiment tests how delayed object observations affect the predicted
future presentation state and the final simulated handover outcome. The
recorded outcomes, including the distinct 0.60 s cases, are summarized in
[Results](results.md) and qualified in
[Technical qualifications](corrections_of_record.md).

The evidence supports a latency/prediction ablation under the documented
simulation conditions; it is not a general robustness or generalization study.

## Reproduction and provenance

After following [Quick start](quickstart.md), a canonical scenario can be run
with:

```bash
scripts/run_scenario.sh longitudinal
```

The other canonical scenarios are `near-ground`, `lateral-low` and `diagonal`.

The corrected latency records are stored under `evidence/latency/`. Earlier
experiment states remain traceable through [Provenance](provenance.md) and the
repository tags.

## Scope

These experiments demonstrate the finite TRIAD decision process and the effect
of prediction latency under the documented simulation conditions. They do not
establish feasible-space coverage, human-subject performance, WCET, or an
end-to-end physical human-to-robot handover validation campaign.

`SCIENTIFIC_BASELINE.sha256` and the source-sync manifests under `docs/` retain
the source provenance needed to distinguish the corresponding implementations
and datasets.
