# Simulation experiments

This page summarizes the experiment set used in the current TRIAD release. The
release contains results produced on more than one historical source state, so
[`provenance.md`](provenance.md) remains the canonical source-state map.

All end-to-end handover results reported here are from simulation.

## Canonical TRIAD scenarios

The four canonical moving-object scenarios are the clearest demonstration of
the complete TRIAD planning decision. For each scenario, TRIAD selects a future
event time, grasp and route from the bounded finite plan bank.

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

These four cases show that the finite selector can produce different complete
`(event time, grasp, route)` decisions for different object motions and
geometries.

See [Simulation](simulation.md) for scenario inputs, labels and expected-output
details.

## Corrected prediction-latency experiment

The current release preserves the corrected nonzero-delay latency sweep under
`evidence/latency/`. This experiment compares compensated and uncompensated
prediction across several configured measurement delays.

The purpose is to test how delayed object observations affect the predicted
future presentation state and the final handover outcome when the same TRIAD
planning logic is used.

The current publication interpretation, including the distinct 0.60 s outcomes,
is summarized in [Results](results.md) and [Corrections of record](corrections_of_record.md).

The experiment is intentionally reported as a latency/prediction ablation, not
as a general robustness or generalization claim.

## Reproduction and provenance

The canonical scenarios can be reproduced through the scenario wrapper after
following [Quick start](quickstart.md):

```bash
scripts/run_scenario.sh longitudinal
```

The other canonical scenarios are `near-ground`, `lateral-low` and `diagonal`.

The corrected latency records are preserved under `evidence/latency/` and can be
checked through the release evidence and source manifests. Historical experiment
states that are not part of the main current experiment story remain traceable
through [Provenance](provenance.md) and the preserved repository tags.

## Scope

These experiments demonstrate the finite TRIAD decision process and the effect
of prediction latency under the documented simulation conditions. They do not
establish feasible-space coverage, human-subject performance, WCET, or an
end-to-end physical human-to-robot handover validation campaign.

`SCIENTIFIC_BASELINE.sha256` and the source-sync manifests under `docs/` retain
the source provenance needed to distinguish historical and current
implementations.
