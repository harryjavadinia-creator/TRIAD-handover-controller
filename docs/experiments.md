# Experiment provenance and manifest

The release contains several experiment batches produced on different source
states. Do not assume every number was produced at the same commit.
[`provenance.md`](provenance.md) is the canonical source-state map.

## Dataset B — canonical finite event-time/grasp/route campaign

This is the clearest demonstration of the complete TRIAD planning decision.

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

See [Simulation](simulation.md) for scenario inputs, labels and expected output
details.

## Dataset A — historical July-19 latency matrix

The historical matrix contains 5 scenarios × 3 latency conditions
(ideal, compensated 0.220 s, uncompensated 0.220 s), for 15 runs.

The run records reported 11 completed and 4 fail-safe outcomes. Every ideal and
compensated run completed. Among uncompensated runs, the four moving-object
scenarios failed while the stationary scenario completed, as a stale position
has no translational effect when object velocity is zero.

Historical source: `e2e194d`, preserved by tag `dataset-a-baseline`.

The reproduction wrapper can prepare or run those cells:

```bash
scripts/reproduce_latency_matrix.sh --all --dry-run
```

For an actual run, provide the mc_rtc installation prefix as documented by the
script and [Quick start](quickstart.md).

## Corrected nonzero-delay ablation

The current release also preserves the corrected latency sweep under
`evidence/latency/`. It evaluates multiple configured delays after fixing the
configuration-mirror read used by the perception path.

The current publication interpretation is summarized in
[Results](results.md) and [Corrections of record](corrections_of_record.md).

## Asynchronous controller performance

The asynchronous publication provides clean-machine in-planning controller
profiles for the four canonical scenarios plus frozen plan-set and
timing-frontier records. See [Performance](performance.md) and
`evidence/async/`.

These measurements characterize one implementation on one machine. They are not
WCET or a formal schedulability proof.

## Baseline integrity

`SCIENTIFIC_BASELINE.sha256` and the source-sync manifests under `docs/` retain
the source provenance needed to distinguish the historical and current
implementations.

No result in this release is an end-to-end physical human-to-robot handover
campaign.
