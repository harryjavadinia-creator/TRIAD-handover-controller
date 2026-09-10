# Reproducibility

Use [Quick start](quickstart.md) for installation, model reconstruction and live
simulation. Use [Results](results.md) to inspect the archived release evidence
without installing mc_rtc.

## Check the repository

From the repository root:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/test_replay_timing_frontier.py
python3 tools/test_setup_gen3_2f85_module.py
python3 tools/test_verify_latency_matrix_cell.py
python3 tools/test_verify_scenario_identity.py
python3 tools/test_scenario_override_yaml.py
python3 tools/check_markdown_links.py
python3 tools/check_publication_claims.py
python3 tools/test_check_planner_core_purity.py
python3 tools/check_planner_core_purity.py
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 --commit scientific-baseline
sha256sum -c docs/source_sync_f56add3.sha256
python3 tools/check_evidence_manifest.py
```

These checks exercise the selectors and checker fixtures, verify source/evidence
contents and check publication-document consistency. Passing them does not
establish physical safety, WCET or complete race freedom.

## Reproduce a canonical scenario

After setup:

```bash
scripts/run_scenario.sh longitudinal
```

Use `near-ground`, `lateral-low` or `diagonal` for the other canonical inputs.
The wrapper preserves the scenario override, log and checker outputs under
`results/`.

Record the checkout identity and environment alongside any new run:

```bash
git rev-parse HEAD
git status --short
```

## What should and should not match

| Quantity | Reproduction contract |
| --- | --- |
| Frozen source and archived evidence | exact content, checked by supplied manifests |
| Generated finite bank and objective calculations | deterministic for the same source, configuration and relevant inputs |
| Timing-admissible set and winner | also depend on actual selector time and numerical tie convention |
| Completion | depends on admission and execution conditions |
| Wall time and visualization | machine-dependent |

## Reproduce analyses from archived records

[Results](results.md) gives the figure-generation command. It reads included
records without executing the controller.

For a log with final timing-diagnostic records:

```bash
python3 tools/replay_timing_frontier.py results/<run>/longitudinal.log \
  --planner-time 3.808 --planner-time 3.976
```

See [Timing frontiers](timing_frontiers.md).

## Preserved experimental protocols

| Protocol | Documentation / preserved version |
| --- | --- |
| Historical five-scenario latency matrix | [Experiments](experiments.md), tag `dataset-a-baseline` |
| Four canonical finite-plan scenarios | [Simulation](simulation.md), tag `scientific-baseline` |
| Exact-serial performance/revalidation | [Performance](performance.md), tag `csi-2026-release` |
| Corrected latency and asynchronous records | [Evidence index](../evidence/README.md) and [Provenance](provenance.md) |

The historical exact-serial manifest verifies at its historical release tag,
not against the current asynchronous source. Interpretation notes in
[Corrections of record](corrections_of_record.md) qualify superseded wording
where necessary.
