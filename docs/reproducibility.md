# Reproducibility

Use [Quick start](quickstart.md) for installation, model reconstruction, and
live simulation. Use [Results](results.md) to inspect the archived evidence
without installing mc_rtc.

## Check the repository

From the repository root, with Git, Python 3, and a C++ compiler:

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

These checks exercise selectors and checker fixtures, verify the included
source/evidence contents, and check documentation consistency. The static
planner guard has the coverage limits stated in [Architecture](architecture.md).
Passing the checks does not establish physical safety or complete race freedom.

[GitHub Actions](../.github/workflows/source-checks.yml) also runs Python and
shell syntax checks. The [validation page](release_validation.md) distinguishes
source, build, runtime, and visualization evidence.

## Reproduce a scenario

After the [setup](quickstart.md):

```bash
scripts/run_scenario.sh longitudinal
```

Use `near-ground`, `lateral-low`, or `diagonal` for the other canonical
inputs. The wrapper preserves the scenario override, log, and checker outputs
under `results/` without editing tracked configuration or persistent mc_rtc
settings. The [simulation reference](simulation.md) explains each outcome.

Record the checkout identity and environment alongside your run:

```bash
git rev-parse HEAD
git status --short
```

A commit identifier is a source-version fingerprint, not a scientific parameter.
The everyday reading path uses descriptive names; exact versions are retained
in [Provenance](provenance.md) so a result can be traced to its source.

## What should and should not match

| Quantity | Reproduction contract |
| --- | --- |
| Frozen source and archived data | Exact content, checked by the supplied manifests |
| Generated bank and objective calculations | Deterministic for the same source, configuration, and relevant inputs |
| Timing-admissible set and winner | Also depend on the actual selector time and numerical tie convention |
| Completion | Depends on admission and execution conditions; report actual checker outputs |
| Wall time, scheduling, and visualization | Machine-dependent; not exact cross-machine outputs |

All runs made from the current checkout are new local runs. The historical
four-scenario exact-serial reruns retain their original attribution. The
asynchronous near-ground comparison retains 229 prior scientific payloads and
adds 54 after normalizing `sourceIndex`; raw record sets are not byte-identical.

## Reproduce analyses from archived records

[Results](results.md) gives the figure-generation command. It reads the
included records without executing the controller.

For a log with final timing-diagnostic records:

```bash
python3 tools/replay_timing_frontier.py results/<run>/longitudinal.log \
  --planner-time 3.808 --planner-time 3.976
```

The replay checks logged timing flags against selector inequalities and derives
the scenario-specific counterfactual boundary. See [Timing frontiers](timing_frontiers.md).

## Earlier experimental protocols

| Protocol | Documentation and preserved version |
| --- | --- |
| Dataset A: five-scenario latency matrix | [Experiments](experiments.md), tag `dataset-a-baseline` |
| Dataset B: four finite-plan reference scenarios | [Simulation](simulation.md), tag `scientific-baseline` |
| Exact-serial performance and revalidation | [Performance](performance.md), tag `csi-2026-release` |
| Held-out, perturbation, corrected-latency, and asynchronous records | [Evidence index](../evidence/README.md) and [Provenance](provenance.md) |

The earlier latency wrapper can generate all 15 condition configurations
without building or running:

```bash
scripts/reproduce_latency_matrix.sh --all --dry-run
```

The historical exact-serial manifest verifies at its historical release tag,
not against the current asynchronous source. Primary archived evidence remains
unchanged; [interpretation notes](corrections_of_record.md) qualify superseded
wording where necessary.
