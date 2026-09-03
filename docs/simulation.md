# Simulation reproduction

## Scope

Dataset B is the four-scenario moving-object campaign for the finite global event-time–grasp–route selector. It is simulation evidence: `allowPhysicalExecution: false`.

Dataset A is an earlier, separate perception-latency study. Its numbers and source attribution are documented in [`experiments.md`](experiments.md).

## Build and install

Use the verified build procedure in the top-level [`README.md`](../README.md). After installation, the dependency-free scientific checks can still be run from the source checkout:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
```

## Scenario selection without tracked-file edits

`scripts/run_scenario.sh` writes a small controller-override YAML file under a temporary `HOME`. mc_rtc merges that fragment over the installed default configuration for the duration of the run.

This mechanism means:

- tracked source/configuration files are not edited;
- the user's persistent `~/.config/mc_rtc/` is not read or written;
- the exact temporary override is preserved with the result.

```bash
export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
scripts/run_scenario.sh <name> [output-dir]
```

The default result directory is `results/<timestamp>_<name>/`.

## Dataset-B scenarios

| Command | Internal label | Initial position `[x,y,z]` | Linear velocity `[vx,vy,vz]` |
| --- | --- | --- | --- |
| `near-ground` | `GROUND_NEAR` | `[0.25, 0.62, 0.15]` | `[0.0, -0.08, 0.0]` |
| `longitudinal` | `PURE_X` | `[0.92, 0.00, 0.55]` | `[-0.08, 0.0, 0.0]` |
| `lateral-low` | `CANONICAL_YZ` | `[0.55, -0.56, 0.15]` | `[0.0, 0.08, 0.0]` |
| `diagonal` | `DIAGONAL_XZ` | `[0.90, 0.00, 0.30]` | `[-0.0565685, 0.0, 0.0565685]` |

The controller also contains a `static_nominal` preset. It is not part of Dataset B and is not exposed by `run_scenario.sh`.

## Frozen Dataset-B outputs

The original scientific campaign is anchored to the frozen `scientific-baseline` provenance described in [`experiments.md`](experiments.md). The following deterministic selection quantities are the reference values:

| Scenario | Event lead (s) | Grasp | Route | `J_global` |
| --- | ---: | --- | --- | ---: |
| near-ground | 4.600 | `axisP_side_45deg` | `ring80mm_0of8` | 0.822892544 |
| longitudinal | 3.700 | `axisP_side_337deg` | `direct` | 0.686806299 |
| lateral-low | 4.150 | `axisN_side_337deg` | `direct` | 0.700830630 |
| diagonal | 4.600 | `axisP_side_337deg` | `ring140mm_2of8` | 0.684634405 |

Additional reference metrics from the frozen campaign are:

| Scenario | Predicted completion (s) | Actual completion (s) | Path length (m) | Min. reach clearance (m) | Joint-velocity utilization | Logical planning elapsed (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| near-ground | 9.801 | 8.469 | 0.631 | 0.079 | 1.00 | ≈1.36 |
| longitudinal | 8.703 | 7.289 | 0.275 | 0.080 | 1.00 | ≈1.39 |
| lateral-low | 8.803 | 7.435 | 0.398 | 0.082 | 1.00 | ≈0.88 |
| diagonal | 9.620 | 8.224 | 0.405 | 0.081 | ≈0.690 | ≈1.39 |

The exact-serial source currently shipped in this repository was revalidated against the four deterministic winner fingerprints above. See [`release_validation.md`](release_validation.md).

## Runtime verification

A successful wrapper run reports:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

`tools/check_global_time_plan_log.py` verifies the global selector/runtime proof. Among other checks it:

- confirms the configured bounded schedule was fully evaluated;
- reconciles valid, invalid and geometry-rejected alternatives;
- verifies selection/commit identity;
- independently reconstructs the frozen seven-term objective;
- uses `[GlobalPlanTimingAdmissibility]` records, when present, to verify the exact argmin over the cost-valid and final-timing-admissible set.

It does not independently prove collision geometry, real-robot behavior or scenario identity.

Scenario identity is checked separately:

```bash
python3 tools/verify_scenario_identity.py <log> --expect-scenario diagonal
```

The identity checker compares the logged initial object position and settled velocity with the named scenario and checks the expected completion class.

## Three timing quantities

The repository distinguishes:

1. **logical/controller planning elapsed** — no-sync simulation time associated with the number of `SolveInterception` cycles;
2. **external planner wall time** — real elapsed computation time;
3. **scenario-specific timing boundary** — a counterfactual planner duration derived from the exact final timing-admission rule.

The first is not a CPU benchmark. The second is machine-dependent. The third is computed from complete-plan timing records and the selector rule.

Replay a run with:

```bash
python3 tools/replay_timing_frontier.py <log> --planner-time 3.976
```

See [`timing_frontiers.md`](timing_frontiers.md).

## Simulation limitations

- Dataset-B runs use `allowPhysicalExecution: false`.
- The virtual load-transfer source is not a physical force measurement.
- Copied-state preview is a predictive approximation, not exact QP-rollout parity.
- Hardware-facing timing replay is a counterfactual analysis, not an end-to-end physical experiment.
