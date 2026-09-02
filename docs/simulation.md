# Simulation reproduction

## Build, install, verify

See the top-level `README.md` for the verified build/install commands and required environment. After installing:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
```

## Scenario selection without editing tracked files

`scripts/run_scenario.sh` selects a scenario by writing a small YAML fragment to a **temporary** `$HOME/.config/mc_rtc/controllers/HandoverInterceptionController.yaml`, for the duration of one run only. This is mc_rtc's own per-controller user-configuration mechanism, pointed at an isolated temporary `$HOME`, so that:

- no tracked file is modified;
- the user's persistent `~/.config/mc_rtc/` is not read or written;
- the working tree remains clean before and after the run.

```bash
export MAIN_ROBOT_MODULE_PATH=/path/to/your/gen3_2f85/module/directory
scripts/run_scenario.sh <name> [output-dir]
```

See `docs/robot_module.md` for reconstruction of the verified `gen3_2f85` module from pinned upstream packages.

The selected scenario, object translation and object velocity are printed before the run. The runtime log and exact override fragment are preserved under `output-dir` (default: `results/<timestamp>_<name>/`).

## Available scenarios

| Name | Internal label | Human description | Initial position `[x,y,z]` | Linear velocity `[vx,vy,vz]` |
| --- | --- | --- | --- | --- |
| `near-ground` | GROUND_NEAR | Near-ground lateral motion | `[0.25, 0.62, 0.15]` | `[0.0, -0.08, 0.0]` |
| `longitudinal` | PURE_X | Longitudinal motion | `[0.92, 0.00, 0.55]` | `[-0.08, 0.0, 0.0]` |
| `lateral-low` | CANONICAL_YZ | Lateral low-height motion | `[0.55, -0.56, 0.15]` | `[0.0, 0.08, 0.0]` |
| `diagonal` | DIAGONAL_XZ | Diagonal forward/upward motion | `[0.90, 0.00, 0.30]` | `[-0.0565685, 0.0, 0.0565685]` |

These are the four moving-object scenarios evaluated in the frozen global finite event-time–grasp–route campaign.

The controller configuration also catalogs a stationary `static_nominal` preset (`[0.55, 0.00, 0.55]`, zero velocity). It is not part of Dataset B and is not currently supported by `scripts/run_scenario.sh`.

An earlier perception-latency study used its own scenario/condition set. See `docs/experiments.md`. Do not merge Dataset-A and Dataset-B results.

## Expected scientific output

All values below are from the frozen scientific baseline. Deterministic quantities such as the selected event lead, selected grasp/route and objective values should reproduce exactly given the same scientific source and configuration. Wall-clock timing can vary with machine/runtime conditions.

| Scenario | Predicted completion (s) | Actual completion (s) | Selected event lead (s) | Path length (m) | Min. reach clearance (m) | Joint-velocity utilization | Logged logical planning elapsed (s) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| near-ground | 9.801 | 8.469 | 4.60 | 0.631 | 0.079 | 1.00 | ≈1.36 |
| longitudinal | 8.703 | 7.289 | 3.70 | 0.275 | 0.080 | 1.00 | ≈1.39 |
| lateral-low | 8.803 | 7.435 | 4.15 | 0.398 | 0.082 | 1.00 | ≈0.88 |
| diagonal | 9.620 | 8.224 | 4.60 | 0.405 | 0.081 | ≈0.690 | ≈1.39 |

The last column is the controller/no-sync simulation's **logical planning elapsed value**, tied to the number of `SolveInterception` control cycles. It must not be presented as a hardware CPU benchmark or as equivalent to external wall-clock planning latency.

Representative diagonal evidence: 14 event hypotheses, 233 complete/cost-valid/timing-admissible plans accumulated across the bounded search, selected event lead 4.600 s, `J_motion ≈ 0.596089263`, `J_global ≈ 0.684634405`.

## Independent verification

```bash
python3 tools/check_global_time_plan_log.py <log>
```

Expected output:

```text
global time-grasp-route runtime log proof: PASS
```

The checker verifies schedule completeness, exactly one selection/commit pair, per-hypothesis reconciliation, the exact finite minimum over cost-valid and timing-admissible records, selection/commit agreement, full-handover completion, and independent reconstruction of the frozen objective from logged terms.

It does **not** verify wall-clock/CPU performance, physical geometry independently of the controller's logged candidate identities, real-robot behavior, or scenario identity.

### Scenario identity

Scenario identity is checked separately:

```bash
python3 tools/verify_scenario_identity.py <log> --expect-scenario diagonal
```

It verifies the initial object position and settled velocity against the named scenario and checks the expected completion class. `scripts/run_scenario.sh` runs both the scientific runtime checker and the scenario-identity checker automatically.

## Timing: three quantities that must not be conflated

TRIAD's evidence now distinguishes three timing notions:

1. **Logical/controller planning elapsed** — determined by the number of no-sync `SolveInterception` control cycles in simulation.
2. **Measured external planner wall time** — actual elapsed wall-clock duration spent computing while the physical world would continue to move.
3. **Scenario-specific timing-admission frontier** — the counterfactual physical planner duration beyond which the timing-admissible set changes or becomes empty.

The previously quoted `3.976 s` value is not universal. It belongs to PURE_X / longitudinal only. Exact scenario-specific frontiers are documented in [`timing_frontiers.md`](timing_frontiers.md).

This distinction is essential for hardware interpretation: a no-sync simulation can preserve a logical decision time even when several seconds of external wall time are consumed by computation.

## Simulation-scope caveats

- `allowPhysicalExecution` is `false` for all reported Dataset-B results.
- The virtual force sensor used by the simulation is not a physical force measurement.
- The copied-state preview used during candidate evaluation is a predictive approximation, not exact QP-rollout parity.
- Hardware-facing timing analysis is a counterfactual replay of the exact selector rule; it is not itself an end-to-end physical-robot experiment.
