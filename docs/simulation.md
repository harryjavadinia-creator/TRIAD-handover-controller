# Simulation reference

For the build and live visualization, start with [Quick start](quickstart.md).
The four named scenarios below are the inputs of the finite-plan campaign (tag
`scientific-baseline`).

## Scope

The four-scenario moving-object campaign exercises the finite-plan selector. It is simulation evidence with
`allowPhysicalExecution: false`.

The perception-latency matrix (tag `dataset-a-baseline`) belongs to a separate
source state and is attributed in [`provenance.md`](provenance.md). The corrected latency experiment
is summarized in [`experiments.md`](experiments.md) and [`results.md`](results.md).

## Build

Use the build procedure in [Quick start](quickstart.md) (the controller runs from
its build tree; nothing is installed). The dependency-free scientific checks run
from the source checkout:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
```

## Scenario selection without tracked-file edits

`scripts/run_scenario.sh` writes a small controller-override YAML file under a
temporary `HOME`. mc_rtc merges that fragment over the installed default
configuration for the duration of the run.

This mechanism means:

- tracked source/configuration files are not edited;
- the user's persistent `~/.config/mc_rtc/` is not read or written;
- the exact temporary override is preserved with the result.

```bash
export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
export TRIAD_BUILD_DIR=/path/to/TRIAD-handover-controller/build
export MC_RTC_INSTALL=/path/to/your/mc_rtc/install
scripts/run_scenario.sh <name> [output-dir]
```

The default result directory is `results/<timestamp>_<name>/`.

## The four scenarios

| Command | Internal label | Initial position `[x,y,z]` | Linear velocity `[vx,vy,vz]` |
| --- | --- | --- | --- |
| `near-ground` | `GROUND_NEAR` | `[0.25, 0.62, 0.15]` | `[0.0, -0.08, 0.0]` |
| `longitudinal` | `PURE_X` | `[0.92, 0.00, 0.55]` | `[-0.08, 0.0, 0.0]` |
| `lateral-low` | `CANONICAL_YZ` | `[0.55, -0.56, 0.15]` | `[0.0, 0.08, 0.0]` |
| `diagonal` | `DIAGONAL_XZ` | `[0.90, 0.00, 0.30]` | `[-0.0565685, 0.0, 0.0565685]` |

A static presentation (`static_nominal`) exists only as a Robot B scenario of
the two-robot runner (`TRIAD_GIVER_SCENARIO=static_nominal`).

## Reference winners

Two record sets exist and they commit different plans, because timing
admission is evaluated at the simulated time the background search returns.

**At the `scientific-baseline` source state** (exact-serial search, listed in
[`provenance.md`](provenance.md); records in `evidence/async/*/frozen_plan_set.txt`):

| Scenario | Event lead (s) | Grasp | Route | `J_global` |
| --- | ---: | --- | --- | ---: |
| near-ground | 4.600 | `axisP_side_45deg` | `ring80mm_0of8` | 0.822892544 |
| longitudinal | 3.700 | `axisP_side_337deg` | `direct` | 0.686806299 |
| lateral-low | 4.150 | `axisN_side_337deg` | `direct` | 0.700830630 |
| diagonal | 4.600 | `axisP_side_337deg` | `ring140mm_2of8` | 0.684634405 |

**From the current sources, run from a fresh clone** (build tree, no viewer;
logs in `evidence/reference_runs/`, table in
[Experiments](experiments.md#reference-run-winners)): longitudinal 5.500 s /
`axisP_side_337deg` / `ring80mm_1of8` / 0.806835232; near-ground 6.850 s /
`axisP_side_68deg` / `ring80mm_1of8` / 1.126642447; lateral-low 8.000 s /
`axisN_side_45deg` / `ring80mm_6of8` / 0.896429684; diagonal 4.600 s /
`axisP_side_337deg` / `ring80mm_1of8` / 0.698600183.

The first table is the exact-serial campaign and is not a measurement of the
background-planning search; the second is. An identical winner is not
guaranteed under a different timing condition (machine, viewer attached,
`TRIAD_SYNC_RATIO`). See [Validation scope](release_validation.md).

## Runtime verification

A successful wrapper run reports:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

`tools/check_global_time_plan_log.py` verifies the global selector/runtime
record. Among other checks it:

- confirms the configured bounded schedule was fully evaluated;
- reconciles valid, invalid and geometry-rejected alternatives;
- verifies selection/commit identity;
- independently reconstructs the frozen seven-term objective;
- uses `[GlobalPlanTimingAdmissibility]` records, when present, to verify the
  exact argmin over the cost-valid and final-timing-admissible set.

It does not independently prove collision geometry, real-robot behavior or
scenario identity.

Scenario identity is checked separately:

```bash
python3 tools/verify_scenario_identity.py <log> --expect-scenario diagonal
```

The identity checker compares the logged initial object position and settled
velocity with the named scenario and checks the expected completion class.

## Three timing quantities

The repository distinguishes:

1. **logical/controller planning elapsed** — no-sync simulation time associated
   with the number of `SolveInterception` cycles;
2. **external planner wall time** — real elapsed computation time;
3. **scenario-specific timing boundary** — a counterfactual planner duration
   derived from the exact final timing-admission rule.

The first is not a CPU benchmark. The second is machine-dependent. The third is
computed from complete-plan timing records and the selector rule.

Replay a run with:

```bash
python3 tools/replay_timing_frontier.py <log> --planner-time 3.976
```

See [`timing_frontiers.md`](timing_frontiers.md).

## Simulation limitations

- The scenario runs use `allowPhysicalExecution: false`.
- The virtual load-transfer source is not a physical force measurement.
- Copied-state preview is a predictive approximation, not exact QP-rollout parity.
- Timing-frontier replay is a counterfactual analysis, not an end-to-end
  physical experiment.
