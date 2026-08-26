# Simulation reproduction

## Build, install, verify

See the top-level `README.md` for the verified build/install commands and
required environment. After installing:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
```

## Scenario selection without editing tracked files

`scripts/run_scenario.sh` selects a scenario by writing a small YAML
fragment to a **temporary** `$HOME/.config/mc_rtc/controllers/
HandoverInterceptionController.yaml`, for the duration of one run only. This
is mc_rtc's own per-controller user-configuration mechanism (a deep merge on
top of the installed default), pointed at an isolated temporary `$HOME` so
that:

- no tracked file is ever modified;
- your real, persistent `~/.config/mc_rtc/` is never read or written;
- the tree's `git status` stays clean before and after every run.

```bash
export MAIN_ROBOT_MODULE_PATH=/path/to/your/gen3_2f85/module/directory
scripts/run_scenario.sh <name> [output-dir]
```

See `docs/robot_module.md` for how to reconstruct the `gen3_2f85` module
directory (`scripts/setup_gen3_2f85_module.py`) from pinned upstream
packages, and for the equivalence evidence against the frozen module that
produced the results tabulated below.

The selected scenario, and its object translation/velocity, are printed
before the run starts. The log and the exact override fragment used are
preserved under `output-dir` (default: `results/<timestamp>_<name>/`).

## Available scenarios

| Name | Human description | Initial position `[x,y,z]` | Linear velocity `[vx,vy,vz]` |
| --- | --- | --- | --- |
| `near-ground` | Near-ground lateral motion | `[0.25, 0.62, 0.15]` | `[0.0, -0.08, 0.0]` |
| `longitudinal` | Longitudinal motion | `[0.92, 0.00, 0.55]` | `[-0.08, 0.0, 0.0]` |
| `lateral-low` | Lateral low-height motion | `[0.55, -0.56, 0.15]` | `[0.0, 0.08, 0.0]` |
| `diagonal` | Diagonal forward/upward motion | `[0.90, 0.00, 0.30]` | `[-0.0565685, 0.0, 0.0565685]` |

These are the four scenarios evaluated in the frozen global finite
time-grasp-route campaign at the scientific baseline commit (see
`docs/experiments.md`). The controller's configuration template also
catalogs a fifth preset, "Static nominal" (`[0.55, 0.00, 0.55]`, zero
velocity — a stationary object) — it is **not** part of that campaign
(there is no arrival-time residual to search over for a stationary object)
and is **not** currently supported by `scripts/run_scenario.sh`. Do not
treat "four scenarios" as the complete set of presets in the configuration
template; it is the complete set of *moving-object* scenarios in the
reported campaign.

Separately, an earlier perception-latency compensation study used its own,
different scenario/condition set — see `docs/experiments.md`. Its results
must not be merged with, or presented as part of, the four-scenario global
campaign above.

## Expected output

All values below are from the frozen scientific baseline (`GLOBAL_*_rep1`
runs). Deterministic quantities (selected event lead, selected grasp/route,
objective values) should match exactly given the same source and
configuration; timing quantities (predicted/actual completion time,
planning-phase elapsed time) can vary slightly run to run and machine to
machine — they are not promised to exact floating-point equality.

| Scenario | Predicted completion (s) | Actual completion (s) | Selected event lead (s) | Path length (m) | Min. reach clearance (m) | Joint-velocity utilization | Planning-phase elapsed (s, no-sync sim) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| near-ground | 9.801 | 8.469 | 4.60 | 0.631 | 0.079 | 1.00 | ≈1.36 |
| longitudinal | 8.703 | 7.289 | 3.70 | 0.275 | 0.080 | 1.00 | ≈1.39 |
| lateral-low | 8.803 | 7.435 | 4.15 | 0.398 | 0.082 | 1.00 | ≈0.88 |
| diagonal | 9.620 | 8.224 | 4.60 | 0.405 | 0.081 | ≈0.690 | ≈1.39 |

"Planning-phase elapsed" is wall-clock time for the no-sync simulation
planning phase, not a CPU benchmark — it has not been separately
CPU-benchmarked.

Representative runtime evidence for the diagonal scenario: 14 event
hypotheses, 233 complete/cost-valid/timing-admissible plans (cumulative
across the whole bounded search, not per hypothesis), selected event lead
4.600 s, `J_motion ≈ 0.596089263`, `J_global ≈ 0.684634405`.

## Independent verification

```bash
python3 tools/check_global_time_plan_log.py <log>
```

Expected output: `global time-grasp-route runtime log proof: PASS`.

**What it verifies**: schedule completeness (every configured event
hypothesis evaluated); exactly one selection/commit pair; per-hypothesis
reconciliation of pooled vs. cost-invalid vs. geometry-rejected candidates;
that the reported minimum is not lower than any logged valid cost; that the
committed candidate is the exact argmin over the cost-valid and
timing-admissible set; selection/commit numeric and identity agreement
within tolerance; full-handover completion; and, independently from a
hard-coded copy of the frozen seven-term weight vector, that every logged
cost reconstructs from `(T,E,L,C,Q,K,V)` and that `V` matches
`clamp01(velocityUtil)^4`.

**What it does not verify**: wall-clock/CPU planning performance; grasp or
route geometric correctness (it trusts the logged candidate names); real
robot behavior; or that the run used a particular scenario (it has no
scenario-identity check).

### Scenario identity

`check_global_time_plan_log.py` verifies selector/runtime consistency but
not which scenario produced the log. `tools/verify_scenario_identity.py` is
a separate, narrower check for that:

```bash
python3 tools/verify_scenario_identity.py <log> --expect-scenario diagonal
```

It confirms the log's own recorded initial object position (from the
`[ObserveObject] ... p0=...` line) and settled velocity (from the *last*
`[ObserveObject] t=.../... ... v=...` sample line) both match the named
scenario's configuration — position within a tolerance derived from the
log's own numeric display precision, velocity within a scenario-identity
tolerance covering both display rounding and the small settled-velocity
estimation variation observed across validated runs (not a physical-
accuracy claim) — and that the completion class (completed vs.
fail-safe) matches what was expected. `SCENARIO_IDENTITY_RESULT=PASS`
requires both the position and velocity checks to pass, reported separately
as `position_ok`/`velocity_ok` alongside `observed_position`/
`observed_velocity`. `scripts/run_scenario.sh` runs both checkers
automatically and reports `RUNTIME_CHECKER_RESULT` and
`SCENARIO_IDENTITY_RESULT` as separate fields.

## Simulation-scope caveats

- `allowPhysicalExecution` is `false` for all reported results — physical
  execution was disabled for these runs.
- The virtual force sensor (`decisionCost` / `physicalBridge` configuration)
  is a simulation source, not a physical force measurement.
- The copied-state preview used during candidate evaluation is a predictive
  approximation, not exact QP-rollout parity.
