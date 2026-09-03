# Reproducibility guide

TRIAD separates four reproducibility targets:

1. source-level selector and checker behavior;
2. build/install reproducibility;
3. robot-model reconstruction;
4. experiment reproduction.

The frozen scientific tags preserve historical evidence, while the current publication source contains the audited exact-serial implementation.

## Level 1: dependency-free checks

These checks require only a C++ compiler, Python 3 and Git:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/test_replay_timing_frontier.py
python3 tools/test_setup_gen3_2f85_module.py
python3 tools/test_verify_latency_matrix_cell.py
python3 tools/test_verify_scenario_identity.py
python3 tools/test_scenario_override_yaml.py
python3 tools/check_markdown_links.py

python3 tools/verify_scientific_baseline.py \
  SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline

sha256sum -c docs/source_sync_82e6eaa.sha256
```

They cover the within-event and cross-event selectors, binding-cost source integration and runtime-checker fixtures, timing-frontier replay logic, robot-module reconstruction safety/validation, latency-cell and scenario-identity verification, scenario-override generation, local documentation links, frozen scientific-baseline integrity and byte identity of the source synchronized from `82e6eaa`.

The GitHub Actions workflow runs this dependency-free layer automatically.

## Level 2: clean build and install

With a working mc_rtc installation:

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH \
  CMAKE_PREFIX_PATH=/path/to/your/mc_rtc/install \
  cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_DISABLE_FIND_PACKAGE_rclcpp=ON

cmake --build build -j"$(nproc)"
cmake --install build
```

The controller installs into the runtime locations associated with the mc_rtc installation used at configure time. See [`troubleshooting.md`](troubleshooting.md).

## Level 3: reconstruct the robot module

The Kinova Gen3 + Robotiq 2F-85 mc_rtc module is reconstructed from pinned upstream artifacts rather than redistributed:

```bash
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf /path/to/kortex_description/robots/gen3_2f85.urdf \
  --kortex-share /path/to/share/kortex_description \
  --robotiq-share /path/to/share/robotiq_description \
  --output /path/to/gen3_2f85_module

export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
```

The setup tool validates the pinned URDF, expected structural transformations and all referenced mesh contents before producing the module. See [`robot_module.md`](robot_module.md).

## Level 4: reproduce Dataset B

| Command | Dataset-B label |
| --- | --- |
| `near-ground` | `GROUND_NEAR` |
| `longitudinal` | `PURE_X` |
| `lateral-low` | `CANONICAL_YZ` |
| `diagonal` | `DIAGONAL_XZ` |

Run one:

```bash
scripts/run_scenario.sh longitudinal
```

A successful reproduction ends with:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

The wrapper preserves the run log, exact temporary scenario override and checker outputs in `results/`.

The most recent publication synchronization was revalidated on all four scenarios; see [`release_validation.md`](release_validation.md).

## Level 5: reproduce Dataset A

Dataset A is the earlier perception-latency study and is intentionally separate from Dataset B. Its historical source state is preserved as `dataset-a-baseline`.

Example:

```bash
scripts/reproduce_latency_matrix.sh \
  --scenario pure_x \
  --condition compensated220 \
  --mc-rtc-prefix /path/to/your/mc_rtc/install
```

Generate all 15 scenario/condition configurations without building or running:

```bash
scripts/reproduce_latency_matrix.sh --all --dry-run
```

See [`experiments.md`](experiments.md) for source attribution and expected outcome classes.

## Reproduce timing-admission analysis

Any Dataset-B run containing the final timing-diagnostic records can be analysed directly:

```bash
python3 tools/replay_timing_frontier.py \
  results/<run>/longitudinal.log \
  --planner-time 3.808 \
  --planner-time 3.976
```

The tool first verifies the logged timing flags against the selector inequalities, then derives the analytical fail-closed boundary and reports the admissible set at requested counterfactual planner durations.

See [`timing_frontiers.md`](timing_frontiers.md).

## Deterministic scientific outputs

Given the same scientific source/configuration, the reproducibility contract covers:

- generated event schedule;
- complete-plan identities;
- selected event lead;
- selected grasp and route;
- objective values;
- timing-admissible set at the logged selector time;
- committed winner fingerprint.

The runtime checker independently reconstructs the binding objective and, where timing-diagnostic records are available, verifies the finite argmin over the final admissible set.

## Machine-dependent outputs

The following are not cross-machine exact outputs:

- external wall-clock planning duration;
- operating-system scheduling;
- wall-time distribution of control cycles;
- GUI timing;
- physical sensor behavior;
- hardware interaction timing.

Performance claims therefore state the timing metric, measurement window and runtime context separately from deterministic scientific outputs.

## Evidence rules

A reported result should identify:

1. source commit/tag or named baseline;
2. scenario and condition;
3. Dataset A or Dataset B;
4. timing metric, when timing is discussed;
5. simulation versus physical hardware;
6. checker result;
7. any attribution limitation documented in [`experiments.md`](experiments.md).

## Release checklist

Before tagging a paper-associated release:

- working tree clean;
- dependency-free suite passes;
- local Markdown links pass;
- frozen scientific-baseline manifest verifies;
- exact-serial source-sync manifest verifies;
- fresh configure/build/install succeeds;
- robot-module reconstruction tests pass;
- all four Dataset-B scenario wrappers pass both runtime and identity checks if controller/config/runtime behavior changed;
- timing-frontier replay tests pass;
- README, mathematics, architecture, simulation, experiments and timing documents use consistent terminology;
- hardware-facing statements remain clearly separated from physical-robot validation.
