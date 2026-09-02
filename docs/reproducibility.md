# Reproducibility guide

This repository is organized so that the scientific method, the experiment provenance, and the runtime verification tools can be inspected separately.

## Reproducibility levels

### Level 1 — source-only checks

These checks do not require mc_rtc or the robot module:

```bash
bash tools/run_binding_cost_checks.sh
python3 tools/test_setup_gen3_2f85_module.py
python3 tools/test_verify_latency_matrix_cell.py
python3 tools/test_verify_scenario_identity.py
python3 tools/test_scenario_override_yaml.py
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
```

They cover selector unit tests, source-level cost checks, runtime-checker fixtures, scenario-identity fixtures, robot-module reconstruction checks and frozen-baseline integrity.

### Level 2 — clean build and install

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

The controller installs into the runtime directories associated with the mc_rtc installation used at configure time. See [`troubleshooting.md`](troubleshooting.md).

### Level 3 — robot-model reconstruction

Reconstruct the verified Kinova Gen3 + Robotiq 2F-85 module from pinned upstream description packages:

```bash
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf /path/to/kortex_description/robots/gen3_2f85.urdf \
  --kortex-share /path/to/share/kortex_description \
  --robotiq-share /path/to/share/robotiq_description \
  --output /path/to/gen3_2f85_module

export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
```

The reconstruction script verifies the pinned upstream URDF and referenced mesh contents before producing the mc_rtc module. See [`robot_module.md`](robot_module.md).

### Level 4 — Dataset-B scenario reproduction

Run one of the four moving-object scenarios:

```bash
scripts/run_scenario.sh longitudinal
```

A valid reproduction ends with:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

The wrapper preserves the runtime log, exact temporary scenario override and checker outputs in the result directory.

Available names are:

```text
near-ground
longitudinal
lateral-low
diagonal
```

See [`simulation.md`](simulation.md) for the exact scenario table and expected deterministic outputs.

### Level 5 — Dataset-A latency reproduction

The historical latency study is a separate experiment and uses the preserved `dataset-a-baseline` source state.

Example:

```bash
scripts/reproduce_latency_matrix.sh \
  --scenario pure_x \
  --condition compensated220 \
  --mc-rtc-prefix /path/to/your/mc_rtc/install
```

To generate all 15 scenario/condition configurations without building or running:

```bash
scripts/reproduce_latency_matrix.sh --all --dry-run
```

See [`experiments.md`](experiments.md) for provenance and expected outcome classes.

## What is deterministic

Given the same scientific source and configuration, the following are treated as deterministic scientific outputs:

- event-hypothesis set;
- complete-plan identities;
- selected event lead;
- selected grasp;
- selected route;
- finite objective values;
- minimum-cost timing-admissible selection;
- winner fingerprint.

The runtime checker independently reconstructs the binding objective from logged terms and verifies that the committed plan is the exact minimum over the logged cost-valid and timing-admissible finite set.

## What is not promised to be bit-identical across machines

Do not treat the following as cross-machine exact outputs:

- wall-clock planning duration;
- operating-system scheduling behavior;
- control-cycle wall-time distribution;
- GUI timing;
- physical sensor behavior;
- hardware interaction timing.

Timing results must therefore be accompanied by the timing metric, measurement window and machine/runtime context.

## Timing-analysis reproducibility

The scenario-specific timing-frontier analysis is defined analytically from complete-plan records and the exact selector rule. See [`timing_frontiers.md`](timing_frontiers.md).

The important distinction is:

- **scientific/logical simulation time** used by the controller during the no-sync run;
- **external planner wall time** that would elapse while the physical world continues to move.

The hardware-facing counterfactual replay substitutes real planner duration into the exact timing-admission inequality; it does not change the controller's policy.

## Evidence and provenance rules

When reporting a result, identify:

1. source commit or named baseline;
2. scenario and condition;
3. whether the evidence is Dataset A or Dataset B;
4. timing metric, if any;
5. whether the result is simulation or physical hardware;
6. checker outcome;
7. any attribution limitation documented in [`experiments.md`](experiments.md).

Do not combine Dataset-A latency results with Dataset-B global finite-plan results as if they were one experiment.

## Publication checklist

Before tagging a paper-associated release, verify:

- clean working tree;
- source-only regression suite passes;
- scientific-baseline manifest verifies;
- fresh configure/build/install succeeds;
- robot-module reconstruction test passes;
- all four Dataset-B wrappers complete with both runtime and identity checkers passing;
- any promoted performance implementation has its own exact-equivalence evidence;
- README, mathematics, simulation, experiments and timing documents agree on terminology;
- no claim describes `3.976 s` as a universal planner deadline;
- no claim describes simulation support as end-to-end physical-robot validation.
