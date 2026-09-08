# Reproducibility guide

TRIAD separates five reproducibility targets:

1. source-level selector/checker behavior;
2. integrity of the frozen/public source snapshots;
3. build/install reproducibility;
4. robot-model reconstruction;
5. experiment/runtime reproduction, with explicit source-state provenance.

Historical tags preserve historical experiments. The current publication branch
contains the frozen asynchronous implementation synchronized from `f56add3`.
Those facts must not be collapsed into a claim that every historical run was
executed at the current branch head.

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

sha256sum -c docs/source_sync_f56add3.sha256

cd evidence && sha256sum -c MANIFEST.sha256 && cd ..
python3 tools/check_evidence_manifest.py
```

They cover the within-event/cross-event selectors, binding-cost source
integration and checker fixtures, timing-frontier replay logic, robot-module
reconstruction tests, latency-cell and scenario-identity verification,
scenario-override generation, local documentation links, frozen
scientific-baseline integrity, source synchronization from `f56add3`, and the
integrity/internal consistency of the reduced evidence package.

`docs/source_sync_82e6eaa.sha256` is retained as the historical exact-serial
record and verifies against `csi-2026-release`, **not** against the current
asynchronous branch.

The GitHub Actions workflow runs the dependency-free layer automatically.
Passing these checks is not physical validation and is not a substitute for a
runtime campaign.

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

The controller installs into the runtime locations associated with the mc_rtc
installation used at configure time. See [`troubleshooting.md`](troubleshooting.md).

The publication pass recorded a clean configure/build for the current
asynchronous branch. That establishes buildability in the tested environment;
it does not by itself establish a fresh current-head simulation or hardware
runtime campaign.

## Level 3: reconstruct the robot module

The Kinova Gen3 + Robotiq 2F-85 mc_rtc module is reconstructed from pinned
upstream artifacts rather than redistributed:

```bash
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf /path/to/kortex_description/robots/gen3_2f85.urdf \
  --kortex-share /path/to/share/kortex_description \
  --robotiq-share /path/to/share/robotiq_description \
  --output /path/to/gen3_2f85_module

export MAIN_ROBOT_MODULE_PATH=/path/to/gen3_2f85_module
```

The setup tool validates the pinned URDF, expected structural transformations
and referenced mesh contents before producing the module. See
[`robot_module.md`](robot_module.md).

## Level 3b: check the published evidence without running the simulator

```bash
cd evidence && sha256sum -c MANIFEST.sha256 && cd ..
python3 tools/check_evidence_manifest.py
```

This verifies the reduced evidence digests, pre-execution hashes for the two
predeclared input sets, published plan-set hashes and headline counts derived
from raw records. The checker also guards several corrected publication
interpretations such as H002's post-hoc diagnostic status and near-ground
`sourceIndex` normalization.

See [`../evidence/README.md`](../evidence/README.md).

## Level 4: historical Dataset-B reproduction

| Command | Dataset-B label |
| --- | --- |
| `near-ground` | `GROUND_NEAR` |
| `longitudinal` | `PURE_X` |
| `lateral-low` | `CANONICAL_YZ` |
| `diagonal` | `DIAGONAL_XZ` |

Example:

```bash
scripts/run_scenario.sh longitudinal
```

The historical exact-serial four-scenario revalidation documented in
[`release_validation.md`](release_validation.md) belongs to the historical
synchronized state around `123be4a` / `a006912`. It must not be silently
relabelled as a new runtime validation of the current asynchronous branch.

When running the current branch yourself, preserve the exact branch/commit,
scenario override, runtime environment and checker output as a **new local run**.
Do not compare machine-dependent wall-clock numbers as if they were deterministic
cross-machine outputs.

## Level 5: historical Dataset-A reproduction

Dataset A is the earlier perception-latency study and is intentionally separate
from Dataset B. Its historical source state is preserved as
`dataset-a-baseline`.

Example:

```bash
scripts/reproduce_latency_matrix.sh \
  --scenario pure_x \
  --condition compensated220 \
  --mc-rtc-prefix /path/to/your/mc_rtc/install
```

Generate all 15 scenario/condition configurations without building/running:

```bash
scripts/reproduce_latency_matrix.sh --all --dry-run
```

See [`experiments.md`](experiments.md) for attribution and expected historical
outcome classes.

## Reproduce timing-admission analysis

A Dataset-B-style log containing final timing-diagnostic records can be analysed
with:

```bash
python3 tools/replay_timing_frontier.py \
  results/<run>/longitudinal.log \
  --planner-time 3.808 \
  --planner-time 3.976
```

The tool verifies logged timing flags against selector inequalities, derives the
scenario-specific analytical fail-closed boundary and reports the admissible set
at requested counterfactual planner durations.

See [`timing_frontiers.md`](timing_frontiers.md).

## Deterministic scientific outputs

Given the same scientific source/configuration and the same relevant timing
inputs, the deterministic contract covers quantities such as:

- generated event schedule;
- complete-plan scientific payload identities;
- selected event lead/grasp/route;
- objective values;
- timing-admissible set at the specified selector time;
- committed winner fingerprint.

For the asynchronous near-ground historical comparison, raw records differ by a
`sourceIndex` field and 54 additional payloads are enumerated; see
[`performance.md`](performance.md) and
[`corrections_of_record.md`](corrections_of_record.md). Do not call the full raw
sets byte-identical.

## Machine-dependent outputs

The following are not cross-machine exact outputs:

- external wall-clock planning duration;
- operating-system scheduling;
- wall-time distribution of control cycles;
- GUI/logging timing;
- physical sensor behavior;
- hardware interaction timing.

Performance claims must state the timing metric, measurement window and runtime
context separately from deterministic scientific outputs. The published
`<2 ms` asynchronous observation applies only to the specific in-planning
`ControllerRun` samples documented in [`performance.md`](performance.md), not to
whole-run controller/global maxima and not as WCET.

## Evidence rules

A reported result should identify:

1. source commit/tag or record-specific run state;
2. scenario and condition;
3. campaign/dataset;
4. timing metric, when timing is discussed;
5. simulation versus physical hardware;
6. checker/evidence source;
7. any attribution limitation documented in [`provenance.md`](provenance.md),
   [`experiments.md`](experiments.md) or
   [`corrections_of_record.md`](corrections_of_record.md).

## Release checklist

Before tagging a paper-associated release:

- publication branch/commit identified and working tree clean;
- dependency-free suite passes;
- local Markdown links pass;
- frozen scientific-baseline manifest verifies;
- `f56add3` source-sync manifest verifies;
- evidence manifest and evidence consistency checker pass;
- fresh configure/build succeeds in the declared environment;
- robot-module reconstruction tests pass;
- any new runtime campaign is labelled with its actual source/config/runtime
  provenance rather than inferred from historical revalidation;
- timing-frontier replay tests pass;
- README, mathematics, architecture, provenance, performance, evidence and
  validation documents use consistent terminology;
- simulation/hardware scope remains explicit;
- no hard-real-time, continuous-space globality, feasible-space coverage,
  physical-human validation or full copied-state-purity claim is introduced.
