# Publication release validation

This page records the validation performed for the curated TRIAD publication branch after synchronizing the audited exact-serial implementation.

## Validated source state

- Publication synchronization date: **2026-09-02**
- Publication synchronization commit: `123be4a`
- Imported implementation source: development commit `82e6eaa`
- Byte-identity manifest: [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256)
- Frozen Dataset-B provenance remains anchored to the `scientific-baseline` tag and `SCIENTIFIC_BASELINE.sha256`.

Only the exact implementation delta was synchronized from `82e6eaa`:

- `src/HandoverInterceptionController.cpp`
- `src/HandoverInterceptionController.h`
- `tools/check_collision_hierarchy_oracle.sh`

## Source and build checks

The synchronized publication worktree passed:

| Check | Result |
| --- | --- |
| finite-plan selector tests | PASS |
| finite event-time selector tests | PASS |
| binding-cost source contract | PASS |
| binding-cost log-checker tests | PASS |
| global time-plan log-checker tests | PASS |
| robot-module reconstruction tests | PASS |
| latency-matrix verifier tests | PASS |
| scenario-identity verifier tests | PASS |
| scenario-override tests | PASS |
| frozen scientific baseline | 35/35 blobs verified |
| `git diff --check` | PASS |
| clean configure/build | PASS, 100% |

GitHub Actions `source-checks` also passed at publication synchronization commit `123be4a`.

## Four-scenario Dataset-B revalidation

All four scenarios were rerun after source synchronization. Each run satisfied:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

The deterministic winners were:

| Scenario | Event lead (s) | Grasp | Route | `J_global` |
| --- | ---: | --- | --- | ---: |
| near-ground / GROUND_NEAR | 4.600 | `axisP_side_45deg` | `ring80mm_0of8` | 0.822892544 |
| longitudinal / PURE_X | 3.700 | `axisP_side_337deg` | `direct` | 0.686806299 |
| lateral-low / CANONICAL_YZ | 4.150 | `axisN_side_337deg` | `direct` | 0.700830630 |
| diagonal / DIAGONAL_XZ | 4.600 | `axisP_side_337deg` | `ring140mm_2of8` | 0.684634405 |

These match the frozen scientific winner fingerprints.

## Scope of this validation

The validation establishes source synchronization, buildability, deterministic simulation selection and runtime-checker consistency for the publication state.

It does not establish end-to-end physical-robot validation. Hardware status and remaining calibration/safety gaps are documented in [`real_robot.md`](real_robot.md).

Generated run directories are intentionally excluded by `.gitignore`; the repository records the reproducible commands, expected deterministic fingerprints, source manifests and checker implementations rather than committing machine-specific runtime logs.
