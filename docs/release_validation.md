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

One further, deliberate difference exists outside that set.
`src/states/HandoverInterceptionController_SolveInterception.cpp` omits a
`developmentMode=true` token from the `[GlobalTimePlanSearchConfiguration]`
log line that the development tree carries. The difference is a single literal
inside one `mc_rtc::log::success` format string: it changes no placeholder, no
argument, no computation, no selection, and no checker input
(`tools/check_global_time_plan_log.py` requires only `enabled=true`, and the
token appears nowhere else in this repository). It is a publication log-label
choice and is intentionally not reverted, because editing the audited planner
source for cosmetic identity would be a worse trade than documenting the
difference.

The clean build and four-scenario Dataset-B rerun reported below were performed at synchronization commit `123be4a`. Every publication-review commit after it — through `48ba86b` and including the present documentation/test pass — changed only documentation, CI and dependency-free analysis/checking utilities. None modified the synchronized controller source, FSM source, controller configuration, robot/object geometry, scientific baseline manifest or source-sync manifest.

This can be checked directly:

```bash
git diff --name-only 123be4a..HEAD -- src etc configs call_object_description
```

which lists no files.

This distinction is deliberate: the scenario rerun evidence is attributed to the exact commit at which it was generated rather than silently re-labelling historical validation as a later branch-head result.

## Source and build checks at `123be4a`

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

## Expanded publication checks after synchronization

The later repository-wide publication pass added dependency-free checks for:

- analytical timing-frontier replay;
- repository-local Markdown links;
- Python syntax;
- shell-script syntax;
- exact-serial source-sync SHA-256 verification in CI.

Those expanded checks passed at review commit `d492176` and again in the present
pass, which corrected the winner-preservation interval endpoints in
[`timing_frontiers.md`](timing_frontiers.md) and extended the timing-replay
regression tests. No controller, FSM, configuration or geometry source was
touched in any of these commits.

## Four-scenario Dataset-B revalidation at `123be4a`

All four scenarios were rerun after exact source synchronization. Each run satisfied:

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

The validation establishes source synchronization, buildability, deterministic simulation selection and runtime-checker consistency for the synchronized publication implementation.

The later documentation/tools-only publication pass establishes consistency and dependency-free validation of the repository presentation around that implementation; it is not presented as a second four-scenario simulation campaign.

This validation does **not** establish end-to-end physical-robot validation. Hardware status and remaining calibration/safety gaps are documented in [`real_robot.md`](real_robot.md).

Generated run directories are intentionally excluded by `.gitignore`; the repository records the reproducible commands, expected deterministic fingerprints, source manifests and checker implementations rather than committing machine-specific runtime logs.
