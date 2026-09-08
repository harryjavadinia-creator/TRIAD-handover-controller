# Validation scope

The current publication contains the frozen asynchronous controller and
archived evidence from several source states. This page identifies what each
validation layer establishes.

## Current publication

| Layer | Scope |
| --- | --- |
| Source identity | The [current source manifest](source_sync_f56add3.sha256) pins the frozen asynchronous implementation |
| Selector/checker tests | Finite selection, timing replay, scenario identity, overrides, and model-setup fixtures |
| Evidence integrity | Archived records, predeclared input digests, and recomputed headline counts |
| Documentation | Local links, publication-claim guards, and script syntax |
| Controller build | Clean configure/build recorded during the asynchronous publication pass |
| Current desktop demonstration | No new publication-head runtime or viewer campaign is asserted |

For the supervisor publication package, model reconstruction was checked
directly from the two upstream tags in [Quick start](quickstart.md).
The URDF and all 26 unique mesh contents passed the pinned checks.
The reconstruction does not itself run the controller or establish runtime
behavior. The published result figures were regenerated from the included
records and visually inspected.

The current source manifest refers to scientific commit `f56add3`.
The [reproduction commands](reproducibility.md) and
[GitHub Actions workflow](../.github/workflows/source-checks.yml) expose the
checks. A static planner-guard pass is subject to the
[live-read and worker-lifecycle qualifications](architecture.md).

The archived asynchronous timing/determinism records retain their own
[provenance](provenance.md). Compatibility with the frozen source does not
mean every record was executed at that exact source state.

## Historical exact-serial runtime revalidation

At publication synchronization `123be4a`, the imported exact-serial
implementation from `82e6eaa` passed a clean build and four Dataset-B
simulation reruns. Its later public release is preserved by `csi-2026-release`.
This is historical exact-serial runtime revalidation,
not current asynchronous-head runtime validation.

Each of those four reruns reported:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

| Scenario | Event lead (s) | Grasp | Route | Global cost |
| --- | ---: | --- | --- | ---: |
| Near-ground | 4.600 | `axisP_side_45deg` | `ring80mm_0of8` | 0.822892544 |
| Longitudinal | 3.700 | `axisP_side_337deg` | `direct` | 0.686806299 |
| Lateral-low | 4.150 | `axisN_side_337deg` | `direct` | 0.700830630 |
| Diagonal | 4.600 | `axisP_side_337deg` | `ring140mm_2of8` | 0.684634405 |

The historical [source manifest](source_sync_82e6eaa.sha256) and original
`scientific-baseline` tag remain available.

## Interpretation

Source checks, model reconstruction, controller builds, simulator execution,
and physical handovers are different validation layers. No cross-machine
exact wall-time, WCET, human-subject, or end-to-end physical-robot validation is
established by these checks. [Hardware status](real_robot.md) documents the
physical commissioning scope.

A reproduced run should preserve its actual checkout, environment, inputs,
and checker outputs. It should not be inferred from an earlier campaign.
