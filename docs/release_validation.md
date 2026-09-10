# Validation scope

The current publication contains the frozen asynchronous controller and a
reduced evidence package. This page identifies what each validation layer
establishes.

## Current publication

| Layer | Scope |
| --- | --- |
| Source identity | the current source manifest pins the frozen asynchronous implementation |
| Selector/checker tests | finite selection, timing replay, scenario identity, overrides and model-setup fixtures |
| Evidence integrity | asynchronous, latency and planner-core records |
| Documentation | local links, publication-claim guards and script syntax |
| Controller build | clean configure/build recorded during the asynchronous publication pass |
| Current desktop demonstration | no new publication-head runtime or viewer campaign is asserted |

For the supervisor package, model reconstruction was checked from the pinned
upstream robot tags. The URDF and mesh contents passed the published checks.
Reconstruction does not itself establish runtime behavior.

The published latency and controller-timing figures are generated from included
records. The current source manifest refers to scientific commit `f56add3`.

## Historical exact-serial runtime revalidation

At publication synchronization `123be4a`, the imported exact-serial
implementation from `82e6eaa` passed a clean build and four Dataset-B
simulation reruns. Its public release is preserved by `csi-2026-release`.
This is historical exact-serial runtime revalidation,
**not current asynchronous-head runtime validation**.

Each rerun reported:

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

## Interpretation

Source checks, model reconstruction, controller builds, simulator execution and
physical handovers are different validation layers. No cross-machine exact
wall-time, WCET, human-subject or end-to-end physical-robot validation is
established by these checks.

A reproduced run should preserve its actual checkout, environment, inputs and
checker outputs rather than inheriting attribution from an earlier campaign.
