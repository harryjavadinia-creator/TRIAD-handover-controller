# Validation scope

This page distinguishes the validation layers associated with the documented
TRIAD source states and evidence.

## Frozen background-planning source

| Layer | Scope |
| --- | --- |
| Source identity | `source_sync_triad-2026-09-24.sha256` pins the controller sources; `source_sync_f56add3.sha256` pins the frozen V1 implementation, verified at commit `ebaedfa` by `tools/verify_frozen_v1_preserved.sh` |
| Selector/checker tests | finite selection, timing replay, scenario identity, overrides and model-setup fixtures |
| Evidence integrity | latency, planner-core, and archived background-planning records |
| Documentation | local links, claim guards and script syntax |
| Controller build | clean configure/build recorded for the frozen background-planning source |
| Runtime scope | no new four-scenario runtime rerun at `f56add3` is attributed without an explicit record |

Robot-model reconstruction was checked from the pinned upstream robot tags. The
URDF and mesh contents passed the documented checks. Reconstruction does not by
itself establish controller runtime behavior.

The latency figure is generated from included records. The current source
manifest refers to scientific source state `f56add3`.

## Historical exact-serial runtime revalidation

For the exact-serial source imported from `82e6eaa`, validation state `123be4a`
passed a clean build and four Dataset-B simulation reruns. The corresponding
source is preserved by tag `csi-2026-release`.

This evidence is **historical exact-serial runtime revalidation**; it is not a
runtime rerun of the later `f56add3` background-planning source.

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

A reproduced run should retain its actual checkout, environment, inputs and
checker outputs rather than inheriting attribution from another source state.
