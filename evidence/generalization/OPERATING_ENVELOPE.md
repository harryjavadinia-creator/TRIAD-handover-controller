# TRIAD operating envelope (derived from the frozen implementation)

Frozen HEAD: 90549ca133f7f88556105fbb4a838ec4cc38c6ed
Sources: etc/HandoverInterceptionController.in.yaml, src/HandoverInterceptionController.{h,cpp},
scripts/reproduce_latency_matrix.sh (development scenario definitions).

## A. Assumptions encoded by TRIAD (implementation-derived)

Timing / event horizon
- presentation lead bank bounded to [minimumPresentationLead 1.8 s,
  maximumPresentationLead 8.0 s], initial lead 3.25 s, step 0.45 s
- at most maximumEventHypotheses = 15 hypotheses (campaign realises 14)
- search wall-time budget maximumEventSearchWallTime = 7.0 s
- commit requires minimumCommitRemainingTime = 1.6 s and
  minimumReachEntryLead = 0.050 s
- scenario-specific fail-closed frontiers established earlier:
  near-ground 3.900000 s, longitudinal 3.975000 s, lateral-low 5.139608 s,
  diagonal 5.735285 s

Prediction model
- constant-twist extrapolation with a scheduled deceleration of
  presentationDecelerationDuration = 0.85 s ending at the presentation event
- velocity estimated with velocityFilterTimeConstant = 0.10 s
- commit freshness guard: maximumObjectTranslationDeviation = 0.015 m,
  maximumObjectRotationDeviation = 0.12 rad
- execution guard: the object must stay inside the committed prediction tube
  (observed rejection at positionError ~0.015 m), with no replanning

Environment / geometry
- ground plane enabled at groundZ = 0.0 with armGroundSafetyMargin = 0.010 m,
  so object height must leave gripper and arm clearance above the plane
- object modelled as a cylinder: handleRadius 0.01125 m,
  handleHalfLength 0.0687 m; sensor body radius 0.017 m, half-length 0.0182 m
- handle offset O_T_H = [0, 0, -0.0869] along the object's local z

Banks
- grasp bank: candidateCount = 16 ring directions x 2 axis signs = 32 candidates
- route bank: 17 routes = 1 direct + 8 directions x 2 radii (80 mm, 140 mm)
- both banks are fixed; no scenario-specific entries exist

Orientation
- object orientation is a genuine configuration input (object.rpy and
  robots.call_object.init_pos.rotation) and the grasp bank is constructed
  generically from the object axis, so orientation is supported in principle.
  Every development scenario uses rpy = [0, 1.5708, 0] and no benchmark varies
  it, so the implementation is only *exercised* at that orientation. This
  campaign therefore holds orientation fixed at the development value and does
  not claim orientation generalization.

## B. Ranges merely exercised by the four development scenarios

- positions: x in [0.25, 0.92], y in [-0.56, 0.62], z in [0.15, 0.55]
- speed: exactly 0.08 m/s in every moving development scenario
- directions: +y, -y, -x, and one xz diagonal
- orientation: single value [0, 1.5708, 0]
- sensing: ideal, except the Dataset-A latency matrix

The held-out set stays inside the exercised position box, holds orientation at
the development value, and varies speed over {0.05, 0.08, 0.11} m/s and
direction over six families including two- and three-axis motion. Speeds above
0.11 m/s and orientations other than the development value are outside what the
implementation has been exercised at and are therefore not claimed.
