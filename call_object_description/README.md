# Handover object description

This directory contains the simple rigid-body object model used by **TRIAD** within the CALL research project.

The runtime identifier remains `call_object` for compatibility with the validated controller configuration. The identifier is an implementation name; it is not the TRIAD method name.

## Frame convention

The root/object frame is located at the center of the central SENSIX sensor body:

- local `+Z`: human-side grey handle
- local `-Z`: robot-side blue receiver handle

The model is symmetric about the sensor center.

## Geometry encoded in the URDF

| Part | Radius (m) | Length (m) | Center along object `Z` (m) |
| --- | ---: | ---: | ---: |
| Central sensor body | 0.01700 | 0.0364 | 0.0000 |
| Human-side handle | 0.01125 | 0.1374 | +0.0869 |
| Robot-side receiver handle | 0.01125 | 0.1374 | -0.0869 |

The handle centers are positioned so that each handle begins exactly at the corresponding end face of the central sensor body. The resulting modeled end-to-end length is **0.3112 m**.

Visual and collision geometry use the same cylinder primitives; no external mesh or convex-hull asset is required.

## Inertial model

The URDF assigns the complete rigid assembly to the root link with:

- mass: `0.346 kg`
- inertia about the root/object frame:
  - `Ixx = Iyy = 0.0011939025 kg m^2`
  - `Izz = 0.0000397634 kg m^2`
  - products of inertia: `0`

The two fixed handle links intentionally carry no separate inertial entries; the values above represent the complete modeled assembly.

**Provenance note:** the repository records these as the physical/model parameters used by the controller, but it does not currently contain a source record establishing whether the mass and inertia tensor were obtained by direct measurement, CAD, or estimation. Treat them as the current simulation-model values unless separate calibration/CAD evidence is available.

## Sensor-model scope

The central cylinder represents the geometry of the SENSIX sensor body used in the handover object. This URDF does **not** define a force/torque measurement interface or claim that a physical SENSIX signal is connected to the controller. Sensor/force integration is a separate runtime concern.

## Use inside this repository

The controller CMake build installs this directory automatically under its runtime namespace, and the generated controller configuration references the installed path. No manual copy is required when building the full TRIAD controller.

## Standalone mc_rtc use

The directory can also be shared and loaded independently with mc_rtc's `object` robot module. Point the module to the directory containing this README and the `urdf/` folder:

```yaml
robots:
  call_object:
    module:
      - object
      - /absolute/path/to/call_object_description
      - call_object
```

The corresponding URDF is `urdf/call_object.urdf`.
