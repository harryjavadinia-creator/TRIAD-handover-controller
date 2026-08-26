# The `gen3_2f85` robot module

TRIAD's simulation reproduction depends on an external mc_rtc robot-module
description (URDF, meshes) for the Kinova Gen3 (7-DOF) arm with a Robotiq
2F-85 gripper, named `gen3_2f85`. The same robot geometry is referenced by
TRIAD's hardware configuration, but real-robot use is not validated — see
`docs/real_robot.md`, which remains the authority on hardware limitations.
This module is not part of TRIAD's scientific source and is not redistributed
by this repository. This page documents its verified provenance and how to
reconstruct it deterministically from pinned upstream packages.

## Upstream source

| Component | Upstream project | Version | License |
| --- | --- | --- | --- |
| Arm description | [Kinovarobotics/ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex), package `kortex_description` | `0.2.6` (Git tag `0.2.6`) | BSD 3-Clause |
| Gripper description | `robotiq_description` (from PickNik's `ros2_robotiq_gripper`) | `0.0.1` | BSD 3-Clause |

The reproducibility anchor is the file already shipped inside the pinned
`kortex_description` `0.2.6` tag itself:

```
kortex_description/robots/gen3_2f85.urdf
```

SHA-256 of that exact file, at that exact tag:

```
9fc90e6afb795644d568467f91fae45f37a27eab8c0bdcb2906eea05f981163d
```

This hash was computed from a local `ros2_kortex` checkout and then
independently cross-checked by fetching
`kortex_description/robots/gen3_2f85.urdf` directly from the `0.2.6` tag of
`Kinovarobotics/ros2_kortex` on GitHub and re-hashing that separately
obtained copy — both hashes matched exactly.

**Use this shipped file directly, not a regeneration from a live/newer
xacro checkout.** A currently-checked-out `ros2_kortex` working tree can
drift from the exact `0.2.6` release (this was directly observed: the arm's
`gen3_macro.xacro` in one available checkout defined different joint-2/4/6
limits than the `0.2.6`-tagged `gen3_2f85.urdf` does). The tagged, shipped
URDF is the artifact that was verified, term for term, to contain the exact
arm/gripper kinematic model and joint limits used by TRIAD's frozen scientific
module — re-running xacro against whatever `kortex_description` checkout
happens to be on hand is not guaranteed to reproduce it.

## The three deterministic portability transformations

Starting from the pinned, hash-verified `gen3_2f85.urdf` above, exactly three
mechanical transformations are applied, and nothing else:

1. **Remove the top-level `<ros2_control>` subtree.** This block configures
   the `kortex2_driver` ROS 2 hardware interface (arm IP, port, credentials,
   joint command/state interfaces). mc_rtc's URDF loading path does not read
   `<ros2_control>` at all — it is a ROS 2-specific extension tag, inert to
   mc_rtc/RBDyn's standard URDF parsing. Removing it has no effect on
   anything mc_rtc simulates.
2. **Remove the 12 wrist-camera links and 12 wrist-camera joints** verified
   absent from TRIAD's frozen module (`wrist_camera_link`,
   `wrist_camera_mount_link`, and the ten `wrist_mounted_camera_*` frames/
   optical frames, plus their twelve corresponding joints). The frozen
   module used for the reported TRIAD results has these camera links absent;
   they carry only mass/visual/collision data for a physical camera bracket
   that plays no role in TRIAD's (ground-truth) object perception.
3. **Rewrite mesh `<mesh filename="...">` URIs** from the upstream build
   machine's absolute paths to the researcher's own resolved
   `kortex_description` and `robotiq_description` package share
   directories, using each URI's package-relative path segment (e.g.
   `arms/gen3/7dof/meshes/base_link.dae`), not a hardcoded absolute prefix.

None of these three transformations alters arm or gripper kinematics,
inertials, or collision geometry. `<ros2_control>` and the camera links are
disjoint, wholly-removable subtrees; the mesh rewrite only changes how a
file is located on disk, not its content.

## Equivalence evidence

Both the arm and gripper portions of the reconstructed module were
programmatically compared, link by link and joint by joint, against TRIAD's
frozen local `kinova_gen3_2f85_mcdesc` module:

- Identical **19-link / 18-joint** model (12 camera links/joints correctly
  absent from both).
- Identical joint graph (parent/child), joint types, origins, axes, and
  limits for every one of the 18 remaining joints.
- Identical inertial origins, masses, and inertia matrices for every one of
  the 19 remaining links.
- Identical visual and collision geometry (origins and mesh/primitive
  descriptors) for every link, after mesh URI resolution.
- Identical mesh file content: every resolved mesh file's SHA-256 matches
  between the two module trees (resolved via package-relative path, not
  merely by URI string).
- **Same mc_rtc convex-hull cache identity**: mc_rtc's own content-derived
  cache directory name for the reconstructed module was identical to the one
  it produces for the frozen module — an independent, tool-internal
  confirmation of geometric equivalence, not just this project's own
  comparison.

The reconstructed module was then loaded through mc_rtc using TRIAD's own
`scripts/run_scenario.sh` mechanism and run through the `diagonal` Dataset B
scenario. The result matched the frozen module's own reported result
exactly, with no retuning:

| Quantity | Value |
| --- | --- |
| Selected event lead | 4.600 s |
| Grasp | `axisP_side_337deg` |
| Route | `ring140mm_2of8` |
| J_motion | 0.596089263 |
| J_global | 0.684634405 |
| Complete / cost-valid / timing-admissible plans | 233 / 233 / 233 |

## Mesh content manifest

The joint/link geometry the equivalence evidence above was measured against
depends on the *content* of 26 unique mesh files (34 `<mesh>` references in
the URDF resolve to 26 unique files, since several links reuse one mesh for
both visual and collision geometry) — a file that merely has the right name
is not sufficient, since TRIAD's feasibility/collision result depends on the
actual mesh content. `configs/gen3_2f85_meshes.sha256` pins the package-
relative path and SHA-256 of all 26. Each hash was established twice and
cross-checked: once from the installed package copy used in the equivalence
test, and independently by fetching the same package-relative file fresh
from the official upstream Git tag (`Kinovarobotics/ros2_kortex` tag `0.2.6`
for `kortex_description/...`, `PicknikRobotics/ros2_robotiq_gripper` tag
`0.0.1` for `robotiq_description/...`) — all 26 matched exactly between the
two independently obtained copies.

`scripts/setup_gen3_2f85_module.py` enforces this manifest before rewriting
anything: it verifies the URDF's referenced mesh path set exactly matches
the manifest (failing on any missing, added, or otherwise unrecognized mesh
reference), then verifies the SHA-256 of every one of the 26 resolved files
against the manifest (failing on any content mismatch, including a
correctly-named but empty or substituted file). Only after every mesh
passes both checks are the URI references rewritten. Package metadata
(`kortex_description` `0.2.6`, `robotiq_description` `0.0.1`) is documented
above for provenance, but is not itself the verification mechanism — content
hashes are the binding check, since a version string alone cannot detect a
locally modified or corrupted mesh file.

## Reconstructing the module

```bash
python3 scripts/setup_gen3_2f85_module.py \
  --upstream-urdf /path/to/ros2_kortex/kortex_description/robots/gen3_2f85.urdf \
  --kortex-share /path/to/installed/share/kortex_description \
  --robotiq-share /path/to/installed/share/robotiq_description \
  --output /path/to/gen3_2f85_module
```

Before touching `--output` in any way, the script verifies (in this order):
that `--upstream-urdf` matches the pinned `kortex_description/robots/
gen3_2f85.urdf` layout exactly (filename `gen3_2f85.urdf`, inside a
`robots/` directory, inside a directory named `kortex_description` — this
also derives the *entire* upstream package source tree that `--output` must
never touch, not merely the `robots/` subdirectory); the upstream URDF's
SHA-256 against the pinned value above; that exactly the 12 verified
wrist-camera links/joints and exactly one `<ros2_control>` subtree are
present to remove; and every mesh referenced by the URDF against the one
pinned `configs/gen3_2f85_meshes.sha256` manifest shipped with this
repository — there is no CLI override for the manifest — (path set and
SHA-256, not just `Path.is_file()`). Only once all of that succeeds does it
build the complete module in a temporary sibling directory and swap it into
`--output` — so a failed invocation, even with `--force`, leaves an
existing `--output` untouched. `--output` is also refused if it would
equal, contain, or lie inside the CALL repository (other than the one
dedicated `gen3_2f85_module` generated-output location), the *entire*
upstream `kortex_description` package source tree, or either package share
directory, so `--force` can never delete something it shouldn't. The
script downloads nothing — `kortex_description` and `robotiq_description`
must already be obtained/installed locally (see `README.md`'s quick-start
sequence). See `scripts/setup_gen3_2f85_module.py` for the exact, auditable
logic, and `tools/test_setup_gen3_2f85_module.py` for its regression tests.

## Licensing

`kortex_description` (`0.2.6`) and `robotiq_description` (`0.0.1`) each
declare the **BSD 3-Clause** license. They are external dependencies,
obtained and referenced locally by the researcher — this repository's setup
script does not redistribute their source, meshes, or any derived URDF file.
Upstream licensing does not by itself determine this repository's own
license, which remains a separate, unresolved decision — see the "License"
section of `README.md`.
