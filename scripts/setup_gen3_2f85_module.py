#!/usr/bin/env python3
"""Deterministically build a public gen3_2f85 mc_rtc robot-module directory
from an already-obtained, pinned upstream ros2_kortex checkout, without
redistributing or embedding any upstream file content in this repository.

Input (all must already exist locally; this script never downloads
anything):

  --upstream-urdf   path to kortex_description/robots/gen3_2f85.urdf from a
                     ros2_kortex 0.2.6 checkout (see docs/robot_module.md
                     for how to obtain it)
  --kortex-share    path to the kortex_description package's installed or
                     source share directory (containing arms/gen3/7dof/
                     meshes/*.dae)
  --robotiq-share   path to the robotiq_description package's installed
                     share directory (containing meshes/{visual,collision}/*)
  --output          destination module directory (created fresh; refuses to
                     overwrite an existing directory unless --force)

The mesh content manifest is always the one pinned scientific manifest
shipped with this repository (configs/gen3_2f85_meshes.sha256) -- there is
no CLI override for it, so a result can never be produced against an
alternate/custom mesh-hash set while still being described as the pinned
TRIAD robot-model reconstruction.

Verification performed before any transformation, and before --output is
touched in any way (a failed run leaves an existing --output untouched):
  - --upstream-urdf must be named gen3_2f85.urdf, inside a directory named
    robots, inside a directory named kortex_description (the pinned
    ros2_kortex layout) -- this also derives the full upstream package
    source tree that --output must never equal or lie inside;
  - the upstream URDF's SHA-256 must match the pinned hash for the official
    ros2_kortex 0.2.6 tag's kortex_description/robots/gen3_2f85.urdf (see
    docs/robot_module.md for how this hash was established and
    independently cross-checked against a fresh copy of that tag);
  - exactly the 12 verified wrist-camera links / 12 joints and exactly 1
    <ros2_control> subtree must be present to remove;
  - every mesh referenced by the URDF must match the pinned
    configs/gen3_2f85_meshes.sha256 manifest exactly -- no missing file, no
    unrecognized/extra reference, no content mismatch (a filename matching
    but content differing, e.g. a zero-byte placeholder, is rejected).

Transformations applied (see docs/robot_module.md for why these three, and
only these three, are correct):
  1. remove the top-level <ros2_control> subtree
  2. remove the 12 wrist-camera links and 12 wrist-camera joints verified
     absent from the frozen TRIAD robot module
  3. rewrite every mesh <mesh filename="..."> URI to the researcher's own
     resolved --kortex-share / --robotiq-share paths

No mesh files are copied into this repository or referenced by content;
only their existence and SHA-256 at the resolved path is verified. The
complete candidate is built in a temporary sibling directory and only
swapped into --output after every check above has succeeded.
"""
from __future__ import annotations

import argparse
import hashlib
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

EXPECTED_UPSTREAM_URDF_SHA256 = (
    "9fc90e6afb795644d568467f91fae45f37a27eab8c0bdcb2906eea05f981163d"
)

CAMERA_LINK_NAMES = {
    "wrist_camera_link",
    "wrist_camera_mount_link",
    "wrist_mounted_camera_bottom_screw_frame",
    "wrist_mounted_camera_color_frame",
    "wrist_mounted_camera_color_optical_frame",
    "wrist_mounted_camera_depth_frame",
    "wrist_mounted_camera_depth_optical_frame",
    "wrist_mounted_camera_infra1_frame",
    "wrist_mounted_camera_infra1_optical_frame",
    "wrist_mounted_camera_infra2_frame",
    "wrist_mounted_camera_infra2_optical_frame",
    "wrist_mounted_camera_link",
}
CAMERA_JOINT_NAMES = {
    "wrist_camera_joint",
    "wrist_camera_mount_joint",
    "wrist_mounted_camera_color_joint",
    "wrist_mounted_camera_color_optical_joint",
    "wrist_mounted_camera_depth_joint",
    "wrist_mounted_camera_depth_optical_joint",
    "wrist_mounted_camera_infra1_joint",
    "wrist_mounted_camera_infra1_optical_joint",
    "wrist_mounted_camera_infra2_joint",
    "wrist_mounted_camera_infra2_optical_joint",
    "wrist_mounted_camera_joint",
    "wrist_mounted_camera_link_joint",
}


def sha256_of(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _is_equal_or_ancestor(candidate: Path, of: Path) -> bool:
    """True if `candidate` equals `of`, or `candidate` is an ancestor
    directory of `of` (i.e. `of` lies inside `candidate`). Uses pathlib
    containment (`Path.parents`), never string-prefix comparison. Both
    arguments must already be resolved absolute paths."""
    return candidate == of or candidate in of.parents


def resolve_upstream_package_root(upstream_urdf: Path) -> Path:
    """Derive the kortex_description package root from the pinned expected
    layout (kortex_description/robots/gen3_2f85.urdf), rather than merely
    taking the URDF's immediate parent directory -- the immediate parent is
    only the 'robots/' subdirectory, which would leave the rest of the
    package source tree (e.g. arms/, grippers/, package.xml) unprotected
    from an --output pointed at a sibling of 'robots/'."""
    resolved = upstream_urdf.resolve()
    if resolved.name != "gen3_2f85.urdf":
        raise SystemExit(
            f"ERROR: --upstream-urdf must be named gen3_2f85.urdf, got: {resolved.name}"
        )
    if resolved.parent.name != "robots":
        raise SystemExit(
            f"ERROR: --upstream-urdf must be inside a 'robots' directory "
            f"(kortex_description/robots/gen3_2f85.urdf), got parent: {resolved.parent}"
        )
    package_root = resolved.parent.parent
    if package_root.name != "kortex_description":
        raise SystemExit(
            f"ERROR: --upstream-urdf's package root must be named kortex_description, "
            f"got: {package_root}"
        )
    return package_root


def refuse_if_unsafe_output(
    output: Path,
    *,
    repo_root: Path,
    script_dir: Path,
    upstream_urdf: Path,
    upstream_package_root: Path,
    kortex_share: Path,
    robotiq_share: Path,
) -> None:
    """Refuse an --output target that --force would shutil.rmtree(). Two
    independent directions are checked:

    A. output must never EQUAL or CONTAIN a protected path (deleting output
       would delete that protected path): /, HOME, CWD, the CALL repository
       root, this script's own source directory, the upstream URDF file,
       --kortex-share, --robotiq-share.

    B. output must never be EQUAL TO or INSIDE a protected source tree: the
       *entire* upstream kortex_description package source tree (not merely
       its robots/ subdirectory), --kortex-share, --robotiq-share, or the
       CALL repository -- except for the one explicitly allowed generated
       root <repo_root>/gen3_2f85_module (or a descendant of it).
    """
    resolved_output = output.resolve()
    resolved_repo_root = repo_root.resolve()
    resolved_script_dir = script_dir.resolve()
    resolved_upstream_urdf = upstream_urdf.resolve()
    resolved_upstream_package_root = upstream_package_root.resolve()
    resolved_kortex_share = kortex_share.resolve()
    resolved_robotiq_share = robotiq_share.resolve()

    # Direction A: output must not equal or contain a protected path.
    equal_or_contains_targets = [
        Path("/"),
        Path.home().resolve(),
        Path.cwd().resolve(),
        resolved_repo_root,
        resolved_script_dir,
        resolved_upstream_urdf,
        resolved_kortex_share,
        resolved_robotiq_share,
    ]
    for protected in equal_or_contains_targets:
        if _is_equal_or_ancestor(resolved_output, protected):
            raise SystemExit(
                f"ERROR: refusing --output {resolved_output} -- it equals or "
                f"would delete the protected path {protected}."
            )

    if len(resolved_output.parts) <= 2:
        raise SystemExit(
            f"ERROR: --output is too close to the filesystem root, refusing: {resolved_output}"
        )

    # Direction B: output must not be equal to or inside a protected source
    # tree -- with one explicit carve-out for the dedicated generated root
    # inside the CALL repository.
    for protected_tree in (resolved_upstream_package_root, resolved_kortex_share, resolved_robotiq_share):
        if _is_equal_or_ancestor(protected_tree, resolved_output):
            raise SystemExit(
                f"ERROR: refusing --output {resolved_output} -- it is inside "
                f"the protected source tree {protected_tree}."
            )

    allowed_repo_subdir = resolved_repo_root / "gen3_2f85_module"
    if _is_equal_or_ancestor(resolved_repo_root, resolved_output):
        if not _is_equal_or_ancestor(allowed_repo_subdir, resolved_output):
            raise SystemExit(
                f"ERROR: refusing --output {resolved_output} -- inside the CALL "
                f"repository, only {allowed_repo_subdir} (or a descendant of it) "
                "is allowed as a generated-output location."
            )


def mesh_relative_path(original_uri: str) -> str:
    """Extract the package-relative path (e.g.
    'kortex_description/arms/gen3/7dof/meshes/base_link.dae') from an
    upstream mesh file:// URI, independent of the researcher's own
    --kortex-share/--robotiq-share locations."""
    if not original_uri.startswith("file://"):
        raise ValueError(f"unrecognized mesh URI scheme (expected file://): {original_uri}")
    path_str = original_uri[len("file://"):]
    for pkg in ("kortex_description", "robotiq_description"):
        marker = f"{pkg}/"
        idx = path_str.rfind(marker)
        if idx != -1:
            return f"{pkg}/{path_str[idx + len(marker):]}"
    raise ValueError(
        f"mesh URI does not contain a recognized kortex_description/ or "
        f"robotiq_description/ package-relative segment, refusing to guess: {original_uri}"
    )


def resolve_mesh_path(relative: str, kortex_share: Path, robotiq_share: Path) -> Path:
    """Map a package-relative mesh path to a path under the researcher's own
    installed package share directories."""
    pkg, sub = relative.split("/", 1)
    base = kortex_share if pkg == "kortex_description" else robotiq_share
    return base / sub


def load_mesh_manifest(manifest_path: Path) -> dict[str, str]:
    """Parse configs/gen3_2f85_meshes.sha256 -> {relative_path: sha256}."""
    manifest: dict[str, str] = {}
    for line in manifest_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        digest, relative = parts
        manifest[relative] = digest
    return manifest


def verify_meshes(
    root: ET.Element, manifest: dict[str, str], kortex_share: Path, robotiq_share: Path
) -> tuple[dict[str, Path], list[str]]:
    """Verify every mesh referenced by the parsed URDF against the pinned
    manifest: exact path-set match (no missing, no unrecognized/extra
    reference), then SHA-256 content match for each. Returns
    (relative -> resolved_local_path, list_of_problem_strings). Performs no
    filesystem writes -- read-only verification only."""
    referenced: set[str] = set()
    problems: list[str] = []
    for mesh in root.iter("mesh"):
        try:
            referenced.add(mesh_relative_path(mesh.get("filename")))
        except ValueError as exc:
            problems.append(str(exc))
    if problems:
        return {}, problems

    expected = set(manifest)
    unrecognized = sorted(referenced - expected)
    missing_from_urdf = sorted(expected - referenced)
    for rel in unrecognized:
        problems.append(f"mesh referenced by the URDF is not in the pinned manifest: {rel}")
    for rel in missing_from_urdf:
        problems.append(f"manifest entry not referenced by the URDF (unexpected drift): {rel}")
    if problems:
        return {}, problems

    resolved_paths: dict[str, Path] = {}
    for rel in sorted(referenced):
        resolved = resolve_mesh_path(rel, kortex_share, robotiq_share)
        if not resolved.is_file():
            problems.append(f"missing mesh file: {resolved} (expected for {rel})")
            continue
        actual = sha256_of(resolved)
        expected_hash = manifest[rel]
        if actual != expected_hash:
            problems.append(
                f"mesh content hash mismatch for {rel} ({resolved}): "
                f"expected {expected_hash}, got {actual}"
            )
            continue
        resolved_paths[rel] = resolved

    return resolved_paths, problems


def build_candidate(
    upstream_urdf: Path, mesh_manifest_path: Path, kortex_share: Path, robotiq_share: Path
) -> ET.Element | None:
    """Perform every validation step and build the final in-memory URDF
    tree. Touches no output path whatsoever -- purely read-only against
    upstream_urdf/mesh_manifest_path/kortex_share/robotiq_share. Returns the
    transformed root element on full success, or None (having printed every
    problem to stderr) on any failure."""
    actual_hash = sha256_of(upstream_urdf)
    if actual_hash != EXPECTED_UPSTREAM_URDF_SHA256:
        print(
            "ERROR: --upstream-urdf does not match the pinned ros2_kortex 0.2.6 "
            "kortex_description/robots/gen3_2f85.urdf hash.\n"
            f"  expected: {EXPECTED_UPSTREAM_URDF_SHA256}\n"
            f"  actual:   {actual_hash}\n"
            "This script only transforms the exact verified upstream artifact; "
            "see docs/robot_module.md.",
            file=sys.stderr,
        )
        return None
    print(f"Verified upstream artifact hash: {actual_hash}")

    root = ET.parse(upstream_urdf).getroot()

    removed_links = 0
    for link in list(root.findall("link")):
        if link.get("name") in CAMERA_LINK_NAMES:
            root.remove(link)
            removed_links += 1
    removed_joints = 0
    for joint in list(root.findall("joint")):
        if joint.get("name") in CAMERA_JOINT_NAMES:
            root.remove(joint)
            removed_joints += 1
    if removed_links != len(CAMERA_LINK_NAMES) or removed_joints != len(CAMERA_JOINT_NAMES):
        print(
            f"ERROR: expected to remove {len(CAMERA_LINK_NAMES)} camera links / "
            f"{len(CAMERA_JOINT_NAMES)} camera joints, actually removed "
            f"{removed_links} links / {removed_joints} joints. The upstream "
            "artifact does not match what this script was verified against.",
            file=sys.stderr,
        )
        return None
    print(f"Removed {removed_links} wrist-camera links, {removed_joints} wrist-camera joints.")

    removed_ros2_control = 0
    for rc in list(root.findall("ros2_control")):
        root.remove(rc)
        removed_ros2_control += 1
    if removed_ros2_control != 1:
        print(
            f"ERROR: expected exactly 1 top-level <ros2_control> subtree, found {removed_ros2_control}.",
            file=sys.stderr,
        )
        return None
    print("Removed the <ros2_control> subtree.")

    if not mesh_manifest_path.is_file():
        print(f"ERROR: mesh manifest not found: {mesh_manifest_path}", file=sys.stderr)
        return None
    manifest = load_mesh_manifest(mesh_manifest_path)
    resolved_paths, problems = verify_meshes(root, manifest, kortex_share, robotiq_share)
    if problems:
        print("ERROR: mesh content verification failed:", file=sys.stderr)
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        return None
    print(f"Verified {len(resolved_paths)} unique mesh file(s) against the pinned manifest (path + SHA-256).")

    rewritten = 0
    for mesh in root.iter("mesh"):
        relative = mesh_relative_path(mesh.get("filename"))
        mesh.set("filename", f"file://{resolved_paths[relative]}")
        rewritten += 1
    print(f"Rewrote {rewritten} mesh URI reference(s).")

    return root


def write_module(root: ET.Element, module_dir: Path) -> None:
    """Write the final module layout into module_dir, which must not
    already exist."""
    urdf_dir = module_dir / "urdf"
    urdf_dir.mkdir(parents=True)
    (module_dir / "convex" / "gen3_2f85").mkdir(parents=True)
    (module_dir / "rsdf" / "gen3_2f85").mkdir(parents=True)
    ET.ElementTree(root).write(urdf_dir / "gen3_2f85.urdf", encoding="unicode", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--upstream-urdf", required=True, type=Path,
                         help="path to kortex_description/robots/gen3_2f85.urdf (ros2_kortex 0.2.6)")
    parser.add_argument("--kortex-share", required=True, type=Path,
                         help="kortex_description package share directory")
    parser.add_argument("--robotiq-share", required=True, type=Path,
                         help="robotiq_description package share directory")
    parser.add_argument("--output", required=True, type=Path,
                         help="destination gen3_2f85 mc_rtc module directory")
    parser.add_argument("--force", action="store_true",
                         help="overwrite --output if it already exists")
    args = parser.parse_args()

    if not args.upstream_urdf.is_file():
        print(f"ERROR: --upstream-urdf not found: {args.upstream_urdf}", file=sys.stderr)
        return 1
    if not args.kortex_share.is_dir():
        print(f"ERROR: --kortex-share is not a directory: {args.kortex_share}", file=sys.stderr)
        return 1
    if not args.robotiq_share.is_dir():
        print(f"ERROR: --robotiq-share is not a directory: {args.robotiq_share}", file=sys.stderr)
        return 1

    upstream_package_root = resolve_upstream_package_root(args.upstream_urdf)

    refuse_if_unsafe_output(
        args.output,
        repo_root=REPO_ROOT,
        script_dir=SCRIPT_DIR,
        upstream_urdf=args.upstream_urdf,
        upstream_package_root=upstream_package_root,
        kortex_share=args.kortex_share,
        robotiq_share=args.robotiq_share,
    )

    # Every validation step happens here, entirely in memory, before
    # --output is touched in any way. An existing --output is guaranteed to
    # survive untouched if this fails. The mesh manifest is always the one
    # pinned scientific manifest shipped with this repository -- there is no
    # CLI override, so a result can never be produced against, and described
    # as, an alternate/custom mesh-hash set.
    mesh_manifest_path = REPO_ROOT / "configs" / "gen3_2f85_meshes.sha256"
    root = build_candidate(args.upstream_urdf, mesh_manifest_path, args.kortex_share, args.robotiq_share)
    if root is None:
        return 1

    if args.output.exists() and not args.force:
        print(f"ERROR: --output already exists: {args.output} (use --force to overwrite)", file=sys.stderr)
        return 1

    # Build the complete candidate in a temporary sibling directory first,
    # and only replace --output (destructively, if --force was given and it
    # already existed) after that build has fully succeeded.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tmp_dir = Path(tempfile.mkdtemp(prefix=f".{args.output.name}.tmp-", dir=args.output.parent))
    tmp_module_dir = tmp_dir / args.output.name
    try:
        write_module(root, tmp_module_dir)
        if args.output.exists():
            shutil.rmtree(args.output)
        shutil.move(str(tmp_module_dir), str(args.output))
    finally:
        if tmp_dir.exists():
            shutil.rmtree(tmp_dir, ignore_errors=True)

    print(f"Wrote {args.output / 'urdf' / 'gen3_2f85.urdf'}")
    print()
    print(f"export MAIN_ROBOT_MODULE_PATH={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
