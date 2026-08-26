#!/usr/bin/env python3
"""Dependency-free regression tests for scripts/setup_gen3_2f85_module.py's
safety and validation logic: path-safety (both containment directions),
mesh content verification, and the no-premature-delete guarantee. Uses only
small synthetic fixtures in temporary directories -- no real ros2_kortex/
robotiq_description installation required.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import setup_gen3_2f85_module as m  # noqa: E402

SETUP_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "setup_gen3_2f85_module.py"


def _sha256_bytes(data: bytes) -> str:
    import hashlib

    return hashlib.sha256(data).hexdigest()


def test_path_safety() -> None:
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        repo_root = root / "repo"
        script_dir = repo_root / "scripts"
        script_dir.mkdir(parents=True)
        kortex_share = root / "opt" / "kortex_description"
        (kortex_share / "arms").mkdir(parents=True)
        robotiq_share = root / "opt" / "robotiq_description"
        (robotiq_share / "meshes").mkdir(parents=True)
        # Pinned upstream layout: kortex_description/robots/gen3_2f85.urdf.
        upstream_package_root_dir = root / "src" / "ros2_kortex" / "kortex_description"
        upstream_dir = upstream_package_root_dir / "robots"
        upstream_dir.mkdir(parents=True)
        upstream_urdf = upstream_dir / "gen3_2f85.urdf"
        upstream_urdf.write_text("<robot/>")
        (repo_root / "docs").mkdir()
        allowed = repo_root / "gen3_2f85_module"

        upstream_package_root = m.resolve_upstream_package_root(upstream_urdf)
        assert upstream_package_root == upstream_package_root_dir

        def refuse(output: Path) -> bool:
            try:
                m.refuse_if_unsafe_output(
                    output,
                    repo_root=repo_root,
                    script_dir=script_dir,
                    upstream_urdf=upstream_urdf,
                    upstream_package_root=upstream_package_root,
                    kortex_share=kortex_share,
                    robotiq_share=robotiq_share,
                )
                return False
            except SystemExit:
                return True

        # Direction A: output equals or CONTAINS a protected path.
        assert refuse(Path("/"))
        assert refuse(Path.home())
        assert refuse(repo_root)
        assert refuse(repo_root.parent)
        assert refuse(kortex_share)
        assert refuse(kortex_share.parent)
        assert refuse(robotiq_share)
        assert refuse(robotiq_share.parent)
        assert refuse(upstream_urdf.parent)
        assert refuse(upstream_urdf.parent.parent)

        # Direction B: output is INSIDE a protected source tree, or inside
        # the repository anywhere other than the one allowed subdir.
        assert refuse(repo_root / "docs")
        assert refuse(repo_root / "scripts" / "tmp")
        assert refuse(kortex_share / "some" / "subdirectory")
        assert refuse(robotiq_share / "some" / "subdirectory")
        assert refuse(upstream_urdf.parent / "tmp")
        # The full upstream kortex_description package source tree must be
        # protected, not merely its robots/ subdirectory -- this is the
        # exact case independently reproduced as unsafe: a sibling of
        # robots/ (e.g. arms/, grippers/, package.xml) was previously
        # reachable via a --output pointed one level above robots/.
        assert refuse(upstream_package_root / "tmp")
        assert refuse(upstream_dir / "tmp")

        # Allowed.
        assert not refuse(root / "tmp_call_test" / "gen3_2f85_module")
        assert not refuse(allowed)
        assert not refuse(allowed / "descendant_ok")

    print("  path-safety: PASS")


def _tiny_root(mesh_uris: list[str]) -> ET.Element:
    root = ET.Element("robot", {"name": "gen3_"})
    link = ET.SubElement(root, "link", {"name": "gen3_base_link"})
    visual = ET.SubElement(link, "visual")
    geometry = ET.SubElement(visual, "geometry")
    for uri in mesh_uris:
        ET.SubElement(geometry, "mesh", {"filename": uri})
    return root


def test_mesh_verification() -> None:
    with tempfile.TemporaryDirectory() as td:
        root_dir = Path(td)
        kortex_share = root_dir / "kortex_description"
        robotiq_share = root_dir / "robotiq_description"
        (kortex_share / "arms").mkdir(parents=True)
        (robotiq_share / "meshes").mkdir(parents=True)

        good_content = b"real mesh content, not empty"
        good_hash = _sha256_bytes(good_content)
        (kortex_share / "arms" / "base_link.dae").write_bytes(good_content)

        manifest = {"kortex_description/arms/base_link.dae": good_hash}
        uri = "file:///root/build/kortex_description/arms/base_link.dae"
        tree_root = _tiny_root([uri])

        # PASS: real validated mesh content.
        resolved, problems = m.verify_meshes(tree_root, manifest, kortex_share, robotiq_share)
        assert problems == [], problems
        assert resolved["kortex_description/arms/base_link.dae"] == kortex_share / "arms" / "base_link.dae"

        # FAIL: one modified mesh (content differs from the pinned hash).
        (kortex_share / "arms" / "base_link.dae").write_bytes(b"tampered content")
        resolved, problems = m.verify_meshes(tree_root, manifest, kortex_share, robotiq_share)
        assert resolved == {}
        assert any("hash mismatch" in p for p in problems), problems

        # FAIL: zero-byte fake mesh with the correct filename.
        (kortex_share / "arms" / "base_link.dae").write_bytes(b"")
        resolved, problems = m.verify_meshes(tree_root, manifest, kortex_share, robotiq_share)
        assert resolved == {}
        assert any("hash mismatch" in p for p in problems), problems

        # FAIL: missing mesh file entirely.
        (kortex_share / "arms" / "base_link.dae").unlink()
        resolved, problems = m.verify_meshes(tree_root, manifest, kortex_share, robotiq_share)
        assert resolved == {}
        assert any("missing mesh file" in p for p in problems), problems

        # FAIL: URDF references a mesh not present in the manifest at all.
        (kortex_share / "arms" / "base_link.dae").write_bytes(good_content)
        extra_uri = "file:///root/build/kortex_description/arms/extra_unexpected.dae"
        tree_root_extra = _tiny_root([uri, extra_uri])
        resolved, problems = m.verify_meshes(tree_root_extra, manifest, kortex_share, robotiq_share)
        assert resolved == {}
        assert any("not in the pinned manifest" in p for p in problems), problems

    print("  mesh-content verification: PASS")


def test_no_premature_delete() -> None:
    with tempfile.TemporaryDirectory() as td:
        root_dir = Path(td)
        kortex_share = root_dir / "kortex_description"
        robotiq_share = root_dir / "robotiq_description"
        (kortex_share / "arms").mkdir(parents=True)
        (robotiq_share / "meshes").mkdir(parents=True)
        # A bad (wrong-hash) upstream URDF -- guaranteed to fail validation
        # immediately, before any mesh or output-directory logic runs.
        bad_upstream_urdf = root_dir / "gen3_2f85.urdf"
        bad_upstream_urdf.write_text("<robot name='not_the_real_one'/>")

        output_dir = root_dir / "existing_output"
        output_dir.mkdir()
        sentinel = output_dir / "sentinel.txt"
        sentinel.write_text("SENTINEL-DO-NOT-DELETE")

        result = subprocess.run(
            [
                sys.executable,
                str(SETUP_SCRIPT),
                "--upstream-urdf",
                str(bad_upstream_urdf),
                "--kortex-share",
                str(kortex_share),
                "--robotiq-share",
                str(robotiq_share),
                "--output",
                str(output_dir),
                "--force",
            ],
            capture_output=True,
            text=True,
        )
        assert result.returncode != 0
        assert sentinel.is_file(), "existing --output was deleted despite a failed --force invocation"
        assert sentinel.read_text() == "SENTINEL-DO-NOT-DELETE"

    print("  no-premature-delete (sentinel survives failed --force): PASS")


def main() -> int:
    test_path_safety()
    test_mesh_verification()
    test_no_premature_delete()
    print("setup_gen3_2f85_module.py safety/validation tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
