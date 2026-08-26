#!/usr/bin/env python3
"""Derive the exact set of install destination paths a CMake build
directory will write to, by parsing its generated cmake_install.cmake
files -- without running 'cmake --install'.

cmake_install.cmake is generated at configure time (before any build or
install step), so this gives the true destination paths from a read-only
source, before anything is installed.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

INSTALL_RE = re.compile(
    r'file\(INSTALL\s+DESTINATION\s+"([^"]+)"\s+TYPE\s+(\S+)\s+FILES\s+((?:"[^"]+"\s*)+)\)',
    re.MULTILINE,
)
FILES_ITEM_RE = re.compile(r'"([^"]+)"')
INCLUDE_RE = re.compile(r'include\(\s*"([^"]+)"\s*\)')


def parse_cmake_install(path: Path, seen: set[Path]) -> list[tuple[str, str]]:
    """Returns a list of (destination_path, install_type)."""
    if path in seen or not path.is_file():
        return []
    seen.add(path)
    text = path.read_text(errors="replace")
    results: list[tuple[str, str]] = []
    for match in INSTALL_RE.finditer(text):
        dest_dir, type_, files_blob = match.groups()
        for file_match in FILES_ITEM_RE.finditer(files_blob):
            src = file_match.group(1)
            dest_path = f"{dest_dir.rstrip('/')}/{Path(src).name}"
            results.append((dest_path, type_))
    for inc in INCLUDE_RE.finditer(text):
        results.extend(parse_cmake_install(Path(inc.group(1)), seen))
    return results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("build_dir", type=Path)
    args = parser.parse_args()

    top = args.build_dir / "cmake_install.cmake"
    if not top.is_file():
        print(f"no cmake_install.cmake found under {args.build_dir}", file=sys.stderr)
        return 1

    entries = parse_cmake_install(top, set())

    directory_types = sorted({d for d, t in entries if t == "DIRECTORY"})
    if directory_types:
        print(
            f"ERROR: {len(directory_types)} DIRECTORY-type install entry/entries "
            "found; this derivation only resolves individual files and does not "
            "expand directory installs, so the derived path set would be "
            "incomplete: " + ", ".join(directory_types),
            file=sys.stderr,
        )
        return 1

    seen_paths: set[str] = set()
    for dest_path, install_type in entries:
        if dest_path not in seen_paths:
            seen_paths.add(dest_path)
            print(dest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
