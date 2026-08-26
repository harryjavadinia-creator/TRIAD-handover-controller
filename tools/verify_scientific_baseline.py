#!/usr/bin/env python3
"""Verify a SCIENTIFIC_BASELINE.sha256 manifest against Git blob
content at that commit, without touching the current working tree.

Unlike 'sha256sum -c' against a checked-out working tree, this reads each
file's content directly from the Git object database via
'git cat-file -p <commit>:<path>', so it gives the same answer regardless
of what branch or commit is currently checked out, and it never runs
'git checkout'.
"""
from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
from pathlib import Path


def git_blob_sha256(commit: str, path: str, repo: Path) -> str:
    proc = subprocess.run(
        ["git", "cat-file", "-p", f"{commit}:{path}"],
        cwd=repo,
        capture_output=True,
        check=True,
    )
    return hashlib.sha256(proc.stdout).hexdigest()


def parse_manifest(manifest: Path) -> list[tuple[str, str]]:
    entries = []
    for line in manifest.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        expected, _, path = line.partition("  ")
        if path.startswith("./"):
            path = path[2:]
        entries.append((expected, path))
    return entries


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--commit", required=True, help="commit the manifest describes")
    parser.add_argument("--repo", type=Path, default=Path("."), help="path to the Git repository")
    args = parser.parse_args()

    entries = parse_manifest(args.manifest)
    if not entries:
        print(f"FAIL: no entries found in {args.manifest}", file=sys.stderr)
        return 1

    failures = []
    for expected, path in entries:
        try:
            actual = git_blob_sha256(args.commit, path, args.repo)
        except subprocess.CalledProcessError:
            print(f"{path}: MISSING at {args.commit}")
            failures.append(path)
            continue
        if actual == expected:
            print(f"{path}: OK")
        else:
            print(f"{path}: FAILED")
            failures.append(path)

    if failures:
        print(
            f"FAIL: {len(failures)} of {len(entries)} blob(s) do not match "
            f"{args.commit}",
            file=sys.stderr,
        )
        return 1
    print(f"All {len(entries)} blobs verified against commit {args.commit}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
