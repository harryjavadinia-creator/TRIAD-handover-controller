#!/usr/bin/env python3
"""Check repository-local Markdown links in reader-facing documentation."""
from __future__ import annotations

from pathlib import Path
import re
import sys
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")


def documentation_files() -> list[Path]:
    files = [ROOT / "README.md", ROOT / "call_object_description" / "README.md"]
    files.extend(sorted((ROOT / "docs").glob("*.md")))
    return [path for path in files if path.is_file()]


def local_target(source: Path, raw: str) -> Path | None:
    target = raw.strip()
    if not target or target.startswith(("#", "http://", "https://", "mailto:")):
        return None
    if " " in target and not target.startswith("<"):
        target = target.split(" ", 1)[0]
    target = target.strip("<>")
    target = unquote(target.split("#", 1)[0])
    if not target:
        return None
    return (source.parent / target).resolve()


def main() -> int:
    errors: list[str] = []
    root = ROOT.resolve()
    for source in documentation_files():
        text = source.read_text(encoding="utf-8")
        for match in LINK_RE.finditer(text):
            raw = match.group(1)
            target = local_target(source, raw)
            if target is None:
                continue
            try:
                target.relative_to(root)
            except ValueError:
                errors.append(
                    f"{source.relative_to(ROOT)}: link escapes repository: {raw}"
                )
                continue
            if not target.exists():
                errors.append(
                    f"{source.relative_to(ROOT)}: missing local link target: {raw}"
                )

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        print(
            f"markdown local-link check: FAIL ({len(errors)} error(s))",
            file=sys.stderr,
        )
        return 1

    print(f"markdown local-link check: PASS ({len(documentation_files())} files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
