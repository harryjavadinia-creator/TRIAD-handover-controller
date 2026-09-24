#!/usr/bin/env python3
"""Set one key of the controller override used on hardware.

usage: set_override_key.py <dotted.key> <value> [override-file]

Default file: $HOME/.config/mc_rtc/controllers/HandoverInterceptionController.yaml
(the file written by two_robot/prepare_hardware_config.sh). The value is written
as YAML scalar text (true/false, numbers, or a bare string). Only the first
top-level occurrence of the path is changed; the rest of the file is untouched.
"""
import os
import re
import sys


def main():
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    key, value = sys.argv[1], sys.argv[2]
    path = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser(
        "~/.config/mc_rtc/controllers/HandoverInterceptionController.yaml")
    parts = key.split(".")
    lines = open(path, encoding="utf-8").read().splitlines(keepends=True)
    depth = 0
    i = 0
    # walk down the mapping path by indentation
    for level, part in enumerate(parts):
        found = False
        while i < len(lines):
            line = lines[i]
            stripped = line.lstrip(" ")
            indent = len(line) - len(stripped)
            if stripped.strip() and not stripped.startswith("#"):
                if indent < depth:
                    break
                if indent == depth and re.match(re.escape(part) + r":", stripped):
                    found = True
                    if level == len(parts) - 1:
                        comment = ""
                        m = re.search(r"\s+#.*$", line.rstrip("\n"))
                        if m:
                            comment = m.group(0)
                        lines[i] = f"{' ' * indent}{part}: {value}{comment}\n"
                        open(path, "w", encoding="utf-8").writelines(lines)
                        print(f"{path}: {key} = {value}")
                        return 0
                    depth = indent + 2
                    i += 1
                    break
            i += 1
        if not found:
            print(f"key not found: {key} (missing '{part}') in {path}", file=sys.stderr)
            return 1
    return 1


if __name__ == "__main__":
    sys.exit(main())
