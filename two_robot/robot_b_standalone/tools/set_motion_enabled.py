#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import yaml


def update(path: Path, enabled: bool) -> None:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid YAML root in {path}")
    data.setdefault("safety", {})["motionEnabled"] = bool(enabled)
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"{path}: motionEnabled={str(enabled).lower()}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("value", choices=["true", "false"])
    args = parser.parse_args()
    enabled = args.value == "true"

    root = Path(__file__).resolve().parents[1]
    source = root / "etc/CALLRobotBFaceToFaceMover.in.yaml"
    runtime = (
        Path.home()
        / "mc_rtc_ws/install/lib/mc_controller/etc"
        / "CALLRobotBFaceToFaceMover.yaml"
    )
    update(source, enabled)
    if runtime.exists():
        update(runtime, enabled)
    else:
        print("Installed YAML not found yet; source YAML updated only.")


if __name__ == "__main__":
    main()
