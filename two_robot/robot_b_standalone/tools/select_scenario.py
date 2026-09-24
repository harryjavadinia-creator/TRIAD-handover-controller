#!/usr/bin/env python3
"""Select a predefined Robot-B start and the matching Robot-A trajectory."""
from __future__ import annotations

import argparse
from pathlib import Path
import math
import yaml


def load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise RuntimeError(f"Invalid YAML root: {path}")
    return data


def update(path: Path, preset: dict) -> None:
    data = load_yaml(path)
    tr = data.setdefault("trajectory", {})
    for key in (
        "scenario", "referenceAStart", "robotAVelocity",
        "constantVelocityDuration", "decelerationDuration", "maximumTravel",
    ):
        tr[key] = preset[key]
    tr["faceToFaceYaw180"] = True

    start = preset["robotBStartPose"]
    data["startPose"] = {
        "configured": True,
        "translation": [float(x) for x in start["translation"]],
        # Rotation is intentionally identity metadata; runtime preserves the
        # tool orientation present when the controller starts.
        "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    }
    data.setdefault("safety", {})["motionEnabled"] = False
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    print("UPDATED:", path)


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    preset_dir = root / "etc/presets"
    available = sorted(p.stem for p in preset_dir.glob("*.yaml"))
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=available)
    args = parser.parse_args()

    preset = load_yaml(preset_dir / f"{args.scenario}.yaml")
    source = root / "etc/CALLRobotBFaceToFaceMover.in.yaml"
    runtime = Path.home() / "mc_rtc_ws/install/lib/mc_controller/etc/CALLRobotBFaceToFaceMover.yaml"
    update(source, preset)
    if runtime.exists():
        update(runtime, preset)

    va = [float(x) for x in preset["robotAVelocity"]]
    vb = [-va[0], -va[1], va[2]]
    effective = float(preset["constantVelocityDuration"]) + 0.5 * float(preset["decelerationDuration"])
    displacement = [v * effective for v in vb]
    start = [float(x) for x in preset["robotBStartPose"]["translation"]]
    final = [start[i] + displacement[i] for i in range(3)]
    distance = math.sqrt(sum(x*x for x in displacement))

    print()
    print("SCENARIO:", args.scenario)
    print("Robot-A initial object position:", preset["referenceAStart"])
    print("Robot-B predefined start:", start)
    print("Robot-B mirrored velocity:", vb)
    print("Robot-B scenario final:", final)
    print(f"Path distance: {distance:.4f} m")
    main_cfg = load_yaml(source)
    wmin = [float(x) for x in main_cfg["safety"]["workspaceMin"]]
    wmax = [float(x) for x in main_cfg["safety"]["workspaceMax"]]
    inside_start = all(wmin[i] <= start[i] <= wmax[i] for i in range(3))
    inside_final = all(wmin[i] <= final[i] <= wmax[i] for i in range(3))
    print("workspace start/final:", "PASS" if inside_start and inside_final else "BLOCKED")
    if not (inside_start and inside_final):
        print("WARNING: this mapped scenario is outside the configured Robot-B workspace and will not execute.")
    print("motionEnabled: false")
    print("On launch: current pose -> predefined start -> READY -> trigger -> scenario motion")


if __name__ == "__main__":
    main()
