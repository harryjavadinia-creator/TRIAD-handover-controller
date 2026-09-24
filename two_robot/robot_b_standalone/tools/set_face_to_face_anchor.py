#!/usr/bin/env python3
"""Set one known Robot-B canonical start and regenerate all 180-degree starts."""
from __future__ import annotations
import argparse
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("x", type=float)
parser.add_argument("y", type=float)
parser.add_argument("z", type=float)
args = parser.parse_args()

pA_ref = [0.55, -0.56, 0.15]
pB_ref = [args.x, args.y, args.z]
for path in sorted((root / "etc/presets").glob("*.yaml")):
    cfg = yaml.safe_load(path.read_text())
    pa = [float(x) for x in cfg["referenceAStart"]]
    d = [pa[i] - pA_ref[i] for i in range(3)]
    pb = [pB_ref[0] - d[0], pB_ref[1] - d[1], pB_ref[2] + d[2]]
    cfg["robotBStartPose"]["configured"] = True
    cfg["robotBStartPose"]["translation"] = [round(x, 7) for x in pb]
    cfg["mapping"] = {
        "referenceScenario": "canonical_yz",
        "referenceAStart": pA_ref,
        "referenceBStart": pB_ref,
        "yawDegrees": 180.0,
    }
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(path.stem, "->", cfg["robotBStartPose"]["translation"])
print("Presets regenerated. Re-run tools/select_scenario.py for the desired scenario.")
