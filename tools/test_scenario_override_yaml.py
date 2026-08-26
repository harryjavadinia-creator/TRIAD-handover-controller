#!/usr/bin/env python3
"""Regression test for scenario-override YAML field placement.

The controller reads the simulated object's linear velocity from
config["movingObject"]["simulatedLinearVelocity"] (confirmed directly from
src/HandoverInterceptionController.cpp at both curated baseline tags: the
read happens inside the `if(config.has("movingObject"))` block via
`moving("simulatedLinearVelocity", v)`, where `moving = config("movingObject")`;
the `decisionCost` block only ever reads selectionMode/eventSelectionMode/
tieTolerance/allowPhysicalExecution/timeReference/effortReference -- never
simulatedLinearVelocity). A `decisionCost.simulatedLinearVelocity` override
is silently inert: the controller falls back to whatever value is baked
into the installed default etc/HandoverInterceptionController.yaml.

This test verifies, without requiring mc_rtc or a build:
  - Dataset A (scripts/reproduce_latency_matrix.sh): the real --dry-run
    mode is invoked for all 5 scenarios, and the actual generated override
    YAML is checked -- simulatedLinearVelocity exists under movingObject
    with the correct per-scenario value, and no decisionCost key appears
    at all.
  - Dataset B (scripts/run_scenario.sh): this script has no dry-run mode,
    so its embedded YAML template and its scenario/velocity case table are
    checked directly from its own source text.
"""
from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
REPRODUCE_SCRIPT = REPO_ROOT / "scripts" / "reproduce_latency_matrix.sh"
RUN_SCENARIO_SCRIPT = REPO_ROOT / "scripts" / "run_scenario.sh"

# scenario -> (translation, velocity), exactly as documented/recovered.
DATASET_A_SCENARIOS = {
    "static_nominal": ("[0.55, 0, 0.55]", "[0, 0, 0]"),
    "canonical_yz": ("[0.55, -0.56, 0.15]", "[0, 0.08, 0]"),
    "pure_x": ("[0.92, 0, 0.55]", "[-0.08, 0, 0]"),
    "diagonal_xz": ("[0.9, 0, 0.3]", "[-0.0565685, 0, 0.0565685]"),
    "ground_near": ("[0.25, 0.62, 0.15]", "[0, -0.08, 0]"),
}

# scenario -> (translation, velocity), exactly matching run_scenario.sh's
# own case statement / docs/simulation.md's published table.
DATASET_B_SCENARIOS = {
    "near-ground": ("[0.25, 0.62, 0.15]", "[0.0, -0.08, 0.0]"),
    "longitudinal": ("[0.92, 0.00, 0.55]", "[-0.08, 0.0, 0.0]"),
    "lateral-low": ("[0.55, -0.56, 0.15]", "[0.0, 0.08, 0.0]"),
    "diagonal": ("[0.90, 0.00, 0.30]", "[-0.0565685, 0.0, 0.0565685]"),
}


def has_top_level_key(yaml_text: str, key: str) -> bool:
    return re.search(rf"^{re.escape(key)}:\s*$", yaml_text, re.MULTILINE) is not None


def extract_nested_scalar(yaml_text: str, top_key: str, child_key: str) -> str | None:
    """Return the value of a 2-space-indented direct child of a top-level
    (column 0) YAML key, using this repo's consistent 2-space-per-level
    override YAML style -- not a general YAML parser."""
    lines = yaml_text.splitlines()
    in_block = False
    for line in lines:
        if re.match(rf"^{re.escape(top_key)}:\s*$", line):
            in_block = True
            continue
        if in_block:
            if re.match(r"^\S", line):
                in_block = False
                continue
            m = re.match(rf"^ {{2}}{re.escape(child_key)}:\s*(.+)$", line)
            if m:
                return m.group(1).strip()
    return None


def test_dataset_a_overrides() -> None:
    if shutil.which("bash") is None:
        raise RuntimeError("bash not found on PATH")
    out_root = REPO_ROOT / "results" / "latency_matrix_dataset_a"
    for scenario, (expected_translation, expected_velocity) in DATASET_A_SCENARIOS.items():
        result = subprocess.run(
            ["bash", str(REPRODUCE_SCRIPT), "--scenario", scenario, "--condition", "ideal", "--dry-run"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
        m = re.search(r"Dry run: wrote (\S+/generated_override\.yaml)", result.stdout)
        assert m, f"{scenario}: could not find generated_override.yaml path in output:\n{result.stdout}"
        override_path = REPO_ROOT / m.group(1)
        text = override_path.read_text()

        assert not has_top_level_key(text, "decisionCost"), (
            f"{scenario}: decisionCost must not appear in the generated override at all"
        )
        actual_velocity = extract_nested_scalar(text, "movingObject", "simulatedLinearVelocity")
        assert actual_velocity == expected_velocity, (
            f"{scenario}: movingObject.simulatedLinearVelocity = {actual_velocity!r}, "
            f"expected {expected_velocity!r}"
        )
        actual_translation = extract_nested_scalar(text, "object", "translation")
        assert actual_translation == expected_translation, (
            f"{scenario}: object.translation = {actual_translation!r}, expected {expected_translation!r}"
        )
    if out_root.exists():
        shutil.rmtree(out_root)
    print("  Dataset A (reproduce_latency_matrix.sh, all 5 scenarios, real --dry-run): PASS")


def test_dataset_b_source_overrides() -> None:
    src = RUN_SCENARIO_SCRIPT.read_text()

    heredoc_match = re.search(r"<<EOF\n(.*?)\nEOF", src, re.DOTALL)
    assert heredoc_match, "could not locate the override YAML heredoc in run_scenario.sh"
    template = heredoc_match.group(1)

    assert "decisionCost" not in template, "decisionCost must not appear in run_scenario.sh's override template"
    assert re.search(r"^movingObject:\n {2}simulatedLinearVelocity: \$\{VELOCITY\}", template, re.MULTILINE), (
        "movingObject.simulatedLinearVelocity (with ${VELOCITY}) not found as expected in the template"
    )

    for scenario, (expected_translation, expected_velocity) in DATASET_B_SCENARIOS.items():
        m = re.search(
            rf'{re.escape(scenario)}\)\n\s+TRANSLATION="(.*?)"\n\s+VELOCITY="(.*?)"',
            src,
        )
        assert m, f"scenario {scenario!r} not found in run_scenario.sh's case statement"
        assert m.group(1) == expected_translation, (
            f"{scenario}: TRANSLATION = {m.group(1)!r}, expected {expected_translation!r}"
        )
        assert m.group(2) == expected_velocity, (
            f"{scenario}: VELOCITY = {m.group(2)!r}, expected {expected_velocity!r}"
        )
    print("  Dataset B (run_scenario.sh, all 4 published scenarios, source-verified): PASS")


def main() -> int:
    test_dataset_a_overrides()
    test_dataset_b_source_overrides()
    print("scenario override YAML field-placement tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
