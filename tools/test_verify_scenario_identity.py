#!/usr/bin/env python3
"""Regression tests for tools/verify_scenario_identity.py's Dataset-B
scenario identity checks (position AND velocity). Uses only small synthetic
log fixtures -- no mc_rtc, build, or real captured logs required.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
VERIFY_SCRIPT = TOOLS_DIR / "verify_scenario_identity.py"

COMPLETED_LINE = "[success] [Completed] full plan-once handover completed: grasp confirmed"


def _p0_line(position: str) -> str:
    return (
        "[warning] [ObserveObject] unified static/moving observation started "
        f"with robot stationary p0={position} duration=0.90s horizon=1.00s"
    )


def _observe_line(t: str, velocity: str) -> str:
    return (
        f"[info] [ObserveObject] t={t}/0.90s samples=0 valid=false moved=0.0000 "
        f"p=[0.000,0.000,0.000] v={velocity} w=[0.0000,0.0000,0.0000] "
        "predicted=[0.000,0.000,0.000] latency=[mode:IDEAL age:0.000s rawErr:0.0000 estErr:0.0000]"
    )


def _run(tmp_dir: Path, log_lines: list[str], scenario: str) -> subprocess.CompletedProcess:
    log_path = tmp_dir / "test.log"
    log_path.write_text("\n".join(log_lines) + "\n")
    return subprocess.run(
        [sys.executable, str(VERIFY_SCRIPT), str(log_path), "--expect-scenario", scenario],
        capture_output=True,
        text=True,
    )


def test_correct_longitudinal_passes() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [
                _p0_line("[0.920,0.000,0.550]"),
                _observe_line("0.00", "[0.0000,0.0000,0.0000]"),
                _observe_line("0.90", "[-0.0800,0.0000,0.0000]"),
                COMPLETED_LINE,
            ],
            "longitudinal",
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert "position_ok=True" in result.stdout
        assert "velocity_ok=True" in result.stdout
        assert "scenario_identity_result=PASS" in result.stdout
    print("  correct non-default longitudinal => PASS: PASS")


def test_correct_diagonal_passes() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [
                _p0_line("[0.900,0.000,0.300]"),
                _observe_line("0.00", "[0.0000,0.0000,0.0000]"),
                _observe_line("0.90", "[-0.0566,0.0000,0.0566]"),
                COMPLETED_LINE,
            ],
            "diagonal",
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert "position_ok=True" in result.stdout
        assert "velocity_ok=True" in result.stdout
        assert "scenario_identity_result=PASS" in result.stdout
    print("  correct diagonal => PASS: PASS")


def test_correct_position_wrong_velocity_fails() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [
                _p0_line("[0.920,0.000,0.550]"),
                _observe_line("0.00", "[0.0000,0.0000,0.0000]"),
                _observe_line("0.90", "[9.9999,0.0000,0.0000]"),
                COMPLETED_LINE,
            ],
            "longitudinal",
        )
        assert result.returncode != 0
        assert "position_ok=True" in result.stdout
        assert "velocity_ok=False" in result.stdout
        assert "scenario_identity_result=FAIL" in result.stdout
    print("  correct position + wrong velocity => FAIL: PASS")


def test_correct_velocity_wrong_position_fails() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [
                _p0_line("[0.550,0.000,0.550]"),
                _observe_line("0.00", "[0.0000,0.0000,0.0000]"),
                _observe_line("0.90", "[-0.0800,0.0000,0.0000]"),
                COMPLETED_LINE,
            ],
            "longitudinal",
        )
        assert result.returncode != 0
        assert "position_ok=False" in result.stdout
        assert "velocity_ok=True" in result.stdout
        assert "scenario_identity_result=FAIL" in result.stdout
    print("  correct velocity + wrong position => FAIL: PASS")


def main() -> int:
    test_correct_longitudinal_passes()
    test_correct_diagonal_passes()
    test_correct_position_wrong_velocity_fails()
    test_correct_velocity_wrong_position_fails()
    print("verify_scenario_identity.py Dataset-B identity tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
