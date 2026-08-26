#!/usr/bin/env python3
"""Regression tests for tools/verify_latency_matrix_cell.py's scenario/
condition identity checks. Uses only small synthetic log fixtures -- no
mc_rtc, build, or real captured historical logs required. Exercises the
module's functions directly rather than invoking check_latency_log.py as a
subprocess, so a fake `--checker` stub supplies the metrics dict.
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
VERIFY_SCRIPT = TOOLS_DIR / "verify_latency_matrix_cell.py"

P0_LINE = (
    "[warning] [ObserveObject] unified static/moving observation started "
    "with robot stationary p0=[0.920,0.000,0.550] duration=0.90s horizon=1.00s"
)
# An early observation sample: velocity has not yet ramped up to its
# settled value. Included alongside V_LINE_FINAL in every non-static test
# fixture below to prove the verifier reads the *last* [ObserveObject]
# sample, not merely the first match -- using the first match would read
# this v=[0,0,0] value and incorrectly fail scenario_velocity_ok for a
# genuinely correct moving-scenario log.
V_LINE_EARLY = (
    "[info] [ObserveObject] t=0.00/0.90s samples=0 valid=false moved=0.0000 "
    "p=[0.920,0.000,0.550] v=[0.0000,0.0000,0.0000] w=[0.0000,0.0000,0.0000] "
    "predicted=[0.920,0.000,0.550] latency=[mode:IDEAL age:0.000s rawErr:0.0000 estErr:0.0000]"
)
V_LINE_FINAL = (
    "[info] [ObserveObject] t=0.90/0.90s samples=900 valid=true moved=0.0721 "
    "p=[0.848,0.000,0.550] v=[-0.0800,0.0000,0.0000] w=[0.0000,0.0000,0.0000] "
    "predicted=[0.768,0.000,0.550] latency=[mode:IDEAL age:0.000s rawErr:0.0000 estErr:0.0000]"
)
# static_nominal has zero object velocity and never emits the moving-object
# "[PresentationSolve] robot stationary, object approaching..." line at
# all -- only [ObserveObject] samples, all v=[0,0,0].
STATIC_P0_LINE = (
    "[warning] [ObserveObject] unified static/moving observation started "
    "with robot stationary p0=[0.550,0.000,0.550] duration=0.90s horizon=1.00s"
)
STATIC_V_LINE = (
    "[info] [ObserveObject] t=0.90/0.90s samples=900 valid=true moved=0.0000 "
    "p=[0.550,0.000,0.550] v=[0.0000,0.0000,0.0000] w=[0.0000,0.0000,0.0000] "
    "predicted=[0.550,0.000,0.550] latency=[mode:IDEAL age:0.000s rawErr:0.0000 estErr:0.0000]"
)
COMPLETED_LINE = "[success] [Completed] full plan-once handover completed: grasp confirmed"
FAILURE_LINE = "Starting state HandoverInterceptionController_Failure"


def _write_stub_checker(tmp_dir: Path, mode: str, configured_delay_s: float, pass_value: bool) -> Path:
    """A minimal stand-in for check_latency_log.py: ignores its log argument
    and always prints the same fixed metrics dict, so these tests do not
    depend on check_latency_log.py's own parsing behavior."""
    stub = tmp_dir / "stub_checker.py"
    metrics = {"mode": mode, "configured_delay_s": configured_delay_s, "pass": pass_value}
    stub.write_text(
        "#!/usr/bin/env python3\n"
        "import json, sys\n"
        f"print(json.dumps({metrics!r}))\n"
    )
    return stub


def _run(
    tmp_dir: Path,
    log_lines: list[str],
    scenario: str,
    condition: str,
    mode: str,
    configured_delay_s: float,
    pass_value: bool = True,
) -> dict:
    log_path = tmp_dir / "test.log"
    log_path.write_text("\n".join(log_lines) + "\n")
    checker = _write_stub_checker(tmp_dir, mode, configured_delay_s, pass_value)
    result = subprocess.run(
        [
            sys.executable,
            str(VERIFY_SCRIPT),
            str(log_path),
            "--scenario",
            scenario,
            "--condition",
            condition,
            "--checker",
            str(checker),
        ],
        capture_output=True,
        text=True,
    )
    return json.loads(result.stdout)


def test_correct_completed_cell_passes() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [P0_LINE, V_LINE_EARLY, V_LINE_FINAL, COMPLETED_LINE],
            scenario="pure_x",
            condition="ideal",
            mode="IDEAL",
            configured_delay_s=0.0,
        )
        assert result["reproduction_result"] == "PASS", result
        assert result["scenario_position_ok"] is True
        assert result["scenario_velocity_ok"] is True
        assert result["condition_mode_ok"] is True
        assert result["condition_delay_ok"] is True
    print("  correct completed cell => PASS: PASS")


def test_correct_expected_failure_cell_passes() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [P0_LINE, V_LINE_EARLY, V_LINE_FINAL, FAILURE_LINE],
            scenario="pure_x",
            condition="uncompensated220",
            mode="DELAYED_UNCOMPENSATED",
            configured_delay_s=0.220,
            pass_value=False,
        )
        assert result["expected_outcome"] == "FAILURE"
        assert result["observed_outcome"] == "FAILURE"
        assert result["reproduction_result"] == "PASS", result
    print("  correct expected-failure cell => PASS: PASS")


def test_wrong_velocity_fails() -> None:
    with tempfile.TemporaryDirectory() as td:
        bad_v_line = V_LINE_FINAL.replace("v=[-0.0800,0.0000,0.0000]", "v=[9.9999,0.0000,0.0000]")
        result = _run(
            Path(td),
            [P0_LINE, V_LINE_EARLY, bad_v_line, COMPLETED_LINE],
            scenario="pure_x",
            condition="ideal",
            mode="IDEAL",
            configured_delay_s=0.0,
        )
        assert result["scenario_velocity_ok"] is False
        assert result["reproduction_result"] == "FAIL"
    print("  wrong velocity => FAIL: PASS")


def test_wrong_delay_fails() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [P0_LINE, V_LINE_EARLY, V_LINE_FINAL, COMPLETED_LINE],
            scenario="pure_x",
            condition="compensated220",
            mode="DELAYED_COMPENSATED",
            configured_delay_s=0.150,
        )
        assert result["condition_delay_ok"] is False
        assert result["reproduction_result"] == "FAIL"
    print("  wrong 0.220 delay => FAIL: PASS")


def test_wrong_mode_fails() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [P0_LINE, V_LINE_EARLY, V_LINE_FINAL, COMPLETED_LINE],
            scenario="pure_x",
            condition="compensated220",
            mode="IDEAL",
            configured_delay_s=0.220,
        )
        assert result["condition_mode_ok"] is False
        assert result["reproduction_result"] == "FAIL"
    print("  wrong mode => FAIL: PASS")


def test_static_nominal_zero_velocity_passes() -> None:
    """static_nominal never emits the moving-object PresentationSolve line;
    this is exactly the case that was found unverifiable when the verifier
    relied on that line alone, before switching to the last [ObserveObject]
    sample as the velocity source."""
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [STATIC_P0_LINE, STATIC_V_LINE, COMPLETED_LINE],
            scenario="static_nominal",
            condition="ideal",
            mode="IDEAL",
            configured_delay_s=0.0,
        )
        assert result["observed_velocity"] == [0.0, 0.0, 0.0], result
        assert result["scenario_velocity_ok"] is True
        assert result["reproduction_result"] == "PASS", result
    print("  static_nominal (zero velocity, no PresentationSolve line) => PASS: PASS")


def test_ambiguous_terminal_markers_fails() -> None:
    with tempfile.TemporaryDirectory() as td:
        result = _run(
            Path(td),
            [P0_LINE, V_LINE_EARLY, V_LINE_FINAL, COMPLETED_LINE, FAILURE_LINE],
            scenario="pure_x",
            condition="ideal",
            mode="IDEAL",
            configured_delay_s=0.0,
        )
        assert result["observed_outcome"] == "AMBIGUOUS"
        assert result["reproduction_result"] == "FAIL"
    print("  both Completed and Failure markers => FAIL: PASS")


def main() -> int:
    test_correct_completed_cell_passes()
    test_correct_expected_failure_cell_passes()
    test_wrong_velocity_fails()
    test_wrong_delay_fails()
    test_wrong_mode_fails()
    test_static_nominal_zero_velocity_passes()
    test_ambiguous_terminal_markers_fails()
    print("verify_latency_matrix_cell.py identity tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
