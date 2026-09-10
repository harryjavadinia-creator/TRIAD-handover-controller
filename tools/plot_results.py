#!/usr/bin/env python3
"""Render the release figures from included records; never run a controller."""
import argparse
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
EV = ROOT / "evidence"
SCENARIOS = ["lateral-low", "near-ground", "longitudinal", "diagonal"]


def style(ax):
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_axisbelow(True)
    ax.tick_params(length=0, pad=7)


def save(fig, name, output, fmt):
    fig.savefig(output / f"{name}.{fmt}", bbox_inches="tight",
                facecolor="white", dpi=160,
                metadata={"Date": None} if fmt == "svg" else None)
    plt.close(fig)


def plot_latency(output, fmt):
    rows = json.loads((EV / "latency" / "sweep_rows.json").read_text())
    fig, ax = plt.subplots(figsize=(9.0, 4.0), layout="constrained")
    for comp, label, color, marker in [
        (True, "Compensated", "#23786b", "o"),
        (False, "Uncompensated", "#586b9d", "s"),
    ]:
        samples = sorted(
            [(float(r["delay"]), 1000 * r["estimate_position_error_m"])
             for r in rows
             if r["estimate_position_error_m"] is not None
             and (r["comp"] == comp or float(r["delay"]) == 0)]
        )
        ax.plot([p[0] for p in samples], [p[1] for p in samples],
                color=color, marker=marker, label=label, linewidth=2)
    ax.set(xlabel="Configured measurement delay (s)",
           ylabel="Recorded position estimate error (mm)", ylim=(-1.5, 44),
           xticks=[0, .1, .22, .3, .4, .5, .6])
    ax.set_title("Corrected latency sweep · straight-motion simulation",
                 loc="left", fontweight="bold", fontsize=13, pad=14)
    ax.grid(color="#e8ebed")
    ax.legend(frameon=False, loc="upper left")
    style(ax)
    fig.get_layout_engine().set(rect=(0, .10, 1, .90))
    fig.text(.13, .01,
             "Uncompensated 0.60 s: ambiguous observation; estimate error unavailable.\n"
             "Points are recorded condition summaries, not confidence intervals.",
             fontsize=9, color="#475569")
    save(fig, "latency", output, fmt)


def timing_summary(scenario):
    text = (EV / "async" / scenario / "perf_analysis_clean_machine.txt").read_text()
    phase = re.search(
        r"in-planning ControllerRun: median ([\d.]+)  p90 ([\d.]+)  "
        r"p95 ([\d.]+)  p99 ([\d.]+)  max ([\d.]+) ms", text
    )
    whole = [re.search(rf"^{metric}\s*:.*max ([\d.]+) ms", text, re.M)
             for metric in ["ControllerRun", "GlobalRun"]]
    if phase is None or any(match is None for match in whole):
        raise ValueError(f"Unrecognized timing summary: {scenario}")
    return [float(phase[i]) for i in [1, 4, 5]], [float(m[1]) for m in whole]


def plot_timing(output, fmt):
    summaries = [timing_summary(s) for s in SCENARIOS]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
    x = list(range(4))
    for index, (label, color, marker) in enumerate([
        ("Median", "#23786b", "o"), ("p99", "#b88631", "s"),
        ("Maximum", "#586b9d", "D")
    ]):
        axes[0].scatter([v + (index - 1) * .15 for v in x],
                        [s[0][index] for s in summaries],
                        label=label, color=color, marker=marker, s=40)
    axes[0].set_title("During planning · ControllerRun", loc="left",
                      fontweight="bold", fontsize=12, pad=14)
    axes[0].set_ylim(0, 1.5)
    for index, (label, color) in enumerate([
        ("ControllerRun", "#23786b"), ("GlobalRun", "#586b9d")
    ]):
        bars = axes[1].bar([v + (index - .5) * .34 for v in x],
                           [s[1][index] for s in summaries],
                           width=.32, color=color, label=label)
        axes[1].bar_label(bars, fmt="%.1f", fontsize=8, padding=3)
    axes[1].set_title("Whole run · maxima", loc="left",
                      fontweight="bold", fontsize=12, pad=14)
    axes[1].set_ylim(0, max(s[1][1] for s in summaries) * 1.25)
    for ax in axes:
        ax.set_xticks(x, ["Lateral\nlow", "Near\nground", "Longitudinal", "Diagonal"])
        ax.set_ylabel("Wall time per cycle (ms)")
        ax.grid(axis="y", color="#e8ebed")
        ax.legend(frameon=False, fontsize=9, loc="upper left")
        style(ax)
    fig.suptitle("Asynchronous controller · clean-machine profiles",
                 fontsize=13, fontweight="bold")
    fig.get_layout_engine().set(rect=(0, .08, 1, .92))
    fig.text(.08, .005,
             "Different panel scales. Empirical profiles; no WCET or hard-real-time guarantee.",
             fontsize=9, color="#475569")
    save(fig, "controller_timing", output, fmt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "docs" / "figures")
    parser.add_argument("--format", choices=["svg", "png"], default="svg")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 10,
        "axes.labelcolor": "#253449", "text.color": "#253449",
        "xtick.color": "#475569", "ytick.color": "#475569",
        "axes.edgecolor": "#c5cdd5", "svg.fonttype": "none",
        "svg.hashsalt": "triad-results",
    })
    for plot in [plot_latency, plot_timing]:
        plot(args.output, args.format)
    print(f"Two figures written to {args.output}")


if __name__ == "__main__":
    main()
