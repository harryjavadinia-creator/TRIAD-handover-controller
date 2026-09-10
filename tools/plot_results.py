#!/usr/bin/env python3
"""Render the current release figure from included records; never run a controller."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
EV = ROOT / "evidence"


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
    plot_latency(args.output, args.format)
    print(f"Latency figure written to {args.output}")


if __name__ == "__main__":
    main()
