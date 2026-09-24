#!/usr/bin/env python3
"""Plot the objective of every certified plan of one TRIAD run and its seven weighted terms.

usage: plot_plan_costs.py <scenario.log> [--out <prefix>] [--title <text>]

Reads the [GlobalPlanCost] and [CompletePlanCost] lines that the controller writes for every
certified complete plan (one per candidate grasp x route x event hypothesis) and the
[PresentationCommit] line of the committed plan, and draws three panels:

  A  J_global of every certified plan against its event lead h, per route family, with the
     best plan of each hypothesis joined and the committed plan marked;
  B  the weighted terms w_k * k of the best plan of each event hypothesis, stacked, so that
     one sees which term moves when the event time moves (the schedule term
     J_global - J_motion is drawn separately, it can be negative);
  C  the ten best plans of the run by J_global with the same stacked terms.

Weights are the frozen ones of docs/mathematics.md section 7; the terms are already normalized
by the controller. Output: <prefix>.png and <prefix>.pdf (default: plan_costs_<scenario> next to
the log).
"""
import argparse
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WEIGHTS = {"T": 0.4210526, "E": 0.1052632, "L": 0.1052632, "C": 0.1578947,
           "Q": 0.0842105, "K": 0.0736842, "V": 0.0526316}
LABELS = {"T": "T execution time", "E": "E effort", "L": "L route length", "C": "C clearance reserve",
          "Q": "Q joint-limit reserve", "K": "K conditioning reserve", "V": "V velocity utilization"}
COLORS = {"T": "#1f4e79", "E": "#7f7f7f", "L": "#b5b5b5", "C": "#2a9d8f", "Q": "#c9713a",
          "K": "#e0a458", "V": "#5c5c5c", "S": "#c0392b"}


def parse(path):
    plans, terms, commit, lead_commit = [], {}, None, None
    rx_g = re.compile(r"\[GlobalPlanCost\] hypothesis=(\d+) eventLead=([\d.]+)s .*?candidate=(\S+) route=(\S+) valid=true motionJ=([\d.]+) .*?globalJ=([\d.]+)")
    rx_c = re.compile(r"\[CompletePlanCost\] candidate=(\S+) route=(\S+) valid=true J=([\d.]+) .*?terms=\[(.*?)\]")
    rx_k = re.compile(r"\[PresentationCommit\] COMMITTED candidate=(\S+) route=(\S+)")
    rx_l = re.compile(r"selectedLead=([\d.]+)s")
    for line in open(path, encoding="utf-8", errors="replace"):
        m = rx_g.search(line)
        if m:
            plans.append(dict(h=int(m.group(1)), lead=float(m.group(2)), cand=m.group(3), route=m.group(4),
                              jm=float(m.group(5)), jg=float(m.group(6))))
            continue
        m = rx_c.search(line)
        if m:
            terms[(m.group(1), m.group(2))] = {k: float(v) for k, v in re.findall(r"([A-Z]):([\d.]+)", m.group(4))}
            continue
        m = rx_k.search(line)
        if m:
            commit = (m.group(1), m.group(2))
        m = rx_l.search(line)
        if m and lead_commit is None:
            lead_commit = float(m.group(1))
    return plans, terms, commit, lead_commit


def stacked(ax, items, terms, label_fn):
    """items: list of plan dicts. Draws stacked weighted terms + schedule term."""
    keys = ["T", "E", "L", "C", "Q", "K", "V"]
    xs = range(len(items))
    bottoms = [0.0] * len(items)
    for k in keys:
        vals = [WEIGHTS[k] * terms.get((p["cand"], p["route"]), {}).get(k, 0.0) for p in items]
        ax.bar(xs, vals, bottom=bottoms, color=COLORS[k], width=0.75, label=LABELS[k])
        bottoms = [b + v for b, v in zip(bottoms, vals)]
    sched = [p["jg"] - p["jm"] for p in items]
    pos = [max(s, 0.0) for s in sched]
    neg = [min(s, 0.0) for s in sched]
    ax.bar(xs, pos, bottom=bottoms, color=COLORS["S"], width=0.75, label="schedule term (J$_{global}$ − J$_{motion}$)")
    ax.bar(xs, neg, bottom=[0.0] * len(items), color=COLORS["S"], width=0.75, alpha=0.45)
    for i, p in enumerate(items):
        ax.text(i, max(bottoms[i] + pos[i], 0) + 0.012, f"{p['jg']:.2f}", ha="center", fontsize=5.8)
    ax.set_xticks(list(xs))
    ax.set_xticklabels([label_fn(p) for p in items], fontsize=5.6, rotation=50, ha="right")
    ax.set_ylim(top=max(max(b + max(s0, 0.0) for b, s0 in zip(bottoms, sched)) * 1.12, 0.1))
    ax.axhline(0, color="black", lw=0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--out", default=None)
    ap.add_argument("--title", default=None)
    a = ap.parse_args()
    plans, terms, commit, lead_commit = parse(a.log)
    if not plans:
        print("no [GlobalPlanCost] lines found in", a.log, file=sys.stderr)
        return 1
    scenario = os.path.splitext(os.path.basename(a.log))[0]
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.log)), f"plan_costs_{scenario}")
    title = a.title or scenario

    best_by_h = {}
    for p in plans:
        if p["h"] not in best_by_h or p["jg"] < best_by_h[p["h"]]["jg"]:
            best_by_h[p["h"]] = p
    per_h = [best_by_h[h] for h in sorted(best_by_h)]
    top = sorted(plans, key=lambda p: p["jg"])[:10]
    committed = None
    if commit:
        cands = [p for p in plans if (p["cand"], p["route"]) == commit]
        if lead_commit is not None:
            cands = [p for p in cands if abs(p["lead"] - lead_commit) < 1e-6] or cands
        committed = min(cands, key=lambda p: p["jg"]) if cands else None

    fig = plt.figure(figsize=(11.5, 8.6))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.25], hspace=0.62, wspace=0.22, bottom=0.2, top=0.95, left=0.07, right=0.98)
    a1 = fig.add_subplot(gs[0, :]); a2 = fig.add_subplot(gs[1, 0]); a3 = fig.add_subplot(gs[1, 1])

    fam = [("direct", "#1f4e79", lambda r: r == "direct"), ("ring 80 mm", "#2a9d8f", lambda r: r.startswith("ring80")),
           ("ring 140 mm", "#c9713a", lambda r: r.startswith("ring140"))]
    for lab, col, sel in fam:
        pts = [p for p in plans if sel(p["route"])]
        a1.scatter([p["lead"] for p in pts], [p["jg"] for p in pts], s=9, color=col, alpha=0.65, label=f"{lab} ({len(pts)})")
    a1.plot([p["lead"] for p in per_h], [p["jg"] for p in per_h], "-", color="black", lw=0.8, label="best plan of each event hypothesis")
    if committed:
        a1.scatter([committed["lead"]], [committed["jg"]], s=160, marker="*", color="#c0392b", zorder=5,
                   label=f"committed: {committed['cand']} / {committed['route']} (h = {committed['lead']:.2f} s, J = {committed['jg']:.3f})")
    a1.set_xlabel("event lead h (s) — the presentation event, measured from the search epoch")
    a1.set_ylabel("J$_{global}$")
    a1.set_title(f"A  every certified complete plan of «{title}»: {len(plans)} plans over {len(best_by_h)} event hypotheses", fontsize=9, loc="left")
    a1.legend(fontsize=6.5, frameon=False, ncol=2); a1.grid(alpha=0.25)

    stacked(a2, per_h, terms, lambda p: f"h={p['lead']:.2f} s\n{p['cand'].replace('axis', '').replace('_side_', ' ')}\n{p['route'].replace('mm_', ' mm #')}")
    a2.set_ylabel("weight × normalized term")
    a2.set_title("B  best plan of each event hypothesis: which term moves with the event time", fontsize=8.5, loc="left")
    a2.legend(fontsize=6.5, frameon=False, ncol=4, loc="upper center", bbox_to_anchor=(1.02, -0.42))

    stacked(a3, top, terms, lambda p: f"h={p['lead']:.2f} s\n{p['cand'].replace('axis', '').replace('_side_', ' ')}\n{p['route'].replace('mm_', ' mm #')}")
    a3.set_title("C  the ten lowest-J$_{global}$ plans of the run", fontsize=8.5, loc="left")
    if committed:
        for i, p in enumerate(top):
            if p is committed or (p["cand"], p["route"], p["lead"]) == (committed["cand"], committed["route"], committed["lead"]):
                a3.get_xticklabels()[i].set_color("#c0392b"); a3.get_xticklabels()[i].set_fontweight("bold")
    for ax in (a1, a2, a3):
        for s in ("top", "right"): ax.spines[s].set_visible(False)
    fig.text(0.01, 0.01, "J_motion = w_T T + w_E E + w_L L + w_C C + w_Q Q + w_K K + w_V V (weights frozen, docs/mathematics.md §7);  "
             "J_global = J_motion + schedule term. Terms as logged by the controller for each certified plan.", fontsize=6.5, color="#555")
    fig.savefig(out + ".png", dpi=170, bbox_inches="tight"); fig.savefig(out + ".pdf", bbox_inches="tight")
    print(f"{out}.png  plans={len(plans)} hypotheses={len(best_by_h)} committed={commit} lead={lead_commit}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
