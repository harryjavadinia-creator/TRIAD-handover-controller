"""Figures of paper/triad_system_paper.tex, generated from this repository's own configuration,
source rules and evidence records. Run from the repository root:

    python3 paper/make_figures.py        ->  paper/figures/fig_*.pdf

Nothing is drawn by hand: the grasp and route banks follow the construction rules in
src/HandoverInterceptionController.cpp (buildCandidate, transitRouteBank) with the values of
etc/HandoverInterceptionController.in.yaml; the prediction figure uses the quintic stop of
docs/mathematics.md; the funnel, cost, latency, mode and two-robot figures read evidence/,
supervisory_mode/evidence/ and two_robot/results/.
"""
import os, re, json, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "paper", "figures")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"font.family": "DejaVu Serif", "font.size": 8, "axes.linewidth": 0.6,
                     "mathtext.fontset": "cm", "pdf.fonttype": 42})
C_OBJ, C_HUM, C_ROB, C_OK, C_BAD, C_MUT = "#39424e", "#b2442b", "#1f4e79", "#2e7d32", "#b2442b", "#6b7480"
def save(fig, name):
    fig.savefig(os.path.join(OUT, name + ".pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig); print(name)

# ------------------------------------------------------------------ configuration values (etc/*.in.yaml)
LEADS = [1.80, 1.90, 2.35, 2.80, 3.25, 3.70, 4.15, 4.60, 5.05, 5.50, 5.95, 6.40, 6.85, 8.00]
CANDIDATE_COUNT, STANDOFF, RETREAT, CAPTURE_DEPTH = 16, 0.120, 0.180, 0.006
ROUTE_DIRECTIONS, APEX = 8, [0.08, 0.14]
DECEL = 0.85
HANDLE_R, HANDLE_HALF, SENSOR_R, SENSOR_HALF, HANDLE_CENTRE = 0.01125, 0.0687, 0.017, 0.0182, 0.0869
WEIGHTS = {"T": 0.4210526, "E": 0.1052632, "L": 0.1052632, "C": 0.1578947, "Q": 0.0842105, "K": 0.0736842, "V": 0.0526316}

# ------------------------------------------------------------------ 1. pipeline
fig, ax = plt.subplots(figsize=(7.2, 2.9)); ax.set_xlim(0, 14); ax.set_ylim(0, 5.2); ax.axis("off")
def box(x, y, w, h, t, fc="white", ec=C_OBJ, fs=7.2, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", fc=fc, ec=ec, lw=0.9))
    ax.text(x + w / 2, y + h / 2, t, ha="center", va="center", fontsize=fs, fontweight="bold" if bold else "normal")
def arr(p, q, **kw):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=9, lw=0.8, color=C_OBJ, **kw))
# control thread
ax.text(0.15, 4.95, "control thread, 1 kHz (mc_rtc FSM + QP)", fontsize=7, color=C_ROB, fontweight="bold")
box(0.2, 3.35, 2.3, 1.25, "observe object,\nestimate twist,\nfreeze snapshot $s_0$", fs=6.6)
box(10.15, 3.35, 2.0, 1.25, "timing admission\nat $t_{sel}$; commit-\ntime consistency", fs=6.0)
box(12.3, 3.35, 1.55, 1.25, "commit once;\nexecute reach,\ncapture, retreat", fc="#eaf2ea", ec=C_OK, fs=5.9)
arr((2.5, 3.6), (3.2, 2.7)); arr((9.8, 2.7), (10.6, 3.35)); arr((12.15, 3.97), (12.3, 3.97))
# worker
ax.add_patch(FancyBboxPatch((2.55, 0.3), 7.45, 2.75, boxstyle="round,pad=0.02,rounding_size=0.15", fc="#f4f3ee", ec=C_MUT, lw=0.7, ls="--"))
ax.text(2.7, 2.8, "background worker: one frozen snapshot per generation; the control thread polls without blocking", fontsize=6.4, color=C_MUT)
box(2.75, 1.25, 1.5, 1.2, "predict $\\Pi(h)$\nfor 14 leads", fs=6.2)
box(4.4, 1.25, 1.8, 1.2, "bank $\\mathcal{T}\\times\\mathcal{G}\\times\\mathcal{R}$\n14 × 32 × 17", fs=6.2)
box(6.35, 1.25, 2.15, 1.2, "certify reach, insertion,\nclosure, retreat\n(25 poses per segment)", fs=5.9)
box(8.65, 1.25, 1.15, 1.2, "rank by\n$J_{global}$", fs=6.2)
arr((4.25, 1.85), (4.4, 1.85)); arr((6.2, 1.85), (6.35, 1.85)); arr((8.5, 1.85), (8.65, 1.85))
ax.text(6.15, 0.65, "records: every complete plan with its checks, reserves and cost", fontsize=6.4, color=C_MUT, ha="center")
save(fig, "fig_pipeline")

# ------------------------------------------------------------------ helpers for the object and frames
def rot_axis(axis, ang):
    axis = axis / np.linalg.norm(axis); K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
def cylinder(ax3, centre, axis, r, half, color, alpha=0.9, n=24):
    axis = axis / np.linalg.norm(axis); e1 = np.cross(axis, [1, 0, 0]);
    if np.linalg.norm(e1) < 1e-6: e1 = np.cross(axis, [0, 1, 0])
    e1 /= np.linalg.norm(e1); e2 = np.cross(axis, e1)
    th = np.linspace(0, 2 * np.pi, n); s = np.array([-half, half])
    TH, S = np.meshgrid(th, s)
    P = centre[None, None, :] + S[..., None] * axis + r * (np.cos(TH)[..., None] * e1 + np.sin(TH)[..., None] * e2)
    ax3.plot_surface(P[..., 0], P[..., 1], P[..., 2], color=color, alpha=alpha, linewidth=0)

def build_candidate(angle, sign, pH, zH, base_outward):
    """buildCandidate() of the controller: mouth frame axes and the capture/standoff/retreat points."""
    zM = sign * zH
    yM = rot_axis(zM, angle) @ base_outward; yM -= zM * zM.dot(yM); yM /= np.linalg.norm(yM)
    xM = np.cross(yM, zM); xM /= np.linalg.norm(xM); yM = np.cross(zM, xM); yM /= np.linalg.norm(yM)
    pC = pH + yM * CAPTURE_DEPTH
    return yM, zM, pC, pC + yM * STANDOFF, pC + yM * RETREAT

# ------------------------------------------------------------------ 2. grasp bank
# object pose of the lateral-low scenario: frame at the sensor centre, rpy [0, 1.5708, 0] -> object z along world -x
Rz = rot_axis(np.array([0, 1, 0]), 1.5708)
zH = Rz @ np.array([0, 0, 1.0])            # object +Z (human handle) in world; blue handle is along -Z
p_obj = np.array([0.0, 0.0, 0.0]); pH_blue = p_obj - zH * HANDLE_CENTRE
mouth_now = np.array([0.35, 0.10, 0.20])   # a current mouth position, only to define the robot-relative base direction
base = mouth_now - pH_blue; base -= zH * zH.dot(base); base /= np.linalg.norm(base)   # "candidate zero" rule
fig = plt.figure(figsize=(7.2, 3.6))
for k, (sign, ttl) in enumerate([(+1, "axisP: mouth $z_M$ along the handle axis"), (-1, "axisN: mouth $z_M$ reversed")]):
    ax3 = fig.add_subplot(1, 2, k + 1, projection="3d"); ax3.set_box_aspect((1, 1, 0.6)); ax3.view_init(elev=22, azim=-50)
    cylinder(ax3, p_obj, zH, SENSOR_R, SENSOR_HALF, C_OBJ); cylinder(ax3, p_obj + zH * HANDLE_CENTRE, zH, HANDLE_R, HANDLE_HALF, "#9aa0a6")
    cylinder(ax3, pH_blue, zH, HANDLE_R, HANDLE_HALF, C_ROB)
    for i in range(CANDIDATE_COUNT):
        ang = 2 * np.pi * i / CANDIDATE_COUNT
        yM, zM, pC, pS, pR = build_candidate(ang, sign, pH_blue, zH, base)
        ax3.plot(*zip(pC, pS), color=C_OK, lw=0.8, alpha=0.9)
        ax3.plot(*zip(pS, pR), color=C_MUT, lw=0.6, ls=":", alpha=0.8)
        ax3.scatter(*pS, color=C_OK, s=6); ax3.scatter(*pR, color=C_MUT, s=4)
        if i == 0: ax3.text(*(pR + 0.01), "0°", fontsize=6, color=C_OK)
    ax3.quiver(*pH_blue, *(zH * sign * 0.09), color=C_ROB, arrow_length_ratio=0.2, lw=1.2)
    ax3.text(*(pH_blue + zH * sign * 0.1), "$z_M$", fontsize=7, color=C_ROB)
    ax3.set_title(ttl, fontsize=7.5, pad=2); ax3.set_axis_off()
    lim = 0.19; ax3.set_xlim(-lim, lim); ax3.set_ylim(-lim, lim); ax3.set_zlim(-lim * 0.6, lim * 0.6)
fig.subplots_adjust(left=0, right=1, top=0.95, bottom=0, wspace=0)
save(fig, "fig_grasp_bank")

# ------------------------------------------------------------------ 3. route bank
yM, zM, pC, pS, pR = build_candidate(np.deg2rad(45), +1, pH_blue, zH, base)
start = mouth_now; chord = pS - start; chord_n = chord / np.linalg.norm(chord)
e1 = np.array([0, 0, 1.0]) - chord_n * chord_n.dot([0, 0, 1.0]); e1 /= np.linalg.norm(e1); e2 = np.cross(chord_n, e1)
fig = plt.figure(figsize=(3.4, 3.0)); ax3 = fig.add_subplot(111, projection="3d"); ax3.view_init(elev=20, azim=-35); ax3.set_box_aspect((1, 1, 0.8))
t = np.linspace(0, 1, 40)
ax3.plot(*zip(start, pS), color=C_OBJ, lw=1.6, label="direct")
for r, col in zip(APEX, [C_OK, C_ROB]):
    for k in range(ROUTE_DIRECTIONS):
        a = 2 * np.pi * k / ROUTE_DIRECTIONS; off = r * (np.cos(a) * e1 + np.sin(a) * e2)
        apex = start + 0.5 * chord + off
        P = (1 - t)[:, None] ** 2 * start + 2 * (1 - t)[:, None] * t[:, None] * (2 * apex - 0.5 * (start + pS)) + t[:, None] ** 2 * pS
        ax3.plot(P[:, 0], P[:, 1], P[:, 2], color=col, lw=0.7, alpha=0.85, label=f"ring {int(r*1000)} mm" if k == 0 else None)
ax3.scatter(*start, color=C_OBJ, s=14); ax3.text(*(start + [0, 0, 0.012]), "start", fontsize=6.5)
ax3.scatter(*pS, color=C_OK, s=14); ax3.text(*(pS + [0, 0, 0.012]), "standoff", fontsize=6.5)
ax3.set_axis_off(); ax3.legend(fontsize=6, loc="lower left", frameon=False)
fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
save(fig, "fig_route_bank")

# ------------------------------------------------------------------ 4. prediction and event-time bank
v = 0.08; D = DECEL
def travel(h):  # effective constant-twist travel: h - 0.5*min(h, D)
    return v * (h - 0.5 * min(h, D))
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.0, 2.2), gridspec_kw={"width_ratios": [1.3, 1]})
u = np.linspace(0, 1, 100); b = 1 - 10 * u**3 + 15 * u**4 - 6 * u**5
a2.plot(u, b, color=C_OBJ, lw=1.5); a2.fill_between(u, b, alpha=0.12, color=C_OBJ)
a2.set_xlabel("stop progress $u$"); a2.set_ylabel("speed multiplier $b(u)$"); a2.set_title("prescribed terminal stop, $\\int_0^1 b = 1/2$", fontsize=7.5)
a2.text(0.55, 0.75, "$b(u)=1-10u^3+15u^4-6u^5$", fontsize=7)
hh = np.linspace(0, 8.2, 300); a1.plot(hh, [travel(h) for h in hh], color=C_MUT, lw=1, label="predicted travel for a stop at lead $h$")
for h in LEADS:
    a1.plot([h], [travel(h)], "o", color=C_ROB, ms=3.5)
a1.plot([], [], "o", color=C_ROB, ms=3.5, label="14 event hypotheses")
a1.set_xlabel("event lead $h$ from the frozen epoch (s)"); a1.set_ylabel("object travel until the event (m)")
a1.set_title("event-time bank on the predicted motion (0.08 m/s, stop 0.85 s)", fontsize=7.5); a1.legend(fontsize=6.5, frameon=False, loc="lower right")
for a in (a1, a2): a.grid(alpha=0.25); [a.spines[s].set_visible(False) for s in ("top", "right")]
save(fig, "fig_prediction")

# ------------------------------------------------------------------ 5. funnel from the evidence records
scen = ["near-ground", "longitudinal", "lateral-low", "diagonal"]
complete, admissible = {}, {}
for s in scen:
    fp = open(os.path.join(ROOT, "evidence", "async", s, "frozen_plan_set.txt")).read()
    complete[s] = int(re.search(r"records=(\d+)", fp).group(1))
    tr = open(os.path.join(ROOT, "evidence", "async", s, "timing_frontier_replay.txt")).read()
    admissible[s] = int(re.search(r"logged_timing_admissible=(\d+)", tr).group(1))
reasons = {}
pc = open(os.path.join(ROOT, "evidence", "async", "near-ground", "normalized", "PlanCapture_async.txt")).read()
for r in re.findall(r"reason=([A-Za-z_]+)", pc): reasons[r] = reasons.get(r, 0) + 1
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.0, 2.5), gridspec_kw={"width_ratios": [1.25, 1]})
x = np.arange(len(scen)); wdt = 0.2
for i, (lab, vals, col) in enumerate([("generated (upper bound)", [7616] * 4, "#c9c4b8"), ("complete, certified", [complete[s] for s in scen], C_OBJ),
                                      ("timing-admissible at receipt", [admissible[s] for s in scen], C_OK), ("committed", [1] * 4, C_ROB)]):
    a1.bar(x + (i - 1.5) * wdt, vals, wdt, label=lab, color=col)
    for xi, vv in zip(x + (i - 1.5) * wdt, vals): a1.text(xi, vv * 1.25, str(vv), ha="center", fontsize=5.2, rotation=90 if vv > 100 else 0, va="bottom")
a1.set_yscale("log"); a1.set_ylim(0.7, 4e4); a1.set_xticks(x); a1.set_xticklabels(scen, fontsize=7); a1.legend(fontsize=5.8, frameon=False, ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.16))
a1.set_ylabel("plans"); a1.set_title("from the bank to one committed plan", fontsize=7.5)
groups = {"IK preview did not converge": ["ik_preview_no_convergence"], "gripper body / finger clearance": ["right_tip_body_", "left_tip_body_", "right_finger_", "left_finger_", "left_pad_shoulder_high", "right_pad_shoulder_high", "right_knuckle_b", "left_knuckle_b"],
          "closure sweep": ["closure"], "base / rear clearance": ["base_rear"], "mouth corridor": ["mouth_corridor"], "generation": ["gen"]}
gv = {g: sum(reasons.get(k, 0) for k in ks) for g, ks in groups.items()}
other = sum(reasons.values()) - sum(gv.values())
if other: gv["other"] = other
a2.barh(list(gv.keys()), list(gv.values()), color=C_BAD, alpha=0.85)
for i, (g, n) in enumerate(gv.items()): a2.text(n + 3, i, str(n), va="center", fontsize=6)
a2.invert_yaxis(); a2.set_xlabel("rejected grasp previews, near-ground, 14 hypotheses", fontsize=7); a2.set_title("why certification rejects", fontsize=7.5); a2.tick_params(axis="y", labelsize=6.5)
for a in (a1, a2): [a.spines[s].set_visible(False) for s in ("top", "right")]
save(fig, "fig_funnel")

# ------------------------------------------------------------------ 6. cost structure (near-ground reference run)
import lzma, tempfile, sys as _sys
_sys.path.insert(0, os.path.join(ROOT, "tools"))
import plot_plan_costs as _ppc
with lzma.open(os.path.join(ROOT, "evidence", "reference_runs", "near-ground.log.xz"), "rt", errors="replace") as fh, \
     tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as tmp:
    tmp.write(fh.read()); _tmpname = tmp.name
plans, terms, commit, lead_commit = _ppc.parse(_tmpname); os.unlink(_tmpname)
best_by_h = {}
for pl in plans:
    if pl["h"] not in best_by_h or pl["jg"] < best_by_h[pl["h"]]["jg"]: best_by_h[pl["h"]] = pl
per_h = [best_by_h[h] for h in sorted(best_by_h)]
committed = min([pl for pl in plans if (pl["cand"], pl["route"]) == commit and abs(pl["lead"] - lead_commit) < 1e-6], key=lambda r: r["jg"])
argmin = min(plans, key=lambda r: r["jg"])
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.0, 2.5), gridspec_kw={"width_ratios": [1.15, 1]})
for lab, col, sel in [("direct", C_OBJ, lambda r: r == "direct"), ("ring 80 mm", C_OK, lambda r: r.startswith("ring80")), ("ring 140 mm", C_ROB, lambda r: r.startswith("ring140"))]:
    pts = [pl for pl in plans if sel(pl["route"])]; a1.scatter([pl["lead"] for pl in pts], [pl["jg"] for pl in pts], s=6, color=col, alpha=0.65, label=f"{lab} ({len(pts)})")
a1.plot([pl["lead"] for pl in per_h], [pl["jg"] for pl in per_h], "-", color="black", lw=0.7, label="best plan per event hypothesis")
a1.scatter([argmin["lead"]], [argmin["jg"]], s=50, facecolor="none", edgecolor=C_MUT, lw=1.0, label=f"argmin over certified plans ($h$ = {argmin['lead']:.2f} s)")
a1.scatter([committed["lead"]], [committed["jg"]], s=80, marker="*", color=C_BAD, zorder=5, label=f"committed after timing admission ($h$ = {committed['lead']:.2f} s)")
a1.set_xlabel("event lead $h$ (s)"); a1.set_ylabel("$J_{\\mathrm{global}}$"); a1.set_title(f"all {len(plans)} certified plans, near-ground reference run", fontsize=7.5)
a1.legend(fontsize=5.4, frameon=False, loc="upper right"); a1.grid(alpha=0.25)
keys = ["T", "E", "L", "C", "Q", "K", "V"]; cols = {"T": C_OBJ, "E": "#7f7f7f", "L": "#b5b5b5", "C": C_OK, "Q": C_ROB, "K": "#e0a458", "V": "#5c5c5c"}
xs = list(range(len(per_h))); bottoms = [0.0] * len(per_h)
for k in keys:
    vals = [_ppc.WEIGHTS[k] * terms.get((pl["cand"], pl["route"]), {}).get(k, 0.0) for pl in per_h]
    a2.bar(xs, vals, bottom=bottoms, color=cols[k], width=0.75, label=k); bottoms = [b + v for b, v in zip(bottoms, vals)]
sched = [pl["jg"] - pl["jm"] for pl in per_h]
a2.bar(xs, [max(v, 0) for v in sched], bottom=bottoms, color=C_BAD, width=0.75, label="schedule")
a2.bar(xs, [min(v, 0) for v in sched], bottom=[0.0] * len(per_h), color=C_BAD, alpha=0.4, width=0.75)
a2.axhline(0, color="black", lw=0.5); a2.set_xticks(xs); a2.set_xticklabels([f"{pl['lead']:.2f}" for pl in per_h], fontsize=6, rotation=60)
a2.set_xlabel("event lead $h$ of the hypothesis (s)"); a2.set_ylabel("weight × term"); a2.set_title("weighted terms of the best plan per hypothesis", fontsize=7.5)
a2.legend(fontsize=5.2, frameon=False, ncol=4, loc="upper left")
for a in (a1, a2): [a.spines[s].set_visible(False) for s in ("top", "right")]
save(fig, "fig_costs")

# ------------------------------------------------------------------ 7. latency sweep
rows = json.load(open(os.path.join(ROOT, "evidence", "latency", "sweep_rows.json")))
fig, ax = plt.subplots(figsize=(3.4, 2.3))
for comp, col, lab in [(True, C_OK, "compensated"), (False, C_BAD, "uncompensated")]:
    rr = sorted([r for r in rows if r["comp"] == comp and float(r["delay"]) > 0], key=lambda r: float(r["delay"]))
    rv = [r for r in rr if r.get("estimate_position_error_m") is not None]      # 0.60 s rows never produced an estimate
    ax.plot([float(r["delay"]) for r in rv], [1e3 * r["estimate_position_error_m"] for r in rv], "-", color=col, lw=1, label=lab)
    for r in rv:
        mk = "o" if r["completed"] else ("x" if r["committed"] else "s")
        ax.plot(float(r["delay"]), 1e3 * r["estimate_position_error_m"], mk, color=col, ms=5, mfc=(col if r["completed"] else "white"))
ax.plot([], [], "o", color=C_MUT, label="completed"); ax.plot([], [], "x", color=C_MUT, label="committed, execution failed"); ax.plot([], [], "s", color=C_MUT, mfc="white", label="rejected before commit")
ax.set_xlabel("configured perception delay (s)"); ax.set_ylabel("object-estimate position error (mm)"); ax.legend(fontsize=5.6, frameon=False); ax.grid(alpha=0.25)
[ax.spines[s].set_visible(False) for s in ("top", "right")]
save(fig, "fig_latency")

# ------------------------------------------------------------------ 8. selector modes
tab = {}
for line in open(os.path.join(ROOT, "supervisory_mode", "evidence", "phaseF_sim", "outcomes.md")):
    m = re.match(r"\| (near-ground|longitudinal|lateral-low|diagonal) \| (\w+) \| (\d)/(\d) \|", line)
    if m: tab.setdefault(m.group(2), {})[m.group(1)] = int(m.group(3))
order = [("reactive", "reactive (B0)"), ("predictive", "predictive (B1)"), ("predictive_capability", "predictive + capability tie-break (B2)"), ("full", "predictive + hard authority filter (FULL)")]
fig, ax = plt.subplots(figsize=(3.4, 2.3)); x = np.arange(4); w = 0.19
for i, (k, lab) in enumerate(order):
    vals = [tab[k][s] for s in scen]; ax.bar(x + (i - 1.5) * w, vals, w, label=f"{lab}: {sum(vals)}/12", color=[C_MUT, C_OK, C_ROB, C_BAD][i])
ax.set_xticks(x); ax.set_xticklabels(scen, fontsize=6.5); ax.set_ylabel("completions of 3"); ax.set_ylim(0, 3.6); ax.legend(fontsize=5.4, frameon=False, loc="upper left", ncol=1)
[ax.spines[s].set_visible(False) for s in ("top", "right")]
save(fig, "fig_modes")

# ------------------------------------------------------------------ 9. two-robot timeline
rows = list(csv.DictReader(open(os.path.join(ROOT, "two_robot", "results", "sim_2026-09-24", "timeline_20ms.csv"))))
t = np.array([float(r["t"]) for r in rows]); ph = np.array([int(r["dual_giver_phase"]) for r in rows]); sp = np.array([float(r["dual_giver_speed"]) for r in rows])
states = [r["Executor_Main"] for r in rows]; names = sorted(set(states), key=lambda s: states.index(s)); idx = np.array([names.index(s) for s in states])
fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.0, 2.6), sharex=True, gridspec_kw={"height_ratios": [1.2, 1]})
a1.step(t, idx, where="post", color=C_ROB, lw=1.2); a1.set_yticks(range(len(names))); a1.set_yticklabels(names, fontsize=6); a1.set_title("Robot A (receiver): FSM state", fontsize=7.5, loc="left")
phase_names = {1: "prepositioning", 2: "start settling", 3: "ready", 4: "executing", 5: "terminal settling", 6: "holding"}
a2.step(t, ph, where="post", color=C_HUM, lw=1.2); a2.set_yticks(sorted(set(ph))); a2.set_yticklabels([phase_names.get(p, str(p)) for p in sorted(set(ph))], fontsize=6)
a2b = a2.twinx(); a2b.plot(t, sp, color=C_MUT, lw=0.7); a2b.set_ylabel("giver tool speed (m/s)", fontsize=6.5, color=C_MUT); a2b.tick_params(labelsize=6)
a2.set_title("Robot B (giver): coordinator phase and tool speed", fontsize=7.5, loc="left"); a2.set_xlabel("time (s)")
for a in (a1, a2): a.grid(alpha=0.25, axis="x"); [a.spines[s].set_visible(False) for s in ("top", "right")]
save(fig, "fig_two_robot")
