# Phase B: receiving-grasp family and cheap reachability funnel

Scope: the grasp front end only. Execution is unchanged, and the default is still `graspFamily: legacy_ring`.
Formulation: `interception_design_review.md` §4–5.
Tags: `CODE` / `DERIVED` / `LIT` / `NUM` (numerical choice) / `EXP` (measured here).

## 1. Grasp family (`src/ReceivingGraspFamily.h`)

**Handle.** Cylinder with centre p_H, axis ĥ, radius R_H = 11.25 mm and half-length L_H = 68.7 mm (`CODE`, planner config).
**Reference.** A robot-relative vector r, projected orthogonal to ĥ, gives e1 = r⊥ and e2 = ĥ × e1.

| Quantity | Equation | Provenance |
|---|---|---|
| closing direction | n(θ) = e1 cos θ + e2 sin θ | `DERIVED`: an antipodal parallel-jaw grasp on a cylinder closes along a diameter through the axis |
| contacts | c± = p_H + s ĥ ± R_H n(θ) | `DERIVED` |
| gripper frame | x_M = n(θ), z_M = σĥ, y_M = z_M × x_M | `DERIVED`: the pad line contact is parallel to the axis, so the roll ψ is fixed by (σ, θ) |
| capture / standoff / retreat | p_H + s ĥ + d·y_M, with d = 6 / 120 / 180 mm | `CODE` interface offsets |
| axial bound | \|s\| ≤ min(L_H − w_f − m, corridor 35 mm) = 35 mm | `DERIVED` (w_f = 12.5 mm finger half-width from the Robotiq tip lattice, m = 3 mm) |
| remaining DOF | (σ, θ, s); ψ is not free | `DERIVED` |
| centred-grasp task constraint | `axialMax: 0` gives s ≡ 0 (unit-tested) | explicit option |
| θ resolution floor | Δθ ≤ min(ε_p/ℓ, ε_R) with ε_p = 15 mm, ε_R = 0.12 rad, ℓ = 0.126 m, so N_θ = 53 per sign | `DERIVED` (resolution rule of the receding-mode characterisation, not part of this repository) |
| legacy (σ, φ) ring | σ = +1: θ = φ − π/2; σ = −1: θ = π/2 − φ | unit test: identical frames |

**Mechanical screen** (necessary conditions only): the axial bound, and every mouth centre above the ground plane.

## 2. Reachability surrogate (`src/Gen3WristReachabilityMap.h`, generated)

**Wrist point.** The Gen3 joint-7 origin sits at a fixed offset (0, 0, −0.061525) m in the tool frame (`CODE`, URDF).

**Reachable set.** Joint 1 is continuous about the base z axis, so the wrist-point reachable set is a solid of revolution. It is described exactly in (ρ, z).

**Construction** (`supervisory_mode/tools/build_gen3_wrist_reachability.py`):
- 4·10⁶ uniform samples of q2..q6 within URDF limits, seed 20260916;
- a 10 mm grid, binary closing, then a signed Euclidean distance transform;
- URDF SHA-256 `3a7728a6…`.

**Use.** The map gives a necessary condition only. It ignores orientation feasibility for joints 5–7, collisions, and the controller IK.

**False negatives** (`evidence/reachability_surrogate_characterization.txt` (not included in this repository), `EXP`). Truth is 400 k FK-generated tool poses with an independent seed; truth and surrogate come from the same kinematic model.

| prune threshold | false-negative rate |
|---|---|
| sdf < 0 | 0.0010 % |
| sdf < −10 mm | 0.0003 % |
| sdf < −20 mm | 0 |

The deepest miss was −12.9 mm. All far probes were rejected.

**Prune tolerance.** `pruneTolerance` = 20 mm, chosen above the deepest observed miss (`NUM`/`EXP`).

## 3. Funnel

G0 → G_mech → G_R (score ≥ −tol) → G_K (shortlist) → exact controller layers.

**Score.** min(sdf at standoff, sdf at capture).

**Shortlist.** Candidates are ordered by score. A diversity pass keeps one per (σ, axial index, θ-bin of 13.6° = 2 × resolution), then the rest fill by score. The Python mirror in `supervisory_mode/tools/funnel_characterization.py` is used for the analysis.

**Upstream comparison** (`LIT`, Akinola/Xu `dynamic_grasping_world.py`): SDF ranking followed by bounded exact IK. Their grasp database, `max_check`, back-offs and thresholds are not reused.

## 4. Characterization (`evidence/phaseB_sim` (not included in this repository), `EXP`)

**Setup.** 4 scenarios in `characterizeAll` mode, where every mechanical hypothesis is evaluated by the exact layers.
- 36 generations;
- 16 535 exact evaluations;
- 1 316 robot-feasible;
- 1 115 admissible.

**Pruning.** At thresholds 0, −20 mm and −50 mm, **no** pruned hypothesis was robot-feasible (1020 / 737 / 363 pruned).

**AUC.**

| Score | robot-feasible | admissible |
|---|---:|---:|
| wrist SDF | 0.827 | 0.838 |
| −reach distance | 0.674 | 0.689 |

**Recall@K.** Recall is the fraction of the 32 generations used for the shortlist statistics with at least one admissible grasp (the phase-B records do not state why 4 of the 36 characterized generations are excluded). "Clearance-best" means the shortlist retains an admissible grasp within the 5 mm clearance band of the full-family best.

| K | shortlist (runtime rule) | pure SDF order | −reach distance | random | clearance-best retained |
|---:|---:|---:|---:|---:|---:|
| 8 | 0.750 | 0.688 | 0.531 | 0.438 | 0.625 |
| 12 | 0.906 | 0.750 | 0.625 | 0.688 | 0.688 |
| 20 | 1.000 | 0.969 | 0.719 | 0.906 | 0.875 |
| 32 | 1.000 | 0.969 | 0.844 | 0.875 | 0.969 |
| 40 | 1.000 | 1.000 | 0.906 | 0.875 | **1.000** |

**Per-scenario minimum K** for clearance-best retention: diagonal 33, longitudinal 24, near-ground 17, lateral-low 15.

**Default K = 40.** This is the smallest tested K with full recall and full clearance-best retention across all characterized generations. It is an in-sample `NUM` choice, not a generalization claim: 4 scenarios and 32 generations. The diversity pass measurably helps: recall@12 is 0.906 with diversity vs 0.750 without.

**Latency.**
- Front end: median 0.27 ms, max 0.98 ms (530 hypotheses).
- Exact layers: median 1.2 ms per hypothesis, p90 3.3 ms.
- Measured at K = 40 (`evidence/phaseB_sim_k40` (not included in this repository)): exact-evaluation median 38–63 ms per selection job by scenario, max 136 ms.
- The legacy ring evaluates 64 hypotheses exactly with no funnel.

**Sampling-resolution convergence** (sub-families of the characterized 53 × 5 family; loss of best admissible clearance vs the full family):

| θ samples/sign | axial values | generations with admissible | median loss | worst loss |
|---:|---|---:|---:|---:|
| 53 | 5 | 32/32 | 0 | 0 |
| 53 | centred only | 32/32 | 0.11 mm | 28.0 mm |
| 26 | 5 | 32/32 | 0.04 mm | 20.4 mm |
| 13 | 5 | 32/32 | 0.12 mm | 39.6 mm |
| 6 | 5 | 31/32 | 0.89 mm | 38.6 mm |

**What the convergence data show.** Admissibility converges by N_θ ≈ 13, and the median clearance by N_θ ≈ 26. **The worst case has not converged** in either θ or s. A finer family (N_θ = 106) was not run, so 53 is the derived resolution floor, not a demonstrated convergence point. Axial sampling matters in some generations (a 28 mm worst-case loss when s ≡ 0).

## 5. In-situ receiving family with K = 40 (descriptive only; execution = the reactive supervisor's tracker (B0))

`evidence/phaseB_sim_k40/summary.txt`:
- lateral-low: acquisition admitted and frozen;
- longitudinal and diagonal: `control_aware_track_clearance_reserve`;
- near-ground: `unsafe_while_held`.

With K = 12 (`phaseB_sim`), 0/4 completed. These are single runs of the reactive tracker. They say nothing about H1 or H2.

## 6. Unsupported / approximate

- The surrogate has no orientation, self-collision or controller-IK information. It ranks candidates; it never admits them.
- K, the prune tolerance and the θ-bin are characterized on 4 scenarios. They are not validated out of sample.
- Worst-case θ and axial convergence is not demonstrated.
