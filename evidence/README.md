# Reduced evidence package

This directory publishes the primary records needed to check the repository's
headline scientific claims. It is a curated subset of a larger archive.

Source-state provenance for every campaign below is given in
[`../docs/provenance.md`](../docs/provenance.md). Nothing in this directory is
a physical-robot result.

Most files below are byte-identical to archived originals and their origins are
recorded in [`ORIGINS.txt`](ORIGINS.txt). Two publication-facing exceptions are
explicitly identified:

1. `robustness/generate_perturbations.py` is a published variant of the
   campaign script in which two hard-coded archive paths were replaced by
   command-line arguments. Its anchor rule and perturbation magnitudes are
   unchanged and it reproduces `perturbations.csv` byte-for-byte.
2. This `README.md` is publication-authored interpretation/documentation and is
   intentionally not an archived experiment artefact.

Where a frozen archived report contains wording superseded by later audit,
the original bytes are preserved and the correction of record is stated in
[`../docs/corrections_of_record.md`](../docs/corrections_of_record.md).

## Verify the package

```bash
cd evidence && sha256sum -c MANIFEST.sha256
python3 ../tools/check_evidence_manifest.py
```

## `generalization/` — 62 predeclared held-out scenarios (source state 4)

The scenario set was generated from a fixed seed and **hashed before any
scenario was executed**; `held_out_scenarios.csv.sha256` is that
pre-registration record. The four development scenarios are excluded by
construction. All runs used ideal sensing.

| outcome | n | share |
| --- | ---: | ---: |
| A — plan found and completed | 36 | 58.1 % |
| B — committed, then execution failed | 2 | 3.2 % |
| C — no physically feasible TRIAD plan | 3 | 4.8 % |
| D — no timing-admissible TRIAD plan | 18 | 29.0 % |
| E — commit freshness rejected | 3 | 4.8 % |
| **total** | **62** | |

Committed **38/62**; of those, **36/38 = 94.7 %** completed.

Completion by object height band:

| band z [m] | completed | n | share |
| --- | ---: | ---: | ---: |
| 0.18-0.25 | 4 | 11 | 36 % |
| 0.30-0.40 | 25 | 42 | 60 % |
| 0.45-0.55 | 7 | 9 | 78 % |

Among the completions the planner selected 9 distinct event hypotheses,
11 distinct grasps, 12 distinct routes and produced 36 distinct frozen plan
sets.

### How these outcomes must be read

Outcomes C, D and E are **fail-closed rejections in which the robot never
moves**; they are not execution failures. Only B is a failure after commitment.

**The 18 category-D outcomes remain unresolved with respect to the true
physical feasible domain.** Every one produced TRIAD complete-plan records, but
none of those records satisfied the final timing-admission rule when the
completed search result was received.

The independent offline analysis under `feasibility_reference/` is deliberately
more permissive than TRIAD: orientation-free endpoint IK, more sampled leads,
multiple restarts, no grasp-corridor/closure certification and only a necessary
gross joint-velocity lower-bound test. It finds relaxed endpoint witnesses for
all 62 scenarios. That excludes gross handle-position unreachability and a gross
joint-speed explanation under the reference assumptions, but it **does not
certify a complete grasp/path/closure/retreat solution and does not prove
sufficient full-path execution time**.

Accordingly, the repository reports the empirical commit/completion rates
directly. It does **not** call them feasible-set coverage and does not interpret
them as lower bounds on true feasible-domain coverage.

> **Supersession notice.** An earlier internal report said the arm could not
> reach the 18 rejected events in time. That interpretation was withdrawn. What
> is established is only that TRIAD generated complete plans for those cases and
> none were timing-admissible at result receipt.

The monotone height trend is an **empirical association**. Any causal
explanation involving the fixed finite grasp/route bank is an interpretation,
not proof.

## `robustness/` — 66 predeclared local perturbations (source state 4)

Six anchors were selected by a rule fixed **before any perturbation ran**,
consulting scenario geometry rather than perturbation outcomes
(`generalization/ROBUSTNESS_ANCHOR_RULE.md`). The perturbation set was hashed
before execution.

| outcome | all 66 | excluding H002 family |
| --- | ---: | ---: |
| completed | 40/66 (60.6 %) | 37/55 (67.3 %) |
| safely rejected | 14/66 (21.2 %) | 14/55 (25.5 %) |
| execution failure after commitment | 12/66 (18.2 %) | 4/55 (7.3 %) |
| completed given commitment | 40/52 (76.9 %) | 37/41 (90.2 %) |

**The left-hand column is the primary result.** The H002 family contains
11 perturbations: **3 complete and 8 fail**. Its unperturbed anchor lies close to
the configured 8 mm dynamic-clearance reserve, which helps explain why this
family contributes many failures, but it is not universally failing.

The right-hand column is a **secondary, post-hoc diagnostic** that removes the
entire H002 family after observing the full results. It is not a pre-specified
exclusion and must not replace the 66-case result.

The perturbation magnitudes (±20 and ±40 mm in position, ±0.02 m/s in speed,
±10° in direction) are experimental values, not system specifications. All
12 post-commit execution failures entered the fail-safe hold.

## `latency/` — corrected perception-latency ablation (source state 5)

`sweep_rows.json` is the corrected sweep. The pre-correction sweep is invalid
for nondefault delays because a configuration-mirror defect pinned the
perception path to 0.220 s; it is deliberately not published as current
evidence.

The unit relation between uncompensated position error and `v·tau` is a direct
consequence of the simulated straight constant-velocity motion, not an
empirical fit.

The byte-identical archived `FINAL_LATENCY_REPORT.md` is preserved, but its
0.60 s prose is superseded by the precise interpretation below:

- compensated 0.60 s: the result is rejected by the freshness gate before
  commitment;
- uncompensated 0.60 s: no finite search is run because observation
  classification is `AMBIGUOUS`; observed displacement is **0.0241 m** against
  the **0.0250 m** moving threshold while estimated linear speed is
  **0.0760 m/s**, above the **0.0100 m/s** static threshold;
- both modes fail after commitment at **0.40 s and 0.50 s**, not at every delay
  greater than or equal to 0.40 s.

At 0.30 s the repeated result remains 4/4 compensated completions versus
0/4 uncompensated completions, with all four uncompensated runs committing
before execution failure.

## `async/` — asynchronous control-loop and plan-set evidence

For each canonical scenario the package contains a frozen plan set, hashes,
worker timing, timing-frontier replay and a clean-machine control-loop profile.

Three scenarios reproduce the historical control-thread plan-set hash exactly.
For near-ground, the supported relation is:

- 229 prior records;
- 283 worker records;
- after removing only the nonsemantic `sourceIndex` field, all 229 prior
  scientific payloads are retained and 54 payloads are added;
- full raw records and full set hashes are **not** identical.

The added payloads belong to a hypothesis not enumerated in the historical
control-thread run after an elapsed-time prune. See
[`../docs/performance.md`](../docs/performance.md) and
[`../docs/corrections_of_record.md`](../docs/corrections_of_record.md).

The published clean-machine in-planning `ControllerRun` values are:

| scenario | median ms | p90 ms | p99 ms | max ms | >1 ms | >2 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| lateral-low | 0.043 | 0.063 | 0.134 | 1.135 | 1 | 0 |
| near-ground | 0.040 | 0.072 | 0.122 | 1.174 | 1 | 0 |
| longitudinal | 0.067 | 0.084 | 0.122 | 1.090 | 1 | 0 |
| diagonal | 0.018 | 0.040 | 0.049 | 1.206 | 1 | 0 |

These values are the public primary basis for the after-profile. A complete
four-scenario before table survives only as historical summary evidence; one
separate raw pre-thread reference profile is published as
`async/perf_analysis_control_thread_reference.txt`.

The observation that no cycle exceeds 2 ms is restricted to the in-planning
`ControllerRun` sample above. Whole-run controller/global maxima are larger and
are documented in the per-scenario files. No hard-real-time/WCET claim is made.

## `feasibility_reference/` — independent offline analysis

A deliberately permissive reference analysis built from the same robot/object
model ingredients but not from TRIAD's finite bank. Read `README_METHOD.md`
before using any number from it.

Its 62/62 relaxed result is **not ground-truth feasibility** and does not
measure TRIAD's feasible-space coverage. The archived `README_METHOD.md` is kept
byte-identical; any statement there implying that a richer independent oracle
would necessarily be circular is superseded by
`docs/corrections_of_record.md`.

## `safety/` — planner-core guard

The repository publishes the static planner-core guard output and its mutation
record. The guard is useful but not exhaustive.

Most candidate kinematics use the frozen copied state. Residual live
fingertip-frame reads determine gripper aperture, and joint-limit accessors
remain live. These paths are outside the guard's coverage. Stable hashes do not
establish full copied-state purity or race freedom; the residual issues remain
unfixed in the frozen scientific source.

See [`../docs/architecture.md`](../docs/architecture.md) and
[`../docs/corrections_of_record.md`](../docs/corrections_of_record.md).
