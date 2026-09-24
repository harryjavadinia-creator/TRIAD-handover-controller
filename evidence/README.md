# Evidence index

This directory contains the evidence records and integrity manifests retained
with the TRIAD repository. Nothing in this directory is an end-to-end
physical-robot result.

Source-state attribution is documented in
[`../docs/provenance.md`](../docs/provenance.md). File origins are recorded in
[`ORIGINS.txt`](ORIGINS.txt).

## Verify the evidence

```bash
cd evidence && sha256sum -c MANIFEST.sha256
python3 ../tools/check_evidence_manifest.py
```

## `latency/` — corrected perception-latency ablation

`sweep_rows.json` contains the corrected delay sweep. The pre-correction sweep
is invalid for nondefault delays because a configuration-mirror defect pinned
the perception path to 0.220 s.

At 0.30 s the repeated result is 4/4 compensated completions versus 0/4
uncompensated completions, with all four uncompensated runs committing before
execution failure.

At 0.60 s:

- compensated: the selected result is rejected by the prediction-consistency
  gate before commitment;
- uncompensated: no finite search is run because observation classification is
  `AMBIGUOUS`; observed displacement is **0.0241 m** against the **0.0250 m**
  moving threshold while estimated linear speed is **0.0760 m/s**, above the
  **0.0100 m/s** static threshold.

Both modes fail after commitment at 0.40 s and 0.50 s. See
[`../docs/results.md`](../docs/results.md) and
[`../docs/corrections_of_record.md`](../docs/corrections_of_record.md).

## `safety/` — planner-core guard

This directory contains the static planner-core guard output and its mutation
record. The guard is useful but not exhaustive.

Most candidate kinematics use the frozen copied state. Residual live
fingertip-frame reads determine gripper aperture, and joint-limit accessors
remain live. These paths are outside the guard's coverage. Stable hashes do not
establish full copied-state purity or race freedom.

See [`../docs/architecture.md`](../docs/architecture.md) and
[`../docs/corrections_of_record.md`](../docs/corrections_of_record.md).

## `async/` — background-planning implementation records

This directory retains implementation-level records from the background-planning
development lineage, including plan-set, timing, replay, and control-loop files.
Their source attribution is described in [`../docs/provenance.md`](../docs/provenance.md).

These records are retained for traceability and implementation analysis. Timing
measurements are machine- and source-state dependent and do not establish a
WCET bound, hard-real-time guarantee, or formal schedulability result.

## `reference_runs/` — the four scenarios run from a fresh clone

For each scenario (`longitudinal`, `near-ground`, `lateral-low`, `diagonal`): the full controller log
(`<scenario>.log.xz`, every certified plan with its seven objective terms, the timing admission, the
commitment and the execution), the scenario override that produced it and the checker output. They were
produced by `scripts/run_scenario.sh` from the build tree without a viewer; `xz -d` restores the log.
`tools/plot_plan_costs.py <log>` draws `docs/figures/plan_costs_<scenario>.png` from it. Your own run
writes the same log under `results/<run>/`.
