# Reduced evidence package

This directory publishes the compact records currently used by the release.
Nothing in this directory is a physical-robot result.

Source-state provenance is given in
[`../docs/provenance.md`](../docs/provenance.md). Archived files are preserved
with their original bytes where possible; origins are recorded in
[`ORIGINS.txt`](ORIGINS.txt).

## Verify the package

```bash
cd evidence && sha256sum -c MANIFEST.sha256
python3 ../tools/check_evidence_manifest.py
```

## `async/` — asynchronous planner evidence

For each canonical moving-object scenario the package contains a frozen plan
set, hashes, worker timing, timing-frontier replay and a clean-machine
control-loop profile.

Three scenarios reproduce the historical control-thread plan-set hash exactly.
For near-ground, the supported relation is:

- 229 prior records;
- 283 worker records;
- after removing only the nonsemantic `sourceIndex` field, all 229 prior
  scientific payloads are retained and 54 payloads are added;
- full raw records and full set hashes are **not** identical.

The added payloads belong to a hypothesis that was not enumerated in the
historical control-thread run after an elapsed-time prune. See
[`../docs/performance.md`](../docs/performance.md) and
[`../docs/corrections_of_record.md`](../docs/corrections_of_record.md).

The published clean-machine in-planning `ControllerRun` values are:

| scenario | median ms | p90 ms | p99 ms | max ms | >1 ms | >2 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| lateral-low | 0.043 | 0.063 | 0.134 | 1.135 | 1 | 0 |
| near-ground | 0.040 | 0.072 | 0.122 | 1.174 | 1 | 0 |
| longitudinal | 0.067 | 0.084 | 0.122 | 1.090 | 1 | 0 |
| diagonal | 0.018 | 0.040 | 0.049 | 1.206 | 1 | 0 |

The statement that no cycle exceeds 2 ms is restricted to this in-planning
`ControllerRun` sample. Whole-run controller/global maxima are larger and are
documented in the per-scenario files. No hard-real-time/WCET claim is made.

## `latency/` — corrected perception-latency ablation

`sweep_rows.json` is the corrected sweep. The pre-correction sweep is invalid
for nondefault delays because a configuration-mirror defect pinned the
perception path to 0.220 s; it is not used as current evidence.

At 0.30 s the repeated result remains 4/4 compensated completions versus
0/4 uncompensated completions, with all four uncompensated runs committing
before execution failure.

At 0.60 s:

- compensated: the selected result is rejected by the prediction-consistency
  gate before commitment;
- uncompensated: no finite search is run because observation classification is
  `AMBIGUOUS`; observed displacement is **0.0241 m** against the **0.0250 m**
  moving threshold while estimated linear speed is **0.0760 m/s**, above the
  **0.0100 m/s** static threshold.

Both modes fail after commitment at 0.40 s and 0.50 s.

## `safety/` — planner-core guard

The repository publishes the static planner-core guard output and its mutation
record. The guard is useful but not exhaustive.

Most candidate kinematics use the frozen copied state. Residual live
fingertip-frame reads determine gripper aperture, and joint-limit accessors
remain live. These paths are outside the guard's coverage. Stable hashes do not
establish full copied-state purity or race freedom.

See [`../docs/architecture.md`](../docs/architecture.md) and
[`../docs/corrections_of_record.md`](../docs/corrections_of_record.md).
