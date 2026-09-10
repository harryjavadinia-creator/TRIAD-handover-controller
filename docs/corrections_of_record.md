# Publication corrections of record

This page records narrow interpretation corrections for publication-facing text
while preserving the frozen scientific implementation and archived evidence.

## Copied-state planning

Most candidate kinematics are evaluated from one frozen copied robot state.
Two residual categories remain outside an absolute copied-state-purity claim:

- the gripper aperture used by corridor checks is derived from live
  fingertip-frame positions;
- joint position/velocity limits are read through live model accessors.

The planner-core guard does not cover those paths. Stable repeated hashes
therefore do not establish full copied-state purity or race freedom.

## Asynchronous worker lifecycle

Ordinary planning-result polling is nonblocking. FSM teardown can call worker
cancellation followed by `join()` through the planning-state teardown path.
No bounded join latency, WCET, hard-real-time guarantee or formal
schedulability result is reported.

## Near-ground 229 → 283 record relation

For the near-ground asynchronous comparison:

- 229 prior records;
- 283 worker records;
- after removing the nonsemantic `sourceIndex` field, all 229 prior scientific
  payloads are retained and 54 payloads are added;
- full raw records and full set hashes are **not** identical.

The added payloads come from a hypothesis not enumerated in the historical
control-thread run after an elapsed-time prune.

## Corrected latency at 0.60 s

The archived latency report is retained under
`evidence/latency/FINAL_LATENCY_REPORT.md`.

- compensated 0.60 s: the selected result is rejected by the
  prediction-consistency gate before commitment;
- uncompensated 0.60 s: no finite search is run because motion classification
  is `AMBIGUOUS`; observed displacement is 0.0241 m against the 0.0250 m moving
  threshold while estimated linear speed is 0.0760 m/s, above the 0.0100 m/s
  static threshold;
- both compensated and uncompensated modes fail after commitment at 0.40 s and
  0.50 s.

## Source-state attribution

The canonical source-state table is [`provenance.md`](provenance.md).

- Dataset A remains historical `e2e194d` evidence.
- Dataset B remains historical `c07368c` evidence.
- Exact-serial performance/revalidation material belongs to the
  `82e6eaa` / public `a006912` state.
- `f56add3` is the frozen asynchronous implementation and source state of the
  corrected nonzero-delay latency sweep.

Some asynchronous timing/determinism/safety records were produced earlier in
the asynchronous development lineage; source compatibility must not be rewritten
as execution at `f56add3` unless a specific record establishes that provenance.

## Clean-clone validation scope

Historical runtime revalidation belongs to the exact-serial release. Later
current-release validation establishes source synchronization, dependency-free
checks, evidence integrity and clean configure/build. It does not silently
re-label historical four-scenario runtime evidence as a current-head rerun and
does not establish physical-robot validation.

## Published clean-machine asynchronous profiles

| scenario | planning cycles | median ms | p90 ms | p99 ms | max ms | >1 ms | >2 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| lateral-low | 3850 | 0.043 | 0.063 | 0.134 | 1.135 | 1 | 0 |
| near-ground | 3023 | 0.040 | 0.072 | 0.122 | 1.174 | 1 | 0 |
| longitudinal | 2792 | 0.067 | 0.084 | 0.122 | 1.090 | 1 | 0 |
| diagonal | 2124 | 0.018 | 0.040 | 0.049 | 1.206 | 1 | 0 |

The statement about cycles below 2 ms applies only to the in-planning
`ControllerRun` sample above. It is not a WCET guarantee.
