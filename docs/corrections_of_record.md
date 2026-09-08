# Publication corrections of record

This page records narrow interpretation corrections for publication-facing text while preserving the frozen scientific implementation and the byte-identical archived evidence copies.

The authoritative scientific implementation remains the frozen asynchronous state synchronized by [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256). Primary evidence files under [`../evidence/`](../evidence/) are not rewritten to erase historical wording; where an archived report contains a superseded interpretation, the correction is stated here and in the surrounding public documentation.

## Copied-state planning

Most candidate kinematics are evaluated from one frozen copied robot state. Two residual categories remain outside an absolute copied-state-purity claim:

- the gripper aperture used by corridor checks is derived from live fingertip-frame positions;
- joint position/velocity limits are read through live model accessors.

The planner-core guard does not cover those paths. Stable repeated hashes therefore do not establish full copied-state purity or race freedom. The residual live-read issues remain documented and unfixed in the frozen source.

## Asynchronous worker lifecycle

Ordinary planning-result polling is nonblocking. The control path performs atomic result-state polling during normal cycles, but FSM teardown can call worker cancellation followed by `join()` through the planning-state teardown path. No bounded join latency, WCET, hard-real-time guarantee or formal schedulability result is reported.

## Near-ground 229 → 283 record relation

For the near-ground asynchronous comparison, the scientifically supported relation is:

- 229 prior records;
- 283 worker records;
- after removing the nonsemantic `sourceIndex` field, all 229 prior scientific payloads are retained and 54 payloads are added;
- full raw records and full set hashes are not identical.

The added payloads come from a hypothesis that was not enumerated in the historical control-thread run after an elapsed-time prune. This does not justify an unconditional statement that result receipt or admission always moves earlier across different runs.

## Robustness anchor H002

The primary robustness result is the full 66-case outcome set: 40 completed, 14 safely rejected and 12 execution failures after commitment. H002 contributes 8 of those 12 execution failures, but its 11-case family also contains 3 completions.

The H002-excluded numbers are retained only as a **secondary, post-hoc diagnostic**. They are not the primary robustness result and are not a pre-specified exclusion.

## Corrected latency at 0.60 s

The byte-identical archived latency report is retained under `evidence/latency/FINAL_LATENCY_REPORT.md`; the precise 0.60 s interpretation is:

- compensated 0.60 s: the selected result is rejected by the freshness gate before commitment;
- uncompensated 0.60 s: no finite search is run because motion classification is `AMBIGUOUS`; observed displacement is 0.0241 m against the 0.0250 m moving threshold while estimated linear speed is 0.0760 m/s, above the 0.0100 m/s static threshold;
- both compensated and uncompensated modes fail after commitment at 0.40 s and 0.50 s, not at every delay greater than or equal to 0.40 s.

The archived JSON fallback label for the uncompensated 0.60 s row is therefore not evidence that no physically feasible plan exists.

## Held-out timing rejections and offline reference

The 18 held-out timing rejections remain unresolved with respect to the true physical feasible domain. The offline reference finds relaxed endpoint witnesses using orientation-free IK and a gross joint-velocity lower-bound test. It does **not** certify a complete grasp/path/closure/retreat solution and does not prove sufficient full-path execution time.

Consequently the empirical commit and completion fractions are reported directly. They are **not** described as feasible-set coverage or as lower bounds on true feasible-domain coverage.

A richer independently implemented oracle would not be logically circular merely because it shares a declared physical model; no such oracle is claimed or required for the present publication package.

## Source-state attribution

The canonical source-state table remains [`provenance.md`](provenance.md). In particular:

- Dataset A remains historical `e2e194d` evidence;
- Dataset B remains historical `c07368c` evidence;
- the exact-serial performance/revalidation material belongs to the `82e6eaa` / public `a006912` state;
- the held-out generalization and robustness campaigns belong to `90549ca` and used ideal sensing;
- `f56add3` is the frozen asynchronous implementation and the source state of the corrected nonzero-delay latency sweep.

Some retained asynchronous timing/determinism/safety records were produced on the asynchronous development lineage before `f56add3`; source compatibility with the frozen implementation must not be rewritten as execution at `f56add3` unless a specific record establishes that provenance.

## Clean-clone validation scope

Historical runtime revalidation at `123be4a` belongs to the exact-serial release. Later current-branch validation establishes source synchronization, dependency-free checks, evidence integrity and clean configure/build. It does not silently re-label the historical four-scenario runtime campaign as a current-HEAD rerun, and it does not establish physical-robot validation.

## Published clean-machine asynchronous profiles

The current reduced evidence package contains these in-planning `ControllerRun` clean-machine profiles:

| scenario | planning cycles | median ms | p90 ms | p99 ms | max ms | >1 ms | >2 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| lateral-low | 3850 | 0.043 | 0.063 | 0.134 | 1.135 | 1 | 0 |
| near-ground | 3023 | 0.040 | 0.072 | 0.122 | 1.174 | 1 | 0 |
| longitudinal | 2792 | 0.067 | 0.084 | 0.122 | 1.090 | 1 | 0 |
| diagonal | 2124 | 0.018 | 0.040 | 0.049 | 1.206 | 1 | 0 |

The complete four-scenario **before** table survives as historical summary evidence; only one separate raw pre-thread control-loop profile is published. Do not claim that the clean-machine files reproduce a different after-profile table.
