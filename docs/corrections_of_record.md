# Publication corrections of record

This page records narrow technical qualifications for publication-facing text
while preserving the frozen scientific implementation and archived evidence.

## Copied-state planning

Most candidate kinematics are evaluated from one frozen copied robot state.
Two residual categories remain outside an absolute copied-state-purity claim:

- the gripper aperture used by corridor checks is derived from live
  fingertip-frame positions;
- joint position/velocity limits are read through live model accessors.

The planner-core guard does not cover those paths. Stable repeated hashes
therefore do not establish full copied-state purity or race freedom.

## Background-planning lifecycle

The finite TRIAD search runs on a background worker so the normal controller
cycle does not have to perform the full search directly. Ordinary result polling
is nonblocking.

Shutdown and reset paths can still cancel and `join()` the worker. The release
therefore does not claim a bounded join latency, WCET, hard-real-time guarantee
or formal schedulability proof.

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

These are distinct outcomes and should not be summarized as one common failure
mode for all large delays.

## Source-state attribution

The canonical source-state table is [`provenance.md`](provenance.md).

- Dataset A remains historical `e2e194d` evidence.
- Dataset B remains historical `c07368c` evidence.
- Exact-serial performance/revalidation material belongs to the
  `82e6eaa` / public `a006912` state.
- `f56add3` is the frozen asynchronous implementation and source state of the
  corrected nonzero-delay latency sweep.

Some background-planning timing/determinism/safety records were produced earlier
in the asynchronous development lineage. Source compatibility must not be
rewritten as execution at `f56add3` unless a specific record establishes that
provenance.

## Clean-clone validation scope

Historical runtime revalidation belongs to the exact-serial release. Later
current-release validation establishes source synchronization, dependency-free
checks, evidence integrity and clean configure/build. It does not silently
re-label historical four-scenario runtime evidence as a current-head rerun and
does not establish physical-robot validation.

Simulation, historical runtime evidence, current-source verification and
physical-robot validation are separate validation layers and should be reported
as such.
