# Technical qualifications

This page records implementation and evidence qualifications that are important
for interpreting the TRIAD results and source provenance.

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

Shutdown and reset paths can still cancel and `join()` the worker. No bounded
join latency, WCET, hard-real-time guarantee or formal schedulability proof is
established.

## Corrected latency at 0.60 s

The latency report is stored under
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

The source-state table is [`provenance.md`](provenance.md).

- Dataset A is historical `e2e194d` evidence.
- Dataset B is historical `c07368c` evidence.
- Exact-serial performance/revalidation material belongs to the
  `82e6eaa` / public `a006912` state.
- `f56add3` is the frozen background-planning implementation and source state
  of the corrected nonzero-delay latency sweep.

## Validation attribution

Some background-planning timing/determinism/safety records were produced earlier
in the asynchronous development lineage; source compatibility does not by itself
establish that such a record was executed at `f56add3`. Historical runtime
revalidation belongs to the exact-serial source state.
Later validation establishes source synchronization, dependency-free checks,
evidence integrity and clean configure/build for the frozen background-planning
source. Historical runtime evidence is not re-labeled as a later-source rerun.

Simulation, historical runtime evidence, source verification and physical-robot
validation are separate validation layers and should be reported separately.
