# Source-state provenance

This repository contains evidence produced on several distinct source states.
Every quoted result should retain its source-state qualification.

## Release source states

| # | source state | role | results that belong to it |
| --- | --- | --- | --- |
| 1 | `e2e194d` — tag `dataset-a-baseline` | historical perception-latency matrix | Dataset A: 5 scenarios × 3 latency conditions |
| 2 | `c07368c` — tag `scientific-baseline` | frozen finite event-time/grasp/route campaign | Dataset B: four deterministic canonical winners and objective values |
| 3 | `82e6eaa` → public `a006912`, tag `csi-2026-release` | audited exact-serial implementation | serial wall-time study, collision-oracle comparison, historical four-scenario revalidation |
| 4 | **`f56add3`** | frozen asynchronous implementation with corrected perception-latency configuration read | corrected nonzero-delay latency ablation and current source synchronization |

State 4 is the scientific implementation published by the current release and
is pinned by [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256).

## Asynchronous evidence qualification

The reduced package also publishes asynchronous control-loop, plan-set,
determinism and planner-core records from the asynchronous development lineage.
Where the supplied checks establish compatibility with the frozen `f56add3`
source, that does **not** by itself prove that every historical record was
executed at `f56add3`.

Use each record's recorded origin and do not rewrite "compatible with the frozen
state" as "measured at `f56add3`" unless the specific record establishes it.

## What changed between exact-serial and asynchronous publication source

The asynchronous implementation redistributes already-defined planning work
away from ordinary controller-cycle execution. The scientific decision contract
remains the finite event-time/grasp/route method: prediction, candidate bank,
hard feasibility, objective, timing admission, tie convention, one-time commit,
and no post-commit global reselection.

No claim is made that historical runtime timing traces were generated at the
later source state.

## Manifests

| manifest | pins | verify against |
| --- | --- | --- |
| [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256) | implementation files published on this branch | this release |
| [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256) | exact-serial implementation files | `csi-2026-release` tag |
| [`../SCIENTIFIC_BASELINE.sha256`](../SCIENTIFIC_BASELINE.sha256) | frozen Dataset-B source snapshot | `scientific-baseline` tag |

The historical exact-serial manifest is retained as provenance; it is not
expected to verify against the asynchronous source.

## Rules for quoting results

1. Keep each result attached to its source state.
2. Do not mix Dataset A and Dataset B numbers or source commits.
3. Distinguish logical controller planning time, external planner wall time and
   selector-time timing admission.
4. State simulation versus physical hardware. No result in this release is
   validated end-to-end on physical hardware.

## Known documentation discrepancies inside frozen source

The configuration file `etc/HandoverInterceptionController.in.yaml` is
published byte-identical to the frozen scientific state and is therefore not
edited, including comments.

Two comments are superseded by the current publication documentation:

| location | frozen comment | established interpretation |
| --- | --- | --- |
| `decisionCost` weight preamble | states that the seven weights were evaluated through exact finite-set weight-space sensitivity analysis | **No weight-space sensitivity result is reported.** The weights are fixed engineering preference values. |
| velocity-reserve note | implies terminal velocity utilisation separately gates terminal-timing audit success/failure | it does not gate that audit; its non-diagnostic role is a finiteness precondition on complete-plan cost |

Other frozen-source/archive qualifications are collected in
[`corrections_of_record.md`](corrections_of_record.md).
