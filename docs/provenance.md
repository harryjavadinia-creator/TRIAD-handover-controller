# Reproducibility and source attribution

This repository contains evidence produced on several distinct source states.
Each reported result should retain its source-state qualification so that
implementation, experiment and validation claims remain traceable.

## Source states

| # | Source state | Role | Results that belong to it |
| --- | --- | --- | --- |
| 1 | `e2e194d` — tag `dataset-a-baseline` | historical perception-latency matrix | Dataset A: 5 scenarios × 3 latency conditions |
| 2 | `c07368c` — tag `scientific-baseline` | frozen finite event-time/grasp/route campaign | Dataset B: four deterministic canonical winners and objective values |
| 3 | `82e6eaa` → public `a006912`, tag `csi-2026-release` | audited exact-serial implementation | serial wall-time study, collision-oracle comparison, historical four-scenario revalidation |
| 4 | **`f56add3`** | frozen background-planning implementation with corrected perception-latency configuration read | corrected nonzero-delay latency ablation and source synchronization |

The `f56add3` implementation is pinned by
[`source_sync_f56add3.sha256`](source_sync_f56add3.sha256).

## Background-planning evidence

The evidence tree also contains control-loop, plan-set, determinism and
planner-core records from the asynchronous development lineage. Compatibility
with the frozen `f56add3` source does **not** by itself establish that every
historical record was executed at `f56add3`.

Each record therefore retains its recorded origin. A result should be described
as measured at `f56add3` only when its own provenance establishes that source
state.

## Exact-serial and background-planning implementations

The background-planning implementation redistributes already-defined planning
work away from ordinary controller-cycle execution. The scientific decision
contract remains the same finite event-time/grasp/route method: prediction,
candidate-bank generation, hard feasibility, objective construction, timing
admission, deterministic tie handling, one-time commitment and no post-commit
global reselection.

Historical runtime timing traces retain their original source attribution.

## Source verification

| Manifest | Pins | Verify against |
| --- | --- | --- |
| [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256) | frozen background-planning implementation files | `f56add3` source state |
| [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256) | exact-serial implementation files | `csi-2026-release` tag |
| [`../SCIENTIFIC_BASELINE.sha256`](../SCIENTIFIC_BASELINE.sha256) | frozen Dataset-B source snapshot | `scientific-baseline` tag |

The exact-serial manifest is historical provenance and is not expected to
verify against the later background-planning source.

## Result-attribution rules

1. Keep each result attached to its source state.
2. Do not mix Dataset A and Dataset B numbers or source commits.
3. Distinguish logical controller planning time, external planner wall time and
   selector-time timing admission when those quantities are discussed.
4. State simulation versus physical hardware explicitly. No result in this
   repository is validated end-to-end on physical hardware.

## Retained configuration comments

The configuration file `etc/HandoverInterceptionController.in.yaml` is retained
byte-identical to the frozen scientific source, including its comments.

Two retained comments require a narrower interpretation than their wording
suggests:

| Location | Retained comment | Established interpretation |
| --- | --- | --- |
| `decisionCost` weight preamble | states that the seven weights were evaluated through exact finite-set weight-space sensitivity analysis | **No weight-space sensitivity result is reported.** The weights are fixed engineering preference values. |
| velocity-reserve note | implies terminal velocity utilisation separately gates terminal-timing audit success/failure | it does not gate that audit; its non-diagnostic role is a finiteness precondition on complete-plan cost |

Additional implementation and evidence qualifications are collected in
[`corrections_of_record.md`](corrections_of_record.md).
