# Source-state provenance

This repository's history contains evidence produced by several distinct
campaigns, on several distinct source states. **Every reported number belongs to
one identified run state or explicitly qualified historical summary, and no
number should be quoted without that provenance.**

This page is the canonical provenance reference for the README, for
[`experiments.md`](experiments.md), [`performance.md`](performance.md) and
[`reproducibility.md`](reproducibility.md).

## The five source states

| # | source state | role | results that belong to it |
| --- | --- | --- | --- |
| 1 | `e2e194d` — tag `dataset-a-baseline` | historical perception-latency matrix | **Dataset A**: 5 scenarios × 3 latency conditions (15 runs) |
| 2 | `c07368c` — tag `scientific-baseline` | frozen finite event-time/grasp/route campaign | **Dataset B**: the four deterministic event/grasp/route winners and their objective values |
| 3 | `82e6eaa` → published as `a006912`, tag `csi-2026-release` | audited exact-serial implementation | the serial wall-time study; 8,168,732 collision-oracle comparisons with zero mismatches; the historical four-scenario revalidation |
| 4 | `90549ca` | asynchronous planner with commit-freshness evidence | the **62-scenario held-out generalization** campaign and the **66-perturbation robustness** campaign |
| 5 | **`f56add3`** — frozen scientific implementation synchronized onto this branch | asynchronous planner + corrected perception-latency configuration read | the **corrected nonzero-delay latency ablation** and the frozen implementation against which current source synchronization is checked |

State 5 is the implementation published on this branch and pinned by
[`source_sync_f56add3.sha256`](source_sync_f56add3.sha256).

### Important qualification for asynchronous timing/determinism/safety evidence

The reduced package also publishes asynchronous control-loop, plan-set,
determinism and safety records from the asynchronous development lineage. Those
records are compatible with the frozen `f56add3` scientific implementation
where the published source/evidence checks establish compatibility, but a later
source-sync check does **not** by itself prove that every historical record was
executed at `f56add3`.

Use each record's origin and archived run state where available. Do not rewrite
"compatible with the frozen state" as "measured at `f56add3`" unless the
specific run record establishes that fact. See
[`corrections_of_record.md`](corrections_of_record.md).

## Why states 4 and 5 may be reported together

States 4 and 5 differ by a two-line correction on the perception path: the
delayed-measurement selection and the perception-buffer trim had been reading a
configuration mirror that still held its declared default of 0.220 s, so any
*other* configured delay was silently ignored.

**The generalization and robustness campaigns are unaffected by that
correction**, because every one of their 128 runs used ideal sensing. This is
recorded directly in the archived per-run records: each row of
`evidence/generalization/outcomes.json` and
`evidence/robustness/outcomes.json` carries

```
"mode": "IDEAL",  "configured_delay_s": 0.0,  "measurement_age_s": 0.0
```

so the delayed-measurement path was never entered in either campaign. The
correction therefore cannot have changed those 128 outcomes.

The 0.220 s cells of the earlier latency work are also unaffected because the
configured value and the mirror's default coincide; the before/after record at
0.220 s has matching mode, measurement age, raw/compensated error and frozen
plan-set hash.

## What changed between state 3 and state 5

Six tracked files were modified and two were added. The complete configuration
delta is a single added key:

```yaml
routeWorkUnitsPerCycle: 128
```

a work-unit scheduling budget that redistributes already-defined route
certification work across control cycles. **No scientific parameter changed**:
the preview integration step, the event, grasp and route banks, hard
feasibility, the objective and its weights, the timing thresholds, the tie
rules and the one-commit / no-replanning semantics are unchanged relative to
state 3.

This statement is about the scientific decision contract. It is not a claim
that historical runtime timing traces were generated at the later state.

## Manifests

| manifest | pins | verify against |
| --- | --- | --- |
| [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256) | implementation files published on this branch | this branch — `sha256sum -c docs/source_sync_f56add3.sha256` |
| [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256) | exact-serial implementation files of state 3 | **the `csi-2026-release` tag, not this branch** |
| [`../SCIENTIFIC_BASELINE.sha256`](../SCIENTIFIC_BASELINE.sha256) | frozen Dataset-B source snapshot | the `scientific-baseline` tag, directly from Git blobs |

The historical exact-serial source-sync manifest is retained as provenance; it
is not expected to verify against the asynchronous branch.

## Rules for quoting results

1. Name the source state or explicitly qualified historical-summary provenance for every reported number.
2. Do not mix Dataset A and Dataset B numbers, or their source commits.
3. Do not attribute a state-5 result to state 3, or a state-3 result to state 5.
4. Do not convert source compatibility with `f56add3` into a claim that an older run executed at `f56add3`.
5. State the timing metric explicitly — logical controller planning time, externally measured planner wall time and counterfactual timing-admissibility boundaries are different quantities.
6. State simulation versus physical hardware. No result in this repository has been validated end-to-end on physical hardware.

## Known documentation discrepancies inside frozen artefacts

The configuration file `etc/HandoverInterceptionController.in.yaml` is published
byte-identical to the frozen scientific state and is therefore **not edited**,
including its comments. Two comments do not match compiled behaviour and are
superseded here:

| location | comment asserts | established behaviour |
| --- | --- | --- |
| `decisionCost` block, weight preamble | the seven objective weights were *"evaluated through exact finite-set weight-space sensitivity analysis"* | **No weight-space sensitivity result exists in this repository and none is reported.** The weights are frozen engineering preference values. A sensitivity study is future work. |
| `decisionCost` block, velocity-reserve note | terminal velocity utilisation *"continues to gate terminal-timing-audit success/failure separately"* | It does **not** gate that audit. It is computed after the terminal timing audit returns; its only non-diagnostic role is a finiteness precondition on complete-plan cost. |

Other frozen-source/archive wording corrections — copied-state live reads,
worker joining, near-ground record normalization, H002 interpretation,
0.60 s latency interpretation and held-out feasibility wording — are collected
in [`corrections_of_record.md`](corrections_of_record.md). Where frozen comments
or archived prose disagree with established behaviour or the corrected
interpretation, the frozen bytes remain preserved and the downstream correction
of record is authoritative.
