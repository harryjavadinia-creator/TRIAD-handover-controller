# Source-state provenance

This repository's history contains evidence produced by several distinct
campaigns, on several distinct source states. **Every reported number belongs to
exactly one of the states below, and no number should be quoted without it.**

This page is the canonical provenance reference for the README, for
[`experiments.md`](experiments.md) and for
[`reproducibility.md`](reproducibility.md).

## The five source states

| # | source state | role | results that belong to it |
| --- | --- | --- | --- |
| 1 | `e2e194d` — tag `dataset-a-baseline` | historical perception-latency matrix | **Dataset A**: 5 scenarios × 3 latency conditions (15 runs) |
| 2 | `c07368c` — tag `scientific-baseline` | frozen finite event-time/grasp/route campaign | **Dataset B**: the four deterministic event/grasp/route winners and their objective values |
| 3 | `82e6eaa` → published as `a006912`, tag `csi-2026-release` | audited exact-serial implementation | the serial wall-time study; 8,168,732 collision-oracle comparisons with zero mismatches; the four-scenario revalidation |
| 4 | `90549ca` | asynchronous planner with commit-freshness evidence | the **62-scenario held-out generalization** campaign and the **66-perturbation robustness** campaign |
| 5 | **`f56add3`** — the frozen scientific state synchronized onto this branch | asynchronous planner + corrected perception-latency configuration read | the four-scenario asynchronous campaign, **control-loop timing**, the **corrected latency ablation**, repeated-run determinism, and the fault-injection safety evidence |

State 5 is the state whose implementation files are published on this branch and
pinned by [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256).

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
correction can therefore not have changed their results.

The 0.220 s cells of the earlier latency work are also unaffected, because the
configured value and the mirror's default coincide — verified directly, with
identical mode, measurement age, raw and compensated errors and frozen plan-set
hash before and after the correction.

## What changed between state 3 (published) and state 5 (this branch)

Six tracked files were modified and two were added. The complete configuration
delta is a single added key:

```yaml
routeWorkUnitsPerCycle: 128
```

a work-unit scheduling budget that redistributes already-defined route
certification work across control cycles. **No scientific parameter changed**:
the preview integration step, the event, grasp and route banks, hard
feasibility, the objective and its weights, the timing thresholds, the tie
rules and the one-commit / no-replanning semantics are identical to state 3.

## Manifests

| manifest | pins | verify against |
| --- | --- | --- |
| [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256) | the implementation files published on this branch | this branch — `sha256sum -c docs/source_sync_f56add3.sha256` |
| [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256) | the exact-serial implementation files of state 3 | **the `csi-2026-release` tag, not this branch.** Retained as the historical record of state 3; it does not verify against the current source and is no longer part of the automated checks. |
| [`../SCIENTIFIC_BASELINE.sha256`](../SCIENTIFIC_BASELINE.sha256) | the frozen Dataset-B source snapshot | the `scientific-baseline` tag, directly from Git blobs |

## Rules for quoting results

1. Name the source state for every reported number.
2. Do not mix Dataset A and Dataset B numbers, or their source commits.
3. Do not attribute a state-5 result to state 3, or the reverse.
4. State the timing metric explicitly — logical controller planning time,
   externally measured planner wall time and the counterfactual
   timing-admissibility boundary are three different quantities.
5. State simulation versus physical hardware. No result in this repository has
   been validated on physical hardware.

## Known documentation discrepancies inside the frozen configuration

The configuration file `etc/HandoverInterceptionController.in.yaml` is published
byte-identical to the frozen scientific state and is therefore **not edited**,
including its comments. Two of those comments do not match the behaviour of the
compiled source, and are corrected here rather than in the file:

| location | comment asserts | established behaviour |
| --- | --- | --- |
| `decisionCost` block, weight preamble | that the seven objective weights were *"evaluated through exact finite-set weight-space sensitivity analysis"* | **No weight-space sensitivity result exists in this repository and none is reported.** The weights are frozen engineering preference values. A weight-sensitivity study is future work. |
| `decisionCost` block, velocity-reserve note | that the terminal velocity utilisation *"continues to gate terminal-timing-audit success/failure separately"* | It does **not** gate that audit. It is computed *after* the terminal timing audit returns; its only non-diagnostic role is a finiteness precondition on the complete-plan cost. |

Where a comment and the compiled behaviour disagree, the behaviour is
authoritative and this page is the correction of record.
