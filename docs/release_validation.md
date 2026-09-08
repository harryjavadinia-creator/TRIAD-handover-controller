# Publication release validation

This page separates **historical exact-serial runtime revalidation** from the
later **asynchronous publication-branch source/build/check validation**. They are
different evidence and must not be silently relabelled as one another.

## Historical exact-serial synchronization and runtime revalidation

The four-scenario Dataset-B rerun described below belongs to the historical
exact-serial publication state:

- synchronization date: **2026-09-02**;
- synchronization commit: `123be4a`;
- imported development source: `82e6eaa`;
- later public release anchor: `a006912` / tag `csi-2026-release`;
- historical source-sync manifest: [`source_sync_82e6eaa.sha256`](source_sync_82e6eaa.sha256);
- Dataset-B scientific provenance remains anchored to `scientific-baseline` / `c07368c` and `SCIENTIFIC_BASELINE.sha256`.

At `123be4a`, the exact-serial source synchronization was followed by dependency
checks, a clean configure/build and four Dataset-B simulation reruns. This is
**historical runtime validation for that state**, not current asynchronous-head
runtime validation.

### Source and build checks at `123be4a`

| Check | Result |
| --- | --- |
| finite-plan selector tests | PASS |
| finite event-time selector tests | PASS |
| binding-cost source contract | PASS |
| binding-cost log-checker tests | PASS |
| global time-plan log-checker tests | PASS |
| robot-module reconstruction tests | PASS |
| latency-matrix verifier tests | PASS |
| scenario-identity verifier tests | PASS |
| scenario-override tests | PASS |
| frozen scientific baseline | 35/35 blobs verified |
| `git diff --check` | PASS |
| clean configure/build | PASS |

GitHub Actions `source-checks` also passed for the historical synchronized
publication state.

### Four-scenario Dataset-B revalidation at `123be4a`

Each historical rerun satisfied:

```text
HANDOVER_COMPLETED=true
RUNTIME_CHECKER_RESULT=PASS
SCENARIO_IDENTITY_RESULT=PASS
```

The deterministic winners were:

| Scenario | Event lead (s) | Grasp | Route | `J_global` |
| --- | ---: | --- | --- | ---: |
| near-ground / GROUND_NEAR | 4.600 | `axisP_side_45deg` | `ring80mm_0of8` | 0.822892544 |
| longitudinal / PURE_X | 3.700 | `axisP_side_337deg` | `direct` | 0.686806299 |
| lateral-low / CANONICAL_YZ | 4.150 | `axisN_side_337deg` | `direct` | 0.700830630 |
| diagonal / DIAGONAL_XZ | 4.600 | `axisP_side_337deg` | `ring140mm_2of8` | 0.684634405 |

These matched the frozen Dataset-B winner fingerprints for the historical
revalidation state.

## Current asynchronous publication branch

The current publication branch later synchronized the frozen asynchronous
scientific implementation at `f56add3`. That synchronization **does modify
controller/FSM/configuration files relative to `123be4a`**. Therefore the old
statement that `git diff --name-only 123be4a..HEAD -- src etc configs
call_object_description` lists no files is no longer true and is superseded by
this page.

The current branch is checked instead by:

- [`source_sync_f56add3.sha256`](source_sync_f56add3.sha256), which pins the
  synchronized frozen implementation files;
- the frozen scientific-baseline verifier;
- dependency-free selector/replay/identity/setup tests;
- the reduced-evidence SHA-256 manifest and consistency checker;
- repository-local Markdown/link and syntax checks;
- clean configure/build validation performed during the publication pass.

These establish **source synchronization, repository consistency, evidence
integrity and buildability** for the asynchronous publication branch. They do
not convert the historical `123be4a` four-scenario reruns into current-HEAD
runtime reruns.

The reduced asynchronous evidence under `evidence/async/` consists of archived
runtime records from the asynchronous development lineage. Their record-specific
provenance and compatibility with the frozen published source are described in
[`provenance.md`](provenance.md) and
[`corrections_of_record.md`](corrections_of_record.md). Do not infer that every
archived record was executed at `f56add3` merely because the current source-sync
manifest verifies against `f56add3`.

## What is and is not validated at the current branch

### Established for the publication branch

- current implementation content is synchronized to the frozen `f56add3` source
  according to the source-sync manifest;
- frozen Dataset-B baseline blobs remain verifiable;
- the reduced evidence package is integrity-checkable and its headline counts
  are recomputable from published primary records;
- dependency-free source/checker tests are represented in GitHub Actions;
- publication-pass clean configure/build validation has been recorded.

### Not established by this page

- a new four-scenario current-HEAD simulation rerun;
- cross-machine exact wall-time reproduction;
- hard-real-time/WCET guarantees;
- end-to-end physical-robot validation;
- human-subject validation.

A future current-HEAD runtime campaign, if performed, must be recorded as a new
campaign with its own source/config/runtime provenance rather than being inferred
from historical validation.

Generated run directories remain excluded by `.gitignore`; the repository
publishes reproducible commands, expected historical fingerprints, current
source/evidence manifests and checker implementations rather than machine-
specific temporary run directories.
