# Experiment provenance and manifest

This repository's history contains evidence from more than one experiment
batch, on more than one commit. This document states, for each result,
exactly which commit produced it and how confident that attribution is —
do not assume every result was produced at the same commit.

Two datasets are reported:

- **Dataset A — July-19 perception-latency matrix**: source target
  `e2e194d22c11db9bc5af929da863b4823d7e668c` (`v6.5-real-grasp-contact-entry`).
  5 scenarios × 3 latency conditions (15 runs); 11 completed, 4 fail-safe
  failures.
- **Dataset B — frozen global finite time-grasp-route campaign**: source
  baseline `c07368ce19fca731c8a5cd4bd8e46bbd67f44771`. Four moving
  scenarios; exact finite argmin over the bounded event/grasp/route search.

These are different experiments with different purposes, different source
commits, and different numeric results. Do not merge their numbers, claims,
or source commits.

## Dataset B — frozen global finite time-grasp-route campaign

| Field | Value |
| --- | --- |
| Scenarios | near-ground, longitudinal, lateral-low, diagonal (see `docs/simulation.md`) |
| Experiment condition | `selectionMode: binding_cost`, `eventSelectionMode: global_time_plan`, `allowPhysicalExecution: false` |
| Source commit | `c07368ce19fca731c8a5cd4bd8e46bbd67f44771` |
| Source branch | `v6.7.1-cost-validity-fix` (also reachable as `release/csi-2026`'s ancestor) |
| Source config | `etc/HandoverInterceptionController.in.yaml` at that commit, per-scenario translation/velocity as in `docs/simulation.md` |
| Log artifacts | `GLOBAL_{CANONICAL_YZ,DIAGONAL_XZ,GROUND_NEAR,PURE_X}_rep{1,2,3}.log`, not included in this repository |
| Checker | `tools/check_global_time_plan_log.py` |
| Scenario labels | `GROUND_NEAR` = `near-ground`, `PURE_X` = `longitudinal`, `CANONICAL_YZ` = `lateral-low`, `DIAGONAL_XZ` = `diagonal` (the `scripts/run_scenario.sh` command names) |
| Attribution confidence | **Strong circumstantial evidence, not a self-declared record.** The controller does not log its own Git commit hash at runtime, so no run-time artifact directly states its source commit. The evidence for `c07368c`: (1) a same-day, ~16-minute-later build/run log (`v6_7_global.log`) reconstructs its `[CompletePlanCost]` J values *exactly* from the frozen seven-term weight vector documented at this commit, while an earlier same-week run (built before the freeze) reconstructs from a *different* weight vector applied to identical raw per-candidate terms — direct numeric evidence the frozen weights were in effect; (2) no commit exists after `c07368c` that could have produced this weight signature. This is weaker than a self-declared, hash-stamped `RUN_METADATA.txt` (see Dataset A for what that looks like when it does exist). |

### Scenario manifest

| Human name | Initial position `[x,y,z]` | Linear velocity `[vx,vy,vz]` | Expected selected event lead (s) | Checker |
| --- | --- | --- | --- | --- |
| Near-ground lateral motion | `[0.25, 0.62, 0.15]` | `[0.0, -0.08, 0.0]` | 4.60 | `tools/check_global_time_plan_log.py`, `tools/verify_scenario_identity.py` |
| Longitudinal motion | `[0.92, 0.00, 0.55]` | `[-0.08, 0.0, 0.0]` | 3.70 | same |
| Lateral low-height motion | `[0.55, -0.56, 0.15]` | `[0.0, 0.08, 0.0]` | 4.15 | same |
| Diagonal forward/upward motion | `[0.90, 0.00, 0.30]` | `[-0.0565685, 0.0, 0.0565685]` | 4.60 | same |

Selected event lead is deterministic and reproduces exactly (diagonal and
longitudinal both verified). See `docs/simulation.md` for the full
expected-output table and which quantities are exact versus expected to
vary slightly.

## Dataset A — July-19 perception-latency matrix

| Field | Value |
| --- | --- |
| Scenarios | static_nominal, canonical_yz, pure_x, diagonal_xz, ground_near (all 5 presets — the only dataset in this project's history covering all 5, including the stationary-object case) |
| Conditions | ideal, compensated220, uncompensated220 |
| Source commit | `e2e194d22c11db9bc5af929da863b4823d7e668c` |
| Source branch | `v6.5-real-grasp-contact-entry` |
| Run dates | 2026-07-19, 17:16:06–17:22:09 (all 15 runs, one continuous session) |
| Runner outcome | **11/15 `COMPLETED`, 4/15 `FAILURE`** (reconciled directly against each of the 15 individual `runner_status=` fields). Every `ideal` and `compensated220` run completed (10/10). Of the 5 `uncompensated220` runs, the 4 **moving-object** scenarios (canonical_yz, pure_x, diagonal_xz, ground_near) all failed; the `static_nominal` `uncompensated220` run **completed**. This is physically expected: `static_nominal` has zero object velocity, so a stale (uncompensated) perception timestamp still reports the same position — the ablation only has an effect when the object is actually moving. |

### Attribution evidence

Reconciled from all 15 individual run-metadata files (external to this
repository):

- Every one of the 15 runs has its own metadata file independently
  recording `controller_head=e2e194d22c11db9bc5af929da863b4823d7e668c` and
  `controller_branch=v6.5-real-grasp-contact-entry`, plus a SHA-256 of the
  installed controller configuration (differs per run, correctly, since
  scenario/condition differ) and of
  `HandoverInterceptionController_controller.so` (identical across all 15
  — expected, since all 15 ran in one session with no rebuild in between).
- **This does not, by itself, prove the specific source change was
  loaded.** The only source difference between `074f6ed` (see "Earlier
  single-scenario latency tooling" below) and `e2e194d` is in
  `src/states/HandoverInterceptionController_CaptureTransfer.cpp`, which
  compiles into a *separate* plugin library
  (`HandoverInterceptionController_CaptureTransfer.so`), not into
  `HandoverInterceptionController_controller.so`. No hash of the
  `CaptureTransfer.so` plugin specifically was recorded in any of the 15
  runs, so the identical main-library hash cannot distinguish `e2e194d`
  from `074f6ed` behavior. No such hash was reconstructed after the fact —
  doing so would require rebuilding historical source and asserting
  reproducible-build equivalence, which was not attempted.
- `e2e194d`'s change adds a runtime marker, `[AcquireContactEntry]`,
  logged only when a specific narrow condition is hit (strict pre-contact
  tolerance violated during `Closing`, before `stableTime_` starts
  accumulating). All 15 logs were searched for this marker: **absent in
  all 15**, including all 11 completed runs. Its presence would have
  proven the `e2e194d` code path executed; its absence proves nothing
  either way — it only means that narrow edge case was never hit.
- The run root, `handover_interception_controller_v6_4_3`, is the same
  worktree later found (on 2026-08-01, roughly two weeks after these runs)
  to have uncommitted local changes on top of a commit. That later
  observation does not retroactively prove the tree was dirty on
  2026-07-19; it is simply unknown for that date.

**Evidence classification: B — strongly attributed to an `e2e194d`-labelled
run state; exact clean-tree and changed-plugin identity not
cryptographically proven.** The 15 metadata files establish that the run
environment itself *declared* `controller_head=e2e194d...` and
`controller_branch=v6.5-real-grasp-contact-entry`, consistently, 15
independent times, with library/config hashes recorded alongside. It stops
short of exact proof because: (1) `CaptureTransfer.so`, the one plugin
that actually differs between `074f6ed` and `e2e194d`, was never
independently hashed; (2) the one runtime marker that could have
corroborated the change firing, `[AcquireContactEntry]`, did not fire in
any of the 15 runs; (3) clean-versus-dirty working-tree state on
2026-07-19 specifically is not cryptographically established. Do not
describe this dataset as "unresolved" (too weak) or as proof of `e2e194d`
behavior having executed (too strong) — classification B, exactly as
worded above.

**Do not confuse this with hardware or dual-robot evidence.** Some raw
material in the same historical evidence archive is organized under a
directory named for a "Robot-B" project, which is `CALLRobotBFaceToFaceMover`
— a genuinely different controller/repository, not `HandoverInterceptionController`.
Nothing in this repository's evidence depends on that project.

### Reproduction

The public release preserves the archived Dataset-A source state as the
`dataset-a-baseline` tag. The reproduction wrapper creates an isolated
worktree from that tag, so the private development commit graph is not
required for reproduction.

```bash
export MAIN_ROBOT_MODULE_PATH=/path/to/your/gen3_2f85/module/directory
scripts/reproduce_latency_matrix.sh --scenario pure_x --condition compensated220 \
  --mc-rtc-prefix /path/to/your/mc_rtc/install
# or, to generate configuration for all 15 combinations without building/running
# (no --mc-rtc-prefix needed for --dry-run):
scripts/reproduce_latency_matrix.sh --all --dry-run
# or, to run the full matrix:
scripts/reproduce_latency_matrix.sh --all --mc-rtc-prefix /path/to/your/mc_rtc/install
```

`--mc-rtc-prefix` (or the `MC_RTC_PREFIX` environment variable) is required
for any non-dry-run invocation — it is the same mc_rtc installation prefix
used to configure this repository itself (see `README.md`'s Build section).
There is no private-workspace default; the script fails clearly if neither
is set rather than guessing a local layout.

`scripts/reproduce_latency_matrix.sh` creates an isolated Git worktree at
`dataset-a-baseline` (`git worktree add --detach`, not a checkout of the current
worktree) and configures it (no build, no install yet). Because mc_rtc
controllers always install into one shared location regardless of source
commit, building this historical commit will overwrite the shared
installation `scripts/run_scenario.sh` depends on. Before that happens,
`tools/derive_cmake_install_paths.py` reads the exact set of install
destination paths directly from the historical build's own generated
`cmake_install.cmake` files (produced at configure time, before any install
runs) — a read-only derivation, not dependent on any prior build of this
release. The script then records the pre-existing state of every one of
those paths, whatever it is (specific bytes, or entirely absent — this
works identically whether or not this repository has ever been built
before). Only then does it build and install the historical source. After
the requested run(s), it restores every recorded path exactly: paths that
existed are restored byte-for-byte and SHA-256-verified; paths that were
absent are removed. This is file-level exact restoration, not a rebuild of
any commit — a parent directory newly created by the historical install can
remain (empty) afterward even though every file underneath it is restored
or removed exactly; this affects no file content and no scientific result.
Only individual FILE install entries are handled: a DIRECTORY-type CMake
install entry would make the derived path set incomplete, so
`tools/derive_cmake_install_paths.py` fails closed (nonzero exit) if it
encounters one rather than silently continuing — for `dataset-a-baseline`, none exist,
so this has been verified to change nothing. The scientific baseline source
itself (this repository's tracked files) is never modified. Cleanup runs
through a single idempotent path (`trap cleanup EXIT`, with `INT`/`TERM`
producing a plain exit so the `EXIT` trap still fires exactly once).
Scenario/condition selection uses the same isolated-`HOME` per-controller
config-override mechanism as `scripts/run_scenario.sh` (see
`docs/simulation.md`). `mc_rtc_ticker` is terminated by the wrapper itself
once a terminal outcome (completed handover or FSM failure state) is
logged, after a small fixed grace interval — no manual intervention is
required; whether the wrapper had to terminate the process or the process
had already exited on its own is recorded as `TICKER_STOP_REASON`.

Each run is verified at two independent levels, reported separately:
`tools/check_latency_log.py`'s raw controller metrics
(`LATENCY_METRICS_PASS`, read from its JSON output, not its process exit
status), and `tools/verify_latency_matrix_cell.py`'s comparison of the
observed completion/failure class against the historically expected class
for that scenario/condition cell (`REPRODUCTION_RESULT`). An expected
`FAILURE` (every `uncompensated220` cell except `static_nominal`) is a
successful reproduction of that cell: `REPRODUCTION_RESULT=PASS` even
though `LATENCY_METRICS_PASS=false` for those cells.

Scenario identity requires **both** the initial object position (`p0`, from
`[ObserveObject]`'s "unified static/moving observation started" line) and
the object's settled linear velocity (`v`, from the *last*
`[ObserveObject]` per-sample line, not the "`[PresentationSolve]` robot
stationary, object approaching..." line — that line is only emitted for a
moving object and would leave `static_nominal`, whose object velocity is
zero, unverifiable) to match the recovered historical table — position
alone cannot distinguish scenarios that start near the same point but move
differently. Condition identity requires **both** the
observed latency mode and the controller-reported configured delay to match
(`0.220s` for `compensated220`/`uncompensated220`; `0.000s` for `ideal`,
since perception-latency simulation is disabled) — mode alone does not catch
a run with the right mode but a mis-configured delay. A log containing both
a `[Completed]` and a Failure-state terminal marker is rejected as
`AMBIGUOUS` rather than resolved either way. All of `observed_position`,
`observed_velocity`, `configured_delay_s`, `scenario_position_ok`,
`scenario_velocity_ok`, `condition_mode_ok`, and `condition_delay_ok` are
reported individually in `reproduction_verdict.json` alongside
`REPRODUCTION_RESULT`.

**Verified** (against captured logs and synthetic fixtures, not a new
campaign): `tools/verify_latency_matrix_cell.py` was run against all 15 real
historical logs from the July-19 matrix itself (5 scenarios × 3 conditions,
each against its own expected outcome — `COMPLETED` for every cell except
the 4 moving-scenario `uncompensated220` cells, which expect `FAILURE`), and
correctly reported `REPRODUCTION_RESULT=PASS` for all 15, including
`static_nominal` (zero object velocity — the case that specifically
exercises the last-`[ObserveObject]`-sample velocity source rather than the
moving-object-only `[PresentationSolve]` line). `tools/
test_verify_latency_matrix_cell.py` (dependency-free, run as part of the
regression suite) additionally proves each identity check independently: a
correct completed cell and a correct expected-failure cell both pass; a
log with the right position but a wrong velocity, a log with the wrong
configured delay, a log with the wrong mode, and a log with both terminal
markers present are all rejected.


## Baseline integrity

`SCIENTIFIC_BASELINE.sha256` records the curated source snapshot stored at
the `scientific-baseline` tag.

Verify it with:

```bash
python3 tools/verify_scientific_baseline.py SCIENTIFIC_BASELINE.sha256 \
  --commit scientific-baseline
```
