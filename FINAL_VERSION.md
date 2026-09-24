# TRIAD — final version (branch `triad/final-2026-09-24`; two-robot extension on `triad/two-robot-2026-09-24`)

This branch is the last state of TRIAD. It is the union of the two lines that existed at the end:

- **Code** from `research/triad-scientific-repair` (ac3392a, 2026-09-17): the V1 controller, the V2 receding
  receiver with a robot-independent giver, the TRIAD-lite control-aware supervisor and predictive
  interception solver, and the repair-branch fixes (object-fixed grasp basis, immutable plan geometry,
  single-epoch snapshot, matched reactive/lookahead/predictive baselines with a shared grasp pool).
- **Documentation** from `main` (ebaedfa, 2026-09-21): the README that attributes the decision structure
  to prior art (Croft, Fenton & Benhabib 1998; Hujić et al. 1998; Akinola et al. 2021), reports the
  TRIAD-lite comparison (FULL 6/12 vs plain predictive 11/12) and claims no new decision method.

## What runs by default

`receiverArchitecture: v1_frozen_prereach` in `etc/HandoverInterceptionController.in.yaml`: the original
finite-bank controller. The V2 receiver is selected with `receiverArchitecture: v2_receding` and
`movingObject.giverTruthModel: independent_scripted`; TRIAD-lite with `supervisorMode: control_aware`.
Acquisition requires the object to be at rest (`requireObjectStopped: true`, 4 mm/s) in every variant.

## Verified on 2026-09-24

- Builds cleanly against mc_rtc (`~/mc_rtc_ws/install`): 27 targets, RelWithDebInfo, Ninja, 1 min 27 s.
- All 9 Python test scripts in `tools/` pass; the three C++ unit test binaries built by
  `tools/run_control_aware_supervisor_unit_tests.sh` pass (supervisor, grasp family, interception).
- The merge of `main` into the repair branch had no conflicts; only README and `triad_lite/` docs came
  from `main`.

## What this version does not claim

- No new decision method, no control-performance gap, no hardware run, no moving-object acquisition.
- The Phase 3–6 laws of V2 (timing, grasp, route, ranking) exist as offline studies only and are not
  implemented in the controller.
- The preregistered H1/H2 campaign of the repair branch was never launched; gate 4 (preview IK versus QP)
  is not passed; the matched reactive and lookahead baselines never committed in their smoke runs.

## Older versions, for the record

- `CALL-handover-controller` (GitHub, private), branch `release/csi-2026`, commit c07368c: the August 2026
  "CALL V6.7" state with the seven-term objective. Superseded; its README still tells the pre-audit story.
- Tag `csi-2026-release` (a006912, 2026-09-03): the first TRIAD publication package. Superseded; omits the
  prior-art attribution.
- `main` (ebaedfa): V1 code only, corrected docs. This branch supersedes it.

## Two-robot extension (branch `triad/two-robot-2026-09-24`, 2026-09-24)

The July 2026 two-arm setup (Robot A receiver, Robot B giver) ported onto this final code: `src/DualGiverCoordinator.*`,
two extra states, four hooks in the controller that are no-ops unless `dualHandover.enabled: true`, the configuration
overlay, the hardware procedure, the Kortex patch and the July evidence. Verified in simulation on 2026-09-24: the full
two-arm handover completes (see `two_robot/results/sim_2026-09-24/TIMELINE.md`); the single-robot scenario still
completes on the same build. Not verified on hardware with this code. Details: `two_robot/README.md`.

## Manuscript (24 September 2026)

`paper/triad_system_paper.tex` / `.pdf` (5 pages, IEEEtran) replaces the drafts of 3, 7 and 10 September. It makes no
novelty claim: the decision structure is attributed to Croft/Fenton/Benhabib 1998, Hujić 1998, Menon 2014, Salehian 2016,
Islam 2020, Akinola 2021 and Yang 2021/2022; it reports the four reference scenarios, the latency sweep, the exact-serial
performance, the matched TRIAD-lite comparison (6/12 vs 11/12), the two-robot setup (simulation completed 2026-09-24;
hardware status of July 2026 stated exactly) and the measured limits. The two 1998 bibliographic entries carry a
"to be verified" note. The 62/66-scenario campaigns of the earlier drafts are not reported (removed from main on
2026-09-10).

## Receiver-side hardware overlay

`two_robot/HandoverInterceptionController.hardware_receiver.yaml`: the switches and gripper calibration of the July 2026
hardware runs (`allowPhysicalExecution: true`, physical bridge on, `closePercent`/`maxPercent` 50.37 measured 16 July;
the repository default 35.0 is the 15 July value). `transfer.source` stays `virtual_sensor`: no physical force path was
ever validated. See `docs/real_robot.md` §4.
