# About TRIAD

TRIAD is a receiver-side controller for human-to-robot handover on a Kinova Gen3 with a Robotiq 2F-85,
implemented on mc_rtc. This repository holds the controller, its configuration, the evidence behind every
reported number, a robot-to-robot test setup, and the paper that describes it.

## What is in `src/`

- The finite-plan controller (`receiverArchitecture: v1_frozen_prereach`, the default): one frozen planning
  state, a bounded bank of complete plans over event time, grasp and route, modelled feasibility through
  acquisition and retreat, timing admission at selection, one commitment.
- The receding receiver (`receiverArchitecture: v2_receding`) with a robot-independent scripted giver.
- The supervisory mode (`supervisorMode: control_aware`): a sampled earliest-feasible rendezvous solver
  over a 530-hypothesis grasp family and a directional QP-authority filter.
- The two-robot giver coordinator and its two commissioning states, inactive unless `dualHandover.enabled: true`.

Acquisition requires the object to be at rest (`requireObjectStopped: true`, 4 mm/s) in every mode.

## Verified

- Builds against mc_rtc; the Python and C++ unit tests in `tools/` pass; the CI checks in
  `.github/workflows/source-checks.yml` pass.
- Four reference scenarios, a perception-latency sweep and an exact-serial performance measurement
  (`docs/results.md`, `docs/performance.md`).
- A matched comparison of the supervisory-mode selectors (`supervisory_mode/`): the plain predictive
  selector completes 11/12 scenarios, the authority supervisor used as a hard filter 6/12.
- The two-robot handover completes in simulation (`two_robot/results/sim_2026-09-24/TIMELINE.md`).

## Scope

- The decision structure builds on earliest-feasible rendezvous and grasp-funnel prior work, attributed
  in `docs/related_work.md` and in the paper.
- The scenario results are simulation; the physical work is the two-robot setup and its videos.
- Acquisition waits for the object to be at rest.

## Paper and hardware

- `paper/triad_system_paper.tex` / `.pdf`: the paper that matches this repository.
- `two_robot/`: the robot-to-robot setup, the receiver's hardware overlay
  (`HandoverInterceptionController.hardware_receiver.yaml`), the hardware procedure, and the July 2026
  log inventory. `docs/real_robot.md` states what has and has not been reached on hardware.
