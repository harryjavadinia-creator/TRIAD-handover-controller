# July 2026 two-robot sessions — mc_rtc log inventory

Logs written by mc_rtc on the laptop that drove both arms (LogTemplate CALL_DUAL_ACTUAL / CALL_DUAL_D0), 17–19 July 2026.
They stay in `~/Downloads` on that laptop; only the 3.3 MB log of 17 July 22:27:33 is copied into this folder.
`hw` = the log contains `kinova_JointSensor_*_motorCurrent`, i.e. the second arm was a physical Kortex robot.
State sequences were extracted with `mc_bin_utils convert --entries Executor_Main dual_giver_phase`.

| date time | controller | size | notes |
|---|---|---|---|
| 2026-07-17 17:26 | dual (A + B) | 490 MB |  |
| 2026-07-17 17:26 | dual (A + B) | 93 MB |  |
| 2026-07-17 17:28 | dual (A + B) | 210 MB |  |
| 2026-07-17 17:29 | dual (A + B) | 184 MB |  |
| 2026-07-17 22:21 | dual (A + B) | 719 MB |  |
| 2026-07-17 22:24 | dual (A + B) | 81 MB |  |
| 2026-07-17 22:26 | dual (A + B) | 16 MB |  |
| 2026-07-17 22:27 | dual (A + B) | 42 MB |  |
| 2026-07-17 22:27 | dual (A + B) | 3 MB |  |
| 2026-07-17 22:27 | dual (A + B) | 11 MB |  |
| 2026-07-17 22:30 | dual (A + B) | 65 MB | hw. RobotBScenarioPreview: Robot B executed the presentation alone on hardware, giver phases 1→6 (Prepositioning, StartSettling, Ready, Executing, TerminalSettling, Holding). Robot A held. |
| 2026-07-17 22:31 | dual (A + B) | 63 MB |  |
| 2026-07-17 22:31 | dual (A + B) | 36 MB |  |
| 2026-07-17 22:32 | dual (A + B) | 46 MB |  |
| 2026-07-17 22:32 | dual (A + B) | 46 MB |  |
| 2026-07-17 22:33 | dual (A + B) | 33 MB |  |
| 2026-07-17 22:51 | dual (A + B) | 57 MB |  |
| 2026-07-17 22:59 | dual (A + B) | 47 MB |  |
| 2026-07-17 23:17 | dual (A + B) | 116 MB | hw. Combined run: Initial → ObserveObject → Failure; giver phase 7 (failed). |
| 2026-07-17 23:23 | dual (A + B) | 52 MB |  |
| 2026-07-17 23:26 | dual (A + B) | 42 MB |  |
| 2026-07-17 23:34 | dual (A + B) | 98 MB |  |
| 2026-07-17 23:36 | dual (A + B) | 24 MB |  |
| 2026-07-18 00:16 | dual (A + B) | 11.5 GB |  |
| 2026-07-18 00:19 | dual (A + B) | 117 MB |  |
| 2026-07-18 00:20 | dual (A + B) | 108 MB |  |
| 2026-07-18 00:25 | dual (A + B) | 168 MB |  |
| 2026-07-18 00:25 | dual (A + B) | 124 MB |  |
| 2026-07-18 00:26 | dual (A + B) | 71 MB |  |
| 2026-07-18 00:27 | dual (A + B) | 95 MB |  |
| 2026-07-18 00:33 | dual (A + B) | 70 MB |  |
| 2026-07-18 00:33 | dual (A + B) | 77 MB |  |
| 2026-07-18 00:39 | dual (A + B) | 82 MB |  |
| 2026-07-18 00:39 | dual (A + B) | 77 MB |  |
| 2026-07-18 00:43 | dual (A + B) | 94 MB |  |
| 2026-07-18 00:45 | dual (A + B) | 97 MB |  |
| 2026-07-18 00:45 | dual (A + B) | 81 MB |  |
| 2026-07-18 00:46 | dual (A + B) | 72 MB |  |
| 2026-07-18 00:46 | dual (A + B) | 65 MB |  |
| 2026-07-18 00:51 | dual (A + B) | 77 MB | hw. Combined run: Initial → ObserveObject → SolveInterception → ExecuteCommittedReach → Failure; giver phases 1→4 then 7. The furthest hardware run. |
| 2026-07-18 00:56 | dual (A + B) | 72 MB |  |
| 2026-07-18 00:57 | dual (A + B) | 74 MB |  |
| 2026-07-18 00:58 | dual (A + B) | 76 MB |  |
| 2026-07-18 01:03 | dual (A + B) | 89 MB |  |
| 2026-07-18 01:04 | dual (A + B) | 70 MB |  |
| 2026-07-18 01:04 | dual (A + B) | 93 MB |  |
| 2026-07-18 01:05 | dual (A + B) | 69 MB |  |
| 2026-07-18 01:19 | dual (A + B) | 103 MB |  |
| 2026-07-18 01:19 | dual (A + B) | 127 MB |  |
| 2026-07-18 01:20 | dual (A + B) | 67 MB |  |
| 2026-07-18 01:20 | dual (A + B) | 116 MB |  |
| 2026-07-18 01:22 | dual (A + B) | 378 MB |  |
| 2026-07-18 01:30 | dual (A + B) | 101 MB |  |
| 2026-07-18 01:32 | dual (A + B) | 105 MB |  |
| 2026-07-18 13:11 | dual (A + B) | 266 MB |  |
| 2026-07-18 13:22 | dual (A + B) | 121 MB |  |
| 2026-07-18 13:38 | dual (A + B) | 848 MB |  |
| 2026-07-18 13:39 | dual (A + B) | 122 MB |  |
| 2026-07-18 13:40 | dual (A + B) | 116 MB |  |
| 2026-07-18 13:49 | dual (A + B) | 34 MB |  |
| 2026-07-18 13:49 | dual (A + B) | 121 kB |  |
| 2026-07-18 13:53 | dual (A + B) | 112 kB |  |
| 2026-07-18 13:54 | dual (A + B) | 156 kB |  |
| 2026-07-18 13:57 | dual (A + B) | 112 MB |  |
| 2026-07-18 13:58 | dual (A + B) | 22 MB |  |
| 2026-07-18 13:59 | dual (A + B) | 314 MB |  |
| 2026-07-18 14:08 | dual (A + B) | 110 MB |  |
| 2026-07-18 14:11 | dual (A + B) | 94 MB | hw. Initial → ObserveObject (log ends). |
| 2026-07-18 14:15 | dual (A + B) | 32 MB |  |
| 2026-07-18 14:16 | dual (A + B) | 130 kB |  |
| 2026-07-18 14:20 | dual (A + B) | 112 kB |  |
| 2026-07-18 14:24 | dual (A + B) | 130 kB |  |
| 2026-07-18 14:26 | dual (A + B) | 108 MB |  |
| 2026-07-18 14:28 | dual (A + B) | 395 MB |  |
| 2026-07-18 14:28 | dual (A + B) | 130 kB |  |
| 2026-07-18 14:29 | dual (A + B) | 20 MB |  |
| 2026-07-18 14:30 | dual (A + B) | 59 kB |  |
| 2026-07-18 14:31 | dual (A + B) | 68 kB |  |
| 2026-07-18 14:33 | dual (A + B) | 50 kB |  |
| 2026-07-18 14:37 | dual (A + B) | 114 MB |  |
| 2026-07-18 14:40 | dual (A + B) | 203 MB |  |
| 2026-07-18 14:41 | dual (A + B) | 99 MB |  |
| 2026-07-18 14:51 | dual (A + B) | 134 MB |  |
| 2026-07-18 14:56 | dual (A + B) | 39 MB |  |
| 2026-07-18 14:57 | dual (A + B) | 2 MB |  |
| 2026-07-18 15:03 | dual (A + B) | 39 MB |  |
| 2026-07-18 15:03 | dual (A + B) | 139 kB |  |
| 2026-07-18 15:04 | dual (A + B) | 130 kB |  |
| 2026-07-18 15:05 | dual (A + B) | 94 kB |  |
| 2026-07-18 15:05 | dual (A + B) | 156 kB |  |
| 2026-07-19 14:38 | dual (A + B) | 102 MB |  |
| 2026-07-18 11:28 | Robot A only | 41 MB |  |
| 2026-07-18 11:29 | Robot A only | 62 MB |  |
| 2026-07-18 11:29 | Robot A only | 11 MB |  |
| 2026-07-18 11:29 | Robot A only | 32 MB |  |
| 2026-07-18 11:32 | Robot A only | 40 MB |  |
| 2026-07-18 11:32 | Robot A only | 35 MB |  |
| 2026-07-18 11:35 | Robot A only | 272 MB |  |
| 2026-07-18 11:40 | Robot A only | 119 MB |  |
| 2026-07-18 13:40 | Robot A only | 37 MB |  |
| 2026-07-18 13:51 | Robot A only | 85 kB |  |
| 2026-07-18 13:57 | Robot A only | 4 MB |  |
| 2026-07-19 16:40 | Robot A only | 1.5 GB |  |
| 2026-07-19 16:57 | Robot A only | 101 MB |  |
| 2026-07-19 17:01 | Robot A only | 96 MB |  |
| 2026-07-19 17:08 | Robot A only | 11 MB |  |
| 2026-07-19 17:09 | Robot A only | 61 MB |  |
| 2026-07-19 17:09 | Robot A only | 61 MB |  |
| 2026-07-19 17:14 | Robot A only | 61 MB |  |
| 2026-07-19 17:14 | Robot A only | 61 MB |  |
| 2026-07-19 17:14 | Robot A only | 61 MB |  |
| 2026-07-19 17:15 | Robot A only | 67 MB |  |
| 2026-07-19 17:15 | Robot A only | 51 MB |  |
| 2026-07-19 17:16 | Robot A only | 61 MB |  |
| 2026-07-19 17:16 | Robot A only | 61 MB | simulation (no Kortex entries). Robot A alone, full FSM to Completed. |
| 2026-07-19 17:17 | Robot A only | 61 MB |  |
| 2026-07-19 17:17 | Robot A only | 67 MB |  |
| 2026-07-19 17:18 | Robot A only | 51 MB |  |
| 2026-07-19 17:18 | Robot A only | 68 MB |  |
| 2026-07-19 17:18 | Robot A only | 66 MB |  |
| 2026-07-19 17:19 | Robot A only | 49 MB |  |
| 2026-07-19 17:19 | Robot A only | 67 MB |  |
| 2026-07-19 17:20 | Robot A only | 67 MB |  |
| 2026-07-19 17:20 | Robot A only | 51 MB |  |
| 2026-07-19 17:20 | Robot A only | 68 MB |  |
| 2026-07-19 17:21 | Robot A only | 70 MB |  |
| 2026-07-19 17:21 | Robot A only | 49 MB |  |
| 2026-07-19 17:22 | Robot A only | 72 MB |  |
| 2026-07-21 08:32 | Robot A only | 776 MB |  |
| 2026-07-17 15:57 | dual (A + B) | 498 kB |  |
| 2026-07-17 15:58 | dual (A + B) | 42 MB |  |
| 2026-07-17 16:05 | dual (A + B) | 63 MB |  |
| 2026-07-17 16:08 | dual (A + B) | 27 MB |  |
| 2026-07-17 16:40 | dual (A + B) | 5.9 GB |  |

134 logs, 31.0 GB in total. In this mc_rtc log inventory, no hardware run is recorded as reaching `CaptureTransfer`.

## Physical dual-robot video evidence

Separate real-world video evidence is included in `../media/`: `dual_robot_physical_01.mp4` and
`dual_robot_physical_02.mp4`. The footage shows both physical Kinova arms operating in the same laboratory
handover setup. In the second clip Robot B supports/presents the bottle while Robot A's Robotiq gripper
approaches and closes around the bottle neck.

This visual evidence is independent of the state-log inventory above. The videos establish physical
dual-robot operation and interaction, but should not be used by themselves to infer a particular logged
FSM state without synchronized controller data. See
[`PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md`](PHYSICAL_DUAL_ROBOT_VIDEO_EVIDENCE.md) for the source-file hashes
and evidence boundary.
