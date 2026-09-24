# Physical dual-robot video evidence

Two real-world phone videos supplied on 24 September 2026 document the July 2026 two-robot laboratory setup with both physical Kinova arms operating together.

## What the footage directly shows

- [`../media/dual_robot_physical_01.mp4`](../media/dual_robot_physical_01.mp4): both physical Kinova arms are active in the same laboratory setup around the bottle/object.
- [`../media/dual_robot_physical_02.mp4`](../media/dual_robot_physical_02.mp4): Robot B supports/presents the bottle while Robot A's Robotiq gripper approaches the bottle neck and closes around it.

This is direct visual evidence of physical dual-robot handover interaction. It is independent of the mc_rtc state-log inventory. The footage alone should not be used to infer a particular logged FSM state such as `CaptureTransfer` without synchronized controller data.

## Source integrity

The repository MP4s are compressed, silent copies made for convenient versioning. The untouched user-supplied originals are identified below.

| source | duration | original size | original SHA-256 |
|---|---:|---:|---|
| `WhatsApp Video 2026-09-24 at 5.23.45 PM.mp4` | 8.53 s | 1,784,950 B | `d5f9a5e132bddc3d83adc3ff32acc0cd1b90366dd89a4f20553e180a2854505e` |
| `WhatsApp Video 2026-09-24 at 5.23.46 PM.mp4` | 10.96 s | 2,271,301 B | `6682afbf0dc4595d028fb8fb9e3d8d16615ea6cd7734d5bf0fdf320012b96dca` |

The untouched originals are also in the repository as `../media/dual_robot_physical_01_original.mp4` and
`../media/dual_robot_physical_02_original.mp4` (1.8 MB and 2.3 MB; the SHA-256 values above).

Repository copies:

- `../media/dual_robot_physical_01.mp4` — compressed H.264 copy; SHA-256 `b4610cc65a8f41b82050246494b91bb2b9669d616d5ad0f73b46e8b78c6ca5d4`.
- `../media/dual_robot_physical_02.mp4` — compressed H.264 copy; SHA-256 `1a396179bc984102f2a3cf6980518ee772d3de16df6df500f790b0d6ad808ecc`.

The videos establish that the two physical robots operated together in the handover setup. The log inventory and the video evidence answer different questions: the logs record controller/FSM progress, while the phone videos visually document the physical dual-robot interaction.