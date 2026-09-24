# Physical dual-robot video evidence

Two real-world phone videos supplied on 24 September 2026 document the July 2026 two-robot laboratory setup with both physical Kinova arms operating together.

## What the footage directly shows

- [`../media/dual_robot_physical_01.mp4`](../media/dual_robot_physical_01.mp4): both physical Kinova arms are active in the same laboratory setup around the bottle/object.
- [`../media/dual_robot_physical_02.mp4`](../media/dual_robot_physical_02.mp4): Robot B supports/presents the bottle while Robot A's Robotiq gripper approaches the bottle neck and closes around it.

This is direct visual evidence of physical dual-robot handover interaction. It is independent of the mc_rtc state-log inventory. The footage alone should not be used to infer a particular logged FSM state such as `CaptureTransfer` without synchronized controller data.

## Source integrity

The repository holds the two phone videos exactly as supplied, unmodified.

| file | duration | size | SHA-256 |
|---|---:|---:|---|
| `../media/dual_robot_physical_01.mp4` (was `WhatsApp Video 2026-09-24 at 5.23.45 PM.mp4`) | 8.53 s | 1,784,950 B | `d5f9a5e132bddc3d83adc3ff32acc0cd1b90366dd89a4f20553e180a2854505e` |
| `../media/dual_robot_physical_02.mp4` (was `WhatsApp Video 2026-09-24 at 5.23.46 PM.mp4`) | 10.96 s | 2,271,301 B | `6682afbf0dc4595d028fb8fb9e3d8d16615ea6cd7734d5bf0fdf320012b96dca` |

Verify with `sha256sum two_robot/media/dual_robot_physical_0*.mp4`.
