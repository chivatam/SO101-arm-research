---
name: so101-auto-calibrate
description: >-
  Auto-calibrate this SO-101 follower (Feetech STS3215, robot id so101_follower)
  with the Maker-Mods calibrator. Use when the user asks to calibrate, auto-calibrate,
  or recalibrate the arm, says it is folded, or asks which calibration command to run.
  Do not use official lerobot-calibrate or the in-tree PR 4826 engine.
---

# SO-101 follower auto-calibration

Released LeRobot 0.6.2 has no auto-calibration command. This arm was calibrated with the Maker-Mods fork (same code as Hugging Face PR 3282). The saved file is what 0.6.2 loads.

`docs/bringup.md` still describes the official manual `lerobot-calibrate` flow. Ignore that section. The user rejected it.

## Do not

- Do not run `lerobot-calibrate`, `scripts/auto_calibrate_follower.py`, or `third_party/lerobot` `auto_calibrate` / `SO_ARM_PLAN`. That engine is the unmerged PR 4826 port. On this STS3215 it writes `Goal_Position` past 4095 and reframes `Homing_Offset` without a flush, then leaves the arm limp and extended.
- Do not checkout the `pr-4826` branch. Its follower code uses the old bus API.
- Do not enable torque, write a goal, or start a sweep until the user is at the arm, has confirmed the folded pose, and is by the power switch. A read-only status check is fine with nobody watching.
- Do not re-run a successful calibration to “tighten” a range. Re-running overwrites servo EEPROM and the JSON.

## This arm

- Robot id `so101_follower`, type `so_follower`. Port `/dev/tty.usbmodem5C821107241` (same chip as `/dev/cu.usbmodem5C821107241`). Baud 1_000_000. Model 777, IDs 1–6: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
- Adapter: WCH USB Single Serial `1A86:55D3`. A broadcast ping returns one servo. A packet timeout under about 250 ms drops replies. Scan IDs one by one.
- Leader arm is not connected. Cameras are not part of calibration.
- Saved file: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json`. Fields are `id`, `drive_mode`, `homing_offset`, `range_min`, `range_max`. 0.6.2 interprets arm joints in degrees. Maker-Mods internally uses `RANGE_M100_100`; that norm mode is not stored in the file.

Current on-servo result (also the JSON):

| Joint | Degrees | Min | Max | Homing offset |
|---|---|---|---|---|
| shoulder_pan | 228.7 | 746 | 3348 | 149 |
| shoulder_lift | 212.3 | 839 | 3255 | 696 |
| elbow_flex | 195.6 | 934 | 3160 | -292 |
| wrist_flex | 199.9 | 910 | 3184 | -475 |
| wrist_roll | 337.5 | 127 | 3967 | 110 |
| gripper | 132.0 | 1296 | 2798 | -139 |

Shoulder and elbow match earlier end-stop sweeps. Shoulder pan is wider than the 185–205° sanity band. Wrist roll is inside 325–360°. Gripper is inside 128–152°. Wrist flex is inside 195–218°, but the final fold stopped near raw 2846, about 30° short of `range_max` 3184. Tuck that joint by hand with torque off. Do not recalibrate for that pose.

## Code and patches

Clone, if `third_party/` is missing (`third_party/` is gitignored):

```bash
git clone --depth 1 https://github.com/makermods-robotics/lerobot-MakerMods.git third_party/lerobot-makermods
uv pip install --python .venv/bin/python "feetech-servo-sdk>=1.0.0" "deepdiff>=7.0.1,<9.0.0"
```

Import with `PYTHONPATH=third_party/lerobot-makermods/src` so it beats the editable 0.6.2 install. Their `main()` forces `interactive=True` and can block on Enter. Call `run_full_calibration(..., interactive=False)`.

Reapply these two local edits after a fresh clone. Upstream will not connect on this adapter.

1. `third_party/lerobot-makermods/src/lerobot/motors/feetech/feetech.py`, `patch_setPacketTimeout`: the added constant at the end of the timeout formula must be `250`, not `50`.
2. `third_party/lerobot-makermods/src/lerobot/scripts/lerobot_auto_calibrate_feetech.py`, `_connect_and_clear`: `connect(handshake=False)`, `set_timeout(1000)`, then ping each motor by name (`num_retry=2`, up to 5 tries). Do not call `connect()` with the default handshake. Their handshake uses `ping()`, and `ping()` returns `None` on any status error, including latched overload.

## Before the arm moves

1. User present, space in front of the base clear, user by the power switch. Folded means packed: upper arm tilted back, forearm folded down onto it, gripper closed and hanging in front of the base. Read positions with the 0.6.2 bus (`SerialMotorsBus`, `default_timeout = 250`, no `PYTHONPATH`). With the saved calibration loaded, a folded follower sits near shoulder_lift `range_min`, elbow_flex `range_max`, and gripper `range_min`. An extended arm has shoulder_lift in the thousands and elbow near the bottom of its range. Stop if that is what you read.
2. A missing-motor error from the Maker-Mods bus is often overload, not a dead servo. 0.6.2 reads through an overload alarm (status bit `0x20`, protocol error 32). Clear it without a sweep: torque off, set `Goal_Position` to the current present (that write itself enables torque on STS3215), wait ~0.4 s, torque off. Confirm status `0` and that the position did not jump.
3. Park goals before their init. Init writes `Homing_Offset` 0 and then enables torque. It does not rewrite `Goal_Position`. A stale goal from a failed sweep (this arm has seen shoulder_lift goal 3859 and elbow goal 293 while folded) makes the arm lunge. With torque off: `Lock` 0, position limits 0 and 4095, `Homing_Offset` 0, then `Goal_Velocity` 0. The velocity write is the flush. A second write of the same homing offset does not update `Present_Position`. Do not write `Operating_Mode` as the flush. Re-read present. If any present is outside 0..4095, stop. `Goal_Position` cannot store those values and the servo clamps them to 4095. Write each goal equal to that present, read the goals back, then disconnect with torque disabled. Confirm torque is 0.

`Goal_Velocity` 0 in position mode is “no speed cap”, not a stop. Holding still is safe only when the goal equals the present.

## Run

From the repo root, only after the park step. Stay with the process. A full run is a few minutes. Each direction waits out its timeout even after the joint has stopped. That wait is this script’s stop detector, not a hang.

```bash
PYTHONPATH=third_party/lerobot-makermods/src .venv/bin/python -u -c "
from lerobot.scripts.lerobot_auto_calibrate_feetech import run_full_calibration
raise SystemExit(run_full_calibration(
    '/dev/tty.usbmodem5C821107241',
    save=True,
    robot_id='so101_follower',
    robot_type='so_follower',
    velocity_limit=1000,
    timeout_s=20,
    interactive=False,
))
"
```

Keep velocity 1000 with timeout 20. 100 steps/s for 20 s covers about 176°, and wrist roll needs ~337°. Lower the speed only together with a longer timeout. `--save` writes EEPROM and the JSON. On any exception the script disables torque when `interactive=False` (it will not wait for Enter).

Tell the user the arm is about to move, then run it.

## After

Read back torque, `Lock`, `Operating_Mode`, `Status`, homing, and limits with the 0.6.2 bus. Torque 0, Lock 0, mode 0, status 0. Homing and limits match the JSON. Compare degrees with the table above. Shoulder pan outside 185–205° is already true of this arm. Wrist roll must have actually stopped. [Seeed SenseCraft](https://wiki.seeedstudio.com/sensecraft_robotics_common_issues) says not to auto-calibrate a wrist with no hard stop: the sweep winds the cable. This wrist roll stopped at 337.5°. If a later run spins a wrist without a stop, kill power. Do not raise the timeout and retry.

## STS3215 facts that caused the failed engine

- `Present_Position` and `Goal_Position` are 16-bit. The goal register clamps to 0..4095.
- `Homing_Offset` is sign-magnitude, sign bit 11, magnitude at most 2047.
- Present applies a new homing offset only after a later write of a different register. `Goal_Velocity` 0 is the flush that worked. Writing the same offset again does nothing.
- `Lock` 1 keeps EEPROM writes in RAM until power cycle. The successful script sets Lock 0 before `write_calibration`.
- `Torque_Limit` is tenths of a percent. Their run uses 380 (38%) and max torque 1000.
