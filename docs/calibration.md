# Calibrate the follower

Released LeRobot 0.6.2 has no auto-calibration command. This arm was calibrated with the Maker-Mods fork. The file it writes is the file 0.6.2 loads.

Use the command in [Run](#run). Two other tools are the wrong ones for this arm:

- `lerobot-calibrate` is the official manual sweep. It was not the procedure used here.
- `scripts/experimental/auto_calibrate_follower.py` is an unmerged port of [PR 4826](https://github.com/huggingface/lerobot/pull/4826). On this STS3215 it writes `Goal_Position` past 4095 and leaves the arm limp and extended. The script refuses to move unless you pass a second flag on purpose. Leave that flag off.

Finish [setup.md](setup.md) first, including the Maker-Mods patch. Re-running a calibration that already succeeded overwrites servo EEPROM and the JSON. Do that only when you mean to replace the file.

## Safety

Be at the arm, with a clear space in front of the base and the power switch in reach. Folded means packed: upper arm tilted back, forearm folded down onto it, gripper closed and hanging in front of the base.

A full run takes a few minutes. Each direction waits out its timeout after the joint has already stopped. That wait is the stop detector.

Wrist roll on this arm has a hard stop and finished at 337.5°. [Seeed’s SenseCraft notes](https://wiki.seeedstudio.com/sensecraft_robotics_common_issues) say not to auto-calibrate a wrist that has no hard stop, because the sweep winds the cable. If a later run spins the wrist without stopping, cut power. Do not raise the timeout and try again.

## Where the file goes

Robot id `so101_follower`, type `so_follower`. The command below writes:

`~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json`

Fields are `id`, `drive_mode`, `homing_offset`, `range_min`, and `range_max`. LeRobot 0.6.2 reads the arm joints in degrees. Maker-Mods uses an internal `RANGE_M100_100` norm while it runs. That norm mode is not stored in the file.

Current file, also what is on the servos:

| Joint | Degrees | Min | Max | Homing offset |
| --- | ---: | ---: | ---: | ---: |
| shoulder_pan | 228.7 | 746 | 3348 | 149 |
| shoulder_lift | 212.3 | 839 | 3255 | 696 |
| elbow_flex | 195.6 | 934 | 3160 | -292 |
| wrist_flex | 199.9 | 910 | 3184 | -475 |
| wrist_roll | 337.5 | 127 | 3967 | 110 |
| gripper | 132.0 | 1296 | 2798 | -139 |

Shoulder pan is wider than a 185–205° sanity band. That is already true of this arm. Wrist roll is inside 325–360°. Gripper is inside 128–152°. Wrist flex is inside 195–218°, and the last fold stopped near raw 2846, about 30° short of `range_max` 3184. Tuck that joint by hand with torque off. Do not recalibrate to change that pose.

## Before the arm moves

Read positions with the installed 0.6.2 bus (`SerialMotorsBus`, `default_timeout = 250`, no `PYTHONPATH`). With the saved calibration loaded, a folded follower sits near shoulder_lift `range_min`, elbow_flex `range_max`, and gripper `range_min`. An extended arm has shoulder_lift in the thousands and elbow near the bottom of its range. Stop if that is what you read.

1. A missing-motor error from the Maker-Mods bus is often an overload alarm, not a dead servo. 0.6.2 still reads through status bit `0x20` (protocol error 32). Clear it without a sweep: torque off, set `Goal_Position` to the current present position (that write enables torque on an STS3215), wait about 0.4 s, torque off. Continue only when status is `0` and the position did not jump.
2. Park the goals before their init. Init writes `Homing_Offset` 0 and then enables torque. It does not rewrite `Goal_Position`. A stale goal from a failed sweep makes the arm lunge. This arm has seen a shoulder_lift goal of 3859 and an elbow goal of 293 while folded. With torque off: `Lock` 0, position limits 0 and 4095, `Homing_Offset` 0, then `Goal_Velocity` 0. The velocity write is the flush. Writing the same homing offset a second time does not update `Present_Position`. Do not use `Operating_Mode` as the flush. Re-read present. If any present is outside 0..4095, stop. `Goal_Position` cannot store those values and the servo clamps them to 4095. Write each goal equal to that present, read the goals back, then disconnect with torque disabled. Confirm torque is 0.

`Goal_Velocity` 0 in position mode means “no speed cap”, not a stop. Holding still is safe when the goal equals the present position.

## Run

From the repository root, after the park step. Stay with the process.

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

Replace the port and robot id with `config/bench.json` when they differ. Keep velocity 1000 with timeout 20. 100 steps/s for 20 s covers about 176°, and wrist roll needs about 337°. Lower the speed only together with a longer timeout. `save=True` writes EEPROM and the JSON. On an exception with `interactive=False`, the script disables torque instead of waiting for Enter.

Their `main()` forces `interactive=True` and can block on Enter. Call `run_full_calibration(..., interactive=False)` as above.

## After

Read back torque, `Lock`, `Operating_Mode`, `Status`, homing, and limits with the 0.6.2 bus. Expect torque 0, Lock 0, mode 0, status 0, and homing and limits that match the JSON. Compare degrees with the table above.

Logs from the runs on this bench are local files under `runs/logs/` when you keep them. They are not in git.
