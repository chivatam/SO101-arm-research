# SO-101 Phase 1 bring-up

Recorded on this Mac on 2026-10-31. LeRobot is installed. The motor adapter and both USB cameras are visible. No servo answered a read-only ping, and there is no calibration file yet. Motor ID programming and calibration stay manual until you are at the arm.

## Environment

LeRobot 0.6.2, commit `3f797de`, editable install from `third_party/lerobot`, extra `feetech`. Python 3.12.14 in `.venv`.

```bash
cd "/Users/venkatasatyanarayanachivatam/research endeavors/SO101-arm-research"
source .venv/bin/activate
```

Recreate the environment with:

```bash
uv venv --python 3.12 .venv
git clone --depth 1 https://github.com/huggingface/lerobot.git third_party/lerobot
uv pip install --python .venv/bin/python -e "third_party/lerobot[feetech]"
```

Official references: [LeRobot SO-101](https://huggingface.co/docs/lerobot/so101) and [NVIDIA calibration module](https://docs.nvidia.com/learning/physical-ai/sim-to-real-so-101/latest/07-calibrating-so101.html).

## What is plugged in

| Device | Identity |
| --- | --- |
| Motor adapter | WCH USB Single Serial, VID:PID `1A86:55D3`, serial `5C82110724` |
| LeRobot port | `/dev/tty.usbmodem5C821107241` (same chip as `/dev/cu.usbmodem5C821107241`) |
| USB location | `2-1.1.2` on the hub |
| Camera A | icspring UVC, AVFoundation index 0, vendor `13030` product `36869` |
| Camera B | icspring UVC, AVFoundation index 1, vendor `13030` product `37409` |
| Other cameras | FaceTime is index 2. iPhone continuity cameras are indexes 3 and 4. Leave those out of the robot config. |
| Leader arm | No second serial adapter is connected. |

`~/.cache/huggingface/lerobot/calibration` does not exist yet. The robot id for this arm is `so101_follower`. Keep that id for every later command.

## Motor scan

`scripts/scan_bus.py` pings STS3215 servos and does not write registers, enable torque, or disable torque.

```bash
.venv/bin/python scripts/scan_bus.py --port /dev/tty.usbmodem5C821107241
```

Result: the serial port opened, and no servo answered at 1_000_000 baud or at the other STS3215 rates. The adapter is alive on USB. The servos are unpowered, or the daisy-chain is not plugged into the controller.

## Do this at the arm

1. Connect the kit power supply to the controller, and connect the daisy-chain so the shoulder-pan end of the chain plugs into the board. On a Waveshare board, both jumpers sit on channel B (USB).
2. Give the arm a clear space. Keep the power switch in reach.
3. Rerun the scan above.
   - Six motors, ids 1 through 6, means the ids are already programmed. Skip motor setup and go to calibration.
   - One motor, or several motors sharing an id, means the ids still need programming. Run setup below, one motor at a time.
   - Still nothing means the power leads or the first 3-pin cable are not seated.

### Program ids, only if the scan does not show ids 1–6

This writes EEPROM. Plug in a single motor for each prompt, never the whole chain. The script assigns gripper=6, wrist_roll=5, wrist_flex=4, elbow_flex=3, shoulder_lift=2, shoulder_pan=1.

```bash
lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem5C821107241
```

After the last motor, daisy-chain them again and rerun the scan. Expect all six names.

### Calibrate

Move every joint to the middle of its travel before you press Enter. Then sweep each joint to its real mechanical stop, one joint at a time. Wrist roll uses almost a full motor turn, so its center has to be right or the encoder wraps. Stop at the joint, not at a pinched cable.

```bash
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem5C821107241 \
  --robot.id=so101_follower
```

The file is saved under `~/.cache/huggingface/lerobot/calibration/` using the id `so101_follower`. That file is the one training and deployment will use.

`lerobot-calibrate` also writes the usual STS3215 position-mode gains the first time the arm connects. That is the official script. The read-only scan does not do it.

### Cameras

macOS listed both icspring cameras, then blocked a frame grab until Camera access is allowed for Cursor. After you allow it:

```bash
.venv/bin/python scripts/capture_cameras.py
```

Frames land in `bringup/camera_frames/avfoundation_0.jpg` and `avfoundation_1.jpg`. Label which file is the wrist view and which is the front view before any policy work.

## Auto-calibration

Released LeRobot 0.6.2 has no automatic calibration command. [PR 3282](https://github.com/huggingface/lerobot/pull/3282) adds `lerobot-auto-calibrate-feetech`, which drives each joint into its stop. That stays unused until the manual file above exists, you are watching the arm, and a comparison shows each joint range within a few degrees of the manual file. The manual file remains the source of truth.
