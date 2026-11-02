# Setup

Tested on macOS with Python 3.12. LeRobot is pinned to 0.6.2 (`3f797de`). The calibrator is the Maker-Mods fork at `788477a`, which is the same approach as [Hugging Face PR 3282](https://github.com/huggingface/lerobot/pull/3282). Both trees live in `third_party/` and stay out of git.

Install [uv](https://docs.astral.sh/uv/), Python 3.12, and `ffmpeg`. Homebrew covers the last one: `brew install ffmpeg`.

## Environment

From the repository root:

```bash
uv venv --python 3.12 .venv
git clone https://github.com/huggingface/lerobot.git third_party/lerobot
git -C third_party/lerobot checkout 3f797de
git clone https://github.com/makermods-robotics/lerobot-MakerMods.git third_party/lerobot-makermods
git -C third_party/lerobot-makermods checkout 788477a
git -C third_party/lerobot apply "$PWD/patches/lerobot-0.6.2-bench.patch"
git -C third_party/lerobot-makermods apply "$PWD/patches/lerobot-makermods-wch-adapter.patch"
uv pip install --python .venv/bin/python -e "third_party/lerobot[feetech,molmoact2]"
uv pip install --python .venv/bin/python "feetech-servo-sdk>=1.0.0" "deepdiff>=7.0.1,<9.0.0"
```

The first patch lets LeRobot 0.6.2 read the released MolmoAct2 config and skip a broken Wall-OSS import. The second patch is required on the WCH `1A86:55D3` adapter: a 50 ms packet timeout drops servo replies, and the stock handshake misses motors. What each patch changes is in [../patches/README.md](../patches/README.md).

`third_party/lerobot-makermods` is not installed. Calibration imports it with `PYTHONPATH` so it wins over the editable 0.6.2 install for that one command. Leave the 0.6.2 tree as the installed package. The prompt page uses that install.

## Bench file

Edit [`config/bench.json`](../config/bench.json) if the serial port, robot id, or camera indexes differ. The scan and the prompt page both read `port` from that file.

On macOS the same adapter shows up as `/dev/tty.usbmodem…` and `/dev/cu.usbmodem…`. Use the `tty` path.

## Check the bus

Power the controller and seat the daisy-chain with the shoulder-pan end into the board. On a Waveshare board both jumpers sit on channel B (USB). Then:

```bash
source .venv/bin/activate
python scripts/scan_bus.py
```

The script pings ids 1–10, one at a time. It does not write a register and it does not enable torque. A good result is six STS3215 servos, ids 1 through 6, at 1_000_000 baud:

| Id | Joint |
| --- | --- |
| 1 | shoulder_pan |
| 2 | shoulder_lift |
| 3 | elbow_flex |
| 4 | wrist_flex |
| 5 | wrist_roll |
| 6 | gripper |

No answer means the supply is off or the first 3-pin cable is loose. One motor, or several motors sharing an id, means the ids still need programming. That step writes EEPROM. Plug in a single motor for each prompt:

```bash
lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port=/dev/tty.usbmodem5C821107241
```

Use the port from `config/bench.json`. The official script assigns gripper=6, wrist_roll=5, wrist_flex=4, elbow_flex=3, shoulder_lift=2, shoulder_pan=1. Daisy-chain them again and rerun the scan.

## Cameras

macOS blocks a frame grab until Camera access is allowed for the app that launches Python (Terminal or Cursor). Then:

```bash
python scripts/capture_cameras.py
```

`ffmpeg` lists AVFoundation devices and saves one 640×480 frame from each icspring camera under `runs/camera_frames/`. FaceTime and iPhone continuity cameras are skipped. Label which file is the table view and which is the gripper view. OpenCV indexes in the prompt page are not the same list as those AVFoundation indexes. Pick the cameras from the pictures, using the steps in [operator.md](operator.md).

## Next

Calibrate with [calibration.md](calibration.md) before the prompt page. The page refuses to move an arm whose saved calibration does not match the servos.
