# SO-101 arm research

Bring-up, calibration, and a local prompt page for one [SO-101](https://huggingface.co/docs/lerobot/so101) follower arm. The arm is a Feetech STS3215 follower, robot id `so101_follower`. A leader arm is not part of this setup.

Start with [docs/setup.md](docs/setup.md). Calibrate from [docs/calibration.md](docs/calibration.md). Run the arm from a text prompt with [docs/operator.md](docs/operator.md).

Stay at the arm whenever torque is on. Keep the power switch in reach. The read-only bus scan does not move the arm. Calibration and the prompt page do.

## Layout

```text
config/bench.json     Serial port, robot id, cameras, policy. Edit this for your machine.
docs/                 Setup, calibration, operator, and the cube-reconstruction note.
operator/server.py    Local page that runs a MolmoAct2 checkpoint on the follower.
scripts/              Read-only motor scan and a one-frame camera grab.
patches/              Local fixes for the pinned LeRobot and Maker-Mods checkouts.
runs/                 Session frames, videos, logs, and meshes. Not committed.
third_party/          LeRobot checkouts. Not committed. Created by the setup doc.
```

`video_to_data/` is an optional separate clone of [nvidia-isaac/video_to_data](https://github.com/nvidia-isaac/video_to_data). It is not part of this repository. [docs/reconstruction.md](docs/reconstruction.md) says when that pipeline does not apply.

## Commands

From the repository root, after setup:

```bash
source .venv/bin/activate
python scripts/scan_bus.py
python scripts/capture_cameras.py
python operator/server.py
```

`scan_bus.py` reads `config/bench.json` for the port. Open the printed URL from `operator/server.py` and confirm the two camera pictures before Act.

## This bench

| Item | Value |
| --- | --- |
| Robot id | `so101_follower` |
| Port | `/dev/tty.usbmodem5C821107241` |
| Adapter | WCH USB Single Serial, VID:PID `1A86:55D3`, serial `5C82110724` |
| Motors | STS3215, ids 1–6, 1_000_000 baud |
| Cameras | Two icspring USB cameras. FaceTime and iPhone continuity cameras stay out of the robot config. |
| Calibration | `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json` |
| Policy | `lerobot/MolmoAct2-SO100_101-LeRobot` |

Change the port in `config/bench.json` if the adapter enumerates somewhere else. `lerobot-find-port` from the LeRobot install lists serial devices.

Cursor sessions on this checkout also read `.cursor/skills/so101-auto-calibrate/SKILL.md`. The human procedure is [docs/calibration.md](docs/calibration.md).
