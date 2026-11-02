# Prompt page

`operator/server.py` serves a page on this machine that runs [`lerobot/MolmoAct2-SO100_101-LeRobot`](https://huggingface.co/lerobot/MolmoAct2-SO100_101-LeRobot) on the follower. The arm stays still until Act is pressed. That button stays disabled until the power-switch box is checked. Stop drops torque.

Calibrate first. On connect, the page compares the servos with the saved file and stops if they disagree. It does not rewrite calibration.

## Start

Power the arm, allow Camera access for the app that launches Python, and from the repository root:

```bash
source .venv/bin/activate
python operator/server.py
```

Open the URL it prints. The default is [http://127.0.0.1:8771](http://127.0.0.1:8771). Host and port come from `operator` in [`config/bench.json`](../config/bench.json). The serial port, robot id, and `policy_id` come from that same file.

The first Act loads the checkpoint before anything moves. The page loads the Allen AI weights once. LeRobot also publishes a second full `model.safetensors` for this repo, and loading that copy again duplicates the 5B weights. The server skips that second load. Inference runs on MPS (`PYTORCH_ENABLE_MPS_FALLBACK=1`).

## Cameras

The page shows two previews. The table picture is the observation named `table`. The gripper picture is `close`. Defaults are OpenCV index 0 and 1, which match this bench’s icspring cameras when FaceTime is a later index. Confirm the pictures. If a preview is the laptop camera, change the dropdown and press Refresh cameras. The two indexes have to differ.

`scripts/capture_cameras.py` uses AVFoundation indexes, which are a different numbering. Use the preview pictures here, not the filenames from that script, when you choose the dropdowns.

Each run records the prompt and both camera streams under `runs/operator_runs/<timestamp>/`.

## Motion limits

The follower is created with `max_relative_target` of 8 degrees, so one policy step cannot command a joint farther than that from where it is. Shoulder lift is clipped to -110°..60°, elbow flex to -70°..110°, and wrist flex to -20°..100°. The ready pose the page moves to before the policy is:

| Joint | Degrees |
| --- | ---: |
| shoulder_pan | 0 |
| shoulder_lift | -30 |
| elbow_flex | 35 |
| wrist_flex | 58 |
| gripper | 40 |

Stop, or an error, disables torque. The arm can drop. Stay at the power switch for the whole run.
