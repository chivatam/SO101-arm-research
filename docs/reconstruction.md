# Cube clips and Isaac

Two reference clips were tracked on this Mac. They are not a capture of the SO-101. `Robot cube pick-and-place — v2.mp4` is an overhead view and `Robot cube pick-and-place — v3` is a close view. Both are 10.125 s, 854×480, 24 fps, 243 frames, and show a white 3D-printed parallel-jaw arm, a navy 3×3 icon cube, and a wood table.

Local outputs, when present, are under `runs/recon/`. That directory is not committed.

| Artifact | What it is |
| --- | --- |
| `v2/overlay.mp4`, `v2/mask.mp4`, `v2/track.json` | Overhead cube mask and per-frame centroid |
| `v3/overlay.mp4`, `v3/mask.mp4`, `v3/track.json` | Close-view cube mask and per-frame centroid |
| `cube.obj`, `cube.mtl`, `cube_top.png` | 56 mm Z-up cube, top face taken from overhead frame 238 |
| `summary.json` | Counts and the scale assumption |

The mask is a sticker-and-blue tracker. It is not SAM 3.1. On the overhead view it stays on the cube for 238 of 243 frames and covers the top face, leaving the side faces out. On the close view it covers the visible faces on most frames, leaves holes where the jaw covers the cube, and locks onto a corner on 8 frames. Gripper contact in the overhead track rises through 5–7 s and is gone by 10 s. The cube is still on the table in the checked frames at 6 s and 9 s, so this track is not a measured lift.

`cube.obj` is a primitive. The 56 mm edge is a standard 3×3 size, not a measurement. The texture includes a sliver of the side face.

## What did not run

- The checkpoint is `facebook/sam3.1`, file `sam3.1_multiplex.pt` (3.5 GB). The repo is gated. This machine was not logged in to Hugging Face, and an unauthenticated download returns “Access denied.” Model card: https://huggingface.co/facebook/sam3.1
- Official video tracking calls CUDA and raises `NotImplementedError` on MPS and CPU. Release notes: https://github.com/facebookresearch/sam3/blob/main/RELEASE_SAM3p1.md
- SAM 3D Objects needs 64-bit Linux and an NVIDIA GPU with at least 32 GB of VRAM, plus its own gated checkpoint. Setup: https://github.com/facebookresearch/sam-3d-objects/blob/main/doc/setup.md
- This computer is an M1 Max with MPS and no CUDA.

## These clips are not a stereo rig

[nvidia-isaac/video_to_data](https://github.com/nvidia-isaac/video_to_data) can be cloned next to this work as `video_to_data/`. It is gitignored. Its HOI reconstruction (`video_to_data/reconstruction/modules/v2d_hoi_object_reconstruction/`) expects one calibrated stereo rig: `frames_meta.json`, rectified left and right frames, and a positive baseline. v2 and v3 are two unsynchronized monocular views of a moving arm and a moving cube. Treating them as a stereo pair invents a baseline.

A Gaussian splat or Isaac NuRec scene has the same problem. NVIDIA’s smartphone-to-Isaac path is a walk around a static scene. These clips move the object, so a fused scene smears the cube. That path is the wrong capture for this pick: https://developer.nvidia.com/blog/reconstruct-a-scene-in-nvidia-isaac-sim-using-only-a-smartphone/

BundleSDF and the SAM 3D stereo scan mode want a stationary or deliberately rotated object and a calibrated stereo rig. The white printed arm in the clips has no URDF and is not a stand-in for the SO-101.

## Put a measured cube in the SO-101 scene

This is the scene that can be simulated. The collider is a cuboid. The reconstructed mesh is only the appearance.

1. Measure the real cube with a caliper and a scale. Until then the stand-in is 0.056 m and about 0.08 kg.
2. Use the SO-101 lesson in Isaac Lab, which spawns the fixed-base arm and six joint-position actuators: https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/2026-10.0/train-a-robot-arm/02-so101-robot.html
3. Add the cube as a free rigid body. Isaac Lab applies mass and collision through schema configs: https://isaac-sim.github.io/IsaacLab/main/source/concepts/schema_cfgs.html

```python
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.schemas import MassCfg, UsdPhysicsCollisionCfg, UsdPhysicsRigidBodyCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg

cube = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/cube",
    spawn=sim_utils.CuboidCfg(
        size=(0.056, 0.056, 0.056),
        rigid_props=UsdPhysicsRigidBodyCfg(),
        mass_props=MassCfg(mass=0.08),
        collision_props=UsdPhysicsCollisionCfg(collision_enabled=True),
        physics_material=RigidBodyMaterialBaseCfg(
            static_friction=0.8,
            dynamic_friction=0.6,
            restitution=0.0,
        ),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(0.25, 0.0, 0.028)),
)
```

Replace `0.056` and `0.08` with the measurement. The friction values are a starting guess for a plastic cube on a hard table. Raise them if the jaws slip. `pos` is an initial guess in front of the base: the table height plus half the cube, with a 1 mm gap so the cube drops and settles. The overhead clip only says the cube sits on the right side of the table and moves a short distance while the gripper is around it. It does not give a metric pose.

4. For appearance, convert `runs/recon/cube.obj` with Isaac Sim’s asset converter. OBJ and glTF become USD. The stage unit is meters and the up axis is Z. Formats: https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/formats.html
5. Parent that visual under the rigid cube and keep the cuboid as the only collider.
6. Leave the cube free. The Isaac Lab SO-101 vial task grasps by contact and never writes the object pose after reset.
7. Match the close camera to v3 by moving the SO-101 joints until the jaws span the cube.

## SAM masks, after access and a CUDA machine

Do this when the cube’s printed faces matter. It does not replace the cuboid collider.

1. Request access to `facebook/sam3.1` and the SAM 3D Objects repo, then `hf auth login`.
2. On Linux with CUDA 12.6 or newer, install PyTorch cu128 and the current `facebookresearch/sam3`. The 3.1 checkpoint is loaded with `build_sam3_multiplex_video_predictor()`.
3. Run each clip as its own session. Text prompts, one concept at a time: `cube`, then `robot gripper`. A separate `robot arm` prompt if the arm body is needed.
4. Keep the masklets. Use a settled frame where the cube is fully visible and outside the jaws: overhead near 1 s or 9.5 s, close view near 1 s.
5. Run SAM 3D Objects on that image and mask and export the GLB. The mesh is in the model’s own scale.
6. Scale the GLB so its axis-aligned box matches the caliper measurement. The silhouette scale step in the video-to-data HOI module needs stereo depth and camera poses these clips do not have.
7. Import the scaled GLB as the visual only. Keep the cuboid as the rigid body.
