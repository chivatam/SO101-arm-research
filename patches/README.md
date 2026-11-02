# Patches

Apply these from the repository root after the checkouts in [docs/setup.md](../docs/setup.md). Bases:

| Patch | Checkout | Commit |
| --- | --- | --- |
| `lerobot-0.6.2-bench.patch` | `third_party/lerobot` | `3f797de` (LeRobot 0.6.2) |
| `lerobot-makermods-wch-adapter.patch` | `third_party/lerobot-makermods` | `788477a` |

`lerobot-0.6.2-bench.patch` accepts the extra fields on the released MolmoAct2 config (`enable_lora_vlm`, `enable_lora_action_expert`, `train_action_expert_only`, `model_dtype`) and lets the process import when this transformers build cannot load Wall-OSS-0.5. MolmoAct2 still loads.

`lerobot-makermods-wch-adapter.patch` raises the Feetech packet-timeout floor from 50 ms to 250 ms and makes calibration connect without the broadcast handshake. On the WCH `1A86:55D3` adapter a 50 ms window drops replies, and `ping()` returns nothing on a latched overload, so the stock handshake reports a missing servo.

```bash
git -C third_party/lerobot apply "$PWD/patches/lerobot-0.6.2-bench.patch"
git -C third_party/lerobot-makermods apply "$PWD/patches/lerobot-makermods-wch-adapter.patch"
```
