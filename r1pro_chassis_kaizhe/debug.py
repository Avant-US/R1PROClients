import time
import toml
import torch
import cv2
from pathlib import Path
from omegaconf import OmegaConf
from core.communication.ros2_bridge import Ros2Bridge
from core.processor.openpi_processor import OpenPIProcessor
from core.inference.websocket_engine import WebSocketClientEngine
torch.set_printoptions(precision=10, sci_mode=False, linewidth=200)
config = toml.load("config.toml")
cfg = OmegaConf.create({})
print("== config check ==")
print("use_websocket:", config["websocket"]["use_websocket"])
print("host:", config["websocket"]["host"])
print("port:", config["websocket"]["port"])
print("processor:", config["model"]["processor"])
print("action_steps:", config["basic"]["action_steps"])
print("default_prompt:", config.get("openpi", {}).get("default_prompt"))
bridge = Ros2Bridge(config, cfg, use_recv_time=True)
processor = OpenPIProcessor(config, cfg)
processor.initialize(None)
engine = WebSocketClientEngine(config, cfg)
engine.load_model()
print("\n== waiting ROS obs ==")
obs = None
for _ in range(50):
    obs_time, obs = bridge.gather_obs()
    if obs is not None:
        break
    time.sleep(0.1)
if obs is None:
    raise RuntimeError("没有拿到 ROS2 观测，请先检查相机和关节 topic")
print("\n== raw obs ==")
for k, v in obs["images"].items():
    print(f"image[{k}] shape={tuple(v.shape)} dtype={v.dtype}")
for k, v in obs["state"].items():
    print(f"state[{k}] shape={tuple(v.shape)} value={v.flatten()[:8]}")
batch = processor.preprocess(obs)
print("\n== after preprocess ==")
for k, v in batch.items():
    if isinstance(v, torch.Tensor):
        print(f"{k}: shape={tuple(v.shape)} dtype={v.dtype}")
    else:
        print(f"{k}: {v}")
state = batch["state"].cpu()
print("\nstate total dim =", state.numel())
print("state[:7]     left_arm      =", state[0:7])
print("state[7:14]   right_arm     =", state[7:14])
print("state[14:15]  left_gripper  =", state[14:15])
print("state[15:16]  right_gripper =", state[15:16])
print("state[16:20]  torso         =", state[16:20])
print("state[20:23]  chassis       =", state[20:23])
for name in ["head_rgb", "left_wrist_rgb", "right_wrist_rgb"]:
    img = batch[name].cpu().numpy()
    print(f"{name}: shape={img.shape} dtype={img.dtype}")
    if img.ndim == 3 and img.shape[-1] == 3:
        pass  # already HWC
    elif img.ndim == 3 and img.shape[0] == 3:
        img = img.transpose(1, 2, 0)  # CHW -> HWC fallback
    out = f"/tmp/{name}.png"
    cv2.imwrite(out, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    print("saved:", out)
print("\n== request server ==")
resp = engine.predict_action(batch)
print("server keys:", resp.keys())
for k, v in resp.items():
    if isinstance(v, torch.Tensor):
        stats_tensor = v.float() if not torch.is_floating_point(v) else v
        print(
            f"resp[{k}] tensor shape={tuple(v.shape)} dtype={v.dtype} "
            f"min={stats_tensor.min().item():.4f} max={stats_tensor.max().item():.4f} "
            f"mean={stats_tensor.mean().item():.4f}"
        )
    else:
        print(f"resp[{k}] = {v}")

raw_actions = resp["actions"]
print("\n== raw actions ==")
print("shape:", tuple(raw_actions.shape))
print("dtype:", raw_actions.dtype)
print("has nan:", torch.isnan(raw_actions).any().item())
print("has inf:", torch.isinf(raw_actions).any().item())
raw_actions_view = raw_actions.unsqueeze(0) if raw_actions.ndim == 2 else raw_actions
print("view shape:", tuple(raw_actions_view.shape))
print("first step:", raw_actions_view[0, 0, :])
print("first 5 steps first 8 dims:\n", raw_actions_view[0, :5, :8])
flat_argmax = torch.argmax(raw_actions_view)
flat_argmin = torch.argmin(raw_actions_view)
num_steps, action_dim = raw_actions_view.shape[1], raw_actions_view.shape[2]
max_step, max_dim = divmod(flat_argmax.item(), action_dim)
min_step, min_dim = divmod(flat_argmin.item(), action_dim)
print("max value location:", {"step": max_step, "dim": max_dim, "value": raw_actions_view[0, max_step, max_dim].item()})
print("min value location:", {"step": min_step, "dim": min_dim, "value": raw_actions_view[0, min_step, min_dim].item()})

raw_actions_txt = Path("/tmp/openpi_raw_actions.txt")
raw_actions_pt = Path("/tmp/openpi_raw_actions.pt")
raw_actions_txt.write_text(repr(raw_actions_view[0]))
torch.save(raw_actions_view.cpu(), raw_actions_pt)
print("saved raw actions txt:", raw_actions_txt)
print("saved raw actions pt:", raw_actions_pt)

actions = processor.postprocess(resp)["action"]
print("\n== action split ==")
processed_actions_txt = Path("/tmp/openpi_processed_actions.txt")
processed_actions_pt = Path("/tmp/openpi_processed_actions.pt")
processed_lines = []
for k, v in actions.items():
    print(
        f"{k}: shape={tuple(v.shape)} min={v.min().item():.4f} "
        f"max={v.max().item():.4f} mean={v.float().mean().item():.4f}"
    )
    print(f"{k} first step:", v[0, 0, :])
    processed_lines.append(f"[{k}]\n{repr(v[0])}\n")
processed_actions_txt.write_text("\n".join(processed_lines))
torch.save({k: v.cpu() for k, v in actions.items()}, processed_actions_pt)
print("saved processed actions txt:", processed_actions_txt)
print("saved processed actions pt:", processed_actions_pt)
bridge.destroy()
print("\nDEBUG OK")