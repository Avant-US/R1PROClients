from pathlib import Path
from typing import Any, Dict

from loguru import logger
import torch
import torch.nn.functional as F

from core.processor.processor import Processor


class OpenPIProcessor(Processor):
    """Adapter between R1Pro observations and OpenPI's API."""

    def __init__(self, config: Dict[str, Any], cfg):
        super().__init__(config, cfg)
        self.default_prompt = config.get("openpi", {}).get("default_prompt")
        self.image_target_size = config.get("openpi", {}).get("image_target_size", 0)
        self.chassis_deadzone = config.get("openpi", {}).get("chassis_deadzone", 0.0)
        if self.chassis_deadzone > 0:
            logger.info(f"Chassis velocity deadzone: {self.chassis_deadzone}")
        if self.image_target_size > 0:
            logger.info(f"Client-side resize_with_pad to {self.image_target_size}x{self.image_target_size}")

    def initialize(self, dataset_stats_path: Path | None) -> None:
        logger.info("Initializing OpenPI processor")

    @staticmethod
    def _to_hwc(img: torch.Tensor) -> torch.Tensor:
        """Convert CHW → HWC to match training dataset format."""
        t = img.squeeze(0)
        if t.ndim == 3 and t.shape[0] in (1, 3):
            t = t.permute(1, 2, 0)
        return t.contiguous()

    @staticmethod
    def _resize_with_pad(img: torch.Tensor, target_h: int, target_w: int) -> torch.Tensor:
        """Replicate tf.image.resize_with_pad / openpi image_tools.resize_with_pad.

        1. Scale the image so it fits inside (target_h, target_w) preserving aspect ratio.
        2. Pad with black (zeros) to exactly (target_h, target_w).
        Input & output: HWC uint8 tensor.
        """
        h, w, c = img.shape
        scale = min(target_h / h, target_w / w)
        new_h, new_w = int(h * scale), int(w * scale)

        nchw = img.permute(2, 0, 1).unsqueeze(0).float()
        resized = F.interpolate(nchw, size=(new_h, new_w), mode="bilinear", align_corners=False)

        pad_top = (target_h - new_h) // 2
        pad_bottom = target_h - new_h - pad_top
        pad_left = (target_w - new_w) // 2
        pad_right = target_w - new_w - pad_left
        # F.pad order: (left, right, top, bottom) for 4-d NCHW
        padded = F.pad(resized, (pad_left, pad_right, pad_top, pad_bottom), value=0.0)

        return padded.squeeze(0).permute(1, 2, 0).to(img.dtype).contiguous()

    def preprocess(self, batch: Dict[str, Any]) -> Dict[str, Any]:
        images = batch["images"]
        state = batch["state"]

        prompt = batch.get("task") or self.default_prompt
        chassis = state["chassis"].float().reshape(-1)
        chassis = chassis[:3]

        state_tensor = torch.cat(
            [
                state["left_arm"].float().reshape(-1),       # 7
                state["right_arm"].float().reshape(-1),      # 7
                state["left_gripper"].float().reshape(-1),   # 1
                state["right_gripper"].float().reshape(-1),  # 1
                state["torso"].float().reshape(-1),          # 4
                chassis,                                     # 3
            ],
            dim=0,
        )

        if state_tensor.numel() != 23:
            raise ValueError(f"Expected 23-dim state, got shape {tuple(state_tensor.shape)}")

        head = self._to_hwc(images["head_rgb"])
        left = self._to_hwc(images["left_wrist_rgb"])
        right = self._to_hwc(images["right_wrist_rgb"])

        sz = self.image_target_size
        if sz > 0:
            head = self._resize_with_pad(head, sz, sz)
            left = self._resize_with_pad(left, sz, sz)
            right = self._resize_with_pad(right, sz, sz)

        obs = {
            "head_rgb": head,
            "left_wrist_rgb": left,
            "right_wrist_rgb": right,
            "state": state_tensor,
        }
        if prompt:
            obs["prompt"] = prompt
        return obs

    def postprocess(self, batch: Dict[str, Any]) -> Dict[str, Any]:
        actions = batch["actions"]
        if isinstance(actions, torch.Tensor):
            actions = actions.float().cpu()
        else:
            actions = torch.as_tensor(actions, dtype=torch.float32)

        if actions.ndim == 2:
            actions = actions.unsqueeze(0)

        if actions.shape[-1] < 23:
            raise ValueError(f"Expected OpenPI actions with at least 23 dims, got shape {tuple(actions.shape)}")

        chassis = actions[:, :, 20:23]
        if self.chassis_deadzone > 0:
            chassis = torch.where(chassis.abs() < self.chassis_deadzone, torch.zeros_like(chassis), chassis)

        action_dict = {
            "left_arm": actions[:, :, 0:7],
            "right_arm": actions[:, :, 7:14],
            "left_gripper": actions[:, :, 14:15],
            "right_gripper": actions[:, :, 15:16],
            "torso": actions[:, :, 16:20],
            "chassis": chassis,
        }
        return {"action": action_dict}
