from pathlib import Path
from typing import Any, Dict

import torch
import torch.nn.functional as F
from loguru import logger

from core.processor.processor import Processor


class OpenPIProcessor(Processor):
    """Adapter between R1Pro observations and OpenPI's API."""

    def __init__(self, config: Dict[str, Any], cfg):
        super().__init__(config, cfg)
        self.default_prompt = config.get("openpi", {}).get("default_prompt")
        self.chassis_deadzone = config.get("openpi", {}).get("chassis_deadzone", 0.0)
        if self.chassis_deadzone > 0:
            logger.info(f"Chassis velocity deadzone: {self.chassis_deadzone}")

        # 每路相机的目标 (H, W)。client 端做：中心裁到目标 aspect → bilinear resize 到 (H, W)。
        # 训练集形状是 (3, H, W)（CHW），这里 config 写 [H, W] 直接对齐。
        target_cfg = config.get("openpi", {}).get("image_target_hw", {})
        self.image_target_hw: Dict[str, tuple[int, int] | None] = {
            "head_rgb": self._parse_hw(target_cfg.get("head_rgb")),
            "left_wrist_rgb": self._parse_hw(target_cfg.get("left_wrist_rgb")),
            "right_wrist_rgb": self._parse_hw(target_cfg.get("right_wrist_rgb")),
        }
        for name, hw in self.image_target_hw.items():
            if hw is not None:
                logger.info(f"{name}: center-crop+resize to (H={hw[0]}, W={hw[1]})")
            else:
                logger.info(f"{name}: no client-side resize (send native)")

    @staticmethod
    def _parse_hw(value: Any) -> tuple[int, int] | None:
        if value is None:
            return None
        if len(value) != 2:
            raise ValueError(f"image_target_hw entry must be [H, W], got {value}")
        return (int(value[0]), int(value[1]))

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
    def _center_crop_resize(img: torch.Tensor, target_h: int, target_w: int) -> torch.Tensor:
        """Center-crop HWC image to the target aspect, then bilinear resize to (target_h, target_w).

        与 _resize_with_pad 的区别：不做黑色 padding，直接对齐目标 aspect 后拉伸到精确尺寸。
        DM0 服务端期望客户端发的就是训练集分辨率，所以这里输出 = (target_h, target_w, C)。
        Input & output: HWC uint8 tensor.
        """
        h, w, c = img.shape
        target_ratio = target_w / target_h
        current_ratio = w / h

        # Step 1: center-crop to target aspect ratio.
        if abs(current_ratio - target_ratio) > 1e-3:
            if current_ratio > target_ratio:
                new_w = int(h * target_ratio)
                offset = (w - new_w) // 2
                img = img[:, offset:offset + new_w, :].contiguous()
            else:
                new_h = int(w / target_ratio)
                offset = (h - new_h) // 2
                img = img[offset:offset + new_h, :, :].contiguous()

        # Step 2: bilinear resize to exact (target_h, target_w).
        if img.shape[0] == target_h and img.shape[1] == target_w:
            return img
        nchw = img.permute(2, 0, 1).unsqueeze(0).float()
        resized = F.interpolate(nchw, size=(target_h, target_w), mode="bilinear", align_corners=False)
        return resized.squeeze(0).permute(1, 2, 0).to(img.dtype).contiguous()

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

        if (hw := self.image_target_hw["head_rgb"]) is not None:
            head = self._center_crop_resize(head, *hw)
        if (hw := self.image_target_hw["left_wrist_rgb"]) is not None:
            left = self._center_crop_resize(left, *hw)
        if (hw := self.image_target_hw["right_wrist_rgb"]) is not None:
            right = self._center_crop_resize(right, *hw)

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
