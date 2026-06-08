from pathlib import Path
from typing import Any, Dict

from loguru import logger
import torch
import torchvision.transforms.functional as TVF

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
            logger.info(f"Client-side stretch resize to {self.image_target_size}x{self.image_target_size}")

        # 每路相机的目标 (H, W)，用 stretch resize 对齐训练时的图像预处理。
        # 优先级高于正方形的 image_target_size。
        target_cfg = config.get("openpi", {}).get("image_target_hw", {})
        self.image_target_hw: Dict[str, tuple[int, int] | None] = {
            "head_rgb": self._parse_hw(target_cfg.get("head_rgb")),
            "left_wrist_rgb": self._parse_hw(target_cfg.get("left_wrist_rgb")),
            "right_wrist_rgb": self._parse_hw(target_cfg.get("right_wrist_rgb")),
        }
        for name, hw in self.image_target_hw.items():
            if hw is not None:
                logger.info(f"{name}: stretch resize to (H={hw[0]}, W={hw[1]})")

    @staticmethod
    def _parse_hw(value: Any) -> "tuple[int, int] | None":
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
    def _resize_stretch(img: torch.Tensor, target_h: int, target_w: int) -> torch.Tensor:
        """Resize to (target_h, target_w), matching FastWAM's torchvision.transforms.Resize.

        Uses bilinear interpolation with antialias=True so the result aligns with the
        cloud-side FastWAM training/inference pipeline. dtype is preserved (uint8 in → uint8 out).
        """
        chw = img.permute(2, 0, 1)  # HWC -> CHW
        resized = TVF.resize(
            chw,
            size=[target_h, target_w],
            interpolation=TVF.InterpolationMode.BILINEAR,
            antialias=True,
        )
        return resized.permute(1, 2, 0).contiguous()  # CHW -> HWC, dtype unchanged

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
        # print("before crop:", head.shape)
        # head = head[600:, :, :]
        # print("after crop:", head.shape)
        left = self._to_hwc(images["left_wrist_rgb"])
        right = self._to_hwc(images["right_wrist_rgb"])

        # 优先用按相机指定的 (H, W)；否则回退到正方形 image_target_size。
        if (hw := self.image_target_hw["head_rgb"]) is not None:
            head = self._resize_stretch(head, *hw)
        elif self.image_target_size > 0:
            head = self._resize_stretch(head, self.image_target_size, self.image_target_size)

        if (hw := self.image_target_hw["left_wrist_rgb"]) is not None:
            left = self._resize_stretch(left, *hw)
        elif self.image_target_size > 0:
            left = self._resize_stretch(left, self.image_target_size, self.image_target_size)

        if (hw := self.image_target_hw["right_wrist_rgb"]) is not None:
            right = self._resize_stretch(right, *hw)
        elif self.image_target_size > 0:
            right = self._resize_stretch(right, self.image_target_size, self.image_target_size)

        obs = {
            "head_rgb": head,
            "left_wrist_rgb": left,
            "right_wrist_rgb": right,
            "state": state_tensor,
            "ref_time": batch["ref_time"],
            # "role": "image",
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
