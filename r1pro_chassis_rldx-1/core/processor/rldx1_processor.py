"""Processor for the rldx-1 VLA protocol.

跟 openpi 主要差别：
  - 每路相机要给 4 帧（delta_indices=[-6, -4, -2, 0]），shape (B, T, H, W, 3) uint8
  - state 按部位拆开，每个都是 (B, 1, D) float32
  - chassis_pose 实际就是 torso（4 维），chassis_velocity 就是底盘速度（3 维）
  - language.task 是 list[list[str]]，shape (B, 1)
服务端返回的 action 也是按部位拆好的 dict（key 见 [rldx1.action_key_map]）。
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import torch
import torch.nn.functional as F
from loguru import logger

from core.processor.processor import Processor


_DEFAULT_DELTA_INDICES: List[int] = [-6, -4, -2, 0]

# rldx-1 action dict key  →  r1pro 发布通道的 part key
_DEFAULT_ACTION_KEY_MAP: Dict[str, str] = {
    "left_arm": "left_arm",
    "right_arm": "right_arm",
    "left_gripper": "left_gripper",
    "right_gripper": "right_gripper",
    "chassis_pose": "torso",          # rldx-1 把 torso 叫 chassis_pose
    "chassis_velocity": "chassis",    # 底盘速度
}


class RLDX1Processor(Processor):
    """Adapter between R1Pro observations and the rldx-1 server API."""

    def __init__(self, config: Dict[str, Any], cfg):
        super().__init__(config, cfg)
        rldx_cfg = config.get("rldx1", {})

        self.default_prompt: str = rldx_cfg.get("default_prompt", "") or ""

        delta_indices = list(rldx_cfg.get("delta_indices", _DEFAULT_DELTA_INDICES))
        if not delta_indices:
            raise ValueError("rldx1.delta_indices must not be empty")
        if max(delta_indices) > 0:
            raise ValueError(
                f"rldx1.delta_indices must all be <= 0 (0 = current frame), got {delta_indices}"
            )
        self.delta_indices: List[int] = delta_indices
        self.history_len: int = -min(delta_indices) + 1
        logger.info(
            f"rldx1 video delta_indices={self.delta_indices}, history buffer length={self.history_len}"
        )

        self.chassis_deadzone: float = float(rldx_cfg.get("chassis_deadzone", 0.0))
        if self.chassis_deadzone > 0:
            logger.info(f"Chassis velocity deadzone: {self.chassis_deadzone}")

        target_cfg = rldx_cfg.get("image_target_hw", {})
        self.image_target_hw: Dict[str, Tuple[int, int] | None] = {
            "head_rgb": self._parse_hw(target_cfg.get("head_rgb")),
            "left_wrist_rgb": self._parse_hw(target_cfg.get("left_wrist_rgb")),
            "right_wrist_rgb": self._parse_hw(target_cfg.get("right_wrist_rgb")),
        }
        for name, hw in self.image_target_hw.items():
            if hw is not None:
                logger.info(f"{name}: center-crop+resize to (H={hw[0]}, W={hw[1]})")
            else:
                logger.info(f"{name}: no client-side resize (send native)")

        # 模型 action dict 的 key → 本机 publish 路径上的 part key
        key_map_cfg = rldx_cfg.get("action_key_map") or {}
        self.action_key_map: Dict[str, str] = {**_DEFAULT_ACTION_KEY_MAP, **dict(key_map_cfg)}
        logger.info(f"rldx1 action_key_map = {self.action_key_map}")

        self._video_history: Dict[str, deque] = {
            "head_rgb": deque(maxlen=self.history_len),
            "left_wrist_rgb": deque(maxlen=self.history_len),
            "right_wrist_rgb": deque(maxlen=self.history_len),
        }

    # ---------------------------------------------------------------- helpers

    @staticmethod
    def _parse_hw(value: Any) -> Tuple[int, int] | None:
        if value is None:
            return None
        if len(value) != 2:
            raise ValueError(f"image_target_hw entry must be [H, W], got {value}")
        return (int(value[0]), int(value[1]))

    @staticmethod
    def _to_hwc(img: torch.Tensor) -> torch.Tensor:
        """Convert CHW (or 1xCHW) → HWC."""
        t = img.squeeze(0)
        if t.ndim == 3 and t.shape[0] in (1, 3):
            t = t.permute(1, 2, 0)
        return t.contiguous()

    @staticmethod
    def _center_crop_resize(img: torch.Tensor, target_h: int, target_w: int) -> torch.Tensor:
        h, w, _ = img.shape
        target_ratio = target_w / target_h
        current_ratio = w / h

        if abs(current_ratio - target_ratio) > 1e-3:
            if current_ratio > target_ratio:
                new_w = int(h * target_ratio)
                offset = (w - new_w) // 2
                img = img[:, offset:offset + new_w, :].contiguous()
            else:
                new_h = int(w / target_ratio)
                offset = (h - new_h) // 2
                img = img[offset:offset + new_h, :, :].contiguous()

        if img.shape[0] == target_h and img.shape[1] == target_w:
            return img
        nchw = img.permute(2, 0, 1).unsqueeze(0).float()
        resized = F.interpolate(nchw, size=(target_h, target_w), mode="bilinear", align_corners=False)
        return resized.squeeze(0).permute(1, 2, 0).to(img.dtype).contiguous()

    def _process_camera(self, name: str, img: torch.Tensor) -> np.ndarray:
        """把当前帧加入历史 buffer，再按 delta_indices 抽 4 帧，返回 (T, H, W, 3) uint8。

        历史不够时（任务刚开始），按 eval 脚本的做法 clamp 到 buffer[0]。
        """
        hwc = self._to_hwc(img)
        if (hw := self.image_target_hw[name]) is not None:
            hwc = self._center_crop_resize(hwc, *hw)
        hwc = hwc.to(torch.uint8).cpu()

        buf = self._video_history[name]
        buf.append(hwc)

        n = len(buf)
        frames: List[torch.Tensor] = []
        for delta in self.delta_indices:
            # delta <= 0; 0 = 当前帧。idx 是「从最新往回数」后落到的 buffer 位置。
            idx = n - 1 + delta
            if idx < 0:
                idx = 0  # clamp 到当前 buffer 里最旧的一帧
            frames.append(buf[idx])
        stacked = torch.stack(frames, dim=0).numpy()
        return stacked  # (T, H, W, 3) uint8

    @staticmethod
    def _state_to_2d(value: Any, expected_dim: int, name: str) -> np.ndarray:
        """把 obs["state"][name] 的张量转成 (1, 1, expected_dim) float32 numpy。"""
        if isinstance(value, torch.Tensor):
            arr = value.detach().float().cpu().reshape(-1).numpy()
        else:
            arr = np.asarray(value, dtype=np.float32).reshape(-1)
        if arr.size < expected_dim:
            pad = np.zeros(expected_dim - arr.size, dtype=np.float32)
            arr = np.concatenate([arr, pad])
        elif arr.size > expected_dim:
            arr = arr[:expected_dim]
        return arr.astype(np.float32).reshape(1, 1, expected_dim)

    # ---------------------------------------------------------------- public

    def initialize(self, dataset_stats_path: Path | None) -> None:
        logger.info("Initializing rldx-1 processor")

    def reset_history(self) -> None:
        """清空 4 帧视频历史 buffer。任务切换/重新启动时建议调用。"""
        for buf in self._video_history.values():
            buf.clear()
        logger.info("rldx1 video history cleared")

    # ---------------------------------------------------------------- pre/post

    def preprocess(self, batch: Dict[str, Any]) -> Dict[str, Any]:
        images = batch["images"]
        state = batch["state"]

        prompt = batch.get("task") or self.default_prompt or ""

        head = self._process_camera("head_rgb", images["head_rgb"])
        left = self._process_camera("left_wrist_rgb", images["left_wrist_rgb"])
        right = self._process_camera("right_wrist_rgb", images["right_wrist_rgb"])

        # 顶层加 batch 维 → (1, T, H, W, 3)
        head = head[np.newaxis, ...]
        left = left[np.newaxis, ...]
        right = right[np.newaxis, ...]

        # state: 全部拍到 (1, 1, D)。chassis_pose 实际取 torso（4 维），
        # chassis_velocity 取底盘速度（3 维）。
        left_arm = self._state_to_2d(state["left_arm"], 7, "left_arm")
        right_arm = self._state_to_2d(state["right_arm"], 7, "right_arm")
        left_gripper = self._state_to_2d(state["left_gripper"], 1, "left_gripper")
        right_gripper = self._state_to_2d(state["right_gripper"], 1, "right_gripper")
        chassis_pose = self._state_to_2d(state["torso"], 4, "torso (-> chassis_pose)")
        chassis_velocity = self._state_to_2d(state["chassis"], 3, "chassis (-> chassis_velocity)")

        observation = {
            "video": {
                "head_rgb": head,
                "left_wrist_rgb": left,
                "right_wrist_rgb": right,
            },
            "state": {
                "left_arm": left_arm,
                "right_arm": right_arm,
                "left_gripper": left_gripper,
                "right_gripper": right_gripper,
                "chassis_pose": chassis_pose,
                "chassis_velocity": chassis_velocity,
            },
            "language": {
                "task": [[prompt]],
            },
        }
        return observation

    # Default per-part action dimensions matching the training protocol.
    # Used only when the server returns a flat tensor instead of a keyed dict.
    _DEFAULT_ACTION_SPLIT: List[Tuple[str, int]] = [
        ("left_arm",        7),
        ("right_arm",       7),
        ("left_gripper",    1),
        ("right_gripper",   1),
        ("chassis_pose",    4),
        ("chassis_velocity", 3),
    ]

    def postprocess(self, batch: Dict[str, Any]) -> Dict[str, Any]:
        """服务端返回按部位拆好的 dict，例如 left_arm/right_arm/.../chassis_pose/chassis_velocity。

        也能处理服务端直接返回 flat tensor (T, D) 或 (B, T, D) 的情况：
        按 _DEFAULT_ACTION_SPLIT 拆分每个部位。
        """
        # ── flat tensor fallback ──────────────────────────────────────────
        if not isinstance(batch, dict):
            logger.warning(
                f"rldx1 postprocess: server returned {type(batch).__name__} "
                f"instead of dict — trying flat-tensor split"
            )
            batch = self._split_flat_action(batch)

        action_dict: Dict[str, torch.Tensor] = {}

        for src_key, dst_key in self.action_key_map.items():
            if src_key not in batch:
                continue
            tensor = self._to_action_tensor(batch[src_key])
            action_dict[dst_key] = tensor

        if not action_dict:
            raise RuntimeError(
                f"rldx1 postprocess: response 中找不到任何已知的 action 字段，"
                f"keys={list(batch.keys())}, expected one of {list(self.action_key_map.keys())}"
            )

        if "chassis" in action_dict and self.chassis_deadzone > 0:
            c = action_dict["chassis"]
            action_dict["chassis"] = torch.where(
                c.abs() < self.chassis_deadzone, torch.zeros_like(c), c
            )

        return {"action": action_dict}

    def _split_flat_action(self, value: Any) -> Dict[str, Any]:
        """把 flat tensor (T, D) 或 (B, T, D) 或 list 按 _DEFAULT_ACTION_SPLIT 拆分成 dict。"""
        if isinstance(value, (list, tuple)):
            try:
                value = torch.as_tensor(np.asarray(value, dtype=np.float32))
            except Exception as e:
                raise RuntimeError(
                    f"rldx1: cannot convert list response to tensor for splitting: {e}"
                ) from e
        if isinstance(value, np.ndarray):
            value = torch.from_numpy(value.copy().astype(np.float32))
        if not isinstance(value, torch.Tensor):
            raise RuntimeError(
                f"rldx1: unexpected server response type {type(value).__name__}, cannot split"
            )

        t = value.float().cpu()
        # Accept (D,), (T, D), or (B, T, D)
        if t.ndim == 1:
            t = t.unsqueeze(0)  # (1, D)
        if t.ndim == 3:
            t = t[0]  # take first batch dim → (T, D)
        # t is now (T, D)
        total_expected = sum(d for _, d in self._DEFAULT_ACTION_SPLIT)
        if t.shape[-1] != total_expected:
            raise RuntimeError(
                f"rldx1: flat action last dim {t.shape[-1]} != expected {total_expected} "
                f"(split={self._DEFAULT_ACTION_SPLIT})"
            )
        logger.info(f"rldx1: splitting flat action shape={tuple(t.shape)} by parts")
        result: Dict[str, torch.Tensor] = {}
        offset = 0
        for key, dim in self._DEFAULT_ACTION_SPLIT:
            result[key] = t[:, offset:offset + dim]
            offset += dim
        return result

    @staticmethod
    def _to_action_tensor(value: Any) -> torch.Tensor:
        """把模型回来的张量整理成 (1, T, D) float32。

        支持输入 shape：
          (D,)        → (1, 1, D)
          (T, D)      → (1, T, D)
          (B, T, D)   → 原样（仅取 B=1 时）
        """
        if isinstance(value, torch.Tensor):
            t = value.detach().float().cpu()
        else:
            t = torch.as_tensor(np.asarray(value), dtype=torch.float32)
        if t.ndim == 1:
            t = t.unsqueeze(0).unsqueeze(0)
        elif t.ndim == 2:
            t = t.unsqueeze(0)
        elif t.ndim == 3:
            pass
        else:
            raise ValueError(f"Unexpected action tensor ndim={t.ndim}, shape={tuple(t.shape)}")
        return t
