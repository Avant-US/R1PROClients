import json
import pathlib
from enum import Enum
from std_msgs.msg import String

from utils.message.message_convert import decode_img_from_base64
import numpy as np
import torch
from loguru import logger

INSTRUCTION_PATH = pathlib.Path(__file__).resolve().parent / "instruction.txt"


def _load_bbox_utils():
    from utils.message.bbox_utils import (
        call_gemini_for_bbox,
        get_bbox_image,
        get_paligemma_box_instruction,
    )

    return call_gemini_for_bbox, get_bbox_image, get_paligemma_box_instruction

class InstructionAction(Enum):
    RESET = "reset"
    CONTINUE = "continue"
    SKIP = "skip"


class InstructionManager:
    def __init__(self, config):
        self.latest_bbox_dict = {"bbox": [], "head_img_base64": ""}
        self.text_instruction_file = INSTRUCTION_PATH
        self.instruction = ""
        
        self.last_instruction = ""
        self.extra_info = None  # 保存 extra_info 作为实例变量

        self.use_vlm = config["use_vlm"]
        self.image_as_condition = config["image_as_condition"]
        self.bbox_as_instruction = config["bbox_as_instruction"]
        self.image_condition_lang_prefix = config["image_condition_lang_prefix"]
        self.pp_lower_half = config["pp_lower_half"]

    def get_instruction(self, obs: dict) -> InstructionAction:
        if self.use_vlm:
            instruction, bbox, head_img_base64 = self._get_instruction_from_vlm()
        else:
            instruction = self._get_instruction_from_file()
        
        logger.info(f"instruction: {instruction}")
        if instruction in ['', 'nothing']:
            self.last_instruction = instruction
            obs["task"] = instruction
            return InstructionAction.SKIP
        
        elif instruction == "reset":
            self.last_instruction = instruction
            obs["task"] = instruction
            return InstructionAction.RESET
        
        elif instruction != self.last_instruction:
            if self.use_vlm:
                self.extra_info = self._get_extra_info_from_vlm(instruction, bbox, head_img_base64)
            else:
                self.extra_info = self._get_extra_info(instruction, obs["images"]["head_rgb"])

            if self.extra_info is not None:
                self.last_instruction = instruction
                if "image" in self.extra_info:
                    obs["images"]["head_condition"] = torch.from_numpy(self.extra_info["image"]).unsqueeze(0).cuda()
                if "instruction" in self.extra_info:
                    obs["task"] = self.extra_info["instruction"]
            return InstructionAction.CONTINUE
        else:
            if "image" in self.extra_info:
                obs["images"]["head_condition"] = torch.from_numpy(self.extra_info["image"]).unsqueeze(0).cuda()
            if "instruction" in self.extra_info:
                obs["task"] = self.extra_info["instruction"]

            return InstructionAction.CONTINUE

    def _get_instruction_from_vlm(self):
        latest_bbox_dict = self.latest_bbox_dict
        latest_bbox = latest_bbox_dict["bbox"]
        latest_head_img_base64 = latest_bbox_dict["head_img_base64"]
        return (self.instruction, latest_bbox, latest_head_img_base64)

    def _get_instruction_from_file(self):
        return self.text_instruction_file.read_text().replace('\n','')

    @staticmethod
    def _tensor_to_hwc_numpy(img: torch.Tensor) -> np.ndarray:
        """(1,C,H,W) or (C,H,W) or (1,H,W,C) / (H,W,C) → (H,W,C) uint8/float numpy."""
        t = img.detach().cpu()
        while t.ndim > 3 and t.shape[0] == 1:
            t = t.squeeze(0)
        if t.ndim == 3 and t.shape[0] in (1, 3):
            t = t.permute(1, 2, 0)
        elif t.ndim != 3 or t.shape[-1] not in (1, 3, 4):
            raise ValueError(
                f"Expected image tensor (1,C,H,W) or (H,W,C), got shape {tuple(img.shape)}"
            )
        return t.numpy()

    def _get_extra_info(self, instruction: str, head_rgb: torch.Tensor):
        extra_info = {}
        need_image = self.image_as_condition or self.bbox_as_instruction or self.pp_lower_half
        if need_image:
            head_rgb = self._tensor_to_hwc_numpy(head_rgb)
            if self.pp_lower_half:
                img_height = head_rgb.shape[0]
                head_rgb = head_rgb[img_height // 2 :, :, :]
        if self.image_as_condition:
            call_gemini_for_bbox, get_bbox_image, _ = _load_bbox_utils()
            bbox = call_gemini_for_bbox(head_rgb, instruction)
            condition_image = get_bbox_image(
                head_rgb, 
                bbox, 
            )
            system_instruction = self.image_condition_lang_prefix
            extra_info["image"] = condition_image
            extra_info["instruction"] = system_instruction
        elif self.bbox_as_instruction:
            call_gemini_for_bbox, _, get_paligemma_box_instruction = _load_bbox_utils()
            bbox = call_gemini_for_bbox(head_rgb, instruction)
            paligemma_instrctuion = get_paligemma_box_instruction(head_rgb, bbox)
            extra_info["instruction"] = paligemma_instrctuion
        else:
            extra_info["instruction"] = instruction
        return extra_info

    def _get_extra_info_from_vlm(self, instruction: str, bbox: list[int], head_img_base64: str):
        extra_info = {}
        if head_img_base64 == "":
            return None
        latest_head_rgb = decode_img_from_base64(head_img_base64, output_format="rgb")
        if self.pp_lower_half:
            img_height = latest_head_rgb.shape[0]
            latest_head_rgb = latest_head_rgb[img_height//2:, :, :]
        if self.image_as_condition:
            _, get_bbox_image, _ = _load_bbox_utils()
            condition_image = get_bbox_image(
                latest_head_rgb, 
                bbox, 
            )
            system_instruction = self.image_condition_lang_prefix
            extra_info["image"] = condition_image
            extra_info["instruction"] = system_instruction
        elif self.bbox_as_instruction:
            _, _, get_paligemma_box_instruction = _load_bbox_utils()
            paligemma_instrctuion = get_paligemma_box_instruction(latest_head_rgb, bbox)
            extra_info["instruction"] = paligemma_instrctuion
        else:
            extra_info["instruction"] = instruction
        return extra_info

    def _refine_ll_instruction(self, instruction):
        if '[Low]' in instruction or instruction in ['reset', 'stop']:
            return instruction
        elif '[low]' in instruction:
            return instruction.replace("[low]", "[Low]")
        else:
            return f"[Low]:{instruction}"
    
    def _ehi_instruction_callback(self, msg: String):
        vlm_data_dict = json.loads(msg.data)
        bbox_dict = {}
        lower_prompt_list = vlm_data_dict["lower_prompt_list"]
        bbox_dict["bbox"] = vlm_data_dict["bbox"]
        bbox_dict["head_img_base64"] = vlm_data_dict["head_img_base64"]
        
        if not len(lower_prompt_list):
            return

        low_level_instruction = lower_prompt_list[0]
        self.instruction = self._refine_ll_instruction(low_level_instruction)
        self.latest_bbox_dict = bbox_dict

