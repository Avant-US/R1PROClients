from typing import Dict, Any, Optional
from omegaconf import DictConfig
from loguru import logger

from core.processor.processor import Processor
from core.processor.openpi_processor import OpenPIProcessor

def create_processor(
    config: Dict[str, Any],
    cfg: DictConfig,
    processor_type: Optional[str] = None
) -> Processor:
    if processor_type is None:
        processor_type = "default"
    
    processor_type = processor_type.lower()
    
    if processor_type == "default":
        from core.processor.base_processor import BaseProcessor

        logger.info("Creating base processor")
        return BaseProcessor(config=config, cfg=cfg)
    elif processor_type == "cuda":
        from core.processor.cuda_processor import CUDAProcessor

        logger.info("Creating cuda processor")
        return CUDAProcessor(config=config, cfg=cfg)
    elif processor_type == "openpi":
        logger.info("Creating OpenPI processor")
        return OpenPIProcessor(config=config, cfg=cfg)
    elif processor_type == "rldx1":
        from core.processor.rldx1_processor import RLDX1Processor

        logger.info("Creating rldx-1 processor")
        return RLDX1Processor(config=config, cfg=cfg)
    else:
        raise ValueError(
            f"Unsupported processor type: {processor_type}. "
            "Supported types: 'default', 'cuda', 'openpi', 'rldx1'"
        )
