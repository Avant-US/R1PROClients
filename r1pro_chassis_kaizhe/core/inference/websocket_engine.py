import logging
import time
from typing import Dict, Optional, Tuple, Any

from typing_extensions import override
import websockets.sync.client

from core.inference.inference_engine import InferenceEngine
from omegaconf import DictConfig
from utils.websocket.msgpack import Packer, unpackb
from utils.torch_utils import dict_apply
import torch
import numpy as np
import os

class WebSocketClientEngine(InferenceEngine):
    def __init__(self, config: Dict[str, Any], cfg: DictConfig):
        super().__init__(config, cfg)
        host = config["websocket"]["host"]
        port = config["websocket"]["port"]

        if host.startswith("ws"):
            self._uri = host
        else:
            self._uri = f"ws://{host}"
        if port is not None:
            self._uri += f":{port}"
        self._packer = Packer()

    def load_model(self):
        self._ws, self._server_metadata = self._wait_for_server()

    def _wait_for_server(self) -> Tuple[websockets.sync.client.ClientConnection, Dict]:
        logging.info(f"Waiting for server at {self._uri}...")
        old_proxy_settings = {
                'http_proxy': os.environ.get('http_proxy'),
                'https_proxy': os.environ.get('https_proxy'),
                'all_proxy': os.environ.get('all_proxy'), 
                "HTTP_PROXY": os.environ.get('HTTP_PROXY'),
                "HTTPS_PROXY": os.environ.get('HTTPS_PROXY'),
        }

        while True:
            try:
                if 'http_proxy' in os.environ:
                    del os.environ['http_proxy']
                if 'https_proxy' in os.environ:
                    del os.environ['https_proxy']
                if 'all_proxy' in os.environ:
                    del os.environ['all_proxy']
                if 'HTTP_PROXY' in os.environ:
                    del os.environ['HTTP_PROXY']
                if 'HTTPS_PROXY' in os.environ:
                    del os.environ['HTTPS_PROXY']
                headers = {"Authorization": f"Api-Key "}
                conn = websockets.sync.client.connect(
                    self._uri, compression=None, max_size=None, additional_headers=headers
                )
                metadata = unpackb(conn.recv())
                return conn, metadata
            except ConnectionRefusedError:
                logging.info("Still waiting for server...")
                time.sleep(5)
            finally:
                for key, value in old_proxy_settings.items():
                    if value is not None:
                        os.environ[key] = value
                    elif key in os.environ:
                        del os.environ[key]

    @override
    def predict_action(self, batch: Dict) -> Dict:  # noqa: UP006
        t0 = time.time()
        batch = dict_apply(batch, lambda x: x.cpu().numpy() if isinstance(x, torch.Tensor) else x)
        data = self._packer.pack(batch)
        payload_kb = len(data) / 1024
        t1 = time.time()
        self._ws.send(data)
        t2 = time.time()
        response = self._ws.recv()
        t3 = time.time()
        if isinstance(response, str):
            raise RuntimeError(f"Error in inference server:\n{response}")
        action = unpackb(response)
        action = dict_apply(
            action,
            lambda x: torch.from_numpy(x.copy()) if isinstance(x, np.ndarray) else x,
        )
        t4 = time.time()
        logging.info(
            f'WS detail: pack={1000*(t1-t0):.0f}ms  '
            f'send={1000*(t2-t1):.0f}ms  '
            f'wait+recv={1000*(t3-t2):.0f}ms  '
            f'unpack={1000*(t4-t3):.0f}ms  '
            f'payload={payload_kb:.0f}KB'
        )
        return action
