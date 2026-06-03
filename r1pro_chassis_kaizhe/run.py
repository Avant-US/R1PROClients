import argparse
import json
import os
import sys
import threading
import toml
from http.server import HTTPServer, BaseHTTPRequestHandler
from scheduler.scheduler import Scheduler


def load_config(model_path=None):
    """Load config from model_path/efmnode.toml if available, else default config.toml.

    When model_path is provided:
      - Use <model_path>/efmnode.toml if it exists
      - Otherwise fall back to default config.toml with a warning
      - Always override ckpt_dir to point to model_path
    """
    default_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.toml")

    if model_path is not None:
        model_config_path = os.path.join(model_path, "efmnode.toml")
        if os.path.isfile(model_config_path):
            print(f"[INFO] Loading config from: {model_config_path}")
            config = toml.load(model_config_path)
        else:
            print(f"[WARNING] {model_config_path} not found, falling back to default config.toml", file=sys.stderr)
            config = toml.load(default_config_path)

        config.setdefault("model", {})
        config["model"]["ckpt_dir"] = model_path
        print(f"[INFO] Model checkpoint dir: {model_path}")
    else:
        config = toml.load(default_config_path)

    return config


_scheduler: "Scheduler | None" = None


class TaskHTTPHandler(BaseHTTPRequestHandler):
    """供 robot-agent 调用的 HTTP 接口。"""

    def _json_response(self, code: int, data: dict):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length))

    # ---------- GET ----------
    def do_GET(self):
        if self.path == "/health":
            self._json_response(200, {"status": "ok"})
        elif self.path == "/status":
            self._json_response(200, _scheduler.get_task_status())
        elif self.path == "/obs":
            if _scheduler._recording:
                result = _scheduler.stop_recording()
                self._json_response(200, {"recording": "stopped", "count": result["count"], "message": f"已保存到 /tmp/recorded_obs.json"})
                import json as _json
                with open("/tmp/recorded_obs.json", "w") as f:
                    _json.dump(result, f, indent=2, ensure_ascii=False)
            else:
                _scheduler.start_recording()
                self._json_response(200, {"recording": "started", "message": "开始录制，再次请求 /obs 停止并保存"})
        else:
            self._json_response(404, {"error": "not found"})

    # ---------- POST ----------
    def do_POST(self):
        if self.path == "/start":
            body = self._read_body()
            instruction = body.get("instruction", "").strip()
            if not instruction:
                self._json_response(400, {"error": "instruction is required"})
                return
            timeout = body.get("timeout", 120.0)
            _scheduler.start_task(instruction, timeout=float(timeout))
            self._json_response(200, {
                "status": "executing",
                "message": "正在执行",
            })

        elif self.path == "/stop":
            _scheduler.stop_task()
            self._json_response(200, {
                "status": "idle",
                "message": "任务已停止",
            })

        else:
            self._json_response(404, {"error": "not found"})

    def log_message(self, format, *args):
        print(f"[HTTP] {args[0]}")


def _run_http_server(port: int):
    server = HTTPServer(("0.0.0.0", port), TaskHTTPHandler)
    print(f"[INFO] HTTP server listening on 0.0.0.0:{port}")
    server.serve_forever()


def main():
    parser = argparse.ArgumentParser(description="R1Pro VLA inference client")
    parser.add_argument("--model-path", type=str, default=None,
                        help="Absolute path to model directory (overrides ckpt_dir in config)")
    parser.add_argument("--http-port", type=int, default=9001,
                        help="Port for the HTTP control API (default: 9001)")
    args = parser.parse_args()

    global _scheduler
    config = load_config(args.model_path)
    _scheduler = Scheduler(config)

    http_thread = threading.Thread(target=_run_http_server, args=(args.http_port,), daemon=True)
    http_thread.start()

    _scheduler.run()


if __name__ == "__main__":
    main()
