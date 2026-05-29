import os
import signal
import sys
import subprocess
import threading
import time
import json
import urllib.request
from contextlib import asynccontextmanager
from enum import Enum
from typing import Optional

from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn


# ─── 配置 ────────────────────────────────────────────────────────────

EFMNODE_URL = "http://localhost:9001"
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

_ROS2_ENV = os.environ.copy()
_ROS2_ENV.update({
    "ROS_DISTRO": "humble",
    "ROS_VERSION": "2",
    "ROS_PYTHON_VERSION": "3",
    "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
    # 不要硬编码 ROS_LOCALHOST_ONLY!
    # 本机相机发布者 (signal_camera/HDAS) 启动时用的 =1,
    # 如果这里强行设 =0, FastDDS SHM segment 配置不匹配, 收不到任何数据
    "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY", "1"),
})

# --wait-matching-subscriptions 0: 不等订阅者出现，发完就走
# 旧机器有 relaxed_ik 等节点订阅; 新机器只有 chassis 自己订阅, 关掉后没人收, 默认会卡 10s 超时
_INIT_COMMANDS = [
    'ros2 topic pub --once --wait-matching-subscriptions 0 /motion_target/target_joint_state_torso '
    'sensor_msgs/msg/JointState "{position: [0.9,-1.5, -0.70, 0.0]}"',
    'ros2 topic pub --once --wait-matching-subscriptions 0 /motion_target/target_joint_state_arm_left '
    'sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"',
    'ros2 topic pub --once --wait-matching-subscriptions 0 /motion_target/target_joint_state_arm_right '
    'sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"',
]

_RESET_COMMANDS = [
    'ros2 topic pub --once --wait-matching-subscriptions 0 /motion_target/target_joint_state_arm_left '
    'sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"',
    'ros2 topic pub --once --wait-matching-subscriptions 0 /motion_target/target_joint_state_arm_right '
    'sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"',
    'ros2 topic pub --once --wait-matching-subscriptions 0 /motion_target/target_joint_state_torso '
    'sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0]}"',
]


# ─── 状态 ────────────────────────────────────────────────────────────

class TaskState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"


class _GlobalState:
    def __init__(self):
        self.task_state: TaskState = TaskState.IDLE
        self.task_message: str = ""
        self.task_instruction: str = ""
        self.run_py_proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._worker: Optional[threading.Thread] = None
        self.ros2_ready: bool = False
        self.ros2_missing_topics: list[str] = []


_state = _GlobalState()

_REQUIRED_TOPICS = [
    "/hdas/feedback_arm_left",
    "/hdas/feedback_arm_right",
    "/hdas/feedback_torso",
    "/hdas/feedback_chassis",
    "/hdas/feedback_gripper_left",
    "/hdas/feedback_gripper_right",
    "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "/hdas/camera_wrist_left/color/image_raw/compressed",
    "/hdas/camera_wrist_right/color/image_raw/compressed",
]


def _check_ros2_topics() -> tuple[bool, list[str]]:
    """检查 HDAS 硬件反馈话题是否存在，返回 (全部就绪, 缺失列表)。"""
    try:
        result = subprocess.run(
            "ros2 topic list",
            shell=True, capture_output=True, text=True,
            timeout=10, env=_ROS2_ENV,
        )
        if result.returncode != 0:
            print(f"[client] ros2 topic list 执行失败: {result.stderr.strip()}")
            return False, _REQUIRED_TOPICS[:]

        active_topics = set(result.stdout.strip().splitlines())
        missing = [t for t in _REQUIRED_TOPICS if t not in active_topics]
        return len(missing) == 0, missing

    except Exception as e:
        print(f"[client] ROS2 话题检查异常: {e}")
        return False, _REQUIRED_TOPICS[:]


# ─── 内部工具 ─────────────────────────────────────────────────────────

def _send_ros2_commands(commands: list[str], label: str, retries: int = 3):
    for cmd in commands:
        topic = cmd.split("/motion_target/")[1].split()[0]
        for attempt in range(1, retries + 1):
            try:
                print(f"[client] {label}: {topic} (attempt {attempt}/{retries})", flush=True)
                subprocess.run(cmd, shell=True, timeout=10, env=_ROS2_ENV)
                break
            except subprocess.TimeoutExpired:
                print(f"[client] {label}: {topic} 超时 (attempt {attempt}/{retries})", flush=True)
                if attempt == retries:
                    raise
        time.sleep(0.5)


def _efm_get(path: str) -> dict | None:
    try:
        with urllib.request.urlopen(f"{EFMNODE_URL}{path}", timeout=3) as resp:
            return json.loads(resp.read())
    except Exception:
        return None


def _efm_post(path: str, data: dict) -> dict | None:
    body = json.dumps(data).encode()
    req = urllib.request.Request(
        f"{EFMNODE_URL}{path}",
        data=body,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read())
    except Exception:
        return None


def _wait_efm_ready(timeout: float = 120.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _efm_get("/health") is not None:
            return True
        time.sleep(2)
    return False


def _ensure_run_py() -> bool:
    """确保 run.py 正在运行，返回是否就绪。"""
    if _efm_get("/health") is not None:
        return True

    print("[client] run.py 未运行，正在启动...")
    cmd = [sys.executable, "run.py"]
    _state.run_py_proc = subprocess.Popen(
        cmd,
        cwd=PROJECT_DIR,
        env=_ROS2_ENV,
        start_new_session=True,
    )
    if not _wait_efm_ready():
        print("[client] run.py 启动超时")
        return False
    print("[client] run.py 已就绪")
    return True


def _kill_run_py():
    """强制终止 run.py 进程组，兜底 pkill 清除残留。"""
    proc = _state.run_py_proc
    if proc is not None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass
        try:
            proc.wait(timeout=5)
        except Exception:
            pass
        _state.run_py_proc = None
        print("[client] run.py 已关闭")

    subprocess.run(
        f"pkill -9 -f '{sys.executable} run.py'",
        shell=True, timeout=5, env=_ROS2_ENV,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def _task_worker(instruction: str, timeout: float, poll_interval: float):
    """在后台线程执行完整的任务流程。"""
    try:
        if not _ensure_run_py():
            _state.task_state = TaskState.FAILED
            _state.task_message = "run.py 启动失败"
            return

        _efm_post("/stop", {})
        time.sleep(0.3)

        for _i in range(3):
            _send_ros2_commands(_INIT_COMMANDS, f"初始姿态(#{_i+1})")
            time.sleep(0.5)
        time.sleep(0.5)

        resp = _efm_post("/start", {"instruction": instruction, "timeout": timeout})
        if resp is None:
            _state.task_state = TaskState.FAILED
            _state.task_message = "发送 /start 失败"
            return
        print(f"[client] 任务已下发: {instruction}")

        while _state.task_state == TaskState.RUNNING:
            time.sleep(poll_interval)
            status = _efm_get("/status")
            if status is None:
                _state.task_state = TaskState.FAILED
                _state.task_message = "与 run.py 连接丢失"
                return
            if status.get("done"):
                success = status.get("success", False)
                _state.task_state = TaskState.SUCCESS if success else TaskState.FAILED
                _state.task_message = status.get("message", "成功" if success else "执行失败")
                print(f"[client] 任务结束: {_state.task_message}")
                return

    except Exception as e:
        print(f"[client] 任务异常: {e}", flush=True)
        _state.task_state = TaskState.FAILED
        _state.task_message = str(e)

    finally:
        _efm_post("/stop", {})
        _kill_run_py()
        _send_ros2_commands(_RESET_COMMANDS, "复位")


# ─── FastAPI ──────────────────────────────────────────────────────────

class StartRequest(BaseModel):
    instruction: str = (
        "Open the door with a downward-press handle, "
        "go through it, and enter the room."
    )
    timeout: float = 95.0
    poll_interval: float = 2.0


class StatusResponse(BaseModel):
    state: TaskState
    done: bool
    success: bool
    instruction: str
    message: str


def _refresh_ros2_status():
    """刷新 ROS2 话题就绪状态，返回是否就绪。"""
    ready, missing = _check_ros2_topics()
    _state.ros2_ready = ready
    _state.ros2_missing_topics = missing
    return ready


@asynccontextmanager
async def lifespan(application: FastAPI):
    if _refresh_ros2_status():
        print("[client] ROS2 话题检查通过 ✓")
    else:
        print(f"[client] ROS2 话题未就绪，缺失: {_state.ros2_missing_topics}")
        print("[client] 服务仍正常启动，等待话题上线后可正常执行任务")
    yield
    _kill_run_py()


app = FastAPI(title="VLA Door Client", lifespan=lifespan)


@app.get("/health")
def health():
    efm_alive = _efm_get("/health") is not None
    _refresh_ros2_status()
    return {
        "ok": True,
        "efm_ready": efm_alive,
        "ros2_ready": _state.ros2_ready,
        "ros2_missing_topics": _state.ros2_missing_topics,
    }


@app.get("/status")
def status() -> StatusResponse:
    done = _state.task_state in (TaskState.SUCCESS, TaskState.FAILED)
    success = _state.task_state == TaskState.SUCCESS
    return StatusResponse(
        state=_state.task_state,
        done=done,
        success=success,
        instruction=_state.task_instruction,
        message=_state.task_message,
    )


@app.post("/start")
def start_task(req: StartRequest):
    if not _refresh_ros2_status():
        return {
            "ok": False,
            "error": "ROS2 话题未就绪，建议跳过本次操作",
            "missing_topics": _state.ros2_missing_topics,
            "skip": True,
        }
    if _state.task_state == TaskState.RUNNING:
        return {"ok": False, "error": "任务正在执行中"}

    _state.task_state = TaskState.RUNNING
    _state.task_message = ""
    _state.task_instruction = req.instruction

    _state._worker = threading.Thread(
        target=_task_worker,
        args=(req.instruction, req.timeout, req.poll_interval),
        daemon=True,
    )
    _state._worker.start()
    return {"ok": True, "instruction": req.instruction}


@app.post("/stop")
def stop_task():
    was_running = _state.task_state == TaskState.RUNNING

    _efm_post("/stop", {})
    _state.task_state = TaskState.FAILED
    _state.task_message = "用户手动停止"

    def _deferred_cleanup():
        worker = _state._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=5)
        _kill_run_py()
        _send_ros2_commands(_RESET_COMMANDS, "复位")

    threading.Thread(target=_deferred_cleanup, daemon=True).start()

    return {"ok": True, "was_running": was_running}


@app.post("/execute")
def execute_task_sync(req: StartRequest):
    """阻塞式执行：下发任务，等待完成后返回结果。"""
    if not _refresh_ros2_status():
        return {
            "ok": False,
            "success": False,
            "error": "ROS2 话题未就绪，建议跳过本次操作",
            "missing_topics": _state.ros2_missing_topics,
            "skip": True,
        }
    if _state.task_state == TaskState.RUNNING:
        return {"ok": False, "error": "任务正在执行中"}

    _state.task_state = TaskState.RUNNING
    _state.task_message = ""
    _state.task_instruction = req.instruction

    worker = threading.Thread(
        target=_task_worker,
        args=(req.instruction, req.timeout, req.poll_interval),
        daemon=True,
    )
    worker.start()
    worker.join()

    return {
        "ok": True,
        "success": _state.task_state == TaskState.SUCCESS,
        "message": _state.task_message,
    }


@app.post("/shutdown")
def shutdown_run_py():
    """关闭 run.py 子进程。"""
    _kill_run_py()
    _state.task_state = TaskState.IDLE
    return {"ok": True}


# ─── 入口 ────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)
