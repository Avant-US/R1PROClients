#!/usr/bin/env bash
# r1pro_chassis 一键重建 Python 虚拟环境
#
# 用法：
#   cd /home/nvidia/kaizhe_ws/r1pro_chassis
#   bash install.sh
#
# 作用：
#   1. 删除已有 .venv（如果存在）
#   2. 用 /usr/bin/python3 创建一个「封闭」venv（不开 --system-site-packages）
#   3. 写 _jetson_torch.pth 把 ~/.local/lib/python3.10/site-packages 注入 venv
#      —— 这一步借用 Jetson 专版 torch、以及若干 ~/.local 已有的纯 Python 包
#   4. 按 requirements.txt 装 venv 自己应该有的包
#   5. 验证 venv 是否能正确 import
#
# 前置条件：
#   - 已 source 过 ROS2 Humble 的 setup.bash（提供 rclpy / sensor_msgs 等）
#   - ~/.local/lib/python3.10/site-packages/torch 存在且为 Jetson 版（含 CUDA）
#   - pip 镜像源已配置（默认走清华源会更快）

set -euo pipefail

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
VENV_DIR="$PROJECT_DIR/.venv"
PY_SYS="/usr/bin/python3"
LOCAL_SITE="/home/nvidia/.local/lib/python3.10/site-packages"

echo "=========================================================="
echo " r1pro_chassis 环境重建"
echo " 项目目录: $PROJECT_DIR"
echo " 系统 Python: $PY_SYS"
echo "=========================================================="

# ---------- 前置检查 ----------
echo "[1/6] 前置检查 ..."

if ! command -v "$PY_SYS" > /dev/null; then
    echo "ERROR: 找不到 $PY_SYS"
    exit 1
fi
echo "  系统 Python: $($PY_SYS --version)"

if [ ! -f "$LOCAL_SITE/torch/__init__.py" ]; then
    echo "ERROR: 找不到 Jetson torch ($LOCAL_SITE/torch)"
    echo "       请先确认本机装了 Jetson 版 PyTorch（apt 或 NVIDIA wheel）"
    exit 1
fi
echo "  Jetson torch: 存在 ($LOCAL_SITE/torch)"

if ! $PY_SYS -c "import rclpy" 2>/dev/null; then
    echo "ERROR: 当前 shell 没 source ROS2 Humble。请先："
    echo "       source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "  ROS2: rclpy 可见"

# ---------- 删旧 venv ----------
if [ -d "$VENV_DIR" ]; then
    echo "[2/6] 删除已有 .venv ..."
    rm -rf "$VENV_DIR"
else
    echo "[2/6] 没有旧 .venv，跳过"
fi

# ---------- 创建封闭 venv ----------
echo "[3/6] 创建封闭 venv (不开 --system-site-packages) ..."
$PY_SYS -m venv "$VENV_DIR"
echo "  venv 创建于 $VENV_DIR"

# ---------- 注入 Jetson torch 路径 ----------
echo "[4/6] 注入 Jetson torch 路径到 venv ..."
PTH_FILE="$VENV_DIR/lib/python3.10/site-packages/_jetson_torch.pth"
echo "$LOCAL_SITE" > "$PTH_FILE"
echo "  写入 $PTH_FILE"
echo "  内容: $LOCAL_SITE"

# ---------- 验证注入生效 ----------
echo "  验证 torch 可见 ..."
"$VENV_DIR/bin/python" -c "
import torch
assert torch.cuda.is_available(), 'CUDA 不可用'
print(f'    torch {torch.__version__} CUDA={torch.cuda.is_available()}')
"

# ---------- 装 venv 内部依赖 ----------
echo "[5/6] 安装 requirements.txt ..."
"$VENV_DIR/bin/python" -m pip install --upgrade pip > /dev/null
# --no-deps 防止 opencv-python 之类把 numpy 升到 2.x
# --prefer-binary 优先 wheel 避免本地编译
"$VENV_DIR/bin/python" -m pip install --prefer-binary -r "$PROJECT_DIR/requirements.txt"

# ---------- 冒烟测试 ----------
echo "[6/6] 冒烟测试 ..."
"$VENV_DIR/bin/python" - <<'PYEOF'
import sys
sys.path.insert(0, '.')

# 1. 第三方包
import torch, numpy, cv2, websockets, msgpack, omegaconf, accelerate
from loguru import logger
assert torch.cuda.is_available()
assert numpy.__version__.startswith('1.'), f'numpy 必须是 1.x，当前 {numpy.__version__}'

# 2. ROS2
import rclpy
from sensor_msgs.msg import JointState, CompressedImage
from geometry_msgs.msg import PoseStamped, TwistStamped

# 3. 项目模块
from scheduler.scheduler import Scheduler  # 最严的导入测试，会触发 accelerate
from core.communication.robot_topics import RobotTopicsConfig
from core.processor.openpi_processor import OpenPIProcessor
from core.inference.websocket_engine import WebSocketClientEngine

# 4. numpy <-> torch 互操作（验证 ABI 兼容）
import numpy as np
t = torch.from_numpy(np.zeros((2, 2), dtype=np.float32)).cuda()
assert t.device.type == 'cuda'

print('  全部通过')
PYEOF

echo "=========================================================="
echo " 完成"
echo "=========================================================="
echo ""
echo " 启动项目："
echo "   cd $PROJECT_DIR"
echo "   .venv/bin/python run.py"
echo ""
echo " 或激活 venv 后："
echo "   source .venv/bin/activate"
echo "   python run.py"
echo ""
