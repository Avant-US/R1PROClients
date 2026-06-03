#!/usr/bin/env bash
# r1pro_chassis 部署前体检（只读，不动系统）
#
# 用法：
#   bash preflight.sh
#
# 作用：
#   检查目标机器是否满足部署前置条件。全部通过才允许跑 deploy.sh。
#
# 退出码：
#   0 = 全部通过
#   1 = 有至少一项不满足

set -o pipefail

# 颜色（只在终端 tty 时启用，避免管道/日志看到乱码）
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    NC='\033[0m'
else
    RED=''
    GREEN=''
    YELLOW=''
    NC=''
fi

PASS=0
FAIL=0
WARN=0

check_ok()   { echo -e "  ${GREEN}[OK]${NC}   $1"; PASS=$((PASS+1)); }
check_fail() { echo -e "  ${RED}[FAIL]${NC} $1"; FAIL=$((FAIL+1)); }
check_warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; WARN=$((WARN+1)); }

echo "=========================================================="
echo " r1pro_chassis 部署前体检"
echo "=========================================================="

# ---------- 1. 硬件平台 ----------
echo
echo "[1] 硬件平台"
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    check_ok "架构 = aarch64（Jetson 或同等 ARM64）"
else
    check_fail "架构 = $ARCH（需要 aarch64，本项目不支持其他平台）"
fi

# ---------- 2. 操作系统 ----------
echo
echo "[2] 操作系统"
if grep -q "Ubuntu 22.04" /etc/os-release 2>/dev/null; then
    check_ok "Ubuntu 22.04"
else
    OS_NAME=$(grep PRETTY_NAME /etc/os-release 2>/dev/null | cut -d'"' -f2)
    check_warn "OS = $OS_NAME（推荐 Ubuntu 22.04，其他版本未测试）"
fi

# ---------- 3. Python ----------
echo
echo "[3] 系统 Python"
if [ -x /usr/bin/python3 ]; then
    PY_VER=$(/usr/bin/python3 --version 2>&1)
    if echo "$PY_VER" | grep -q "3.10"; then
        check_ok "/usr/bin/python3 = $PY_VER"
    else
        check_warn "/usr/bin/python3 = $PY_VER（推荐 3.10.x）"
    fi
else
    check_fail "找不到 /usr/bin/python3"
fi

# python3-venv 包
if /usr/bin/python3 -c "import venv" 2>/dev/null; then
    check_ok "python3-venv 模块可用"
else
    check_fail "python3-venv 缺失。装一下：sudo apt install python3-venv python3-pip"
fi

# ---------- 4. ROS2 Humble ----------
echo
echo "[4] ROS2 Humble"
if [ -f /opt/ros/humble/setup.bash ]; then
    check_ok "/opt/ros/humble/setup.bash 存在"

    # source 一下临时验证 rclpy 可见
    if (source /opt/ros/humble/setup.bash && python3 -c "import rclpy" 2>/dev/null); then
        check_ok "source 后 rclpy 可见"
    else
        check_fail "source 后仍然 import rclpy 失败"
    fi
else
    check_fail "/opt/ros/humble/setup.bash 不存在。装 ROS2 Humble：参考 https://docs.ros.org/en/humble/Installation.html"
fi

# ---------- 5. Galaxea 机器人栈 ----------
echo
echo "[5] Galaxea 机器人控制栈"
if [ -f /home/nvidia/galaxea/install/setup.bash ]; then
    check_ok "/home/nvidia/galaxea/install/setup.bash 存在"
else
    check_fail "/home/nvidia/galaxea/install/setup.bash 不存在。需要找运维装 Galaxea 包"
fi

# 检查 /hdas/ 话题（说明机器人控制节点在跑）
if [ -f /opt/ros/humble/setup.bash ] && [ -f /home/nvidia/galaxea/install/setup.bash ]; then
    HDAS_COUNT=$(bash -c '
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /home/nvidia/galaxea/install/setup.bash 2>/dev/null
        timeout 5 ros2 topic list 2>/dev/null | grep -c "^/hdas/"
    ' || echo "0")
    HDAS_COUNT=${HDAS_COUNT:-0}
    if [ "$HDAS_COUNT" -ge 10 ]; then
        check_ok "ros2 topic list 看到 $HDAS_COUNT 个 /hdas/ 话题（机器人节点已在跑）"
    elif [ "$HDAS_COUNT" -gt 0 ]; then
        check_warn "只看到 $HDAS_COUNT 个 /hdas/ 话题（期望 >= 10，机器人节点可能没完全起来）"
    else
        check_warn "没看到 /hdas/ 话题（机器人节点没启动？或 ROS_DOMAIN_ID 不对？）"
    fi
fi

# ---------- 6. Jetson PyTorch ----------
echo
echo "[6] Jetson 版 PyTorch"
TORCH_PATH="/home/nvidia/.local/lib/python3.10/site-packages/torch"
if [ -f "$TORCH_PATH/__init__.py" ]; then
    check_ok "$TORCH_PATH 存在"

    # 直接读 torch 的 version.py 拿版本号，不实际 import
    # （因为系统 Python + ~/.local 可能有别的兼容性问题，不该在 preflight 阶段暴露）
    TORCH_VER=$(grep -oE "__version__\s*=\s*['\"][^'\"]+['\"]" "$TORCH_PATH/version.py" 2>/dev/null | \
                grep -oE "['\"][^'\"]+['\"]" | tr -d "'\"")
    if [ -z "$TORCH_VER" ]; then
        check_warn "读不到 torch 版本号（version.py 不在或格式异常）"
    elif echo "$TORCH_VER" | grep -qE "nv[0-9]+|\.nv|jetson"; then
        check_ok "torch $TORCH_VER（版本号包含 nv 标识，是 Jetson 专版）"
    else
        check_fail "torch $TORCH_VER（不像 Jetson 专版 —— 期望版本号包含 nv 字样，例如 2.4.0a0+xxx.nv24.05）"
        echo "         若误判，可在 deploy.sh 后手动用 venv 验证：.venv/bin/python -c 'import torch; print(torch.cuda.is_available())'"
    fi
else
    check_fail "找不到 Jetson torch。装一下：参考 https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/"
fi

# ---------- 7. 推理服务器网络 ----------
echo
echo "[7] 推理服务器连通性"
SERVER_HOST="34.32.242.109"
SERVER_PORT="8000"
if command -v nc >/dev/null 2>&1; then
    if timeout 5 nc -zv "$SERVER_HOST" "$SERVER_PORT" 2>&1 | grep -q "succeeded\|open"; then
        check_ok "$SERVER_HOST:$SERVER_PORT 可连通"
    else
        check_warn "$SERVER_HOST:$SERVER_PORT 连不上（请联系运维或修改 config.toml 里的 server_endpoint）"
    fi
else
    check_warn "找不到 nc 命令，跳过网络检查"
fi

# ---------- 8. sudo 权限 ----------
echo
echo "[8] sudo 权限（装 systemd 服务需要）"
if sudo -n true 2>/dev/null; then
    check_ok "sudo 免密可用"
else
    check_warn "sudo 需要密码（deploy.sh 装 systemd 服务时会问你密码）"
fi

# ---------- 9. 端口占用 ----------
echo
echo "[9] 端口占用检查"
if command -v lsof >/dev/null 2>&1; then
    if sudo lsof -i :8088 -t >/dev/null 2>&1; then
        OCCUPIER=$(sudo lsof -i :8088 -t 2>/dev/null | head -1)
        check_warn "端口 8088 已被 PID=$OCCUPIER 占用（部署后会冲突，可能是旧版本服务）"
    else
        check_ok "端口 8088 空闲"
    fi
    if sudo lsof -i :9001 -t >/dev/null 2>&1; then
        OCCUPIER=$(sudo lsof -i :9001 -t 2>/dev/null | head -1)
        check_warn "端口 9001 已被 PID=$OCCUPIER 占用"
    else
        check_ok "端口 9001 空闲"
    fi
fi

# ---------- 汇总 ----------
echo
echo "=========================================================="
echo -e " 体检结果: ${GREEN}PASS=$PASS${NC}  ${YELLOW}WARN=$WARN${NC}  ${RED}FAIL=$FAIL${NC}"
echo "=========================================================="

if [ "$FAIL" -gt 0 ]; then
    echo
    echo -e "${RED}有 $FAIL 项硬性条件不满足，无法继续部署。${NC}"
    echo "请按上面 [FAIL] 的提示先解决，再重新跑 preflight.sh。"
    exit 1
fi

if [ "$WARN" -gt 0 ]; then
    echo
    echo -e "${YELLOW}有 $WARN 项警告，请确认是否影响部署。${NC}"
    echo "如果确认无影响，可以继续跑 deploy.sh。"
fi

echo
echo "下一步："
echo "  bash deploy.sh"
echo
exit 0
