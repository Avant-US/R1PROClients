#!/usr/bin/env bash
# r1pro_chassis 一键部署脚本
#
# 用法（在目标机器人上）：
#   cd /home/nvidia/kaizhe_ws/r1pro_chassis
#   bash deploy.sh
#
# 这个脚本做这些事：
#   1) 跑 preflight.sh 体检（不通过就退出）
#   2) source ROS2 + Galaxea
#   3) 调用 install.sh 建 venv + 装依赖 + 冒烟测试
#   4) 装 systemd 服务（需要 sudo）
#   5) 启动服务并验证 HTTP 接口
#
# 跑完后：
#   - vla_door_client 在 8088 端口持续运行
#   - 测试人员可以直接调 API.md 里描述的接口

set -euo pipefail

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SERVICE_NAME="door-skill.service"
SERVICE_TEMPLATE="$PROJECT_DIR/docs/door-skill.service.template"
SERVICE_TARGET="/etc/systemd/system/$SERVICE_NAME"

# 颜色（只在终端 tty 时启用）
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[0;33m'
    BLUE='\033[0;34m'
    NC='\033[0m'
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    NC=''
fi

log_step() { echo -e "\n${BLUE}=== $1 ===${NC}"; }
log_ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_fail() { echo -e "${RED}[FAIL]${NC} $1"; }

cd "$PROJECT_DIR"

echo "=========================================================="
echo " r1pro_chassis 一键部署"
echo " 项目目录: $PROJECT_DIR"
echo " 当前用户: $(whoami)"
echo "=========================================================="

# ============================================================
# 步骤 1：体检
# ============================================================
log_step "步骤 1/5: 运行 preflight.sh 体检"
if [ ! -f "$PROJECT_DIR/preflight.sh" ]; then
    log_fail "缺少 preflight.sh，无法体检"
    exit 1
fi

if ! bash "$PROJECT_DIR/preflight.sh"; then
    log_fail "体检不通过，部署中止"
    echo "请按 preflight.sh 的提示先修好环境，再重跑 deploy.sh"
    exit 1
fi
log_ok "体检通过"

# ============================================================
# 步骤 2：source ROS2 + Galaxea
# ============================================================
log_step "步骤 2/5: source ROS2 + Galaxea"
set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash
set -u
log_ok "ROS2 Humble + Galaxea 已 source"

# ============================================================
# 步骤 3：跑 install.sh 建 venv
# ============================================================
log_step "步骤 3/5: 建 venv + 装依赖（调用 install.sh）"
if [ ! -f "$PROJECT_DIR/install.sh" ]; then
    log_fail "缺少 install.sh"
    exit 1
fi

bash "$PROJECT_DIR/install.sh"
log_ok "venv 构建完成"

# ============================================================
# 步骤 4：装 systemd 服务
# ============================================================
log_step "步骤 4/5: 装 systemd 服务（需要 sudo 密码）"

if [ ! -f "$SERVICE_TEMPLATE" ]; then
    log_fail "找不到服务模板 $SERVICE_TEMPLATE"
    exit 1
fi

# 渲染模板：替换 __USER__ / __HOME__ / __PROJECT_DIR__
CURRENT_USER=$(whoami)
CURRENT_HOME="$HOME"
TMP_SERVICE=$(mktemp)
sed -e "s|__USER__|$CURRENT_USER|g" \
    -e "s|__HOME__|$CURRENT_HOME|g" \
    -e "s|__PROJECT_DIR__|$PROJECT_DIR|g" \
    "$SERVICE_TEMPLATE" > "$TMP_SERVICE"

echo "  渲染好的服务文件预览:"
sed 's/^/    /' "$TMP_SERVICE"
echo

# 装到 /etc/systemd/system/
sudo cp "$TMP_SERVICE" "$SERVICE_TARGET"
sudo chmod 644 "$SERVICE_TARGET"
rm -f "$TMP_SERVICE"
log_ok "服务文件已安装到 $SERVICE_TARGET"

sudo systemctl daemon-reload
log_ok "systemctl daemon-reload 完成"

# 如果已经在跑，先停一下
if sudo systemctl is-active --quiet "$SERVICE_NAME"; then
    log_warn "$SERVICE_NAME 已在运行，先停止"
    sudo systemctl stop "$SERVICE_NAME"
fi

sudo systemctl enable "$SERVICE_NAME" 2>&1 | grep -v "^Created symlink" || true
log_ok "$SERVICE_NAME 已设置为开机自启"

# ============================================================
# 步骤 5：启动服务 + 验证
# ============================================================
log_step "步骤 5/5: 启动服务并验证"

sudo systemctl start "$SERVICE_NAME"
log_ok "$SERVICE_NAME 已启动"

# 等服务起来（最多 30 秒）
echo "  等待 HTTP 接口 ready（最多 30 秒）..."
READY=0
for i in $(seq 1 30); do
    if curl -fsS http://localhost:8088/health >/dev/null 2>&1; then
        READY=1
        break
    fi
    sleep 1
done

if [ "$READY" -eq 0 ]; then
    log_fail "30 秒后 HTTP 接口仍未 ready"
    echo "  看日志："
    echo "    journalctl -u $SERVICE_NAME -n 50 --no-pager"
    exit 1
fi
log_ok "HTTP 接口已 ready (http://localhost:8088)"

# 跑健康检查
echo
echo "  健康检查："
HEALTH_JSON=$(curl -fsS http://localhost:8088/health)
echo "    $HEALTH_JSON"

if echo "$HEALTH_JSON" | grep -q '"ros2_ready":true'; then
    log_ok "ROS2 桥接 ready"
else
    log_warn "ROS2 桥接 NOT ready，可能机器人节点没起来（检查 robot-teleop 服务）"
fi

# 跑状态检查
echo
echo "  任务状态："
STATUS_JSON=$(curl -fsS http://localhost:8088/status)
echo "    $STATUS_JSON"

# ============================================================
# 完成
# ============================================================
echo
echo "=========================================================="
echo -e " ${GREEN}部署完成${NC}"
echo "=========================================================="
echo
echo " 服务管理："
echo "   sudo systemctl status $SERVICE_NAME      # 看状态"
echo "   sudo systemctl restart $SERVICE_NAME     # 重启"
echo "   sudo systemctl stop $SERVICE_NAME        # 停止"
echo "   journalctl -u $SERVICE_NAME -f           # 实时日志"
echo
echo " 测试任务下发（注意：会动机器人）："
echo "   curl -X POST http://localhost:8088/execute \\"
echo "     -H 'Content-Type: application/json' \\"
echo "     -d '{\"instruction\":\"Open the door...\",\"timeout\":140}'"
echo
echo " 完整 API 文档：API.md"
echo " 部署详细说明：docs/DEPLOYMENT.md"
echo
