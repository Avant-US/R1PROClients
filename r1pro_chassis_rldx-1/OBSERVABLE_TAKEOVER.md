# R1Pro VLA 推理服务 —— "可观测、可接管"技术方案

## 系统架构

```
远程巡检平台 ──HTTP:8088──→ vla_door_client.py (FastAPI网关) ──HTTP:9001──→ run.py (推理引擎)
                                     │                                        │
                                     └──── ROS2 话题 ────→ HDAS (电机/相机) ←──┘
```

| 层 | 端口 | 职责 |
|----|------|------|
| 网关层 `vla_door_client.py` | 8088 | 对外接口；管理 run.py 生命周期；ROS2 就绪检查；初始姿态/复位 |
| 内核层 `run.py` | 9001 | 推理循环；任务状态机；动作发布；观测录制 |
| 硬件层 HDAS | ROS2 | 电机控制、相机数据、关节反馈 |

## 对外接口（:8088）

| 方法 | 路径 | 功能 | 说明 |
|------|------|------|------|
| GET | `/health` | 健康检查 | 返回 `efm_ready`、`ros2_ready`、`ros2_missing_topics` |
| GET | `/status` | 任务状态 | 返回 `state`/`done`/`success`/`instruction`/`message` |
| POST | `/start` | 启动任务 | 非阻塞，通过 `/status` 轮询结果 |
| POST | `/execute` | 执行任务 | 阻塞等完成后返回 `success`/`message` |
| POST | `/stop` | 停止+复位 | 停止任务 → 杀 run.py → 复位关节到零位 |
| POST | `/shutdown` | 关闭推理 | 仅杀 run.py，不复位 |

### POST /start 请求体

```json
{"instruction": "Open the door...", "timeout": 95, "poll_interval": 2}
```

三种响应：成功 `{"ok":true}` / ROS2未就绪 `{"ok":false,"skip":true}` / 任务冲突 `{"ok":false,"error":"任务正在执行中"}`

## 服务生命周期

**启动**
```bash
cd /home/nvidia/zwy_WS/r1pro_chassis
source venv/bin/activate
python3 vla_door_client.py
```

run.py 由网关按需拉起/杀掉，无需手动管理。

**停止**

| 操作 | 效果 |
|------|------|
| `POST /stop` | 停止任务 → 杀 run.py → 复位关节 |
| `POST /shutdown` | 仅杀 run.py |
| Ctrl+C 网关 | lifespan 钩子自动清理 |

**暂停/恢复**：当前无显式接口。`/stop` 会复位，`/start` 重新初始化。如需保持姿态暂停，需扩展。

## 状态机

```
idle ──POST /start──→ running ──→ success (空闲5s自动判定)
 ↑                      │    ──→ failed  (超时/卡住/断连/手动stop)
 └──自动复位+杀进程──────┘
```

| state | done | 含义 |
|-------|------|------|
| idle | false | 空闲 |
| running | false | 执行中 |
| success | true | 完成（手臂+底盘静止 5s） |
| failed | true | 超时 / 左臂卡住检测 / 连接丢失 / 手动停止 |

## 可观测性

### 日志

| 来源 | 格式 | 示例 |
|------|------|------|
| 网关 | `[client] ...` | `[client] run.py 已就绪` |
| 推理/调度 | loguru `时间\|LEVEL\|模块 - 消息` | `NEW chunk#77 prev_chunk#76 executed 17/50 steps lag=1.04s` |
| 推理性能 | loguru | `client_total=487ms policy_infer=29ms network=443ms` |

### 关键日志事件

- 任务：`Task started` / `Task stopped` / `Task finished: status=completed`
- 轨迹：`NEW chunk#N executed M/50 steps lag=Xs` / `NONE skipped N expired frames`
- 异常：`Head camera buffer is empty` / `检测到左手重复动作，判定为卡住`

### 监控工具

| 工具 | 用途 |
|------|------|
| `GET :9001/obs` | toggle 录制观测+动作，停录写入 `/tmp/recorded_obs.json` |
| `scripts/monitor.py` | 独立 ROS2 节点，订阅指令+反馈话题，输出 CSV + 终端摘要 |
| `scripts/analyze_monitor.py` | 分析 monitor CSV，统计频率和异常 |

## ROS2 话题依赖

**输入（9个，启动前检查）**：`/hdas/feedback_{arm_left,arm_right,torso,chassis,gripper_left,gripper_right}` + 3 个相机 compressed image

**输出（6个）**：`/motion_target/target_joint_state_{arm_left,arm_right,torso}` + `target_position_gripper_{left,right}` + `target_speed_chassis`

## 对远程巡检平台的要求

| 要求 | 说明 |
|------|------|
| 先检查再执行 | 任务前 `GET /health` 确认 `ros2_ready=true` |
| 处理 skip | 收到 `"skip":true` 跳过操作，不视为失败 |
| 超时兜底 | 平台侧 timeout ≥ 请求 timeout + 30s |
| 异常必 stop | 任何异常调 `POST /stop` 确保复位 |
| 不直接访问 :9001 | run.py 由网关管理 |

### 接管操作

| 场景 | 操作 |
|------|------|
| 紧急停止 | `POST /stop` |
| 任务超时 | 平台检测超时 → `POST /stop` |
| 服务异常 | `POST /shutdown` → 重新 `POST /start` |
| 完全重启 | kill 网关 → 重新启动 |

## 配置速查（config.toml）

| 字段 | 说明 | 默认值 |
|------|------|--------|
| `[robot] enable_publish` | 控制发布的部件 | left/right_arm, gripper, chassis |
| `[basic] control_frequency` | 控制频率 | 15 Hz |
| `[basic] action_steps` | 每次推理步数 | 50 |
| `[trajectory] ensemble_mode` | 轨迹融合 | NONE / RTG / HATO |
| `[websocket] host/port` | 远程推理服务器 | 34.32.242.109:8000 |
| `[openpi] chassis_deadzone` | 底盘死区 | 0.01 |

## 扩展建议

| 项 | 建议 |
|----|------|
| 暂停/恢复 | 增加 `POST /pause`（保持姿态）、`POST /resume` |
| 结构化日志 | 统一 JSON 格式，接入日志收集 |
| Metrics | Prometheus 暴露推理延迟、帧率、成功率 |
| 实时推送 | WebSocket 端点替代轮询 |
