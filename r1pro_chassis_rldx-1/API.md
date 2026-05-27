# VLA Door Client API 文档

## 概述

`vla_door_client.py` 是 R1Pro 机器人开门任务的 FastAPI 网关服务。它封装了以下流程：

1. 检查 ROS2 话题就绪状态
2. 自动启动 / 关闭 `run.py` 推理进程
3. 发送初始姿态 → 下发任务 → 轮询结果 → 复位 → 杀掉推理进程

Agent 只需调用本服务的 HTTP 接口，无需关心底层 `run.py` 和 ROS2 细节。

**默认地址**: `http://localhost:8088`

## 启动方式

```bash
cd /home/nvidia/zwy_WS/r1pro_chassis
source venv/bin/activate

# 默认端口 8088
python3 vla_door_client.py

# 自定义端口
python3 vla_door_client.py --port 9090
```

启动时会自动检查 ROS2 HDAS 话题（相机、关节反馈等）。即使话题未就绪，服务仍会正常启动，等话题上线后即可正常执行任务。

---

## 接口列表

| 方法 | 路径       | 说明                               |
|------|------------|-----------------------------------|
| GET  | /health    | 健康检查（含 ROS2 话题状态）         |
| GET  | /status    | 查询当前任务状态                    |
| POST | /start     | 启动任务（非阻塞，立即返回）         |
| POST | /execute   | 执行任务（阻塞，等完成后返回结果）    |
| POST | /stop      | 停止正在执行的任务                  |
| POST | /shutdown  | 关闭 run.py 子进程                  |

---

## GET /health

健康检查，返回服务状态和 ROS2 话题就绪情况。

**请求**

```bash
curl http://localhost:8088/health
```

**响应**

```json
{
    "ok": true,
    "efm_ready": false,
    "ros2_ready": true,
    "ros2_missing_topics": []
}
```

| 字段                | 类型     | 说明                          |
|---------------------|----------|------------------------------|
| ok                  | boolean  | 服务本身是否正常               |
| efm_ready           | boolean  | run.py 推理进程是否在运行       |
| ros2_ready          | boolean  | HDAS 话题是否全部就绪           |
| ros2_missing_topics | string[] | 缺失的话题列表（就绪时为空数组）  |

---

## GET /status

查询当前任务的执行状态。

**请求**

```bash
curl http://localhost:8088/status
```

**响应**

```json
{
    "state": "idle",
    "done": false,
    "success": false,
    "instruction": "",
    "message": ""
}
```

| 字段        | 类型    | 说明                                                   |
|-------------|---------|-------------------------------------------------------|
| state       | string  | `idle` / `running` / `success` / `failed`              |
| done        | boolean | `true` = 任务已结束（成功或失败），`false` = 空闲或执行中  |
| success     | boolean | 仅当 `done=true` 时有意义。`true` = 成功                |
| instruction | string  | 当前/上次执行的指令                                      |
| message     | string  | 状态描述信息                                            |

---

## POST /start

下发任务（非阻塞）。立即返回，任务在后台执行，通过 `GET /status` 轮询结果。

**请求**

```bash
curl -X POST http://localhost:8088/start \
  -H "Content-Type: application/json" \
  -d '{
    "instruction": "Open the door with a downward-press handle, go through it, and enter the room.",
    "timeout": 140,
    "poll_interval": 2
  }'
```

| 字段          | 类型   | 必填 | 默认值 | 说明                 |
|---------------|--------|------|--------|---------------------|
| instruction   | string | 否   | 开门指令 | 任务指令（英文）      |
| timeout       | number | 否   | 70     | 超时秒数             |
| poll_interval | number | 否   | 2      | 内部轮询间隔秒数      |

**成功响应**

```json
{"ok": true, "instruction": "Open the door..."}
```

**错误响应**

```json
{"ok": false, "error": "任务正在执行中"}
```

**ROS2 未就绪响应**

```json
{
    "ok": false,
    "error": "ROS2 话题未就绪，建议跳过本次操作",
    "missing_topics": ["/hdas/camera_head/left_raw/image_raw_color/compressed"],
    "skip": true
}
```

> Agent 收到 `skip: true` 时应跳过本次操作，而非视为执行失败。

---

## POST /execute

阻塞式执行任务。完整流程为：启动 run.py → 发送初始姿态 → 下发任务 → 等待完成 → 复位 → 关闭 run.py → 返回结果。

**请求**

```bash
curl -X POST http://localhost:8088/execute \
  -H "Content-Type: application/json" \
  -d '{
    "instruction": "Open the door with a downward-press handle, go through it, and enter the room.",
    "timeout": 140
  }'
```

请求体字段与 `/start` 相同。

**成功响应**

```json
{"ok": true, "success": true, "message": "成功"}
```

**失败响应**

```json
{"ok": true, "success": false, "message": "执行失败"}
```

**ROS2 未就绪响应**

```json
{
    "ok": false,
    "success": false,
    "error": "ROS2 话题未就绪，建议跳过本次操作",
    "missing_topics": [...],
    "skip": true
}
```

---

## POST /stop

停止正在执行的任务。

**请求**

```bash
curl -X POST http://localhost:8088/stop
```

**响应**

```json
{"ok": true}
```

---

## POST /shutdown

手动关闭 run.py 推理子进程。

**请求**

```bash
curl -X POST http://localhost:8088/shutdown
```

**响应**

```json
{"ok": true}
```

---

## 典型调用流程

### 方式一：阻塞式（推荐 Agent 使用）

```
Agent                              vla_door_client.py (localhost:8088)
  │                                       │
  │── GET /health ───────────────────────→│  1. 确认服务存活 + ROS2 就绪
  │←── {"ok":true,"ros2_ready":true} ────│
  │                                       │
  │── POST /execute ─────────────────────→│  2. 阻塞等待任务完成
  │   {"instruction":"Open the door..."}  │     (内部自动: 启动run.py → 初始姿态
  │                                       │      → 下发任务 → 轮询 → 复位 → 关闭run.py)
  │←── {"ok":true,"success":true} ───────│  3. 返回结果
  │                                       │
```

### 方式二：非阻塞式（手动测试用）

```
用户                               vla_door_client.py (localhost:8088)
  │                                       │
  │── POST /start ───────────────────────→│  1. 启动任务
  │←── {"ok":true} ─────────────────────│
  │                                       │
  │── GET /status (轮询) ────────────────→│  2. 轮询状态
  │←── {"done":false} ─────────────────│
  │                                       │
  │── GET /status ───────────────────────→│  3. 再次轮询
  │←── {"done":true,"success":true} ────│     任务完成
  │                                       │
  │── POST /stop（如需紧急停止）──────────→│
  │                                       │
```

---

## ROS2 话题依赖

服务启动时及每次执行任务前，会检查以下 HDAS 话题是否存在：

| 话题                                               | 说明         |
|---------------------------------------------------|-------------|
| /hdas/feedback_arm_left                            | 左臂关节反馈  |
| /hdas/feedback_arm_right                           | 右臂关节反馈  |
| /hdas/feedback_torso                               | 躯干反馈     |
| /hdas/feedback_chassis                             | 底盘反馈     |
| /hdas/feedback_gripper_left                        | 左夹爪反馈   |
| /hdas/feedback_gripper_right                       | 右夹爪反馈   |
| /hdas/camera_head/left_raw/image_raw_color/compressed | 头部相机    |
| /hdas/camera_wrist_left/color/image_raw/compressed    | 左腕相机    |
| /hdas/camera_wrist_right/color/image_raw/compressed   | 右腕相机    |

如有缺失，接口返回 `skip: true`，Agent 应跳过操作。

---

## 架构说明

```
┌──────────┐    HTTP :8088    ┌──────────────┐   HTTP :9001   ┌──────────┐
│  Agent   │ ───────────────→ │ vla_door_    │ ─────────────→ │  run.py  │
│ (door.py)│ ←─────────────── │ client.py    │ ←───────────── │(Scheduler│
└──────────┘   success/fail   │  (FastAPI)   │   done/status  │ +推理)   │
                              │  管理生命周期  │                └──────────┘
                              │  初始姿态/复位 │───→ ROS2 话题
                              └──────────────┘
```

- **Agent** 只与 `vla_door_client.py` 通信（端口 8088）
- **vla_door_client.py** 自动管理 `run.py` 的启停，Agent 无需关心
- **run.py** 是内部推理进程（端口 9001），每次任务结束后会被关闭
