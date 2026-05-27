# R1Pro VLA 推理服务 —— “可观测、可接管”技术方案汇报

## 1. 核心设计理念
- **双层架构，网关代理**：系统分为网关层（端口 8088）和推理引擎层（端口 9001）。
- **外部隔离**：上层平台仅与网关通信，网关全权负责底层生命周期管理。
- **可观测、可接管**：确保上层应用清晰掌握机器人状态，且在突发情况下具备绝对控制权。

## 2. 准备阶段：启动与状态检查
**服务启动**：
```bash
cd /home/nvidia/zwy_WS/r1pro_chassis
source venv/bin/activate
python3 vla_door_client.py
```
*(注：该命令启动 8088 网关服务，用于接收上层平台的 `/health`、`/start`、`/status`、`/stop` 等控制信号；启动网关时不拉起推理引擎，按需分配资源。)*

**执行健康检查 (`GET /health`)**：
- 任务下发前，调用此接口检查底层 ROS2 话题就绪状态（2 臂、2 夹爪、躯干、底盘、3 相机）。
- 只有返回 `ros2_ready: true` 方可执行任务；否则根据返回的缺失话题列表，触发 `skip` 逻辑。
- 请求字段：无。
- 返回字段：
  - `ok`：网关接口是否正常返回。
  - `efm_ready`：推理引擎 `run.py` 是否已经存活；网关刚启动时通常为 `false`，任务下发后才会按需拉起。
  - `ros2_ready`：ROS2 话题是否全部就绪。
  - `ros2_missing_topics`：缺失的话题列表；为空表示 ROS2 话题检查通过。

## 3. 执行阶段：任务下发与状态轮询
**下发任务 (`POST /start` 或 `/execute`)**：
- 接收自然语言指令。
- 请求字段：
  - `instruction`：自然语言任务指令；默认值为开门并进入房间的任务描述。
  - `timeout`：任务超时时间，单位为秒，默认 `95.0`。
  - `poll_interval`：网关轮询推理引擎状态的间隔，单位为秒，默认 `2.0`。
- `/start` 为异步接口，返回后平台继续通过 `/status` 查询状态；返回字段包括：
  - `ok`：是否成功受理任务。
  - `instruction`：本次受理的任务指令。
  - `error`：失败原因；例如 ROS2 未就绪或已有任务正在执行。
  - `missing_topics`：ROS2 未就绪时返回的缺失话题列表。
  - `skip`：是否建议平台跳过本次任务；ROS2 未就绪时为 `true`。
- `/execute` 为阻塞式接口，会等待任务结束后再返回；返回字段包括：
  - `ok`：接口是否正常执行。
  - `success`：任务是否成功完成。
  - `message`：任务结束说明或失败原因。
  - `error`：任务无法启动时的失败原因。
  - `missing_topics`：ROS2 未就绪时返回的缺失话题列表。
  - `skip`：是否建议平台跳过本次任务；ROS2 未就绪时为 `true`。
- **自动化执行链**：
  1. 自动拉起 `run.py` 推理引擎。
  2. 发送默认姿态，复位关节。
  3. 执行推理，根据相机反馈输出动作。

**状态轮询 (`GET /status`)**：
- 执行期间状态为 `running`，建议平台每 2~3 秒轮询一次。
- 返回字段包括：
  - `state`：当前任务状态，可能为 `idle`、`running`、`success`、`failed`。
  - `done`：任务是否已经结束；当 `state` 为 `success` 或 `failed` 时返回 `true`。
  - `success`：任务是否成功完成；仅成功结束时为 `true`。
  - `instruction`：当前执行的自然语言任务指令。
  - `message`：状态说明或失败原因，例如“用户手动停止”“与 run.py 连接丢失”“执行失败”等。
- 平台侧可根据 `done` 判断是否停止轮询，根据 `success` 判断执行结果，并通过 `message` 展示或记录异常原因。

## 4. 判定阶段：智能结束机制
无需人工干预，系统提供三种自动结束机制：
1. **正常完成（Success）**：空闲检测机制。机械臂与底盘连续 5 秒变化低于阈值。
2. **卡死保护（Failed）**：重复动作检测。50 步窗口期内左手动作极差过小，判定死锁。
3. **超时保护（Failed）**：执行时间超出平台设定的 `timeout` 阈值。

*(注：任务结束后，网关会自动复位机器人姿态并销毁推理进程。)*

## 5. 接管阶段：紧急干预机制
面对突发危险（如即将碰撞）或异常，提供多级接管手段：
- **常规急停 (`POST /stop`)**：立即截断推理指令，下发归零指令，使机器人恢复默认姿态。
  - 请求字段：无。
  - 返回字段：
    - `ok`：急停请求是否已被网关受理。
    - `was_running`：触发急停时是否存在正在执行的任务。
- **强制关闭 (`POST /shutdown`)**：针对底层进程死锁，直接强杀推理进程组作为兜底。
  - 请求字段：无。
  - 返回字段：
    - `ok`：推理进程关闭请求是否已被网关受理。


## 6. 对接要求规范
为保证上下游稳定集成，对上层平台提出四项要求：
1. **先检查状态**：必须调用 `/health`，遵循 `skip` 指示。
2. **异步通信**：优先采用 `/start` + `/status` 轮询架构。
3. **平台侧超时控制**：平台端必须维护独立的超时计时器，超时强制触发 `/stop`。
4. **禁止越权**：严格限制仅通过 8088 端口调用，禁止绕过网关直连内部进程。

## 7. 接口调用示例
以下示例均以网关地址 `http://127.0.0.1:8088` 为例；实际部署时可替换为机器人网关 IP。

**健康检查 (`GET /health`)**：
```bash
curl http://127.0.0.1:8088/health
```

返回示例：
```json
{
  "ok": true,
  "efm_ready": false,
  "ros2_ready": true,
  "ros2_missing_topics": []
}
```

**异步下发任务 (`POST /start`)**：
```bash
curl -X POST http://127.0.0.1:8088/start \
  -H "Content-Type: application/json" \
  -d '{
    "instruction": "Open the door with a downward-press handle, go through it, and enter the room.",
    "timeout": 95.0,
    "poll_interval": 2.0
  }'
```

成功返回示例：
```json
{
  "ok": true,
  "instruction": "Open the door with a downward-press handle, go through it, and enter the room."
}
```

失败返回示例：
```json
{
  "ok": false,
  "error": "ROS2 话题未就绪，建议跳过本次操作",
  "missing_topics": ["/camera/color/image_raw"],
  "skip": true
}
```

**状态轮询 (`GET /status`)**：
```bash
curl http://127.0.0.1:8088/status
```

执行中返回示例：
```json
{
  "state": "running",
  "done": false,
  "success": false,
  "instruction": "Open the door with a downward-press handle, go through it, and enter the room.",
  "message": ""
}
```

完成后返回示例：
```json
{
  "state": "success",
  "done": true,
  "success": true,
  "instruction": "Open the door with a downward-press handle, go through it, and enter the room.",
  "message": "任务完成"
}
```

**阻塞式执行 (`POST /execute`)**：
```bash
curl -X POST http://127.0.0.1:8088/execute \
  -H "Content-Type: application/json" \
  -d '{
    "instruction": "Open the door with a downward-press handle, go through it, and enter the room.",
    "timeout": 95.0,
    "poll_interval": 2.0
  }'
```

返回示例：
```json
{
  "ok": true,
  "success": true,
  "message": "任务完成"
}
```

**常规急停 (`POST /stop`)**：
```bash
curl -X POST http://127.0.0.1:8088/stop
```

返回示例：
```json
{
  "ok": true,
  "was_running": true
}
```

**强制关闭推理进程 (`POST /shutdown`)**：
```bash
curl -X POST http://127.0.0.1:8088/shutdown
```

返回示例：
```json
{
  "ok": true
}
```