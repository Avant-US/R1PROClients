# R1Pro × rldx-1 接入指南

本目录是 `r1pro_chassis` 的 **rldx-1** 协议版本，机器人本机程序在原版基础上只动了三处（processor / factory / scheduler / config），其余 ROS2 桥、轨迹管理、网关 `vla_door_client.py` 等全部沿用。

云端推理服务（GCP）的协议要求见下文「**协议对接**」。

---

## 1. 与原版的差异速查

| 项 | 原版 (openpi) | rldx-1 |
|---|---|---|
| `[model] processor` | `openpi` | **`rldx1`** |
| 视频帧数 | 单帧 | **4 帧**，`delta_indices = [-6, -4, -2, 0]`（0 = 当前帧） |
| 视频形状 | `(H, W, 3)` uint8 | **`(1, 4, H, W, 3)` uint8** |
| state 组织 | 拼成 23 维一条 | **按部位拆开，每个 `(1, 1, D)` float32** |
| `chassis_*` 字段 | 只有 `chassis`（3 维速度） | **`chassis_pose` (4 维) + `chassis_velocity` (3 维)** |
| `language.task` | 字符串 `"prompt"` | **`[["prompt"]]`**（list[list[str]]） |
| action 返回 | 单一 `(B, T, 23)` 张量 | **按部位拆好的 dict**，由 `action_key_map` 映射回本机 part 名 |

> 重要别名：rldx-1 训练时的 `chassis_pose` 实际上就是 r1pro 的 **`torso`**（4 维 JointState position），`chassis_velocity` 是底盘速度（3 维）。

---

## 2. 启动

```bash
cd /home/nvidia/zwy_WS/r1pro_chassis_rldx-1
source venv/bin/activate

# 默认 :8088（和原版一致；如果原版也在跑，换个端口避免冲突）
python3 vla_door_client.py --port 8089
```

对外 HTTP 接口（`/health`、`/status`、`/start`、`/execute`、`/stop`、`/shutdown`）和原版一致，见 [`API.md`](API.md)。

> `vla_door_client.py` 本身没改 —— 它只是 FastAPI 网关，不接触 obs/action 数据格式，所有协议差异都在 processor 和 config 里。

---

## 3. 关键配置（`config.toml`）

### `[model]`

```toml
processor = "rldx1"
```

### `[websocket]`

指向你的 rldx-1 云端 WebSocket 服务：

```toml
[websocket]
use_websocket = true
host = "34.6.175.178"   # 或 "wss://your.domain"
port = 8000
```

### `[rldx1]`

```toml
[rldx1]
default_prompt = "Open the door with a downward-press handle, go through it, and enter the room."

# 视频历史帧偏移；0 = 当前帧。任务刚开始历史不够时自动 clamp 到 buffer[0]。
delta_indices = [-6, -4, -2, 0]

# 底盘速度死区，绝对值小于该阈值的分量置零；0 = 不过滤。
chassis_deadzone = 0.01

# 每路相机的目标 [H, W]（client 端做 center-crop 按 aspect + bilinear resize）
[rldx1.image_target_hw]
head_rgb = [224, 224]
left_wrist_rgb = [224, 224]
right_wrist_rgb = [224, 224]

# 服务端 action dict 的 key  →  本机 publish 通道的 part key
# rldx-1 训练里 chassis_pose 实际就是 torso、chassis_velocity 是底盘速度
[rldx1.action_key_map]
left_arm = "left_arm"
right_arm = "right_arm"
left_gripper = "left_gripper"
right_gripper = "right_gripper"
chassis_pose = "torso"
chassis_velocity = "chassis"
```

### `[robot] enable_publish`

如果你的 rldx-1 模型会输出 `chassis_pose`（= torso 动作）并希望执行，把 `torso` 放进 publish 列表：

```toml
enable_publish = ["left_arm", "right_arm", "left_gripper", "right_gripper",
"torso",     # 想执行模型预测的躯干动作时取消注释
 "chassis"]
```

不想让 torso 跟着模型动，注释掉这一行即可（当前默认就是注释掉的）。

---

## 4. 数据流

```text
                                    HTTP :8088                   HTTP :9001
┌──────────┐                  ┌──────────────────┐         ┌─────────────────┐
│  Agent   │ ───────────────→ │ vla_door_client  │ ──────→ │     run.py      │
│ (door.py)│ ←─────────────── │  (FastAPI 网关)  │ ←────── │  (Scheduler +   │
└──────────┘                  └──────────────────┘         │   RLDX1Proc.)   │
                                                            └────────┬────────┘
                                                                     │
                                              msgpack / WebSocket    │
                                                                     ▼
                                                              ┌──────────────┐
                                                              │  云端 rldx-1 │
                                                              │  推理服务    │
                                                              └──────────────┘
```

机器人本机的关键调用顺序：

1. `Ros2Bridge.gather_obs()` 拿到当前帧 obs（单帧图像 + 各部位 state）
2. `RLDX1Processor.preprocess(obs)`：
   - 把当前帧推入每路相机的 `deque(maxlen=7)`
   - 按 `delta_indices` 抽 4 帧（不够 clamp 到 buffer[0]）
   - 拼出 `{video, state, language}` 三层 dict
3. `WebSocketClientEngine.predict_action(obs_dict)` → msgpack 发到云端 → 收回 action dict
4. `RLDX1Processor.postprocess(response)`：按 `action_key_map` 把 `chassis_pose / chassis_velocity` 映射成本机的 `torso / chassis`，整理成 `(1, T, D)`
5. `Scheduler.step()` → `actions_dict_to_trajectory` → ROS2 话题

---

## 5. 协议对接（写云端服务时按此实现）

复用本仓库现有的 WebSocket + msgpack（自定义 `__ndarray__` 扩展，见 `utils/websocket/msgpack.py`）。

### 连接建立

服务端先发**一条 msgpack 字典**当 metadata（可以是 `{}`），客户端会握手收掉。之后就是按 step 来回的 obs / action 二进制消息。

### 客户端 → 云端（每步 obs）

msgpack 解出后是嵌套 dict：

```python
{
    "video": {
        "head_rgb":         ndarray,  # uint8, (1, 4, H, W, 3)
        "left_wrist_rgb":   ndarray,  # uint8, (1, 4, H, W, 3)
        "right_wrist_rgb":  ndarray,  # uint8, (1, 4, H, W, 3)
    },
    "state": {
        "left_arm":         ndarray,  # float32, (1, 1, 7)
        "right_arm":        ndarray,  # float32, (1, 1, 7)
        "left_gripper":     ndarray,  # float32, (1, 1, 1)
        "right_gripper":    ndarray,  # float32, (1, 1, 1)
        "chassis_pose":     ndarray,  # float32, (1, 1, 4)  ← 来自机器人 torso
        "chassis_velocity": ndarray,  # float32, (1, 1, 3)  ← 来自底盘
    },
    "language": {
        "task": [["open the door ..."]],   # list[list[str]], shape (1, 1)
    },
}
```

### 云端 → 客户端（每步 action）

按部位 dict 返回。形状可以是 `(D,)` / `(T, D)` / `(1, T, D)`，processor 会自动整理成 `(1, T, D)`：

```python
{
    "left_arm":         ndarray,  # float32, (1, T, 7)
    "right_arm":        ndarray,  # float32, (1, T, 7)
    "left_gripper":     ndarray,  # float32, (1, T, 1)
    "right_gripper":    ndarray,  # float32, (1, T, 1)
    "chassis_pose":     ndarray,  # float32, (1, T, 4)
    "chassis_velocity": ndarray,  # float32, (1, T, 3)
    # 可选 timing，不影响控制：
    "server_timing": {"infer_ms": 12.3},
    "policy_timing": {"infer_ms": 10.1},
}
```

`T` 建议等于 `[basic] action_steps`（默认 50）。

错误约定：服务端出错时**发字符串**再关连接，客户端会把字符串当 traceback 抛 `RuntimeError`。

---

## 6. 调试与微调

| 想做的事 | 改哪里 |
|---|---|
| 换云端服务地址 | `config.toml` 的 `[websocket]` |
| 换 prompt | `config.toml` 的 `[rldx1] default_prompt`，或调 `/start` 时传 `instruction` |
| 改视频历史偏移 | `[rldx1] delta_indices`（必须 ≤ 0） |
| 改图像分辨率 | `[rldx1.image_target_hw]` |
| 服务端用不同的 key 名（比如 `arm_left` 而不是 `left_arm`） | 只改 `[rldx1.action_key_map]`，**不要碰 Python** |
| 底盘速度毛刺多 | 调大 `[rldx1] chassis_deadzone` |
| 想看每步 obs/action 数据 | 调 `GET :9001/obs` 开始/停止录制，结果写到 `/tmp/recorded_obs.json` |
| 想看推理耗时拆分 | 看 `run.py` 日志里 `preprocess / postprocess / client_total / policy_infer / server_infer / network` |

### 任务切换时重置帧历史

`Scheduler.start_task()` 已经会自动调用 `processor.reset_history()`，新任务从空 buffer 开始。如果你单独跑 `run.py` 测试且复用同一进程多次任务，也会自动清。

---

## 7. 涉及到的文件

| 文件 | 改动 |
|---|---|
| `core/processor/rldx1_processor.py` | **新增** — rldx-1 协议适配器 |
| `core/processor/factory.py` | 注册 `"rldx1"` 分支 |
| `scheduler/scheduler.py` | 把 rldx1 也归入「dict 进 dict 出」的远端路径；任务启动时调 `reset_history()` |
| `config.toml` | `processor = "rldx1"`；新增 `[rldx1]` 配置块 |
| `vla_door_client.py` | **未改** —— 网关层与协议无关 |
| `run.py` / `Ros2Bridge` / 轨迹管理等 | **未改** |

---

## 8. 故障速排

| 现象 | 可能原因 |
|---|---|
| `Buffer xxx is empty, skipping` | ROS2 话题未就绪；`/health` 看 `ros2_missing_topics` |
| `rldx1 postprocess: response 中找不到任何已知的 action 字段` | 云端返回 key 名和 `action_key_map` 对不上，改 `[rldx1.action_key_map]` |
| `Expected ... dim, got shape ...`（state） | obs 里某个部位维度异常；processor 会按目标维度补零或截断并打日志 |
| 视频前几帧看着不正常 | 历史 clamp 到 buffer[0] 的过渡阶段，1 秒内自然收敛 |
| 连不上服务端 | `[websocket] host/port`、防火墙、或本机有 `http_proxy`（client 内部已尝试临时清掉） |
| `address already in use 8088` | 之前的网关没退干净 ：`fuser -k 8088/tcp`，或换 `--port` |
