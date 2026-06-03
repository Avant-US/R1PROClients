# r1pro_chassis

R1Pro 机器人的 VLA 控制客户端。订阅机器人 ROS2 话题，把观测发给远端推理服务器拿回动作，再通过 ROS2 发回机器人执行。

对外提供一个 HTTP API（端口 8088），方便下发"开门""进房间"这类任务。

---

## 目录

- [给测试人员（看这里就够了）](#给测试人员看这里就够了)
  - [第 1 步：把代码放对地方](#第-1-步把代码放对地方)
  - [第 2 步：体检](#第-2-步体检检查机器满不满足条件)
  - [第 3 步：安装环境 + 起服务](#第-3-步安装环境--起服务)
  - [第 4 步：测试](#第-4-步测试)
- [前置条件清单](#前置条件清单)
- [遇到问题怎么办（故障排查）](#遇到问题怎么办故障排查)
- [安全须知](#安全须知)
- [给运维 / 二次开发](#给运维--二次开发)

---

## 给测试人员（看这里就够了）

下面 4 步，每一步都给了"做什么 + 看到什么算成功"，按顺序做就行。

### 第 1 步：把代码放对地方

代码必须放在 `/home/nvidia/kaizhe_ws/r1pro_chassis/`（不能换路径，systemd 服务文件里写死了）。

```bash
# 如果是从 U 盘或者其他机器复制过来的：
mkdir -p /home/nvidia/kaizhe_ws
cd /home/nvidia/kaizhe_ws
# 然后把 r1pro_chassis 整个文件夹放到这里
```

**确认放对了**：

```bash
ls /home/nvidia/kaizhe_ws/r1pro_chassis/install.sh
# 看到文件路径输出 = OK
```

如果不小心把别人机器上的 `.venv` 也复制过来了，先删掉（很大且不能跨机器用）：

```bash
rm -rf /home/nvidia/kaizhe_ws/r1pro_chassis/.venv
```

### 第 2 步：体检（检查机器满不满足条件）

```bash
cd /home/nvidia/kaizhe_ws/r1pro_chassis
bash preflight.sh
```

**看到什么算成功**：最后一行打印类似

```
体检结果: PASS=13  WARN=1  FAIL=0

下一步：
  bash deploy.sh
```

`FAIL=0` 就算过。WARN 可以忽略。

**如果有 FAIL**：先看下面"[前置条件清单](#前置条件清单)"对照找责任人，然后再看"[遇到问题怎么办](#遇到问题怎么办故障排查)"。

### 第 3 步：安装环境 + 起服务

```bash
bash deploy.sh
```

中间会**问你 sudo 密码**（装 systemd 服务用），输一下就行。

整个过程大约 **2-5 分钟**（取决于网速）。

**看到什么算成功**：最后打印类似

```
==========================================================
 部署完成
==========================================================

 服务管理：
   sudo systemctl status door-skill.service      # 看状态
   ...
```

**如果中途失败**：脚本会立刻退出并告诉你失败原因。看"[遇到问题怎么办](#遇到问题怎么办故障排查)"。

### 第 4 步：测试

#### 4.1 不动机器人的测试（先做这个）

```bash
# 测试 1：服务是不是在跑
sudo systemctl status door-skill.service
# 期望看到: Active: active (running)

# 测试 2：HTTP 接口通不通
curl http://localhost:8088/health
# 期望返回类似: {"ok":true,"efm_ready":false,"ros2_ready":true,"ros2_missing_topics":[]}
# 关键: ros2_ready 是 true，ros2_missing_topics 是空数组

# 测试 3：任务状态接口
curl http://localhost:8088/status
# 期望返回: {"state":"idle", ...}
```

这 3 项都过 = 部署成功。

> 关于 `efm_ready: false`：正常现象。推理进程是按需启动的，没下任务时它不跑。

#### 4.2 动机器人的测试（确认环境安全再做）

**做之前必须确认**（看后面"[安全须知](#安全须知)"）：
- 机器人**周围没有人**
- **紧急停止按钮**伸手可及
- 任务期间**全程有人看守**

```bash
# 下发"开门并进房间"任务（任务大约 60-140 秒）
curl -X POST http://localhost:8088/execute \
  -H "Content-Type: application/json" \
  -d '{
    "instruction": "Open the door with a downward-press handle, go through it, and enter the room.",
    "timeout": 140
  }'
```

任务结束 curl 会返回结果。结束时机器人关节会自动复位到零点（**手臂可能甩动**，注意周围）。

如果想中途紧急停止：

```bash
curl -X POST http://localhost:8088/stop
```

更多 API（查状态、改指令、停止等）见 [`API.md`](API.md)。

---

## 前置条件清单

`preflight.sh` 会自动检查下面这些。**任何一项 FAIL，必须先解决再继续部署**。

| # | 项目 | 谁负责 | 不满足怎么办 |
|---|---|---|---|
| 1 | aarch64 架构（Jetson Orin） | 硬件 | 换机器，本项目只支持 Jetson |
| 2 | Ubuntu 22.04 | 装机时刷的 JetPack | 重刷或忽略警告 |
| 3 | Python 3.10 + python3-venv | apt | `sudo apt install python3-venv python3-pip` |
| 4 | ROS2 Humble + rclpy | 运维装的 | 找运维 |
| 5 | Galaxea 包 (`/home/nvidia/galaxea/install/`) | Galaxea 团队 | 找 Galaxea 团队要部署脚本 |
| 6 | Jetson 版 PyTorch (`~/.local/lib/python3.10/site-packages/torch`) | NVIDIA wheel | 参考 [NVIDIA 官方教程](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/) |
| 7 | 推理服务器 `34.32.242.109:8000` 可访问 | 网络管理员 | 找运维开网络；或修改 `config.toml` 里的 `[websocket] server_endpoint` |
| 8 | sudo 权限 | 自己 | 测试人员账号一般都有 |
| 9 | 端口 8088 / 9001 没被占 | 自己 | 看下面排查 6.3 |

> **关键点**：前置 4、5、6 不是本项目能解决的，是系统级安装。`deploy.sh` 不会自动装这些，只会建 venv 装 Python 包。

---

## 遇到问题怎么办（故障排查）

### 排查 1：`preflight.sh` 报 FAIL

按报错信息找责任人，参考上面"[前置条件清单](#前置条件清单)"的"谁负责"列。

### 排查 2：`deploy.sh` 在 install.sh 阶段失败（装 venv / pip 装包阶段）

最常见三种：

| 现象 | 原因 | 解决 |
|---|---|---|
| `pip install` 网络超时 | pip 镜像没配 | 配清华源后重跑：<br>`mkdir -p ~/.config/pip && echo -e "[global]\nindex-url = https://pypi.tuna.tsinghua.edu.cn/simple" > ~/.config/pip/pip.conf` |
| `numpy 必须是 1.x` | 装包时 numpy 被升到 2.x（torch ABI 不兼容） | 删 venv 重跑：`rm -rf .venv && bash deploy.sh` |
| 冒烟测试 `from scheduler.scheduler import Scheduler` 报错 | 项目代码本身有问题 | 联系项目维护者 |

### 排查 3：`deploy.sh` 在 systemd 阶段失败

```bash
# 看 systemd 加载服务时报什么错
sudo systemctl status door-skill.service
journalctl -u door-skill.service -n 50 --no-pager
```

| 日志关键词 | 原因 | 解决 |
|---|---|---|
| `address already in use` | 8088 被占 | `sudo lsof -i :8088` 看谁占着；通常是旧的 `vla_door_client` 进程，`sudo kill -9 <pid>` 后重跑 deploy |
| `ModuleNotFoundError` | venv 没装好 | `bash install.sh` 重建 venv |
| `Permission denied` 写 `/etc/systemd/` | 没 sudo | 重跑 `deploy.sh` 并输入 sudo 密码 |

### 排查 4：服务起来了但 `ros2_ready: false`

```bash
# 看 systemd 进程的环境变量是否带 Galaxea 路径
sudo cat /proc/$(pgrep -f vla_door_client | head -1)/environ | tr '\0' '\n' | grep AMENT_PREFIX_PATH
# 期望: 路径里既有 /opt/ros/humble，又有 /home/nvidia/galaxea/install/*
```

如果只有 ROS2 没有 Galaxea，说明服务文件里 source Galaxea 那行失败了。检查 `/etc/systemd/system/door-skill.service` 里的 `ExecStart` 行，确认 `/home/nvidia/galaxea/install/setup.bash` 路径存在且可读。改对后：

```bash
sudo systemctl daemon-reload
sudo systemctl restart door-skill.service
```

### 排查 5：任务下发后立即 `state: failed`，message 是 `run.py 启动失败`

```bash
# 看 run.py 启动时的错误堆栈
journalctl -u door-skill.service --since "5 minutes ago" | grep -A 30 "启动失败\|Traceback"
```

两种最常见：

- `ModuleNotFoundError: No module named 'xxx'` — venv 缺包。`bash install.sh` 重建
- `ConnectionRefusedError: 34.32.242.109:8000` — 推理服务器连不上。查前置条件 7

### 排查 6：任务下发后 `state: running` 但机器人不动

看推理服务器是否真有响应：

```bash
journalctl -u door-skill.service --since "1 minute ago" | grep -E "Infer cost|preprocess"
# 期望: 每秒看到 1-2 行
```

- 完全没有 `Infer cost` → 推理服务器没回包，找推理服务器维护方
- 有 `Infer cost` 但机器人不动 → 看 `config.toml` 的 `[robot] enable_publish` 列表是不是空的（空列表 = 屏蔽所有动作发布）

---

## 安全须知

- **`config.toml` 里 `[robot] enable_publish` 列表**：列表里有什么部位，对应部位才会被发动作命令。**调试时建议清空这个列表**，机器人就不会动。
- **改完代码 / 改完 `config.toml`**：必须 `sudo systemctl restart door-skill.service` 才生效。
- **远端推理服务器** `34.32.242.109:8000` 是公共服务器，不要瞎发请求，会影响其他用户。
- **任务执行中**：不要反复 Ctrl+C 终端，可能触发清理逻辑残留进程。要中断用 `curl -X POST http://localhost:8088/stop`。
- **关节复位时手臂可能甩动**：每次 `/stop` 或任务完成机器人会自动归零，注意周围别站人。

---

## 给运维 / 二次开发

### 项目大致是怎么回事

```
                              远端推理服务器
                         (34.32.242.109:8000)
                                  ▲
                                  │ websocket
                                  ▼
   ┌──────────────────────┐    拉起    ┌──────────────────┐
   │  vla_door_client.py  │ ─────────► │     run.py       │
   │   (FastAPI 网关)     │            │  (推理调度器)     │
   │   端口 8088          │            │   端口 9001       │
   └──────────────────────┘            └──────────────────┘
            ▲                                   ▲ │
            │ HTTP                              │ │ ROS2
            │ 任务下发                          │ ▼
       测试 / Agent                       机器人 ROS2 节点
                                         (Galaxea 提供)
```

两个进程都在本机。`vla_door_client` 由 systemd 自启常驻，`run.py` 由 `vla_door_client` 按需拉起。

### 关键文件

| 文件 | 作用 |
|---|---|
| `config.toml` | 项目配置（改推理服务器地址、改启用的机器人部位） |
| `vla_door_client.py` | HTTP 网关入口 |
| `run.py` | 推理调度器入口（不直接启动，由网关拉起） |
| `preflight.sh` | 部署前体检脚本 |
| `deploy.sh` | 一键部署脚本（装 venv + 装 systemd） |
| `install.sh` | 只建 venv（被 `deploy.sh` 调用） |
| `requirements.txt` | venv 内 pip 装的包 |
| `requirements-system.txt` | 系统级依赖说明（不用装，仅供查阅） |
| `docs/door-skill.service.template` | systemd 服务模板 |
| `API.md` | HTTP 接口详细文档 |
| `docs/r1pro_ros2_topics_and_cameras.md` | 项目订阅的 ROS2 话题清单 |

### 日常运维命令

| 操作 | 命令 |
|---|---|
| 看服务状态 | `sudo systemctl status door-skill.service` |
| 重启服务（改完代码必跑） | `sudo systemctl restart door-skill.service` |
| 停止服务 | `sudo systemctl stop door-skill.service` |
| 实时日志 | `journalctl -u door-skill.service -f` |
| 最近 100 行日志 | `journalctl -u door-skill.service -n 100` |
| 重建 venv | `bash install.sh` |
| 看 venv 装了啥 | `.venv/bin/pip list` |
| 紧急停止任务 | `curl -X POST http://localhost:8088/stop` |

### 卸载

```bash
sudo systemctl disable --now door-skill.service
sudo rm /etc/systemd/system/door-skill.service
sudo systemctl daemon-reload
# 然后删项目目录：rm -rf /home/nvidia/kaizhe_ws/r1pro_chassis
```

---

## 联系方式

部署/使用问题，联系项目维护者。
