# R1 Pro 机器人 — ROS2 话题与相机参数总结

> 采集时间：2026-05-07，ROS_DOMAIN_ID=41
> 采集机器人：当前 nvidia-desktop 上的 R1 Pro

---

## 一、订阅话题 — 状态反馈（State）

代码位置：`core/communication/robot_topics.py` → `RobotTopicsConfig.state`

| 键名 | 话题 | 消息类型 | 发布频率 | 发布节点 |
|---|---|---|---|---|
| `left_arm` | `/hdas/feedback_arm_left` | `JointState` | ~200 Hz | HDAS |
| `right_arm` | `/hdas/feedback_arm_right` | `JointState` | ~167 Hz | HDAS |
| `torso` | `/hdas/feedback_torso` | `JointState` | ~500 Hz | HDAS |
| `chassis` | `/hdas/feedback_chassis` | `JointState` | ~200 Hz | HDAS |
| `left_ee_pose` | `/motion_control/pose_ee_arm_left` | `PoseStamped` | ~50 Hz | motion_control |
| `right_ee_pose` | `/motion_control/pose_ee_arm_right` | `PoseStamped` | ~50 Hz | motion_control |
| `left_gripper` | `/hdas/feedback_gripper_left` | `JointState` | ~200 Hz | HDAS |
| `right_gripper` | `/hdas/feedback_gripper_right` | `JointState` | ~167 Hz | HDAS |

**QoS 配置（订阅端）**：`BEST_EFFORT` / `KEEP_LAST(1)` / `VOLATILE`

**缓冲区**：`state_deque_length = 80`（按 >400 Hz 计，覆盖约 0.2s）

---

## 二、订阅话题 — 图像（Images）

代码位置：`core/communication/robot_topics.py` → `RobotTopicsConfig.images`

| 键名 | 话题 | 消息类型 | 发布频率 | 发布节点 |
|---|---|---|---|---|
| `head_rgb` | `/hdas/camera_head/left_raw/image_raw_color/compressed` | `CompressedImage` | ~15 Hz | signal_camera |
| `left_wrist_rgb` | `/hdas/camera_wrist_left/color/image_raw/compressed` | `CompressedImage` | ~15 Hz | RealSense |
| `right_wrist_rgb` | `/hdas/camera_wrist_right/color/image_raw/compressed` | `CompressedImage` | ~15 Hz | RealSense |

**QoS 配置（订阅端）**：`BEST_EFFORT` / `KEEP_LAST(1)` / `VOLATILE`

**缓冲区**：`camera_deque_length = 3`（按 15 Hz 计，覆盖约 0.2s）

---

## 三、发布话题 — 动作指令（Action）

代码位置：`core/communication/robot_topics.py` → `RobotTopicsConfig.action`

| 键名 | 话题 | 消息类型 | 用途 |
|---|---|---|---|
| `left_arm` | `/motion_target/target_joint_state_arm_left` | `JointState` | 左臂关节目标 |
| `right_arm` | `/motion_target/target_joint_state_arm_right` | `JointState` | 右臂关节目标 |
| `torso` | `/motion_target/target_joint_state_torso` | `JointState` | 躯干关节目标 |
| `chassis` | `/motion_target/target_speed_chassis` | `TwistStamped` | 底盘速度目标 |
| `left_ee_pose` | `/motion_target/target_pose_arm_left` | `PoseStamped` | 左臂末端位姿目标 |
| `right_ee_pose` | `/motion_target/target_pose_arm_right` | `PoseStamped` | 右臂末端位姿目标 |
| `left_gripper` | `/motion_target/target_position_gripper_left` | `JointState` | 左夹爪目标 |
| `right_gripper` | `/motion_target/target_position_gripper_right` | `JointState` | 右夹爪目标 |

**QoS 配置（发布端）**：`RELIABLE` / `KEEP_LAST(1)` / `VOLATILE`

> 注意：`vla_door_client.py` 中通过 `ros2 topic pub --once` 硬编码了以下 3 个话题用于初始/复位姿态：
> - `/motion_target/target_joint_state_torso`
> - `/motion_target/target_joint_state_arm_left`
> - `/motion_target/target_joint_state_arm_right`

---

## 四、相机原始参数

### 4.1 头部双目相机（Head Stereo Camera）

由 `signal_camera` 节点发布，无 `camera_info` 话题（不发布内参标定数据）。

#### 左目

| 参数 | 值 |
|---|---|
| 话题 | `/hdas/camera_head/left_raw/image_raw_color/compressed` |
| 分辨率 | **640 x 360** |
| 帧率 | ~15 Hz |
| 格式 | JPEG 压缩 |
| 单帧大小 | ~17 KB |

#### 右目（项目未使用，仅供参考）

| 参数 | 值 |
|---|---|
| 话题 | `/hdas/camera_head/right_raw/image_raw_color/compressed` |
| 分辨率 | **1920 x 1080** |
| 帧率 | ~30 Hz |
| 格式 | JPEG 压缩 |
| 单帧大小 | ~106 KB |

### 4.2 左腕相机（Wrist Left — RealSense）

#### 彩色流

| 参数 | 值 |
|---|---|
| 话题 | `/hdas/camera_wrist_left/color/image_raw/compressed` |
| 分辨率 | **640 x 480** |
| 帧率 | ~15 Hz |
| frame_id | `/hdas/camera_wrist_left_color_optical_frame` |
| 畸变模型 | `plumb_bob`（5 参数径向 + 切向） |
| 畸变系数 D | `[-0.0509, 0.0617, -0.0006, 0.0011, -0.0217]` |
| 内参 fx | 436.79 |
| 内参 fy | 436.27 |
| 内参 cx | 315.94 |
| 内参 cy | 240.31 |

#### 深度流

| 参数 | 值 |
|---|---|
| 话题 | `/hdas/camera_wrist_left/depth/image_raw` |
| 分辨率 | **640 x 480** |
| 畸变 D | 全为 0 |
| 内参 fx=fy | 388.13 |
| 内参 cx | 320.40 |
| 内参 cy | 238.49 |

#### Depth → Color 外参

| 参数 | 值 |
|---|---|
| 旋转 | 近似单位阵（偏差 < 0.006 rad） |
| 平移 | ≈ (0, 0, 0) mm |

### 4.3 右腕相机（Wrist Right — RealSense）

#### 彩色流

| 参数 | 值 |
|---|---|
| 话题 | `/hdas/camera_wrist_right/color/image_raw/compressed` |
| 分辨率 | **640 x 480** |
| 帧率 | ~15 Hz |
| frame_id | `/hdas/camera_wrist_right_color_optical_frame` |
| 畸变模型 | `plumb_bob` |
| 畸变系数 D | `[-0.0531, 0.0603, -0.0002, 0.0009, -0.0201]` |
| 内参 fx | 434.77 |
| 内参 fy | 434.25 |
| 内参 cx | 322.27 |
| 内参 cy | 246.09 |

#### 深度流

| 参数 | 值 |
|---|---|
| 话题 | `/hdas/camera_wrist_right/depth/image_raw` |
| 分辨率 | **640 x 480** |
| 畸变 D | 全为 0 |
| 内参 fx=fy | 385.76 |
| 内参 cx | 325.56 |
| 内参 cy | 244.71 |

---

## 五、相机参数汇总对比

| 相机 | 分辨率 | 帧率 | 有内参标定 | 有深度 | 项目是否使用 |
|---|---|---|---|---|---|
| 头部左目 | 640x360 | ~15 Hz | 否 | 否 | 是 |
| 头部右目 | 1920x1080 | ~30 Hz | 否 | 否 | 否 |
| 左腕 RGB | 640x480 | ~15 Hz | 是 | 是 | 是 |
| 右腕 RGB | 640x480 | ~15 Hz | 是 | 是 | 是 |

**代码中的统一处理**：所有图像在 `openpi_processor.py` 中被 resize 到 `image_target_size`（默认 224x224，由 `config.toml` 配置）。

---

## 六、话题频率汇总

| 话题 | 频率 |
|---|---|
| `/hdas/feedback_arm_left` | ~200 Hz |
| `/hdas/feedback_arm_right` | ~167 Hz |
| `/hdas/feedback_torso` | ~500 Hz |
| `/hdas/feedback_chassis` | ~200 Hz |
| `/motion_control/pose_ee_arm_left` | ~50 Hz |
| `/motion_control/pose_ee_arm_right` | ~50 Hz |
| `/hdas/feedback_gripper_left` | ~200 Hz |
| `/hdas/feedback_gripper_right` | ~167 Hz |
| `/hdas/camera_head/left_raw/image_raw_color/compressed` | ~15 Hz |
| `/hdas/camera_wrist_left/color/image_raw/compressed` | ~15 Hz |
| `/hdas/camera_wrist_right/color/image_raw/compressed` | ~15 Hz |

> 动作指令话题（`/motion_target/*`）为按需发布，无固定频率。

---

## 七、移植到新机器人注意事项

1. **话题名称**：确认新机器人是否使用相同的 `/hdas/` 和 `/motion_target/` 命名空间。如不同，修改 `robot_topics.py` 和 `vla_door_client.py` 中的硬编码话题。
2. **QoS 匹配**：确认发布端的 QoS（Reliability、Durability）与代码中的订阅端 QoS 兼容。
3. **相机分辨率**：如果分辨率不同，resize 到 224x224 的裁剪比例会变化，可能影响模型效果。
4. **相机内参**：每台 RealSense 出厂标定不同，若模型只用 RGB 不涉及点云投影，可忽略。
5. **状态反馈频率**：如果新机器人反馈频率更低，需调整 `state_deque_length` 以保证 0.2s 窗口覆盖。
6. **`ROS_DOMAIN_ID`**：确保新机器人的 `ROS_DOMAIN_ID` 一致（当前为 41）。

---

## 八、常用诊断命令

以下命令均需先设置环境变量：

```bash
export ROS_DOMAIN_ID=41
```

### 8.1 查看所有活跃话题

```bash
ros2 topic list
```

### 8.2 查看某个话题的详细信息（发布者、订阅者、QoS）

```bash
ros2 topic info /hdas/feedback_arm_left --verbose
```

### 8.3 测量话题发布频率

```bash
ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed
# Ctrl+C 停止，观察 average rate
```

### 8.4 查看相机内参（camera_info）

```bash
# 腕部相机有 camera_info 话题
ros2 topic echo /hdas/camera_wrist_left/color/camera_info --once

# 头部相机无 camera_info，需从 JPEG 头解析分辨率
```

### 8.5 从压缩图像解析分辨率（当没有 camera_info 时）

```python
# 保存为 check_resolution.py，执行: python3 check_resolution.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

TOPIC = "/hdas/camera_head/left_raw/image_raw_color/compressed"

rclpy.init()
node = Node("resolution_checker")
done = False

def cb(msg):
    global done
    if done:
        return
    done = True
    data = bytes(msg.data)
    i = 0
    while i < len(data) - 9:
        if data[i] == 0xFF and data[i + 1] in (0xC0, 0xC2):
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
            print(f"Resolution: {w}x{h}")
            print(f"JPEG size : {len(data)} bytes")
            print(f"Format    : {msg.format}")
            return
        i += 1
    print(f"No SOF marker found. Data size: {len(data)}")

qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)
node.create_subscription(CompressedImage, TOPIC, cb, qos)

import time
t0 = time.time()
while rclpy.ok() and not done and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.5)
if not done:
    print("Timeout: no message received")
rclpy.shutdown()
```

### 8.6 查看深度到彩色的外参

```bash
ros2 topic echo /hdas/camera_wrist_left/extrinsics/depth_to_color --once
```

### 8.7 查看相机元数据（帧号、硬件时间戳等）

```bash
ros2 topic echo /hdas/camera_wrist_left/color/metadata --once
```

### 8.8 查看某话题的发布者数量（排查冲突）

```bash
ros2 topic info /motion_target/target_joint_state_arm_left --verbose
# 检查 Publisher count 是否 > 1
```

### 8.9 列出所有 ROS2 节点

```bash
ros2 node list
```

### 8.10 查看某节点的详细信息（订阅/发布了哪些话题）

```bash
ros2 node info /HDAS
```

### 8.11 过滤查看特定前缀的话题

```bash
ros2 topic list | grep camera_wrist
ros2 topic list | grep feedback
ros2 topic list | grep motion_target
```
