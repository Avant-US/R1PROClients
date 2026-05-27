# R1 Pro Visualizer

轻量本地网页可视化器，用来回放 `/tmp/openpi_processed_actions.txt` 到
`/home/nvidia/juncheng_ws/urdf/r1pro/r1_pro_with_gripper.urdf`。

## 启动

```bash
cd /home/nvidia/kaizhe_ws/r1pro_chassis
python3 visualizer/server.py
```

默认访问地址：

```text
http://127.0.0.1:8765
```

## 自定义路径

```bash
python3 visualizer/server.py \
  --urdf /home/nvidia/juncheng_ws/urdf/r1pro/r1_pro_with_gripper.urdf \
  --actions /tmp/openpi_processed_actions.txt \
  --host 0.0.0.0 \
  --port 8765
```

## 当前能力

- 加载 R1 Pro URDF 与 OBJ mesh
- 解析 `openpi_processed_actions.txt`
- 回放 `left_arm/right_arm/torso/chassis`
- 用单个夹爪标量驱动两个指爪关节
- 根据 `chassis(vx, vy, wz)` 积分出底盘平面移动轨迹
- 显示当前帧原始值与实际写入 URDF 的关节值

## 注意

- 夹爪标量语义暂时不确定，页面里提供了 `scale` 和 `offset` 用于试调
- 底盘速度值通常很小，页面默认加了可视化缩放，方便观察移动效果
- 第一版是离线回放，不订阅 ROS2 实时话题
