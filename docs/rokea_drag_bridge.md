# Rokea 拖拽 → Marvin ServoJ（rokea_drag_bridge）

本机对 Rokae AR3 双臂开关节空间拖拽，按 **50 Hz** 发布已轴映射的 `JointState`；nimo-012 上 `dual_arm_node` 在 **ServoJ 遥操作** 门控下 `ServoJ` 跟随。

## 网络

| 角色 | 网段 / 地址 |
|------|-------------|
| Rokae 控制器 | 默认 `192.168.2.160` / `.161`，本机口 `192.168.2.100` |
| Marvin / GUI / Orin | 例如 GUI `192.168.0.92`，nimo-012 `192.168.0.12` |
| DDS | 两边 `ROS_DOMAIN_ID=0` |

跑 bridge 的机器须能同时访问 **Rokae 网段** 与 **机器人 DDS 网段**（双网卡或路由）。本机须有 `local_ip`（默认 `192.168.2.100`）网卡地址。

SDK 日志写在 `~/.cache/rokea_drag_bridge/logs/`。若曾用 root 跑过并在当前目录留下 `logs/global_logger_*.log`，会导致进程直接 Aborted；可删或改属主：`sudo chown -R "$USER" ~/rokea_control/logs`。

## 轴映射（只在 bridge）

Rokea → Marvin（`tj_control` URDF）：

```text
q_tj = [q1, -q2, q3, -q4, q5, -q6, q7]_rokea
```

后端与 GUI **不再**翻符号。参数 `sign_flip` 可改。

## 本机编译 / 启动

```bash
source /opt/ros/jazzy/setup.bash
cd /home/yxc/rokea_control/ws   # 或含 rokea_drag_bridge 的 colcon ws
colcon build --symlink-install --packages-select rokea_drag_bridge
source install/setup.bash
export ROS_DOMAIN_ID=0
# launch 内已设 FastDDS 单播 peer→192.168.0.12（与 GUI 一致；WiFi 组播不通）

ros2 launch rokea_drag_bridge drag_bridge.launch.py
# 或改 IP：
# ros2 launch rokea_drag_bridge drag_bridge.launch.py \
#   local_ip:=192.168.2.100 ip_a:=192.168.2.160 ip_b:=192.168.2.161 rate_hz:=50.0
```

话题：

- 发布 `arm/left/target_joints`、`arm/right/target_joints`（已连接即发，供 MovJ 对齐；Marvin 符号）
- 发布 `rokea/drag_bridge/status`（`std_msgs/Bool`，true = 双臂 drag 开）
- 订阅 `rokea/drag_bridge/enable`（`std_msgs/Bool`，由 `dual_arm_node` 镜像 `servoj_teleop_enabled`）
  - `true` → `enableDrag`（需下电态）
  - `false` / 未勾选 ServoJ / 自动退出 → `disableDrag` + 下电（保持连接）

启动时先连臂并下电、drag 关，但关节话题已在发。勾选 GUI「ServoJ」→ 后端 MovJ 对齐 → `enable=true` 开 Rokae 拖拽。SIGINT 时 `disableDrag` 并断开。

## 远程后端（nimo-012）

`servoj_interp_time_s:=0.02`，`servoj_align_tol_rad:=0.25`，`servoj_joint_timeout_s:=0.25`。

ServoJ 使能上升沿若 `|q_cmd-q_ref|` 超容差则先 `MovJ` 再流式跟随。

**自动退出 ServoJ 遥操作**（清 `servoj_teleop_enabled`，GUI 勾选随 status 灭）：

- 任一侧 Fault / EStop / Alarm
- 对齐 `MovJ` 或流式 `ServoJ` 被拒绝
- 已开始跟随后 `target_joints` 超时（默认 0.25 s）

自动退出 ServoJ 时同步发 `rokea/drag_bridge/enable=false`，本机 bridge 会停拖拽并下电。

## GUI

勾选 **ServoJ遥操作模式** 即开 Rokea 拖拽；取消勾选（或自动退出）即下电。`bridge` 灯反映 `rokea/drag_bridge/status`。

## 推荐顺序

1. nimo-012：`backend.launch` 已起  
2. GUI：Init → Enable → ControlMode  
3. 本机：起 `rokea_drag_bridge`（此时臂已下电、未开拖拽）  
4. GUI：勾选 ServoJ 遥操作 → Rokea enableDrag  
5. 人推 Rokae；结束取消勾选 → Rokea 下电；再停 bridge 进程  

与 Pico 遥操作、轨迹复现互斥。
