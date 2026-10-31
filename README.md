# rokea_control

珞石 AR3 双臂相关资产库（**非 ROS 2 工程**）：整机 description、解析运动学、厂商文档，以及右臂连通性检查示例。

## 目录

| 路径 | 说明 |
|------|------|
| [`src/rokea_description`](src/rokea_description) | 台架 + 左右臂 meshes / URDF |
| [`src/rokea_kinematics`](src/rokea_kinematics) | 7-DoF PoE 解析正逆解 |
| [`src/rokea_drag_bridge`](src/rokea_drag_bridge) | ROS 2：双臂关节拖拽 + 50 Hz 发布（Marvin 轴映射） |
| [`docs/rokea_docs`](docs/rokea_docs) | 珞石硬件手册、xCore SDK 手册等 |
| [`docs/sdk_overview.md`](docs/sdk_overview.md) | SDK 文档索引与 AR3 相关说明（对照[官网](https://docs.rokae.com/docs/SDK/cpp)） |
| [`docs/connection_check.md`](docs/connection_check.md) | 网络 ping 与双臂接线说明 |
| [`docs/rokea_drag_bridge.md`](docs/rokea_drag_bridge.md) | 拖拽桥接启停、轴映射、与 nimo ServoJ 联调 |
| [`examples/ar3_right_arm_status`](examples/ar3_right_arm_status) | 右臂（A-LAN1）只读状态最小示例 |

## 首次连通（当前接 A-LAN1）

一根网线即可（A-LAN1 = **右臂**，默认 `192.168.1.160`）。详细步骤见 [docs/connection_check.md](docs/connection_check.md)。

```bash
# 本机有线口设为 192.168.1.100/24 后：
ping -c 4 192.168.1.160
```

SDK 读状态见 [`examples/ar3_right_arm_status`](examples/ar3_right_arm_status)。

## 依赖 SDK

珞石 xCore C++ SDK 以 git submodule 放在 `third_party/xCoreSDK-CPP`：

```bash
git clone --recurse-submodules https://github.com/Asukayoo/rokea_control.git
# 若已克隆未拉 submodule：
git submodule update --init --recursive
```

## 参考

- [xCoreSDK-CPP](https://github.com/RokaeRobot/xCoreSDK-CPP)
- [珞石 SDK 在线文档](https://docs.rokae.com/docs/SDK/cpp)
