# xCore SDK（C++）文档说明 — AR3 双臂

本仓库相关材料与官方文档索引。官方在线文档入口：[docs.rokae.com/docs/SDK/cpp](https://docs.rokae.com/docs/SDK/cpp)（版本选 **0.7.x**；AR 机型可对照 **0.7.x-AR**）。

本地 PDF（与控制器 V3.2.1 对应）：

- [`rokea_docs/rokea/xCore_SDK_C++使用手册V3.2_A.pdf`](rokea_docs/rokea/xCore_SDK_C++使用手册V3.2_A.pdf)
- 硬件网口：[`rokea_docs/rokea/AR3产品硬件安装手册-20260923.pdf`](rokea_docs/rokea/AR3产品硬件安装手册-20260923.pdf)
- 本机连通步骤：[connection_check.md](connection_check.md)
- 源码仓库：[RokaeRobot/xCoreSDK-CPP](https://github.com/RokaeRobot/xCoreSDK-CPP)

---

## 1. 文档从哪里读

| 来源 | 内容 | 用途 |
|------|------|------|
| [SDK 文档总览](https://docs.rokae.com/docs/SDK/cpp) | 简介 / 接口 / 错误码 / FAQ / 场景示例 / 附录 | 日常查 API |
| [快速开始](https://docs.rokae.com/docs/SDK/quick_start/) | 网络、编译、日志、实时超时配置 | 首次搭环境 |
| [SDK-C++接口说明](https://docs.rokae.com/docs/SDK/cpp/cpp_API/) | 各接口一览表 | 找函数名 |
| 本地 PDF 手册 | 与上类似 + 示例代码片段 + 完整附录 | 离线、与 V3.2.1 对齐 |
| SDK 仓 `example/` | 可编译示例 | 对照实现 |

在线文档主要栏目（C++）：

1. SDK-C++简介  
2. SDK-C++接口说明  
3. 错误码和异常说明  
4. 非实时指令常见问题  
5. 实时模式常见问题（含网络、力矩、**AR 机型专栏**等）  
6. 场景示例  
7. 附录：类型 / 数据结构 / 方法说明  

另有 Python / C# / 弧焊 SDK，结构平行，本仓库当前用 **C++**。

---

## 2. 与本机 AR3 相关的关键结论

实测右臂（A-LAN1）：

| 项 | 值 |
|----|-----|
| 类 | `rokae::xMateErProRobot`（协作 **7** 轴） |
| 机器人 IP | `192.168.1.160`（A-LAN1） |
| 本机 IP | `192.168.1.165`（示例） |
| 机型字符串 | `AR3-3_0.7R-W4C5C5` |
| 控制器 | ≥ `3.2.1`（实测 `3.2.1.C224...`） |
| 读关节 | `jointPos()` → `std::array<double,7>`，单位 **rad** |

注意：

- 官网「直连默认 `192.168.0.160`」是**通用单机**说法；**双臂 MAC 柜以硬件手册为准**（A-LAN1=`192.168.1.160`，A-LAN2=`192.168.2.160`）。见 [connection_check.md](connection_check.md)。
- 左右臂各一块核心板 → **两个 IP、两个 Robot 实例**，没有「一个网口控双臂」。
- 构造时传入 `(robot_ip, local_ip)`；实时模式更依赖正确的本机 IP。

本仓库只读示例：[`examples/ar3_right_arm_status`](../examples/ar3_right_arm_status)。

---

## 3. 控制模式概览

### 3.1 非实时（NRT）

控制器内部做轨迹规划，上位机发目标点即可。

- 运动：`MoveAbsJ` / `MoveJ` / `MoveL` / `MoveC` / `MoveCF` / `MoveSP`（`moveAppend` + `moveStart` 等）
- 状态：`robotInfo` / `jointPos` / `jointVel` / `jointTorque` / `posture` / `powerState` …
- Jog、IO、寄存器、RL 工程、拖动与路径回放（协作）、碰撞检测、软限位等

适合：点到点、示教回放、首次联调读状态（**当前阶段**）。

### 3.2 实时（RT，最高约 1 kHz）

协作（含 7 轴）常见模式：

- 轴空间位置（含 ServoJ）
- 笛卡尔空间位置
- 轴 / 笛卡尔阻抗
- 直接力矩

状态：`startReceiveRobotState` → `updateRobotState` → `getStateData`（如 `jointPos_m`）。  
实时建议有线直连；UDP 相关端口需放行（常见排查见[网络通信问题](https://docs.rokae.com/docs/SDK/cpp/%E5%AE%9E%E6%97%B6%E6%A8%A1%E5%BC%8F%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98/cpp_rt_network_communication/)）。

---

## 4. 机器人类选择（必对）

| 类名 | 适用 |
|------|------|
| `xMateRobot` | 协作 6 轴 |
| **`xMateErProRobot`** | **协作 7 轴（AR3）** |
| `StandardRobot` | 工业 6 轴 |
| `xMateCr5Robot` | 协作 CR5 |
| `PCB4Robot` / `PCB3Robot` | 工业 4/3 轴 |

类型与轴数不匹配会在连接/校验时报错。

---

## 5. 只读状态常用 API（已验证）

```cpp
rokae::xMateErProRobot robot("192.168.1.160", "192.168.1.165");
error_code ec;

auto info = robot.robotInfo(ec);   // version / type / joint_num
auto q    = robot.jointPos(ec);    // 7 × rad
auto v    = robot.jointVel(ec);    // rad/s
auto tau  = robot.jointTorque(ec); // Nm
auto ps   = robot.powerState(ec);  // on / off / Estop / Gstop
auto om   = robot.operateMode(ec); // automatic / manual
auto os   = robot.operationState(ec);

robot.disconnectFromRobot(ec);
```

在线表：[接口说明 · 基本操作及信息查询](https://docs.rokae.com/docs/SDK/cpp/cpp_API/)。

`PowerState` 数值含义（与手册一致）：上电 / 下电 / **急停** / 安全门等。实测右臂曾出现急停态，只读不受影响；若要运动需先复位急停。

---

## 6. 若要开始运动（尚未做）

非实时最小路径概念：

1. `setMotionControlMode(NrtCommand)`  
2. `setOperateMode(automatic)`、解除急停、`setPowerState(true)`（视现场安全与示教器配置）  
3. `moveAppend(...)` → `moveStart()`，或 `executeCommand(...)`  
4. 用 `operationState` / 事件回调等待结束；`stop()` 急停级停止  

实时：切 `RtCommand`，`getRtMotionController()`，再按模式 `startMove` + `sendCommand`。  
细节与坑见官网「非实时指令常见问题」「实时模式常见问题」，以及 PDF §5、§6。

**本仓库当前约定：只读状态、不做控制。**

---

## 7. 工程包与编译（摘要）

官方仓结构：

```text
xCoreSDK-CPP/
  include/     # rokae/*.h
  lib/         # Release 预编译库（需另行下载解压）
  external/    # Eigen
  example/     # 官方示例
```

- CMake ≥ 3.12，C++17  
- 预编译库：见 [lib/README](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/lib/README.md) / Release `v0.7.1`  
- 本仓库示例构建：`-DXCORE_SDK_ROOT=.../third_party/xCoreSDK-CPP`  

日志默认：`_rokae_log_/<uid>/`；可用 `user_log_settings.json` 改路径。  
实时超时：可执行目录下 `xcoresdk_config.json` 的 `rt._timeout_`（1–4 ms）。

---

## 8. 本地 PDF 目录对照（便于翻页）

`xCore_SDK_C++使用手册V3.2_A.pdf`：

| 章 | 内容 |
|----|------|
| 2 | 兼容性、非实时/实时能力 |
| 3 | 网络、编译、日志、Linux RT 内核（可选） |
| 4 | 接口总表 |
| 5 | Assist 并存、RCI、实时注意、网络排查 |
| 6 | 非实时 / 实时示例代码 |
| 8 附录 | 枚举、结构体、方法签名 |

---

## 9. 建议阅读顺序（本项目）

1. [connection_check.md](connection_check.md) — 先保证 A-LAN1 ping 通  
2. 跑通 [`examples/ar3_right_arm_status`](../examples/ar3_right_arm_status) — `robotInfo` + `jointPos`  
3. [快速开始](https://docs.rokae.com/docs/SDK/quick_start/) + [接口说明](https://docs.rokae.com/docs/SDK/cpp/cpp_API/) — 查后续 API  
4. 需要实时/ServoJ 时再读「实时模式常见问题」与 `example/rt/`  
5. 双臂：第二根线或改 IP 后，再建第二个 `xMateErProRobot`
