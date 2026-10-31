# ar3_right_arm_status

右臂（**A-LAN1**，默认 `192.168.1.160`）连通性检查：连接后读取 `robotInfo` / `jointPos`，**不运动、不上电**。

机型类：`rokae::xMateErProRobot`（协作 7 轴）。

## 前置

1. 按 [docs/connection_check.md](../../docs/connection_check.md) 完成 ping。
2. 准备 [xCoreSDK-CPP](https://github.com/RokaeRobot/xCoreSDK-CPP)：
   - 克隆仓库
   - 打开与版本匹配的 [Releases](https://github.com/RokaeRobot/xCoreSDK-CPP/releases)（当前常见为 v0.7.1）
   - 下载 `xCoreSDK-*-linux-<arch>.tar.gz`，在 SDK **仓库根目录**解压，使库落入 `lib/Linux/...`
3. 本机有线口：`192.168.1.100/24`

## 编译

```bash
cd examples/ar3_right_arm_status
cmake -S . -B build -DXCORE_SDK_ROOT=/path/to/xCoreSDK-CPP
cmake --build build
```

## 运行

```bash
./build/ar3_right_arm_status
# 或指定 IP
./build/ar3_right_arm_status 192.168.1.160 192.168.1.100
```

成功时会打印控制器版本、机型、轴数与 7 维关节角（rad）。

若报「功能未授权」，需联系珞石开通 xCoreSDK 授权（与网络无关）。

## 双臂 / 空跑 1 kHz

规范栈：`PREEMPT_RT` + `isolcpus=2,4`（grub，改完需重启）+ `scripts/rt_performance.sh`（performance / IRQ 屏蔽 / `cpu_dma_latency`）+ 用户态 `SCHED_FIFO(80)` + 绑核 + `mlockall`。

- 臂：`dual_arm_rt_1khz` — `RtCommand` + `startReceiveRobotState(1ms)`；只接一臂时自动单臂  
- 空跑：`empty_1khz` — 绝对 `nanosleep` + 末 100µs spin  

当前默认 IP：本机 `192.168.2.100`，臂 `192.168.2.160` / `.161`（type 里 `0.7L`/`0.7R`）。

```bash
# 确认隔离（重启后应打印 2,4）
cat /sys/devices/system/cpu/isolated

sudo bash ../../scripts/rt_performance.sh
sudo ./build/empty_1khz 5 2 80
sudo ./build/empty_1khz 5 4 80
sudo ./build/dual_arm_rt_1khz 5 2 4

sudo bash ../../scripts/rt_performance.sh off   # 测完可恢复省电
```
