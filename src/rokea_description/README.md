# rokea_description

台架 + 左右 AR3 网格与整机 URDF（自包含，不依赖外部 description 包）。

> 本仓库整体**不是** ROS 2 工程；本目录仍保留可选的 `package.xml` / `CMakeLists.txt`，便于日后在 ROS 工作空间中引用。

## 目录结构

```
rokea_description/
├── package.xml
├── CMakeLists.txt
├── meshes/
│   ├── torso/torso.stl
│   ├── left/{base,link1..link7}.stl
│   └── right/{base,link1..link7}.stl
├── urdf/
│   ├── rokea.urdf.xacro
│   ├── rokea.urdf
│   └── arms/{left,right}_arm.xacro
├── launch/display.launch.py
├── rviz/rokea.rviz
└── scripts/
```

## 坐标系

| 帧 | 定义 |
|----|------|
| `base_link` 原点 | 底板底面中心 |
| Z | 台柱向上 |
| Y | 顶端居中圆孔轴向 |
| X | 朝前 |

臂安装（装入顶端圆孔）：

| 臂 | xyz (m) | rpy | Z |
|----|---------|-----|---|
| 左 | `(0, +0.025, 1.240263)` | `(-π/2, 0, 0)` | +Y |
| 右 | `(0, -0.025, 1.240263)` | `(+π/2, 0, 0)` | −Y |

## 重新导出台架 STL

```bash
python3 src/rokea_description/scripts/step_to_torso_stl.py
```
