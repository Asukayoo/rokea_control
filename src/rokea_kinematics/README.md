# rokea_kinematics

AR3 / Rokea 7-DoF 臂 PoE 正逆解（解析臂角 + 关节限位）。

完整公式见 [ANALYTICAL_IK.md](ANALYTICAL_IK.md)。

## 要点

- 臂角 $\psi$：BSW→SEW 绕 $SW$（与 TJ 一致）
- **单解**：`IkCfg {elbow, shoulder, psi}` → `ik(T, cfg)`
- **枚举**：`analytical_ik(T, psi)` 最多 4 解（默认过滤限位）
- 关节限位取自 URDF `AR3-3_07L/R`（`Q_MIN_DEFAULT` / `Q_MAX_DEFAULT`）
- 躯干圆柱：临界 $\psi^\star$ 设定值 → IK（无临界根视为失败）

## 限位 [rad]

| 关节 | min | max |
|------|-----|-----|
| q1 | -3.1067 | 3.1067 |
| q2 | -2.0944 | 2.0944 |
| q3 | -3.1067 | 3.1067 |
| q4 | -1.0472 | 2.5307 |
| q5 | -3.1067 | 3.1067 |
| q6 | -1.0472 | 1.0472 |
| q7 | -1.5708 | 1.5708 |

## 运行

```bash
python3 analytical_ik.py
```

```python
from analytical_ik import (
    ArmParams, IkCfg, fk_poe, cfg_from_q, ik, analytical_ik,
    compute_arm_angle, within_limits,
)

params = ArmParams()
T = fk_poe(q, params)
cfg = cfg_from_q(q, params)          # 由 q 提构型
q_sol = ik(T, cfg, params)           # 单解（超限位 → None）
sols = analytical_ik(T, cfg.psi, params)  # ≤4，已滤限位
```

## 文件

| 文件 | 说明 |
|------|------|
| `analytical_ik.py` | FK / IK / 臂角 / 圆柱回避 |
| `ANALYTICAL_IK.md` | 公式与流程 |
| `derive_poe_analytical_ik.py` | sympy 推导草稿 |
| `scripts/` | 圆柱回避仿真与出图 |
| `figs/` | 示例 PNG |
