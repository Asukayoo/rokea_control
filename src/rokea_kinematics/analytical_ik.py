#!/usr/bin/env python3
"""AR3 / Rokea 7-DoF PoE 解析 IK（臂角 ψ）+ FK。

计算约定（与 ANALYTICAL_IK.md §7 一致）：
  - 能复用的中间量只算一次（R_ψ, R123, R04, R57, …）
  - 矩阵块抱起：R57=R04.T@R_a 等，不强制完全展开成 s,c 多项式
  - 数值实现里 sin/cos 在组装旋转前求一次；推导上 s_i,c_i 留到最后回代
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

Array = np.ndarray


# ---------------------------------------------------------------------------
# 参数（预计算定长，避免每次 IK 重复 hypot/atan2）
# ---------------------------------------------------------------------------
# URDF AR3-3_07L/R 关节限位 [rad]（左右臂相同）
Q_MIN_DEFAULT = np.array([-3.1067, -2.0944, -3.1067, -1.0472, -3.1067, -1.0472, -1.5708])
Q_MAX_DEFAULT = np.array([3.1067, 2.0944, 3.1067, 2.5307, 3.1067, 1.0472, 1.5708])


@dataclass(frozen=True)
class ArmParams:
    d1: float = 0.17
    d3: float = 0.281
    a3: float = 0.015
    d5: float = 0.276
    d_wf: float = 0.093

    # 以下在 __post_init__ 填入；frozen 用 object.__setattr__
    SE: float = 0.0
    EW: float = 0.0
    SE2: float = 0.0
    EW2: float = 0.0
    two_SE_EW: float = 0.0
    angle_SEA: float = 0.0
    angle_WEB: float = 0.0
    angle_SEA_WEB: float = 0.0  # ∠SEA+∠WEB，q4 共用
    z_p3: float = 0.0            # d1+d3
    z_p7: float = 0.0            # d1+d3+d5
    z_M: float = 0.0             # d1+d3+d5+d_wf
    pS: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    p7: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    q_min: Tuple[float, ...] = tuple(float(x) for x in Q_MIN_DEFAULT)
    q_max: Tuple[float, ...] = tuple(float(x) for x in Q_MAX_DEFAULT)

    def __post_init__(self) -> None:
        se = float(np.hypot(self.a3, self.d3))
        ew = float(np.hypot(self.a3, self.d5))
        sea = float(np.arctan2(self.d3, self.a3))
        web = float(np.arctan2(self.d5, self.a3))
        zp3 = self.d1 + self.d3
        zp7 = zp3 + self.d5
        zM = zp7 + self.d_wf
        object.__setattr__(self, "SE", se)
        object.__setattr__(self, "EW", ew)
        object.__setattr__(self, "SE2", se * se)
        object.__setattr__(self, "EW2", ew * ew)
        object.__setattr__(self, "two_SE_EW", 2.0 * se * ew)
        object.__setattr__(self, "angle_SEA", sea)
        object.__setattr__(self, "angle_WEB", web)
        object.__setattr__(self, "angle_SEA_WEB", sea + web)
        object.__setattr__(self, "z_p3", zp3)
        object.__setattr__(self, "z_p7", zp7)
        object.__setattr__(self, "z_M", zM)
        object.__setattr__(self, "pS", (0.0, 0.0, self.d1))
        object.__setattr__(self, "p7", (0.0, 0.0, zp7))


@dataclass(frozen=True)
class IkCfg:
    """单解 IK 构型（与 TJ Config 同构）。"""

    elbow: int  # ±1 → σ4
    shoulder: int  # ±1 → ZYZ 的 s2 符号
    psi: float  # 臂角 [rad]
    q1: float = 0.0  # |s2|≈0 时填充
    q3: float = 0.0


def within_limits(
    q: Sequence[float],
    params: ArmParams = ArmParams(),
    tol: float = 1e-9,
) -> bool:
    """关节是否在 [q_min, q_max] 内（含容差）。"""
    q = np.asarray(q, dtype=float)
    lo = np.asarray(params.q_min, dtype=float)
    hi = np.asarray(params.q_max, dtype=float)
    return bool(np.all(q >= lo - tol) and np.all(q <= hi + tol))

    def M(self) -> Array:
        T = np.eye(4)
        T[2, 3] = self.z_M
        return T

    def M_inv(self) -> Array:
        """M = Trans(z_M)，逆为 Trans(-z_M)。"""
        T = np.eye(4)
        T[2, 3] = -self.z_M
        return T


# ---------------------------------------------------------------------------
# 由 (s,c) 组装旋转（不再内部调 sin/cos）
# ---------------------------------------------------------------------------
def Rz_sc(c: float, s: float) -> Array:
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry_sc(c: float, s: float) -> Array:
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def Rx_sc(c: float, s: float) -> Array:
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_axis_angle_sc(u: Array, c: float, s: float) -> Array:
    """Rodrigues，u 已单位化；传入 cosθ,sinθ。"""
    x, y, z = float(u[0]), float(u[1]), float(u[2])
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def wrap(a: float) -> float:
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


def wrap_vec(a: Array) -> Array:
    return np.array([wrap(x) for x in a])


def sc(th: float) -> Tuple[float, float]:
    """唯一回代入口：由角得到 (sin, cos)。"""
    return float(np.sin(th)), float(np.cos(th))


# ---------------------------------------------------------------------------
# FK：先回代全部 s,c，再累乘
# ---------------------------------------------------------------------------
def fk_poe(q: Sequence[float], params: ArmParams = ArmParams()) -> Array:
    q = np.asarray(q, dtype=float)
    # 一次性回代全部 s,c
    scs = [sc(float(q[i])) for i in range(7)]

    d1, a3, zp3, zp7 = params.d1, params.a3, params.z_p3, params.z_p7
    axes_R = (
        Rz_sc, Ry_sc, Rz_sc, Ry_sc, Rz_sc, Ry_sc, Rx_sc,
    )
    pts = (
        (0.0, 0.0, d1),
        (0.0, 0.0, d1),
        (0.0, 0.0, zp3),
        (a3, 0.0, zp3),
        (0.0, 0.0, zp7),
        (0.0, 0.0, zp7),
        (0.0, 0.0, zp7),
    )

    R = np.eye(3)
    t = np.zeros(3)
    for (si, ci), Rmaker, p in zip(scs, axes_R, pts):
        Rm = Rmaker(ci, si)
        p = np.asarray(p)
        t = R @ (p - Rm @ p) + t
        R = R @ Rm

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    T[2, 3] += params.z_M
    return T


# ---------------------------------------------------------------------------
# 子问题（最小运算）
# ---------------------------------------------------------------------------
def _p7p_from_sc4(s4: float, c4: float, params: ArmParams) -> Tuple[float, float, float]:
    """p7p = Exp4(q4) p7，仅用 s4,c4。"""
    a3, d5, zp3 = params.a3, params.d5, params.z_p3
    return (
        a3 * (1.0 - c4) + d5 * s4,
        0.0,
        zp3 + a3 * s4 + d5 * c4,
    )


def _virtual_sc_from_q2(
    q2v: float,
    rx: float,
    rz: float,
    wx: float,
    wy: float,
) -> Tuple[float, float, float, float]:
    """给定 q2v → (s1v,c1v,s2v,c2v)。"""
    s2, c2 = sc(q2v)
    rho = rx * c2 + rz * s2
    if abs(rho) < 1e-12:
        s1, c1 = 0.0, 1.0
    else:
        s1, c1 = sc(float(np.arctan2(wy / rho, wx / rho)))
    return s1, c1, s2, c2


def _canonical_virtual_sc(
    rx: float,
    rz: float,
    wx: float,
    wy: float,
    wz: float,
    e_home: Optional[Array] = None,
    S: Optional[Array] = None,
) -> Optional[Tuple[float, float, float, float]]:
    """规范虚拟肩 → (s1v,c1v,s2v,c2v)。

    两支 q2v=φ±δ 中取使肘落在 BSW 参考半空间者（n_BSW·n_SEW 最大，与 TJ 一致）；
    每个角只 sc 回代一次。
    """
    A, B = rz, -rx  # A c2 + B s2 = wz
    R2 = A * A + B * B
    if R2 < 1e-24:
        return None
    Rmag = float(np.sqrt(R2))
    if abs(wz) > Rmag + 1e-8:
        return None
    phi = np.arctan2(B, A)
    delta = np.arccos(float(np.clip(wz / Rmag, -1.0, 1.0)))
    cands = (phi + delta, phi - delta)

    if e_home is None or S is None:
        q2v = max(cands)
        return _virtual_sc_from_q2(q2v, rx, rz, wx, wy)

    W = S + np.array([wx, wy, wz])
    nb = np.cross(S, W)
    best = None
    best_score = -np.inf
    for q2v in cands:
        s1, c1, s2, c2 = _virtual_sc_from_q2(q2v, rx, rz, wx, wy)
        Rref = Rz_sc(c1, s1) @ Ry_sc(c2, s2)
        E = S + Rref @ e_home
        ns = np.cross(E - S, W - S)
        score = float(np.dot(nb, ns))
        if score > best_score:
            best_score = score
            best = (s1, c1, s2, c2)
    return best


def arm_angle_from_SEW(S: Array, E: Array, W: Array) -> float:
    """臂角 ψ：平面 BSW→SEW 绕 SW 的有向角（B=臂原点 0；与 TJ 一致）。

    n_BSW ∝ S×W，n_SEW ∝ (E−S)×(W−S)，k=SŴ，
    ψ = atan2(k·(n_BSW×n_SEW), n_BSW·n_SEW)。
    """
    nb = np.cross(S, W)
    ns = np.cross(E - S, W - S)
    nb_n = float(np.linalg.norm(nb))
    ns_n = float(np.linalg.norm(ns))
    if nb_n < 1e-12 or ns_n < 1e-12:
        raise ValueError("arm angle undefined (BSW or SEW degenerate)")
    nb = nb / nb_n
    ns = ns / ns_n
    k = W - S
    kn = float(np.linalg.norm(k))
    if kn < 1e-12:
        raise ValueError("SW≈0, arm angle undefined")
    k = k / kn
    return wrap(float(np.arctan2(float(np.dot(k, np.cross(nb, ns))), float(np.dot(nb, ns)))))


def _zyz_from_R_cfg(
    R: Array,
    shoulder: int,
    q1_ref: float = 0.0,
    q3_ref: float = 0.0,
    s2_eps: float = 1e-9,
) -> Tuple[float, float, float]:
    """R=Rz Ry Rz，由 shoulder=±1 唯一确定一支；|s2|≈0 时用 q1_ref,q3_ref。"""
    r11, r21 = R[0, 0], R[1, 0]
    r13, r23, r33 = R[0, 2], R[1, 2], R[2, 2]
    r31, r32 = R[2, 0], R[2, 1]
    c2 = float(r33)
    s2 = float(shoulder) * float(np.sqrt(max(0.0, 1.0 - c2 * c2)))
    if abs(s2) < s2_eps:
        if c2 >= 0.0:
            sm = float(np.arctan2(r21, r11))
            q1 = wrap(0.5 * (sm + q1_ref - q3_ref))
            q3 = wrap(sm - q1)
            return q1, 0.0, q3
        df = float(np.arctan2(-r21, -r11))
        q1 = wrap(0.5 * (df + q1_ref + q3_ref))
        q3 = wrap(q1 - df)
        return q1, wrap(np.pi), q3
    inv = 1.0 / s2
    q1 = wrap(float(np.arctan2(r23 * inv, r13 * inv)))
    q2 = wrap(float(np.arctan2(s2, c2)))
    q3 = wrap(float(np.arctan2(r32 * inv, -r31 * inv)))
    return q1, q2, q3


def _wrist_from_R57(
    R57: Array,
    params: ArmParams = ArmParams(),
) -> Optional[Tuple[float, float, float]]:
    """ZYX 腕；q6 超出 params.q_min/max[5] 则 None。"""
    r11, r21, r31 = R57[0, 0], R57[1, 0], R57[2, 0]
    r32, r33 = R57[2, 1], R57[2, 2]
    c6 = float(np.hypot(r11, r21))
    q6 = float(np.arctan2(-r31, c6))
    if q6 < params.q_min[5] - 1e-9 or q6 > params.q_max[5] + 1e-9:
        return None
    if c6 < 1e-10:
        q5 = 0.0
        q7 = float(np.arctan2(-R57[0, 1], R57[1, 1]))
    else:
        inv = 1.0 / c6
        q5 = float(np.arctan2(r21 * inv, r11 * inv))
        q7 = float(np.arctan2(r32 * inv, r33 * inv))
    return wrap(q5), wrap(q6), wrap(q7)


# ---------------------------------------------------------------------------
# 解析 IK：Config 单解 / 枚举
# ---------------------------------------------------------------------------
def ik(
    T_target: Array,
    cfg: IkCfg,
    params: ArmParams = ArmParams(),
    check_limits: bool = True,
) -> Optional[Array]:
    """给定 T 与 IkCfg，解析求唯一解；超限位或不可达返回 None。"""
    T_a = T_target.copy()
    T_a[2, 3] -= params.z_M
    R_a = T_a[:3, :3]
    p7z = params.z_p7
    pW = R_a @ np.array([0.0, 0.0, p7z]) + T_a[:3, 3]
    S = np.asarray(params.pS, dtype=float)
    w = pW - S
    SW2 = float(np.dot(w, w))
    if SW2 < 1e-24:
        return None
    SW = float(np.sqrt(SW2))
    if SW > params.SE + params.EW + 1e-9 or SW < abs(params.SE - params.EW) - 1e-9:
        return None

    cos_sew = float(np.clip((params.SE2 + params.EW2 - SW2) / params.two_SE_EW, -1.0, 1.0))
    ang_SEW = float(np.arccos(cos_sew))
    sigma4 = 1.0 if cfg.elbow >= 0 else -1.0
    q4 = wrap(sigma4 * ang_SEW - params.angle_SEA_WEB)
    s4, c4 = sc(q4)

    wx, wy, wz = float(w[0]), float(w[1]), float(w[2])
    u = w / SW
    s_psi, c_psi = sc(cfg.psi)
    R_psi = rot_axis_angle_sc(u, c_psi, s_psi)

    px, _, pz = _p7p_from_sc4(s4, c4, params)
    e_home = np.array([params.a3, 0.0, params.d3])
    virt = _canonical_virtual_sc(
        px - S[0], pz - S[2], wx, wy, wz, e_home=e_home, S=S
    )
    if virt is None:
        return None
    s1v, c1v, s2v, c2v = virt
    Rref = Rz_sc(c1v, s1v) @ Ry_sc(c2v, s2v)
    R123 = R_psi @ Rref
    q1, q2, q3 = _zyz_from_R_cfg(R123, cfg.shoulder, cfg.q1, cfg.q3)

    R04 = R123 @ Ry_sc(c4, s4)
    wrist = _wrist_from_R57(R04.T @ R_a, params)
    if wrist is None:
        return None
    q5, q6, q7 = wrist
    qsol = np.array([q1, q2, q3, q4, q5, q6, q7])
    if check_limits and not within_limits(qsol, params):
        return None
    return qsol


def cfg_from_q(q: Sequence[float], params: ArmParams = ArmParams()) -> Optional[IkCfg]:
    """由关节角提取 IkCfg（肘/肩分支 + 臂角）。"""
    q = np.asarray(q, dtype=float)
    try:
        psi = compute_arm_angle(q, params)
    except ValueError:
        return None
    S = np.asarray(params.pS, dtype=float)
    T = fk_poe(q, params)
    T_a = T.copy()
    T_a[2, 3] -= params.z_M
    W = T_a[:3, :3] @ np.array([0.0, 0.0, params.z_p7]) + T_a[:3, 3]
    SW = float(np.linalg.norm(W - S))
    cos_sew = float(np.clip((params.SE2 + params.EW2 - SW * SW) / params.two_SE_EW, -1.0, 1.0))
    ang = float(np.arccos(cos_sew))
    q4p = wrap(+ang - params.angle_SEA_WEB)
    q4m = wrap(-ang - params.angle_SEA_WEB)
    elbow = 1 if abs(wrap(float(q[3]) - q4p)) <= abs(wrap(float(q[3]) - q4m)) else -1
    s2 = float(np.sin(q[1]))
    shoulder = 1 if abs(s2) < 1e-9 else (1 if s2 > 0.0 else -1)
    return IkCfg(elbow=elbow, shoulder=shoulder, psi=psi, q1=float(q[0]), q3=float(q[2]))


def analytical_ik(
    T_target: Array,
    psi: float,
    params: ArmParams = ArmParams(),
    check_limits: bool = True,
    q1_ref: float = 0.0,
    q3_ref: float = 0.0,
) -> List[Array]:
    """枚举 elbow×shoulder 最多 4 解（默认过滤关节限位）。"""
    sols: List[Array] = []
    for elbow in (1, -1):
        for shoulder in (1, -1):
            cfg = IkCfg(
                elbow=elbow,
                shoulder=shoulder,
                psi=psi,
                q1=q1_ref,
                q3=q3_ref,
            )
            q = ik(T_target, cfg, params=params, check_limits=check_limits)
            if q is None:
                continue
            if all(np.linalg.norm(wrap_vec(q - s)) > 1e-6 for s in sols):
                sols.append(q)
    return sols


def nearest_ik(
    T_target: Array,
    psi: float,
    q_seed: Optional[Sequence[float]] = None,
    params: ArmParams = ArmParams(),
    check_limits: bool = True,
) -> Optional[Array]:
    """在限位内解中取与 q_seed 最近者；若有 seed 则优先用其 cfg 单解。"""
    if q_seed is not None:
        cfg = cfg_from_q(q_seed, params)
        if cfg is not None:
            cfg = IkCfg(
                elbow=cfg.elbow,
                shoulder=cfg.shoulder,
                psi=psi,
                q1=float(q_seed[0]),
                q3=float(q_seed[2]),
            )
            q = ik(T_target, cfg, params=params, check_limits=check_limits)
            if q is not None:
                return q
    q1_ref = float(q_seed[0]) if q_seed is not None else 0.0
    q3_ref = float(q_seed[2]) if q_seed is not None else 0.0
    sols = analytical_ik(
        T_target, psi, params, check_limits=check_limits, q1_ref=q1_ref, q3_ref=q3_ref
    )
    if not sols:
        return None
    if q_seed is None:
        return sols[0]
    seed = np.asarray(q_seed, dtype=float)
    return min(sols, key=lambda q: np.linalg.norm(wrap_vec(q - seed)))


def _fk_cumulants(
    q: Sequence[float], params: ArmParams
) -> Tuple[List[Array], List[Array], Array, Array]:
    """累乘 PoE：当前空间轴 (ω_i^s, q_i^s) 与末端旋转/平移 (R,t)（未乘 M）。"""
    q = np.asarray(q, dtype=float)
    scs = [sc(float(q[i])) for i in range(7)]
    d1, a3, zp3, zp7 = params.d1, params.a3, params.z_p3, params.z_p7
    axes_R = (Rz_sc, Ry_sc, Rz_sc, Ry_sc, Rz_sc, Ry_sc, Rx_sc)
    omega0 = (
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
    )
    pts0 = (
        np.array([0.0, 0.0, d1]),
        np.array([0.0, 0.0, d1]),
        np.array([0.0, 0.0, zp3]),
        np.array([a3, 0.0, zp3]),
        np.array([0.0, 0.0, zp7]),
        np.array([0.0, 0.0, zp7]),
        np.array([0.0, 0.0, zp7]),
    )

    R = np.eye(3)
    t = np.zeros(3)
    omegas_s: List[Array] = []
    points_s: List[Array] = []
    for (si, ci), Rmaker, w0, p0 in zip(scs, axes_R, omega0, pts0):
        omegas_s.append(R @ w0)
        points_s.append(R @ p0 + t)
        Rm = Rmaker(ci, si)
        t = R @ (p0 - Rm @ p0) + t
        R = R @ Rm
    return omegas_s, points_s, R, t


def _point_jacobian(omegas_s: Sequence[Array], points_s: Sequence[Array], P: Array) -> Array:
    """点 P 的线速度雅可比 3×7：J_i = ω_i^s × (P - q_i^s)。"""
    J = np.zeros((3, 7))
    for i in range(7):
        J[:, i] = np.cross(omegas_s[i], P - points_s[i])
    return J


def _shoulder_wrist_elbow(
    q: Sequence[float], params: ArmParams
) -> Tuple[Array, Array, Array, Array, Array, Array]:
    """返回 S, E, W, R123, Rref, u。

    E：轴 4 上参考点（零位 p4）当前位姿；W：腕心；Rref：规范虚拟肩。
    """
    q = np.asarray(q, dtype=float)
    sx, sy, sz = params.pS
    S = np.array([sx, sy, sz])

    s1, c1 = sc(float(q[0]))
    s2, c2 = sc(float(q[1]))
    s3, c3 = sc(float(q[2]))
    R123 = Rz_sc(c1, s1) @ Ry_sc(c2, s2) @ Rz_sc(c3, s3)

    # 肩球心转动：E = S + R123 (p4-S)
    e_home = np.array([params.a3, 0.0, params.d3])
    E = S + R123 @ e_home

    # 腕心：用 FK 去 M 后作用 p7
    T = fk_poe(q, params)
    T_a = T.copy()
    T_a[2, 3] -= params.z_M
    W = T_a[:3, :3] @ np.array([0.0, 0.0, params.z_p7]) + T_a[:3, 3]

    w = W - S
    SW = float(np.linalg.norm(w))
    if SW < 1e-12:
        raise ValueError("SW≈0, arm angle undefined")
    u = w / SW

    s4, c4 = sc(float(q[3]))
    px, _, pz = _p7p_from_sc4(s4, c4, params)
    e_home = np.array([params.a3, 0.0, params.d3])
    virt = _canonical_virtual_sc(
        px - sx, pz - sz, float(w[0]), float(w[1]), float(w[2]), e_home=e_home, S=S
    )
    if virt is None:
        raise ValueError("virtual shoulder failed")
    s1v, c1v, s2v, c2v = virt
    Rref = Rz_sc(c1v, s1v) @ Ry_sc(c2v, s2v)
    return S, E, W, R123, Rref, u


def compute_arm_angle(q: Sequence[float], params: ArmParams = ArmParams()) -> float:
    """反求臂角 ψ = BSW→SEW 绕 SW 的有向角（与 analytical_ik / TJ 一致）。"""
    S, E, W, *_ = _shoulder_wrist_elbow(q, params)
    return arm_angle_from_SEW(S, E, W)


def arm_angle_jacobian(q: Sequence[float], params: ArmParams = ArmParams()) -> Array:
    """臂角雅可比 J_ψ (1×7)，ψ̇ = J_ψ q̇。

    对 BSW→SEW 定义（参考面随 S,W 几何固定，瞬时与 Kreutz-Delgado 同构）：
        J_ψ = [ u·ω1^s, u·ω2^s, u·ω3^s, 0, 0, 0, 0 ]
    """
    q = np.asarray(q, dtype=float)
    *_, u = _shoulder_wrist_elbow(q, params)
    omegas_s, _, _, _ = _fk_cumulants(q, params)
    J = np.zeros(7)
    for i in range(3):
        J[i] = float(np.dot(u, omegas_s[i]))
    return J


def augmented_jacobian(
    q: Sequence[float],
    params: ArmParams = ArmParams(),
    ee_point: Optional[Array] = None,
) -> Array:
    """增广雅可比 J_A ∈ R^{7×7}：[ω; v; ψ̇] = J_A q̇（空间系）。

    前 6 行：法兰（或 ee_point）空间速度；末行：J_ψ。
    参考 Kreutz-Delgado et al. 1992 的 augmented Jacobian。
    """
    q = np.asarray(q, dtype=float)
    omegas_s, points_s, R, t = _fk_cumulants(q, params)
    if ee_point is None:
        ee_point = t + R @ np.array([0.0, 0.0, params.z_M])
    J = np.zeros((7, 7))
    for i in range(7):
        J[0:3, i] = omegas_s[i]
        J[3:6, i] = np.cross(omegas_s[i], ee_point - points_s[i])
    J[6, :] = arm_angle_jacobian(q, params)
    return J


# ---------------------------------------------------------------------------
# 躯干圆柱回避：解析 ψ̇（积分 ψ → IK），非数值优化
# ---------------------------------------------------------------------------
def mount_right_world() -> Tuple[Array, Array]:
    """右臂 base → base_link：t=(0,-0.025,1.240263), R=Rx(+π/2)。"""
    s, c = 1.0, 0.0  # sin/cos(π/2)
    R = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    t = np.array([0.0, -0.025, 1.2402628])
    return R, t


def mount_left_world() -> Tuple[Array, Array]:
    """左臂 base → base_link：t=(0,+0.025,1.240263), R=Rx(-π/2)。"""
    s, c = -1.0, 0.0
    R = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    t = np.array([0.0, 0.025, 1.2402628])
    return R, t


def _seg_radial_closest(
    A: Array, B: Array
) -> Tuple[float, Array, float]:
    """水平面内原点到线段的最近点：返回 (ρ, P*, t*)。"""
    Ah = A[:2]
    Bh = B[:2]
    d = Bh - Ah
    den = float(np.dot(d, d))
    if den < 1e-24:
        t = 0.0
        P = A.copy()
    else:
        t = float(np.clip(-np.dot(Ah, d) / den, 0.0, 1.0))
        P = A + t * (B - A)
    rho = float(np.hypot(P[0], P[1]))
    return rho, P, t


def segment_cylinder_clearance(
    A: Array,
    B: Array,
    R_cyl: float = 0.15,
) -> Tuple[float, Array, float, Array]:
    """竖直无限高圆柱间隙 d=ρ(P*)-R（解析）。

    返回 (d, P*, t∈[0,1] 相对 AB, n 水平单位径向)。
    """
    rho, P, t = _seg_radial_closest(A, B)
    if rho < 1e-12:
        n = np.array([1.0, 0.0, 0.0])
    else:
        n = np.array([P[0] / rho, P[1] / rho, 0.0])
    return rho - R_cyl, P, t, n


def psi_dot_cylinder_repulsion(
    q: Sequence[float],
    params: ArmParams = ArmParams(),
    R_mount: Optional[Array] = None,
    t_mount: Optional[Array] = None,
    R_cyl: float = 0.15,
    phi0: float = 0.03,
    k_gain: float = 20.0,
    psi_dot_max: float = 2.0,
) -> Tuple[float, float]:
    """解析臂角斥力速度 ψ̇_rep（世界系竖直无限高圆柱 vs SE/EW）。

    固定末端意义下 dE/dψ = u×(E−S)；φ=min(d_SE,d_EW)；
    ψ̇ = k max(0, φ0−φ) · ∂φ/∂ψ，并限幅。

    返回 (psi_dot, phi)。R_mount/t_mount 默认右臂安装。
    """
    if R_mount is None or t_mount is None:
        R_mount, t_mount = mount_right_world()

    S_a, E_a, W_a, _, _, u_a = _shoulder_wrist_elbow(q, params)
    S = R_mount @ S_a + t_mount
    E = R_mount @ E_a + t_mount
    W = R_mount @ W_a + t_mount
    dE_dpsi = R_mount @ np.cross(u_a, E_a - S_a)

    d_se, P_se, t_se, n_se = segment_cylinder_clearance(S, E, R_cyl)
    d_ew, P_ew, t_ew, n_ew = segment_cylinder_clearance(E, W, R_cyl)

    if d_se <= d_ew:
        phi, n, t_E = d_se, n_se, t_se  # P=(1-t)S+tE → ∂P/∂ψ = t dE/dψ
    else:
        phi, n, t_E = d_ew, n_ew, 1.0 - t_ew  # P=(1-t)E+tW → ∂P/∂ψ = (1-t) dE/dψ

    dphi_dpsi = float(np.dot(n, t_E * dE_dpsi))
    gap = max(0.0, phi0 - phi)
    psi_dot = k_gain * gap * dphi_dpsi
    if abs(psi_dot) > psi_dot_max:
        psi_dot = psi_dot_max if psi_dot > 0.0 else -psi_dot_max
    return psi_dot, phi


def integrate_psi_and_ik(
    T_target: Array,
    q_ref: Sequence[float],
    dt: float,
    params: ArmParams = ArmParams(),
    side: str = "right",
    **repulsion_kw,
) -> Tuple[Optional[Array], float, float]:
    """ψ ← ψ+ψ̇ dt，再 nearest_ik(T, ψ, q_ref)。返回 (q_sol, psi_new, phi)。"""
    R_m, t_m = mount_right_world() if side == "right" else mount_left_world()
    psi = compute_arm_angle(q_ref, params)
    psi_dot, phi = psi_dot_cylinder_repulsion(
        q_ref, params, R_mount=R_m, t_mount=t_m, **repulsion_kw
    )
    psi_new = wrap(psi + psi_dot * dt)
    q_sol = nearest_ik(T_target, psi_new, q_seed=q_ref, params=params)
    return q_sol, psi_new, phi


# ---------------------------------------------------------------------------
# 临界臂角：固定 T 时 φ(ψ)=ρ_min(SE,EW)-R_c 的水平解析条件
# ---------------------------------------------------------------------------
def elbow_sigma4(q: Sequence[float], params: ArmParams = ArmParams()) -> float:
    """由当前 q4 判定肘支 σ4=±1（与 analytical_ik 分支一致）。"""
    q = np.asarray(q, dtype=float)
    S = np.asarray(params.pS, dtype=float)
    T = fk_poe(q, params)
    T_a = T.copy()
    T_a[2, 3] -= params.z_M
    W = T_a[:3, :3] @ np.array([0.0, 0.0, params.z_p7]) + T_a[:3, 3]
    SW2 = float(np.dot(W - S, W - S))
    cos_sew = float(np.clip((params.SE2 + params.EW2 - SW2) / params.two_SE_EW, -1.0, 1.0))
    ang = float(np.arccos(cos_sew))
    q4p = wrap(+ang - params.angle_SEA_WEB)
    q4m = wrap(-ang - params.angle_SEA_WEB)
    return 1.0 if abs(wrap(float(q[3]) - q4p)) <= abs(wrap(float(q[3]) - q4m)) else -1.0


def _sw_elbow0_from_T(
    T_target: Array,
    params: ArmParams,
    sigma4: float = 1.0,
) -> Optional[Tuple[Array, Array, Array, Array]]:
    """固定 T、肘支 σ4 → (S, W, u, e0)，使 E(ψ)=S+Rot(u,ψ)e0。"""
    T_a = T_target.copy()
    T_a[2, 3] -= params.z_M
    R_a = T_a[:3, :3]
    p7z = params.z_p7
    W = R_a @ np.array([0.0, 0.0, p7z]) + T_a[:3, 3]
    S = np.asarray(params.pS, dtype=float)
    w = W - S
    SW2 = float(np.dot(w, w))
    if SW2 < 1e-24:
        return None
    SW = float(np.sqrt(SW2))
    if SW > params.SE + params.EW + 1e-9 or SW < abs(params.SE - params.EW) - 1e-9:
        return None
    cos_sew = float(np.clip((params.SE2 + params.EW2 - SW2) / params.two_SE_EW, -1.0, 1.0))
    q4 = wrap(sigma4 * float(np.arccos(cos_sew)) - params.angle_SEA_WEB)
    s4, c4 = sc(q4)
    px, _, pz = _p7p_from_sc4(s4, c4, params)
    e_home = np.array([params.a3, 0.0, params.d3])
    virt = _canonical_virtual_sc(
        px - S[0],
        pz - S[2],
        float(w[0]),
        float(w[1]),
        float(w[2]),
        e_home=e_home,
        S=S,
    )
    if virt is None:
        return None
    s1v, c1v, s2v, c2v = virt
    Rref = Rz_sc(c1v, s1v) @ Ry_sc(c2v, s2v)
    e0 = Rref @ e_home
    u = w / SW
    return S, W, u, e0


def _weierstrass_roots(
    a0: float,
    ac: float,
    as_: float,
    acc: float,
    ass: float,
    acs: float,
) -> List[float]:
    """解 a0 + ac c + as s + acc c² + ass s² + acs c s = 0，c²+s²=1。

    t=tan(ψ/2) → 至多 4 实根；ψ=±π（t→∞）单独检查。
    """
    # * (1+t²)²:
    # a0(1+2t²+t⁴) + ac(1-t²)(1+t²) + as(2t)(1+t²)
    # + acc(1-t²)² + ass(4t²) + acs·2t(1-t²) = 0
    # (1-t²)(1+t²)=1-t⁴
    # (1-t²)²=1-2t²+t⁴
    # 2t(1+t²)=2t+2t³
    # 2t(1-t²)=2t-2t³
    # a0(1+2t²+t⁴) + ac(1-t⁴) + as(2t+2t³)
    # + acc(1-2t²+t⁴) + ass(4t²) + acs(2t-2t³) = 0
    p4 = a0 - ac + acc
    p3 = 2.0 * as_ - 2.0 * acs
    p2 = 2.0 * a0 - 2.0 * acc + 4.0 * ass
    p1 = 2.0 * as_ + 2.0 * acs
    p0 = a0 + ac + acc
    coeffs = [p4, p3, p2, p1, p0]
    if all(abs(c) < 1e-15 for c in coeffs):
        return []  # 恒成立，交给外层稠密采样处理
    # 去掉前导零
    while len(coeffs) > 1 and abs(coeffs[0]) < 1e-15:
        coeffs.pop(0)
    roots = np.roots(coeffs)
    psis: List[float] = []
    for r in roots:
        if abs(r.imag) > 1e-8:
            continue
        t = float(r.real)
        psis.append(wrap(2.0 * np.arctan(t)))
    # ψ=±π：t→∞，看最高次；用 c=-1,s=0 直接验
    c_pi, s_pi = -1.0, 0.0
    val_pi = (
        a0
        + ac * c_pi
        + as_ * s_pi
        + acc * c_pi * c_pi
        + ass * s_pi * s_pi
        + acs * c_pi * s_pi
    )
    if abs(val_pi) < 1e-9:
        psis.append(np.pi)  # wrap → -π 亦可；后面去重
    # 去重
    out: List[float] = []
    for p in psis:
        pw = wrap(p)
        if all(abs(wrap(pw - q)) > 1e-7 for q in out):
            out.append(pw)
    return out


def _line_circle_trig_coeffs(
    Ah: Array,
    P0: Array,
    V1: Array,
    V2: Array,
    R: float,
) -> Tuple[float, float, float, float, float, float]:
    """线段端 A 固定、B=P0+c V1+s V2 时，|A×B|² - R²||B-A||² = 0 的三角系数。

    2D 叉积标量 Ax By - Ay Bx。
    """
    ax, ay = float(Ah[0]), float(Ah[1])
    # Ax Ey - Ay Ex = k0 + kc c + ks s
    k0 = ax * P0[1] - ay * P0[0]
    kc = ax * V1[1] - ay * V1[0]
    ks = ax * V2[1] - ay * V2[0]
    # B-A = (P0-A) + c V1 + s V2
    D0 = P0 - Ah
    # ||B-A||² = ||D0||² + c²||V1||² + s²||V2||² + 2c D0·V1 + 2s D0·V2 + 2cs V1·V2
    d00 = float(np.dot(D0, D0))
    v11 = float(np.dot(V1, V1))
    v22 = float(np.dot(V2, V2))
    d0v1 = float(np.dot(D0, V1))
    d0v2 = float(np.dot(D0, V2))
    v12 = float(np.dot(V1, V2))
    R2 = R * R
    # (k0 + kc c + ks s)² - R² || ||² = 0
    # k0² + kc² c² + ks² s² + 2 k0 kc c + 2 k0 ks s + 2 kc ks c s
    #  - R² (d00 + v11 c² + v22 s² + 2 d0v1 c + 2 d0v2 s + 2 v12 c s)
    a0 = k0 * k0 - R2 * d00
    ac = 2.0 * k0 * kc - R2 * 2.0 * d0v1
    as_ = 2.0 * k0 * ks - R2 * 2.0 * d0v2
    acc = kc * kc - R2 * v11
    ass = ks * ks - R2 * v22
    acs = 2.0 * kc * ks - R2 * 2.0 * v12
    return a0, ac, as_, acc, ass, acs


def _point_circle_trig_coeffs(
    P0: Array, V1: Array, V2: Array, R: float
) -> Tuple[float, float, float, float, float, float]:
    """||P0+c V1+s V2||² - R² = 0。"""
    R2 = R * R
    p00 = float(np.dot(P0, P0))
    v11 = float(np.dot(V1, V1))
    v22 = float(np.dot(V2, V2))
    p0v1 = float(np.dot(P0, V1))
    p0v2 = float(np.dot(P0, V2))
    v12 = float(np.dot(V1, V2))
    a0 = p00 - R2
    ac = 2.0 * p0v1
    as_ = 2.0 * p0v2
    acc = v11
    ass = v22
    acs = 2.0 * v12
    return a0, ac, as_, acc, ass, acs


def clearance_phi_at_psi(
    S_w: Array,
    W_w: Array,
    u_w: Array,
    e0_w: Array,
    psi: float,
    R_cyl: float = 0.15,
) -> Tuple[float, float, float, Array]:
    """世界系：给定 ψ 返回 (φ, d_SE, d_EW, E_w)。"""
    s, c = sc(psi)
    E_w = S_w + rot_axis_angle_sc(u_w, c, s) @ e0_w
    d_se, _, _, _ = segment_cylinder_clearance(S_w, E_w, R_cyl)
    d_ew, _, _, _ = segment_cylinder_clearance(E_w, W_w, R_cyl)
    return min(d_se, d_ew), d_se, d_ew, E_w


def critical_arm_angles_cylinder(
    T_target: Array,
    params: ArmParams = ArmParams(),
    R_mount: Optional[Array] = None,
    t_mount: Optional[Array] = None,
    R_cyl: float = 0.15,
    phi_star: float = 0.0,
    sigma4: float = 1.0,
    tol: float = 1e-5,
    q_seed: Optional[Sequence[float]] = None,
) -> List[dict]:
    """解析求临界臂角：φ(ψ)=φ* 的 ψ（固定末端 T）。

    几何：E(ψ)=S+Rot(u,ψ)e0 水平投影为椭圆；临界候选为
      (1) 直线 SE / EW 与水平圆 ρ=R_c+φ* 相切（内点最近点）
      (2) 肘点 E 落在该圆上
    各化为 c=cosψ,s=sinψ 的二次三角式，Weierstrass → ≤4 次代数方程。

    若给 q_seed，则用其 q4 自动选 σ4（覆盖传入的 sigma4）。

    返回列表元素：{psi, phi, d_se, d_ew, kind, sigma4}，
    kind∈{SE_tangent,EW_tangent,E_on_circle}。
    """
    if R_mount is None or t_mount is None:
        R_mount, t_mount = mount_right_world()
    if q_seed is not None:
        sigma4 = elbow_sigma4(q_seed, params)
    geo = _sw_elbow0_from_T(T_target, params, sigma4=sigma4)
    if geo is None:
        return []
    S_a, W_a, u_a, e0_a = geo
    S_w = R_mount @ S_a + t_mount
    W_w = R_mount @ W_a + t_mount
    u_w = R_mount @ u_a
    u_w = u_w / np.linalg.norm(u_w)
    e0_w = R_mount @ e0_a

    R_star = R_cyl + phi_star
    if R_star < 0.0:
        return []

    # E_w = S_w + c e0_w + (1-c)(u·e0)u + s (u×e0)  — Rodrigues
    # 水平：E_h = P0 + c V1 + s V2
    ue = float(np.dot(u_w, e0_w))
    ux_e = np.cross(u_w, e0_w)
    # Rot(u,ψ) e0 = c e0 + (1-c)(u·e0)u + s (u×e0)
    # = [ue u] + c (e0 - ue u) + s (u×e0)
    P0 = S_w + ue * u_w  # constant part when using form below with V1=(e0-ue u)
    V1 = e0_w - ue * u_w
    V2 = ux_e
    P0h, V1h, V2h = P0[:2], V1[:2], V2[:2]
    Sh, Wh = S_w[:2], W_w[:2]

    candidates: List[Tuple[float, str]] = []
    for coeffs, kind in (
        (_line_circle_trig_coeffs(Sh, P0h, V1h, V2h, R_star), "SE_tangent"),
        (_line_circle_trig_coeffs(Wh, P0h, V1h, V2h, R_star), "EW_tangent"),
        (_point_circle_trig_coeffs(P0h, V1h, V2h, R_star), "E_on_circle"),
    ):
        for psi in _weierstrass_roots(*coeffs):
            candidates.append((psi, kind))

    results: List[dict] = []
    for psi, kind in candidates:
        phi, d_se, d_ew, _ = clearance_phi_at_psi(S_w, W_w, u_w, e0_w, psi, R_cyl)
        if abs(phi - phi_star) > tol:
            # 相切方程可能对应延长线相切；要求段上真实间隙
            continue
        # 对 tangent：再确认活跃段匹配
        if kind == "SE_tangent" and abs(d_se - phi_star) > tol:
            continue
        if kind == "EW_tangent" and abs(d_ew - phi_star) > tol:
            continue
        if kind == "E_on_circle":
            s, c = sc(psi)
            Eh = (S_w + rot_axis_angle_sc(u_w, c, s) @ e0_w)[:2]
            if abs(float(np.linalg.norm(Eh)) - R_star) > tol:
                continue
        entry = {
            "psi": wrap(psi),
            "phi": phi,
            "d_se": d_se,
            "d_ew": d_ew,
            "kind": kind,
            "sigma4": sigma4,
        }
        if all(
            abs(wrap(entry["psi"] - r["psi"])) > 1e-6 or entry["kind"] != r["kind"]
            for r in results
        ):
            results.append(entry)
    results.sort(key=lambda r: r["psi"])
    return results


def critical_psi_interval(
    T_target: Array,
    params: ArmParams = ArmParams(),
    R_mount: Optional[Array] = None,
    t_mount: Optional[Array] = None,
    R_cyl: float = 0.15,
    phi_star: float = 0.0,
    sigma4: float = 1.0,
    n_sample: int = 720,
    q_seed: Optional[Sequence[float]] = None,
) -> Tuple[List[dict], List[Tuple[float, float]]]:
    """临界 ψ 列表 + 满足 φ≥φ* 的臂角区间（稠密采样确认连通性）。

    区间用于选取「外侧」可行臂角；临界点由 critical_arm_angles_cylinder 解析给出。
    """
    if q_seed is not None:
        sigma4 = elbow_sigma4(q_seed, params)
    crit = critical_arm_angles_cylinder(
        T_target,
        params=params,
        R_mount=R_mount,
        t_mount=t_mount,
        R_cyl=R_cyl,
        phi_star=phi_star,
        sigma4=sigma4,
    )
    if R_mount is None or t_mount is None:
        R_mount, t_mount = mount_right_world()
    geo = _sw_elbow0_from_T(T_target, params, sigma4=sigma4)
    if geo is None:
        return crit, []
    S_a, W_a, u_a, e0_a = geo
    S_w = R_mount @ S_a + t_mount
    W_w = R_mount @ W_a + t_mount
    u_w = R_mount @ u_a
    u_w = u_w / np.linalg.norm(u_w)
    e0_w = R_mount @ e0_a

    psis = np.linspace(-np.pi, np.pi, n_sample, endpoint=False)
    ok = np.array(
        [
            clearance_phi_at_psi(S_w, W_w, u_w, e0_w, float(p), R_cyl)[0] >= phi_star - 1e-9
            for p in psis
        ]
    )
    intervals: List[Tuple[float, float]] = []
    if not np.any(ok):
        return crit, intervals
    idx = np.where(ok)[0]
    # 分段连续下标 → [psi_start, psi_end]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate([[idx[0]], idx[breaks + 1]])
    ends = np.concatenate([idx[breaks], [idx[-1]]])
    for a, b in zip(starts, ends):
        intervals.append((float(psis[a]), float(psis[b])))
    # 若首尾都可行且覆盖 ±π，合并为跨界区间 (end_last, end_first) 用 (ψ_lo, ψ_hi) 且 lo>hi 表示跨界
    if (
        len(intervals) >= 2
        and ok[0]
        and ok[-1]
        and abs(intervals[0][0] + np.pi) < 2 * np.pi / n_sample + 1e-6
        and abs(intervals[-1][1] - (np.pi - 2 * np.pi / n_sample)) < 2 * np.pi / n_sample + 1e-6
    ):
        lo = intervals[-1][0]
        hi = intervals[0][1]
        intervals = [(lo, hi)] + intervals[1:-1]
    return crit, intervals


def psi_setpoint_cylinder(
    T_target: Array,
    psi_ref: float,
    params: ArmParams = ArmParams(),
    R_mount: Optional[Array] = None,
    t_mount: Optional[Array] = None,
    R_cyl: float = 0.15,
    phi0: float = 0.03,
    sigma4: float = 1.0,
    q_seed: Optional[Sequence[float]] = None,
) -> Tuple[Optional[float], float, str]:
    """将临界臂角直接作为设定值（替代 ψ̇ 斥力积分）。

    规则：
      - 当前 φ(ψ_ref) ≥ φ0 → 保持 ψ_ref（mode=hold）
      - φ < φ0 且存在 φ=φ0 的解析临界根 → 取与 ψ_ref 最近的临界 ψ（mode=critical）
      - 否则无法达到 φ0 → 视为失败（psi_set=None, mode=fail）

    返回 (psi_set|None, phi_at_ref, mode)。
    """
    if R_mount is None or t_mount is None:
        R_mount, t_mount = mount_right_world()
    if q_seed is not None:
        sigma4 = elbow_sigma4(q_seed, params)
    geo = _sw_elbow0_from_T(T_target, params, sigma4=sigma4)
    if geo is None:
        return None, float("nan"), "fail"
    S_a, W_a, u_a, e0_a = geo
    S_w = R_mount @ S_a + t_mount
    W_w = R_mount @ W_a + t_mount
    u_w = R_mount @ u_a
    u_w = u_w / np.linalg.norm(u_w)
    e0_w = R_mount @ e0_a

    phi_ref, _, _, _ = clearance_phi_at_psi(S_w, W_w, u_w, e0_w, psi_ref, R_cyl)
    if phi_ref >= phi0 - 1e-12:
        return wrap(psi_ref), phi_ref, "hold"

    crit = critical_arm_angles_cylinder(
        T_target,
        params=params,
        R_mount=R_mount,
        t_mount=t_mount,
        R_cyl=R_cyl,
        phi_star=phi0,
        sigma4=sigma4,
    )
    if crit:
        best = min(crit, key=lambda r: abs(wrap(r["psi"] - psi_ref)))
        return wrap(float(best["psi"])), phi_ref, "critical"

    return None, phi_ref, "fail"


def apply_psi_setpoint_ik(
    T_target: Array,
    q_ref: Sequence[float],
    params: ArmParams = ArmParams(),
    side: str = "right",
    R_cyl: float = 0.15,
    phi0: float = 0.03,
) -> Tuple[Optional[Array], Optional[float], float, str]:
    """临界设定 ψ → nearest_ik。无可行临界时视为 IK 失败。

    返回 (q_sol|None, psi_set|None, phi_ref, mode)。
    """
    R_m, t_m = mount_right_world() if side == "right" else mount_left_world()
    psi_ref = compute_arm_angle(q_ref, params)
    psi_set, phi_ref, mode = psi_setpoint_cylinder(
        T_target,
        psi_ref,
        params=params,
        R_mount=R_m,
        t_mount=t_m,
        R_cyl=R_cyl,
        phi0=phi0,
        q_seed=q_ref,
    )
    if psi_set is None or mode == "fail":
        return None, None, phi_ref, "fail"
    q_sol = nearest_ik(T_target, psi_set, q_seed=q_ref, params=params)
    if q_sol is None:
        return None, psi_set, phi_ref, "fail"
    return q_sol, psi_set, phi_ref, mode


# ---------------------------------------------------------------------------
# 自检
# ---------------------------------------------------------------------------
def _pose_err(Ta: Array, Tb: Array) -> Tuple[float, float]:
    dp = np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])
    dR = Ta[:3, :3].T @ Tb[:3, :3]
    ang = np.arccos(float(np.clip((np.trace(dR) - 1.0) * 0.5, -1.0, 1.0)))
    return dp, ang


def self_test(n: int = 20, seed: int = 0) -> None:
    rng = np.random.default_rng(seed)
    params = ArmParams()
    lo = np.asarray(params.q_min)
    hi = np.asarray(params.q_max)
    ok_pose = ok_exact = ok_cfg = 0
    for i in range(n):
        q_gt = rng.uniform(lo + 0.05, hi - 0.05)
        T = fk_poe(q_gt, params)
        cfg = cfg_from_q(q_gt, params)
        if cfg is None:
            print(f"[{i}] FAIL cfg_from_q")
            continue
        q_ik = ik(T, cfg, params)
        if q_ik is None:
            print(f"[{i}] FAIL ik None")
            continue
        ok_cfg += 1
        dp, dang = _pose_err(T, fk_poe(q_ik, params))
        if dp < 1e-5 and dang < 1e-5:
            ok_pose += 1
        if np.linalg.norm(wrap_vec(q_ik - q_gt)) < 1e-5:
            ok_exact += 1
        if dp >= 1e-5 or dang >= 1e-5:
            print(f"[{i}] FAIL pose err dp={dp} dang={dang}")
    print(f"self_test: cfg+ik {ok_cfg}/{n}, pose {ok_pose}/{n}, exact-q {ok_exact}/{n}")


def _psi_frozen_r(q: Array, r0: Array, params: ArmParams) -> float:
    """与 J_ψ 一致：参考 r 冻结在标称构型。"""
    S, E, W, _, _, u = _shoulder_wrist_elbow(q, params)
    e = E - S
    p = e - u * np.dot(u, e)
    return wrap(float(np.arctan2(float(np.dot(u, np.cross(r0, p))), float(np.dot(r0, p)))))


def _fd_arm_angle_jacobian(q: Array, params: ArmParams, eps: float = 1e-7) -> Array:
    """中心差分；r 取标称构型并冻结（与解析 J_ψ 定义一致）。"""
    _, _, _, _, Rref, u = _shoulder_wrist_elbow(q, params)
    r0 = Rref @ np.array([params.a3, 0.0, params.d3])
    r0 = r0 - u * np.dot(u, r0)
    J = np.zeros(7)
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = eps
        J[i] = wrap(_psi_frozen_r(q + dq, r0, params) - _psi_frozen_r(q - dq, r0, params)) / (
            2.0 * eps
        )
    return J


if __name__ == "__main__":
    p = ArmParams()
    print(f"SE={p.SE:.6f} EW={p.EW:.6f} SEA+WEB={p.angle_SEA_WEB:.6f}")

    print(f"q_min={np.round(p.q_min, 4)}")
    print(f"q_max={np.round(p.q_max, 4)}")

    q_gt = np.array([0.3, -0.4, 0.5, 0.6, -0.2, 0.3, 0.1])
    T = fk_poe(q_gt, p)
    cfg = cfg_from_q(q_gt, p)
    assert cfg is not None
    q_ik = ik(T, cfg, p)
    print(f"ψ={cfg.psi:.6f} elbow={cfg.elbow} shoulder={cfg.shoulder}")
    print(f"ik dq={np.linalg.norm(wrap_vec(q_ik - q_gt)):.3e} err={_pose_err(T, fk_poe(q_ik, p))}")
    print(f"within_limits={within_limits(q_ik, p)} nsols(ψ)={len(analytical_ik(T, cfg.psi, p))}")

    Ja = arm_angle_jacobian(q_gt, p)
    Jf = _fd_arm_angle_jacobian(q_gt, p)
    print(f"\nJ_ψ anal: {np.round(Ja, 5)}")
    print(f"J_ψ FD  : {np.round(Jf, 5)}")
    print(f"J_ψ |anal-FD|_∞ = {np.max(np.abs(Ja - Jf)):.3e}")

    self_test(20, seed=1)
