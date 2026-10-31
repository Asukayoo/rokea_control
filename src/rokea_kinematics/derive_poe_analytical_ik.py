#!/usr/bin/env python3
"""Sympy 推导 AR3 / Rokea 7-DoF 臂 PoE 解析 IK（臂角 ψ 参数化）。

构型（零位空间系）：
  轴点 p1..p7:
    p1=p2=[0,0,d1], p3=[0,0,d1+d3], p4=[a3,0,d1+d3],
    p5=p6=p7=[0,0,d1+d3+d5]
  轴向 ω: Z Y Z Y Z Y X
  M: R=I, p=[0,0,d1+d3+d5+d_wf]

几何辅助：
  pA — 3 轴上离 4 轴最近点（零位 = p3）
  pB — 5 轴上离 4 轴最近点（零位 = p3）
  pS = p1（肩）, pE = p4（肘轴上参考点）, pW = 腕心

分支：
  q4:  ∠SEW 取 ± 两支
  q2:  肩部 ZYZ 通常 ± 两支
  q6:  由限位只取一支（本脚本仍给出 asin 主值）
"""

from __future__ import annotations

import sympy as sp


# ---------------------------------------------------------------------------
# 符号与基本对象
# ---------------------------------------------------------------------------
d1, d3, a3, d5, d_wf = sp.symbols("d1 d3 a3 d5 d_wf", positive=True)
q1, q2, q3, q4, q5, q6, q7 = sp.symbols("q1:8", real=True)
psi = sp.symbols("psi", real=True)

# 目标位姿（符号占位，数值 IK 时代入）
# T_a = T_target * M^{-1} 的旋转与腕心
Wx, Wy, Wz = sp.symbols("W_x W_y W_z", real=True)
# R_a 的 9 元
r11, r12, r13, r21, r22, r23, r31, r32, r33 = sp.symbols(
    "r11 r12 r13 r21 r22 r23 r31 r32 r33", real=True
)


def skew(w: sp.Matrix) -> sp.Matrix:
    x, y, z = w
    return sp.Matrix([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rot_axis_angle(w: sp.Matrix, th: sp.Expr) -> sp.Matrix:
    """Rodrigues: 单位轴 w 转角 th。"""
    W = skew(w)
    return sp.eye(3) + sp.sin(th) * W + (1 - sp.cos(th)) * (W * W)


def se3_revolute(w: sp.Matrix, q: sp.Matrix, th: sp.Expr) -> sp.Matrix:
    """空间系零位螺旋：ω, 轴上一点 q → Exp([ξ]θ) ∈ SE(3)。"""
    R = rot_axis_angle(w, th)
    # 绕轴上点 q 旋转: p ↦ R(p-q)+q  ⇒  t=(I-R)q
    # （等价于 v=-ω×q 的 Exp 闭式）
    t = (sp.eye(3) - R) * q
    T = sp.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return sp.simplify(T)


def apply_T(T: sp.Matrix, p: sp.Matrix) -> sp.Matrix:
    ph = T * sp.Matrix.vstack(p, sp.Matrix([1]))
    return sp.simplify(ph[:3, 0])


def mat3(a00, a01, a02, a10, a11, a12, a20, a21, a22) -> sp.Matrix:
    return sp.Matrix([[a00, a01, a02], [a10, a11, a12], [a20, a21, a22]])


# 零位点
p1 = sp.Matrix([0, 0, d1])
p2 = sp.Matrix([0, 0, d1])
p3 = sp.Matrix([0, 0, d1 + d3])
p4 = sp.Matrix([a3, 0, d1 + d3])
p5 = sp.Matrix([0, 0, d1 + d3 + d5])
p6 = p5.copy()
p7 = p5.copy()
pA = p3.copy()  # 零位
pB = p3.copy()  # 零位
pS = p1.copy()
pE = p4.copy()
pW_home = p7.copy()

w1 = sp.Matrix([0, 0, 1])
w2 = sp.Matrix([0, 1, 0])
w3 = sp.Matrix([0, 0, 1])
w4 = sp.Matrix([0, 1, 0])
w5 = sp.Matrix([0, 0, 1])
w6 = sp.Matrix([0, 1, 0])
w7 = sp.Matrix([1, 0, 0])

M = sp.eye(4)
M[:3, 3] = sp.Matrix([0, 0, d1 + d3 + d5 + d_wf])


def section(title: str) -> None:
    print("\n" + "=" * 72)
    print(title)
    print("=" * 72)


def show(name: str, expr: sp.Expr | sp.Matrix) -> None:
    print(f"\n{name} =")
    sp.pprint(expr, use_unicode=True)


# ---------------------------------------------------------------------------
# 1) 定长 SE, EW 与 ∠SEW → q4
# ---------------------------------------------------------------------------
section("1) 勾股定长 SE/EW 与余弦定理 ∠SEW → q4（± 两支）")

SE = sp.sqrt(a3**2 + d3**2)
EW = sp.sqrt(a3**2 + d5**2)
SW = sp.symbols("SW", positive=True)

# 三角形 SAE（直角在 A）、EBW（直角在 B）
angle_SEA = sp.atan2(d3, a3)  # ∠ at E in △SAE
angle_WEB = sp.atan2(d5, a3)  # ∠ at E in △EBW
angle_SEW = sp.acos((SE**2 + EW**2 - SW**2) / (2 * SE * EW))

# q4 = σ * ∠SEW - ∠SEA - ∠WEB,  σ=±1
sigma = sp.symbols("sigma", real=True)  # ±1
q4_expr = sigma * angle_SEW - angle_SEA - angle_WEB

show("SE", SE)
show("EW", EW)
show("∠SEA = atan2(d3,a3)", angle_SEA)
show("∠WEB = atan2(d5,a3)", angle_WEB)
show("∠SEW", angle_SEW)
show("q4(σ=±1)", q4_expr)

# 零位校核：SW0 = d3+d5 → ∠SEW = ∠SEA+∠WEB → q4(+)=0
SW0 = d3 + d5
q4_home_plus = sp.simplify(q4_expr.subs({SW: SW0, sigma: 1}))
show("q4|_{SW=d3+d5, σ=+1} (应为 0)", q4_home_plus)


# ---------------------------------------------------------------------------
# 2) p7p = Exp4(q4) p7
# ---------------------------------------------------------------------------
section("2) p7p = Exp([ξ4] q4) p7  （仅肘转后的腕心，空间零位）")

T4 = se3_revolute(w4, p4, q4)
p7p = apply_T(T4, p7)
show("p7p(q4)", p7p)

# 相对肩的向量（用于虚拟 q1v,q2v）
r_p = sp.simplify(p7p - pS)
show("p7p - pS", r_p)


# ---------------------------------------------------------------------------
# 3) 虚拟肩 q3=0：Exp1v Exp2v p7p = pW
# ---------------------------------------------------------------------------
section("3) 虚拟解 q3=0：由 (p7p-pS) 与 (pW-pS) 解析 q2v,q1v")

rx, rz = r_p[0], r_p[2]  # 零位下 y=0
# Exp2: 绕过 S 的 Y；Exp1: 绕 Z。对以 S 为原点的向量：
# R_z(q1) R_y(q2) [rx,0,rz]^T = [Wx,Wy,Wz]^T
#
# R_y(q2)[rx,0,rz] = [rx c + rz s, 0, -rx s + rz c]
# R_z(q1) 后:
#   Wx = (rx c2 + rz s2) c1
#   Wy = (rx c2 + rz s2) s1
#   Wz = -rx s2 + rz c2

Wz_s = Wz  # 符号
# Wz = rz cos q2v - rx sin q2v  ⇒  A cos + B sin = Wz, A=rz, B=-rx
A_v, B_v = rz, -rx
R_v = sp.sqrt(sp.simplify(A_v**2 + B_v**2))
# cos(q2v - phi) = Wz/R  with cos phi=A/R, sin phi=B/R
# q2v = atan2(B,A) ± acos(Wz/R)
phi_v = sp.atan2(B_v, A_v)
q2v_expr = phi_v + sigma * sp.acos(Wz_s / R_v)  # 用同一 sigma 表示 ±；实际肩分支独立

# 更干净的两支写法：
q2v_plus = sp.atan2(B_v, A_v) + sp.acos(Wz_s / R_v)
q2v_minus = sp.atan2(B_v, A_v) - sp.acos(Wz_s / R_v)

rho = rx * sp.cos(q2) + rz * sp.sin(q2)  # 代入 q2v 后
q1v_expr = sp.atan2(Wy, Wx)  # 当 rho>0；若 rho<0 则 atan2(-Wy,-Wx) 且与 q2 支配合

show("|p7p-pS| (=SW)", sp.simplify(sp.sqrt(rx**2 + rz**2)))
show("q2v (+)", sp.simplify(q2v_plus))
show("q2v (−)", sp.simplify(q2v_minus))
show("q1v (rho>0)", q1v_expr)
print(
    "\n注: rho = rx cos(q2v)+rz sin(q2v)；"
    "若 rho<0，则 q1v=atan2(-Wy,-Wx)，并与对应 q2v 支配对。"
)


# ---------------------------------------------------------------------------
# 4) 臂角 ψ：T123 = Exp(SW,ψ) Exp1v Exp2v  → ZYZ 提取 q1,q2,q3
# ---------------------------------------------------------------------------
section("4) 臂角 ψ 与肩部 ZYZ：R = R_sw(ψ) Rz(q1v) Ry(q2v) = Rz(q1) Ry(q2) Rz(q3)")

# 单位 SW 轴（过 pS）
ux, uy, uz = sp.symbols("u_x u_y u_z", real=True)
u = sp.Matrix([ux, uy, uz])
R_sw = rot_axis_angle(u, psi)
R_ref = rot_axis_angle(w1, q1) * rot_axis_angle(w2, q2)  # 占位：实际用 q1v,q2v
# 符号分解：给定 R ∈ SO(3)，ZYZ:
# R = Rz(q1) Ry(q2) Rz(q3)
R = mat3(r11, r12, r13, r21, r22, r23, r31, r32, r33)

# 标准 ZYZ（内禀 q3 绕移动 z / 外禀等价）：
# q2 = atan2(±sqrt(r13^2+r23^2), r33)
# q1 = atan2(r23/s2, r13/s2)
# q3 = atan2(r32/s2, -r31/s2)
s2 = sp.sqrt(r13**2 + r23**2)
q2_from_R_p = sp.atan2(s2, r33)
q2_from_R_m = sp.atan2(-s2, r33)
q1_from_R = sp.atan2(r23, r13)  # 当 s2≠0，对应 q2>0 支要与符号一致
q3_from_R = sp.atan2(r32, -r31)

show("q2 (+支)", q2_from_R_p)
show("q2 (−支)", q2_from_R_m)
show("q1 (s2≠0, 与 +q2 配套时)", sp.atan2(r23 / sp.sin(q2), r13 / sp.sin(q2)))
show("q3 (s2≠0, 与 +q2 配套时)", sp.atan2(r32 / sp.sin(q2), -r31 / sp.sin(q2)))
print(
    """
分解步骤（数值）：
  1. u = (pW-pS)/SW
  2. R_ref = Rot(u,ψ) @ Rz(q1v) @ Ry(q2v)
  3. 对 R_ref 做 ZYZ 欧拉分解得 (q1,q2,q3)；q2 两支
"""
)


# ---------------------------------------------------------------------------
# 5) 腕部 R57 = R04^{-1} R → ZYX → q5,q6,q7
# ---------------------------------------------------------------------------
section("5) 腕部：R57 = R04^{-1} R_a = Rz(q5) Ry(q6) Rx(q7)")

# ZYX 欧拉（固定角或等价）：
# r = R57
# q6 = atan2(-r31, sqrt(r11^2+r21^2))  或 asin；限位 |q6|≤π/3 → 无额外支
# q5 = atan2(r21/c6, r11/c6)
# q7 = atan2(r32/c6, r33/c6)

q6_expr = sp.atan2(-r31, sp.sqrt(r11**2 + r21**2))
q5_expr = sp.atan2(r21, r11)
q7_expr = sp.atan2(r32, r33)

show("q6 (主值, 无翻转支)", q6_expr)
show("q5 | cos q6>0", sp.atan2(r21 / sp.cos(q6), r11 / sp.cos(q6)))
show("q7 | cos q6>0", sp.atan2(r32 / sp.cos(q6), r33 / sp.cos(q6)))


# ---------------------------------------------------------------------------
# 6) 汇总伪代码
# ---------------------------------------------------------------------------
section("6) 解析 IK 流程汇总")
print(
    """
给定 T_target, ψ:
  M_inv = M^{-1}
  T_a   = T_target @ M_inv
  R_a   = T_a[:3,:3]
  pW    = T_a[:3,:3] @ p7 + T_a[:3,3]     # = (T_a @ p7^h)
  pS    = p1
  SW    = ||pW - pS||

  # --- 肘 ---
  SE = hypot(a3,d3);  EW = hypot(a3,d5)
  ang_SEW = acos(clamp((SE^2+EW^2-SW^2)/(2*SE*EW)))
  ang_SEA = atan2(d3,a3);  ang_WEB = atan2(d5,a3)
  for σ4 in (+1,-1):
      q4 = σ4*ang_SEW - ang_SEA - ang_WEB

      # --- 虚拟肩 (q3=0) ---
      p7p = Exp4(q4) @ p7
      r   = p7p - pS;  w = pW - pS
      # 解 q2v:  rz*c - rx*s = wz, 两支 σ2v
      # 再 q1v = atan2(wy, wx)（按 rho 符号修正）

      for σ2v in (+1,-1):
          # --- 臂角 ---
          u = w / SW
          R123 = Rot(u,ψ) @ Rz(q1v) @ Ry(q2v)
          # ZYZ 分解 → (q1,q2,q3)，q2 再 ±（可与 σ2v 合并筛选）

          # --- 腕 ---
          R04 = Rz(q1)@Ry(q2)@Rz(q3)@Ry(q4)   # 与 PoE 肩+肘一致（轴交于 S/几何）
          R57 = R04.T @ R_a
          q6 = atan2(-R57[2,0], hypot(R57[0,0], R57[1,0]))  # 限位内单支
          q5 = atan2(R57[1,0]/c6, R57[0,0]/c6)
          q7 = atan2(R57[2,1]/c6, R57[2,2]/c6)

注：肩 1-2-3 零位轴线交于 pS，故 SO(3) 子问题可用 ZYZ；
    平移部分由腕心约束 + 臂角绕 SW 旋转确定。
"""
)


# ---------------------------------------------------------------------------
# 数值常量（URDF）下打印一次具体表达式
# ---------------------------------------------------------------------------
section("7) 代入 URDF 数值参数后的定长")
subs_urdf = {d1: sp.Rational(17, 100), d3: sp.Rational(281, 1000),
             a3: sp.Rational(15, 1000), d5: sp.Rational(276, 1000),
             d_wf: sp.Rational(93, 1000)}
show("SE (m)", SE.subs(subs_urdf).evalf(6))
show("EW (m)", EW.subs(subs_urdf).evalf(6))
show("∠SEA (rad)", angle_SEA.subs(subs_urdf).evalf(6))
show("∠WEB (rad)", angle_WEB.subs(subs_urdf).evalf(6))
show("p7p(q4)", sp.simplify(p7p))


if __name__ == "__main__":
    pass
