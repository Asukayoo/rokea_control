# Rokea / AR3 7-DoF PoE 解析逆解（臂角 $\psi$）

本文整理该臂的 Product of Exponentials（PoE）模型与**臂角参数化解析 IK**。Python 参考实现见 `analytical_ik.py`，符号推导见 `derive_poe_analytical_ik.py`。臂角雅可比见 §8，C++ 逆解接口见 §9。

---

## 1. 构型

零位、空间系下：

| 量 | 定义 |
|---|---|
| 轴点 | $p_1=p_2=[0,0,d_1]$，$p_3=[0,0,d_1+d_3]$，$p_4=[a_3,0,d_1+d_3]$，$p_5=p_6=p_7=[0,0,d_1+d_3+d_5]$ |
| 轴向 $\omega$ | $Z,\,Y,\,Z,\,Y,\,Z,\,Y,\,X$ |
| 零位 $M$ | $R=I$，$p_M=[0,0,d_1+d_3+d_5+d_{wf}]$ |
| URDF 默认 | $d_1=0.17$，$d_3=0.281$，$a_3=0.015$，$d_5=0.276$，$d_{wf}=0.093$ |

辅助几何：

- $p_S=p_1$：肩心（轴 $1/2/3$ 交点）
- $p_A$：3 轴上离 4 轴最近点；零位与 $p_3$ 重合
- $p_B$：5 轴上离 4 轴最近点；零位与 $p_3$ 重合
- $p_E=p_4$：肘轴上参考点
- $p_W$：腕心（轴 $5/6/7$ 交点）

正运动学：

$$
T(q)=e^{[\xi_1]q_1}\cdots e^{[\xi_7]q_7}\,M
$$

其中 $e^{[\xi_i]\theta}$ 为绕零位轴 $(\omega_i,p_i)$ 的刚体转动：$p\mapsto R(p-p_i)+p_i$。

---

## 2. 给定目标与臂角

给定法兰目标 $T_{\mathrm{target}}$ 与臂角 $\psi$：

$$
T_a=T_{\mathrm{target}}\,M^{-1},\quad
R_a=T_a[1:3,1:3],\quad
p_W=T_a\,p_7,\quad
p_S=p_1,\quad
SW=\|p_W-p_S\|
$$

（关节 $5$–$7$ 不改变腕心，故 $p_W=e^{[\xi_1]q_1}\cdots e^{[\xi_4]q_4}p_7$。）

若存在用户坐标系 / 工具坐标系，则实际送入 IK 的目标为

$$
T_{\mathrm{eff}}
= {}^{\mathrm{base}}T_{\mathrm{user}}\,
  T_{\mathrm{target}}\,
  {}^{\mathrm{flange}}T_{\mathrm{tool}}^{-1}
$$

（见 §8 中 `user`、`tool` 指针；均为单位阵时可省略。）

---

## 3. 定长与 $q_4$（2 支）

三角形 $SAE$、$EBW$ 在 $A$/$B$ 处为直角（勾股）：

$$
SE=\sqrt{a_3^2+d_3^2},\qquad
EW=\sqrt{a_3^2+d_5^2}
$$

$$
\angle SEA=\operatorname{atan2}(d_3,a_3),\qquad
\angle WEB=\operatorname{atan2}(d_5,a_3)
$$

由余弦定理：

$$
\angle SEW=\arccos\!\left(\frac{SE^2+EW^2-SW^2}{2\,SE\,EW}\right)
\in[0,\pi]
$$

肘角两支（$\sigma_4=\pm 1$，对应配置项 `cfg.elbow`）：

$$
\boxed{q_4=\sigma_4\,\angle SEW-\angle SEA-\angle WEB}
$$

零位校核：$SW=d_3+d_5$ 时 $\sigma_4=+1$ 给出 $q_4=0$。

可达条件：$|SE-EW|\le SW\le SE+EW$。

对每个 $q_4$：

$$
p_{7p}=e^{[\xi_4]q_4}p_7
=
\begin{bmatrix}
a_3(1-\cos q_4)+d_5\sin q_4\\
0\\
d_1+d_3+a_3\sin q_4+d_5\cos q_4
\end{bmatrix}
$$

---

## 4. 臂角与虚拟肩

### 4.1 臂角定义（与 TJ 一致）

臂坐标系原点为 $B=0$，$S=(0,0,d_1)$ 在基座 $z$ 上。令

$$
n_{\mathrm{BSW}}=\frac{S\times W}{\|\cdot\|},\quad
n_{\mathrm{SEW}}=\frac{(E-S)\times(W-S)}{\|\cdot\|},\quad
k=\widehat{SW}.
$$

$$
\boxed{
\psi
=
\operatorname{atan2}\bigl(
  k\cdot(n_{\mathrm{BSW}}\times n_{\mathrm{SEW}}),\;
  n_{\mathrm{BSW}}\cdot n_{\mathrm{SEW}}
\bigr)
}
$$

即平面 **BSW → SEW** 绕 $SW$ 的有向角。$\psi=0$ 时肘在 BSW 上（BSW $=$ $SW$ 与轴 1/$z$ 所张平面）。  
实现：`arm_angle_from_SEW` / `compute_arm_angle`。

### 4.2 IK 参数化（虚拟肩）

约束 $e^{[\xi_1]q_1}e^{[\xi_2]q_2}e^{[\xi_3]q_3}p_{7p}=p_W$。令 $q_3=0$ 求虚拟解 $(q_{1v},q_{2v})$：

$$
R_z(q_{1v})R_y(q_{2v})\,(p_{7p}-p_S)=p_W-p_S.
$$

几何上有 2 支；**规范**：取使虚拟肘落在 BSW 参考半空间（$n_{\mathrm{BSW}}\cdot n_{\mathrm{SEW}}$ 最大）的一支，记 $R_{\mathrm{ref}}=R_z(q_{1v}^\star)R_y(q_{2v}^\star)$。对本构型该零位与 §4.1 的 $\psi=0$ 一致。

$$
\boxed{
R_{123}
=\mathrm{Rot}(\widehat{SW},\psi)\,R_{\mathrm{ref}}
=R_z(q_1)R_y(q_2)R_z(q_3)
}
$$

对 $R_{123}$ 做 ZYZ 分解得 $(q_1,q_2,q_3)$，其中 **$q_2$ 有 $\pm$ 两支**（`cfg.shoulder`）。若两支虚拟参考都保留再乘同一 $\psi$，会虚增到 8 解（另一零位差 $\pi$）。

---

## 5. 腕部 $q_5,q_6,q_7$（$q_6$ 单支）

同一 $R_{123}$ 的两条 ZYZ 解表示**同一姿态**，故

$$
R_{04}=R_{123}\,R_y(q_4)
$$

对两条肩部分支**只算一次**。随后

$$
R_{57}=R_{04}^\top R_a
=R_z(q_5)R_y(q_6)R_x(q_7)
$$

$$
q_6=\operatorname{atan2}\!\big(-r_{31},\,\sqrt{r_{11}^2+r_{21}^2}\big)
\quad\text{（主值；}|q_6|\text{ 限位内不再取翻转支）}
$$

$$
q_5=\operatorname{atan2}(r_{21}/c_6,\,r_{11}/c_6),\qquad
q_7=\operatorname{atan2}(r_{32}/c_6,\,r_{33}/c_6)
$$

其中 $c_6=\sqrt{r_{11}^2+r_{21}^2}$。

---

## 6. 解的个数

| 因素 | 支数 | 说明 |
|------|------|------|
| $q_4$（$\sigma_4=\pm$） | $2$ | 肘三角形翻折 → `cfg.elbow` |
| $q_2$（ZYZ $\pm$） | $2$ | 肩部同一 $R_{123}$ 的两种参数 → `cfg.shoulder` |
| 虚拟肩参考 | **固定 $1$** | 否则会虚增到 $8$ |
| $q_6$ 翻转 | **$0$（限位）** | URDF $|q_6|\le 1.0472$ |
| **每个 $\psi$** | **最多 $4$** | $2\times 2$ |

限位、奇异（$SW$ 近 $0$、肩部 $q_2\approx 0/\pi$ 等）下实际解数可能少于 $4$。

调用侧用 `cfg.shoulder`、`cfg.elbow` 在 $4$ 支中选定一支；`q_ref` 可用于在同支不可达时就近回退（实现可选）。

---

## 7. 最小计算流程（复用 + 抱起 + $s_i,c_i$ 最后回代）

原则：

1. **能复用的不算第二次**（$R_\psi$、$R_{123}$、$R_{04}$、$R_{57}$、定长、$SW^2$ 等）
2. **$s_i=\sin q_i,\;c_i=\cos q_i$ 留到最后再回代**：推导与中间式一律保留符号 $s_i,c_i$（或矩阵因子），**不**提倡“先求出数值角再立刻 `sin/cos`”；只有在需要数值输出或无法再保留时才回代
3. **能抱起就不完全展开**：中间量保持矩阵/乘积块（如 $R_{57}=R_{04}^\top R_a$），从块的元素提取角度；除非展开后明显更省或不可避免，否则不做逐元多项式展开
4. $M=\mathrm{Trans}(z_M)$，$T_a$ 只需 $t_z\leftarrow t_z-z_M$，不必求 $4\times 4$ 逆

### 7.1 预计算（臂参数构造时一次）

$$
SE,\,EW,\,SE^2,\,EW^2,\,2\,SE\,EW,\;
\angle SEA+\angle WEB,\;
z_{p3},\,z_{p7},\,z_M,\;p_S,\,p_7
$$

### 7.2 每次 IK（与 $\sigma_4$ 无关，算一次）

```
T_a:   t_z ← T_eff.t_z − z_M；  R_a ← T_eff.R
pW:    R_a[:,2]·z_p7 + t_a
w:     pW − pS
SW²:   w·w；  SW ← √SW²；  可达性
∠SEW:  acos( (SE²+EW²−SW²)/(2 SE EW) )
q4_off: −(∠SEA+∠WEB)
u:     w/SW
R_ψ:   Rot(u, ψ)     # 以 (s_ψ,c_ψ) 符号写入 Rodrigues，最后与 ψ 一并回代
```

### 7.3 每个 $\sigma_4$（肘支）——抱起写法

```
q4   ← σ4·∠SEW + q4_off
p7p  ← p7p(s4,c4)          # 闭式，保留 s4,c4，不展开后续用到处
Rref ← Rz(q1v*) Ry(q2v*)   # 虚拟肩：内部可用 s1v,c1v,s2v,c2v 符号

R123 ← R_ψ · Rref          # ★ 抱起：不展开成 9 个关于 sψ,s1v,… 的多项式
R04  ← R123 · Ry(q4)       # ★ 抱起：不展开
R57  ← R04ᵀ · R_a          # ★ 抱起：腕部只依赖该 3×3 块

(q5,q6,q7) ← ZYX_extract(R57)   # 只用 R57 的若干元素，两支 ZYZ 共用
(q1,q2,q3) ← ZYZ_extract(R123) # 按 cfg.shoulder 取一支

# —— 最后回代 ——
# 对已确定的 q4,q1v*,q2v*,ψ（及若 FK 需要的 qi）统一：
#   si,ci ← sin qi, cos qi
# 再代入上述矩阵块做数值乘；或在符号推导末尾一次性 subs
```

选定支：$\sigma_4=+1$ 若 `cfg.elbow$\ge 0$`，否则 $-1$；肩部取 ZYZ 中与 `cfg.shoulder` 同号的 $q_2$ 支（见 §9）。

### 7.4 腕部提取（典型“抱起、不展开”）

不把 $R_{04}^\top R_a$ 乘成关于全部 $s_i,c_i$ 的显式式，只算一次矩阵积得 $R_{57}=(r_{ij})$，再：

$$
\begin{aligned}
q_6&=\operatorname{atan2}\big(-r_{31},\,\sqrt{r_{11}^2+r_{21}^2}\big),\\
q_5&=\operatorname{atan2}(r_{21}/c_6,\,r_{11}/c_6),\\
q_7&=\operatorname{atan2}(r_{32}/c_6,\,r_{33}/c_6).
\end{aligned}
$$

同理 $q_{1,2,3}$ 由 $R_{123}$ 的元素提取，不必把 $R_\psi R_{\mathrm{ref}}$ 完全展开。

### 7.5 虚拟肩（保留 $s,c$ 至块组装）

$$
A=r_z,\; B=-r_x,\quad
\phi=\operatorname{atan2}(B,A),\quad
\delta=\arccos\!\left(w_z/\sqrt{A^2+B^2}\right)
$$

$q_{2v}^\star=\max(\phi\pm\delta)$，$q_{1v}^\star=\operatorname{atan2}(w_y/\rho,\,w_x/\rho)$（$\rho=r_x c_{2v}+r_z s_{2v}$）。  
随后 $R_{\mathrm{ref}}=R_z(q_{1v}^\star)R_y(q_{2v}^\star)$ **以矩阵块保留**；$s_{1v},c_{1v},s_{2v},c_{2v}$ 与其它角的 $s,c$ 一起在流程末尾回代。

### 7.6 FK

$$
T=\Big(\prod_{i=1}^{7} e^{[\xi_i]q_i}\Big)M
$$

实现上：各因子用 $R_i(c_i,s_i)$ 累乘（**不**把整条链乘成单一巨型展开式）；全部 $s_i,c_i$ 在进入累乘前一次性由 $q$ 回代即可。

反求臂角：§4.1 的 BSW→SEW 公式（`arm_angle_from_SEW`）。

---

## 8. 臂角雅可比 $J_\psi$（与本文 $\psi$ 定义匹配）

参考文献：

> K. Kreutz-Delgado, M. Long, H. Seraji,  
> *Kinematic Analysis of 7-DOF Manipulators*,  
> The International Journal of Robotics Research, **11**(5):469–481, **1992**

原文用肘平面相对竖直参考平面的夹角；本文参考面取 **BSW**（基座 $z$+$SW$，与 TJ 相同），瞬时结构同构。

### 8.1 定义回顾

$\psi$ 见 §4.1；IK 侧 $R_{123}=\mathrm{Rot}(u,\psi)\,R_{\mathrm{ref}}$（§4.2）。

### 8.2 解析雅可比

记 $s=u^\top(r\times p)$，$c=r^\top p$，$\Delta=s^2+c^2$。点雅可比（空间系，注意中间点只含祖先关节）：

$$
(J_E)_{*,i}=\omega_i^s\times(E-q_i^s)\ (i=1,2,3),\quad
(J_W)_{*,i}=\omega_i^s\times(W-q_i^s)\ (i=1..4).
$$

$$
J_u=\frac{(I-uu^\top)J_W}{\|w\|},\quad
J_p=(I-uu^\top)J_E-(u^\top e)J_u-u(e^\top J_u).
$$

冻结 $r$（与原文常值 $V$ 同构；**固定末端自运动**时 $W,q_4$ 不变，$r$ 恰为常量）：

$$
\frac{\partial s}{\partial q}=(r\times p)^\top J_u+(u\times r)^\top J_p,\quad
\frac{\partial c}{\partial q}=r^\top J_p,
\quad
J_\psi=\frac{c\,\partial s/\partial q-s\,\partial c/\partial q}{\Delta}.
$$

该式在冻结 $r$ 下**恒等价于**更省算的闭式（实现采用此式）：

$$
\boxed{
J_\psi
=
\big[
u^\top\omega_1^s,\ 
u^\top\omega_2^s,\ 
u^\top\omega_3^s,\ 
0,\ 0,\ 0,\ 0
\big]
}
$$

理由：$R_{123}=\mathrm{Rot}(u,\psi)R_{\mathrm{ref}}$ 且 $u,R_{\mathrm{ref}}$ 冻结时 $\omega_{123}=u\dot{\psi}$，故 $\dot{\psi}=u^\top\omega_{123}$；且冻结 $r$ 时 $\partial\psi/\partial q_4=0$。

奇异：$\|p\|\to 0$（肘与 $SW$ 共线）或参考平面不定 → 与原文 algorithmic singularity 一致。

### 8.3 增广雅可比

$$
J_A(q)
=
\begin{bmatrix}
J_{\omega}\\
J_{v}\\
J_\psi
\end{bmatrix}
\in\mathbb{R}^{7\times 7},
\qquad
\begin{bmatrix}\omega\\ v\\ \dot{\psi}\end{bmatrix}
=J_A\dot{q}.
$$

用于位姿 + 臂角的配置控制 / 冗余分辨率（Seraji configuration control 框架）。

### 8.4 C++ 建议接口

```cpp
/** J_psi: 1×7 行主序；成功返回 0 */
int arm_angle_jacobian(const double q[7], double J_psi[7]);

/** J_A: 7×7 行主序，末行为 J_psi */
int augmented_jacobian(const double q[7], double J_A[49]);
```

Python：`arm_angle_jacobian(q)`，`augmented_jacobian(q)`。

### 8.5 躯干圆柱回避：解析 $\dot\psi$ → 积分 → IK（非数值优化）

目标：世界系竖直**无限高**圆柱（轴 $=z$，半径 $R_c=0.15$）与线段 $SE$、$EW$ 保持间隙；缓冲带 $\phi_0=0.03$（水平半径警戒到 $R_c+\phi_0=0.18$）。**不**用数值搜索/优化，只用闭式 $\dot\psi$，积分得 $\psi$，再代入解析 IK。

#### 流程（推荐：临界设定值）

$$
\psi^\star(\phi=\phi_0)
\;\xrightarrow{\ \psi\leftarrow\psi^\star\ }
\;\xrightarrow{\ \mathrm{ik}(T,\psi)\ }
\;q
$$

备选：积分离散 $\dot\psi_{\mathrm{rep}}$ 再 IK。末端位姿 $T$ 由任务给定；回避只改臂角通道。

#### 固定 $T$ 时肘点对 $\psi$ 的运动（解析）

$S,W,u,R_{\mathrm{ref}}$ 不变时：

$$
E(\psi)=S+\mathrm{Rot}(u,\psi)\,e_0,\quad
e_0=R_{\mathrm{ref}}(p_4-S)
$$

$$
\boxed{
\frac{\mathrm{d}E}{\mathrm{d}\psi}=u\times(E-S)
}
$$

（$SE$ 的 $S$、$EW$ 的 $W$ 固定，仅 $E$ 动。）

#### 线段相对竖直圆柱的间隙（解析）

在水平面内求原点到线段 $A_hB_h$ 的最近点（与 $z$ 无关）：

$$
t^\star=\mathrm{clip}\!\left(
\frac{-A_h\cdot(B_h-A_h)}{\|B_h-A_h\|^2},\,0,\,1\right),\quad
P^\star=A+t^\star(B-A)
$$

$$
d(AB)=\rho(P^\star)-R_c,\qquad
\rho=\sqrt{P_x^2+P_y^2},\qquad
\mathbf{n}=\frac{1}{\rho}(P_x,P_y,0)^\top
$$

（$\rho\to0$ 时单独保护。）

对 $SE$（$A=S,B=E$）与 $EW$（$A=E,B=W$）各算 $d_{SE},d_{EW}$，取

$$
\phi=\min(d_{SE},d_{EW})
$$

#### 间隙对臂角的导数（解析）

仅 $E$ 依赖 $\psi$。记活跃段上 $P^\star=(1-t^\star)A+t^\star B$：

- 若 $P^\star$ 随 $E$：$A$ 或 $B$ 为 $E$，或内点 $t^\star\in(0,1)$ 且 $E$ 为端点之一，则  
  $$
  \frac{\partial P^\star}{\partial\psi}=t_E\,\frac{\mathrm{d}E}{\mathrm{d}\psi}
  $$
  （$t_E=1$ 若 $B=E$；$t_E=0$ 若 $A=E$ 且 $t^\star=0$；内点对 $SE$ 为 $t^\star$，对 $EW$ 为 $1-t^\star$ 等，按端点角色取值。）
- 若最近点钉在固定端 $S$ 或 $W$：$\partial\phi/\partial\psi=0$（该段）。

活跃间隙：

$$
\boxed{
\frac{\partial\phi}{\partial\psi}
=\mathbf{n}(P^\star)^\top
\frac{\partial P^\star}{\partial\psi}
=\mathbf{n}(P^\star)^\top
\big(t_E\,u\times(E-S)\big)
}
$$

#### 斥力臂角速度（解析，可积分）

势函数（$\phi<\phi_0$ 时）：

$$
U=\frac12\eta\left(\frac{1}{\phi}-\frac{1}{\phi_0}\right)^2
\quad(\phi>0),\qquad
\dot\psi_{\mathrm{rep}}
=-\frac{\partial U}{\partial\psi}
=\eta\left(\frac{1}{\phi}-\frac{1}{\phi_0}\right)\frac{1}{\phi^2}
\frac{\partial\phi}{\partial\psi}
$$

或更简单的线性区：

$$
\dot\psi_{\mathrm{rep}}
=k\cdot\max(0,\,\phi_0-\phi)\cdot\operatorname{sign}\!\left(\frac{\partial\phi}{\partial\psi}\right)
$$

（取使 $\phi$ **增大** 的符号，即 $\dot\psi \propto +\partial\phi/\partial\psi$ 若用 $k\max(0,\phi_0-\phi)$。）

推荐：

$$
\boxed{
\dot\psi_{\mathrm{rep}}
=k\,\max(0,\,\phi_0-\phi)\,
\frac{\partial\phi}{\partial\psi}
}
$$

积分：$\psi\leftarrow\psi+\dot\psi_{\mathrm{rep}}\Delta t$（可限幅），再 `ik(T_target, ψ, …)`。

#### 坐标系

圆柱在 `base_link` / 世界系；臂内 $S,E,W,u$ 先变到世界系再算间隙：

$$
P^w=R_{\mathrm{mount}}P^{\mathrm{arm}}+t_{\mathrm{mount}}
$$

左：$t=(0,+0.025,1.240263),\;R=R_x(-\pi/2)$；右：$t=(0,-0.025,1.240263),\;R=R_x(+\pi/2)$。

$$
\frac{\mathrm{d}E^w}{\mathrm{d}\psi}=R_{\mathrm{mount}}\big(u^{\mathrm{arm}}\times(E^{\mathrm{arm}}-S^{\mathrm{arm}})\big)
$$

#### 与 $J_\psi$ 的关系

本方案**不**在关节空间用 $J_\psi$ 做数值投影；$\dot\psi$ 已是标量命令，IK 直接吃 $\psi$。  
$J_\psi$ 仅在需要 $\dot q=J_\psi^+\dot\psi$ 的速度级控制时使用；位置级「积分 $\psi$ + 解析 IK」不必经过 $J_\psi$。

Python：`psi_dot_cylinder_repulsion(...)`（见 `analytical_ik.py`）。  
**推荐设定值方式**（不再积分离散 $\dot\psi$）：`psi_setpoint_cylinder` / `apply_psi_setpoint_ik`——把 $\phi=\phi_0$ 的解析临界臂角直接当作 $\psi$ 指令；已满足间隙则保持；若无临界根（$\max_\psi\phi<\phi_0$）则**视为 IK 失败**。

#### 临界臂角（解析）

固定末端 $T$ 时 $S,W,u$ 不变，肘点
$$
E(\psi)=S+\mathrm{Rot}(u,\psi)\,e_0,\quad e_0=R_{\mathrm{ref}}(p_4-S)
$$
（$e_0$ 依赖肘支 $\sigma_4$）。水平投影 $E_h(\psi)=P_0+c\,V_1+s\,V_2$。

给定目标间隙 $\phi^\star$（接触取 $0$，缓冲取 $\phi_0$），令 $R_\star=R_c+\phi^\star$。临界候选：

1. 直线 $SE$ / $EW$ 与水平圆 $\rho=R_\star$ **相切**：$|A_h\times E_h|^2=R_\star^2\|E_h-A_h\|^2$
2. 肘点落在该圆上：$\|E_h\|^2=R_\star^2$

均为 $c=\cos\psi,\,s=\sin\psi$ 的二次三角式；Weierstrass $t=\tan(\psi/2)$ 化为 **次数 ≤4** 的代数方程，`np.roots` 求实根后再用真实线段间隙 $\phi(\psi)$ 过滤（去掉延长线相切）。

API：`critical_arm_angles_cylinder(T, …, phi_star=…, q_seed=…)`；可行区间（采样）见 `critical_psi_interval`。注意必须用与当前构型一致的 $\sigma_4$（`elbow_sigma4(q)` / `q_seed`）。

若某 $T$ 上 $\max_\psi\phi(\psi)<\phi^\star$，则**无临界臂角**，设定值接口返回失败（与 IK 无解同等处理）。

---

## 9. C++ 逆解接口

### 9.1 配置结构体

```cpp
/** 解析 IK 分支与臂角配置。
 *  shoulder : 肩部 ZYZ 的 q2 支，+1 → q2>=0 支，-1 → q2<0 支（或 0/+1 约定见实现）
 *  elbow    : 肘部 σ4 支，+1 → σ4=+1，-1 → σ4=-1
 *  psi      : 臂角 ψ [rad]，BSW→SEW 绕 SŴ（与 TJ 一致）
 */
struct IkCfg {
  int    shoulder;  // ±1
  int    elbow;     // ±1
  double psi;       // [rad]
};
```

### 9.2 逆解函数

```cpp
/** 7-DoF 解析 IK（选定 shoulder×elbow 一支）。
 *
 * @param T_target  目标位姿 4×4（行主或列主与工程约定一致），法兰系相对 base
 * @param q_ref     参考关节角 7×1，用于连续/就近（限位内微调或失败回退）
 * @param cfg       分支与臂角，见 IkCfg
 * @param q_sol     输出关节角 7×1
 * @param user      可选：base→user 变换 4×4；nullptr 视为 I
 * @param tool      可选：flange→tool 变换 4×4；nullptr 视为 I
 * @return          0 成功；非 0 失败（不可达 / 奇异 / 超限等）
 *
 * 有效目标：
 *   T_eff = user * T_target * inv(tool)   (nullptr 处用 I)
 */
int ik(const double T_target[16],
       const double q_ref[7],
       const IkCfg& cfg,
       double q_sol[7],
       const double* user = nullptr,
       const double* tool = nullptr);
```

调用示例：

```cpp
IkCfg cfg{/*shoulder=*/+1, /*elbow=*/-1, /*psi=*/0.3};
double q_sol[7];
int ret = ik(T_target, q_ref, cfg, q_sol, /*user=*/nullptr, /*tool=*/nullptr);
if (ret != 0) {
  // 不可达或该分支失败
}
```

分支与几何量对应：

| `cfg` 字段 | 几何 |
|------------|------|
| `cfg.elbow` | $\sigma_4=\operatorname{sign}(\texttt{elbow})$，$q_4=\sigma_4\angle SEW-\angle SEA-\angle WEB$ |
| `cfg.shoulder` | ZYZ 分解中选取 $q_2$ 符号与 $\operatorname{sign}(\texttt{shoulder})$ 一致的一支 |
| `cfg.psi` | $R_{123}=\mathrm{Rot}(\widehat{SW},\psi)\,R_z(q_{1v}^\star)R_y(q_{2v}^\star)$ |

完整 $4$ 支枚举时可对 `shoulder,elbow $\in\{+1,-1\}$` 循环调用，或另提供批量接口（实现可选）。

---

## 10. Python 参考入口

关节限位（URDF `AR3-3_07L/R`，左右相同）存在 `ArmParams.q_min/q_max`；`ik` / `analytical_ik` 默认丢弃超限解。

```python
from analytical_ik import (
    ArmParams, IkCfg, fk_poe, cfg_from_q, ik, analytical_ik,
    compute_arm_angle, within_limits, nearest_ik,
)

params = ArmParams()
T = fk_poe(q, params)
cfg = cfg_from_q(q, params)       # elbow, shoulder, psi
q_sol = ik(T, cfg, params)        # 单解；超限 → None
sols = analytical_ik(T, cfg.psi, params)  # ≤4，已滤限位
```

```bash
python3 analytical_ik.py                 # 自检：cfg+ik / 限位
python3 derive_poe_analytical_ik.py      # sympy 草稿（可选）
```

---

## 11. 与 URDF 的对应

`rokea_description/urdf/arms/right_arm.xacro`：

| 关节 | 轴 | 相对原点累加 |
|------|----|----------------|
| $1$ | $Z$ | $0$（轴过 $p_S$ 亦可取 $p_1$） |
| $2$ | $Y$ | $z=d_1=0.17$ |
| $3$ | $Z$ | $z=d_1+d_3=0.451$ |
| $4$ | $Y$ | $x=a_3=0.015$ |
| $5$ | $Z$ | $x=0,\;z=d_1+d_3+d_5=0.727$ |
| $6$ | $Y$ | 同腕心 |
| $7$ | $X$ | 同腕心 |
| flange | — | $z+=d_{wf}=0.093$ → $0.82$ |

关节限位（`limit lower/upper`，L/R 相同）→ `Q_MIN_DEFAULT` / `Q_MAX_DEFAULT`：

| $q$ | lower | upper |
|-----|-------|-------|
| 1 | $-3.1067$ | $3.1067$ |
| 2 | $-2.0944$ | $2.0944$ |
| 3 | $-3.1067$ | $3.1067$ |
| 4 | $-1.0472$ | $2.5307$ |
| 5 | $-3.1067$ | $3.1067$ |
| 6 | $-1.0472$ | $1.0472$ |
| 7 | $-1.5708$ | $1.5708$ |
