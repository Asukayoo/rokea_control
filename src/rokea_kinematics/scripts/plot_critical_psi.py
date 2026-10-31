#!/usr/bin/env python3
"""左臂沿 -Y：解析临界臂角 ψ*(φ=0 / φ=φ0) 与固定 ψ=-90° 间隙对比。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import analytical_ik as ik  # noqa: E402

DEG = np.pi / 180.0
OUT = ROOT / "figs"
OUT.mkdir(exist_ok=True)


def main():
    params = ik.ArmParams()
    Rm, tm = ik.mount_left_world()
    q0 = np.array([90, 90, -90, 90, 0, 0, 0], dtype=float) * DEG
    T0 = ik.fk_poe(q0, params)
    sig = ik.elbow_sigma4(q0, params)
    Tw0 = np.eye(4)
    Tw0[:3, :3] = Rm @ T0[:3, :3]
    Tw0[:3, 3] = Rm @ T0[:3, 3] + tm

    ss, phi_fix, psi_c0, psi_c03, phi_max = [], [], [], [], []
    for s in np.arange(0.0, 0.401, 0.005):
        Tw = Tw0.copy()
        Tw[1, 3] -= s
        Ta = np.eye(4)
        Ta[:3, :3] = Rm.T @ Tw[:3, :3]
        Ta[:3, 3] = Rm.T @ (Tw[:3, 3] - tm)
        geo = ik._sw_elbow0_from_T(Ta, params, sig)
        if geo is None:
            break
        Sa, Wa, ua, e0a = geo
        Sw, Ww = Rm @ Sa + tm, Rm @ Wa + tm
        uw = Rm @ ua
        uw = uw / np.linalg.norm(uw)
        e0w = Rm @ e0a
        phi_f = ik.clearance_phi_at_psi(Sw, Ww, uw, e0w, -0.5 * np.pi, 0.15)[0]
        phis = [
            ik.clearance_phi_at_psi(Sw, Ww, uw, e0w, float(p), 0.15)[0]
            for p in np.linspace(-np.pi, np.pi, 360, endpoint=False)
        ]
        c0 = ik.critical_arm_angles_cylinder(
            Ta, params, Rm, tm, 0.15, 0.0, sigma4=sig
        )
        c03 = ik.critical_arm_angles_cylinder(
            Ta, params, Rm, tm, 0.15, 0.03, sigma4=sig
        )
        ss.append(s)
        phi_fix.append(phi_f)
        phi_max.append(max(phis))
        # 相对 -90° 最近的临界根（便于对照斥力方向）
        def nearest(crit):
            if not crit:
                return np.nan
            return min(crit, key=lambda r: abs(ik.wrap(r["psi"] + 0.5 * np.pi)))["psi"] / DEG

        psi_c0.append(nearest(c0))
        psi_c03.append(nearest(c03))

    ss = np.asarray(ss)
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True, constrained_layout=True)
    fig.suptitle(
        rf"Critical arm angle (analytical) | left arm, $\sigma_4={sig:+.0f}$"
        r" | $R_c=0.15$"
    )

    ax = axes[0]
    ax.plot(ss, phi_fix, lw=2.0, label=r"$\phi$ at $\psi=-90^\circ$")
    ax.plot(ss, phi_max, lw=2.0, ls="--", label=r"$\max_\psi\,\phi(\psi)$")
    ax.axhline(0.03, color="C3", ls=":", lw=1.2, label=r"$\phi_0=0.03$")
    ax.axhline(0.0, color="0.4", lw=0.8)
    ax.set_ylabel(r"clearance $\phi$ [m]")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", fontsize=9)

    ax = axes[1]
    ax.plot(ss, psi_c03, "o-", ms=3, lw=1.5, label=r"nearest $\psi^\star(\phi=0.03)$ to $-90^\circ$")
    ax.plot(ss, psi_c0, "s--", ms=3, lw=1.5, label=r"nearest $\psi^\star(\phi=0)$ to $-90^\circ$")
    ax.axhline(-90.0, color="0.5", ls=":", lw=1.0, label=r"fixed $\psi=-90^\circ$")
    ax.set_xlabel(r"$s$ along world $-Y$ [m]")
    ax.set_ylabel(r"critical $\psi$ [deg]")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", fontsize=9)

    path = OUT / "cyl_critical_psi.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"wrote {path}")
    print(f"sigma4={sig}")
    # 起点数值
    Ta0 = T0
    for star in (0.0, 0.03):
        crit = ik.critical_arm_angles_cylinder(
            Ta0, params, Rm, tm, 0.15, star, q_seed=q0
        )
        print(f"s=0 phi*={star}:")
        for r in crit:
            print(
                f"  psi={r['psi']/DEG:7.2f} deg  kind={r['kind']:12s}  "
                f"dSE={r['d_se']:.4f} dEW={r['d_ew']:.4f}"
            )


if __name__ == "__main__":
    main()
