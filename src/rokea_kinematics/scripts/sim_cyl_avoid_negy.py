#!/usr/bin/env python3
"""左臂沿世界 -Y 运动至 IK 无解；对比固定臂角 / 临界臂角设定值（φ*=0, Rc=0.15）；输出 CSV + PNG。"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import analytical_ik as ik  # noqa: E402

DEG = np.pi / 180.0
OUT = ROOT / "figs"
OUT.mkdir(exist_ok=True)

C_FIXED = "C0"
C_SET = "C1"


def world_to_arm(Tw, Rm, tm):
    Ta = np.eye(4)
    Ta[:3, :3] = Rm.T @ Tw[:3, :3]
    Ta[:3, 3] = Rm.T @ (Tw[:3, 3] - tm)
    return Ta


def clearance(q, params, Rm, tm):
    Sa, Ea, Wa, *_ = ik._shoulder_wrist_elbow(q, params)
    S, E, W = Rm @ Sa + tm, Rm @ Ea + tm, Rm @ Wa + tm
    dse, _, _, _ = ik.segment_cylinder_clearance(S, E)
    dew, _, _, _ = ik.segment_cylinder_clearance(E, W)
    if dse <= dew:
        return dse, dse, dew, "SE", S, E, W
    return dew, dse, dew, "EW", S, E, W


def run(use_setpoint, q0, Tw0, Rm, tm, params, ds=0.005, s_max=0.8):
    q = q0.copy()
    psi = ik.compute_arm_angle(q, params)
    rows = []
    s = 0.0
    while s <= s_max + 1e-12:
        Tw = Tw0.copy()
        Tw[1, 3] -= s
        Ta = world_to_arm(Tw, Rm, tm)
        mode = "fixed"
        if use_setpoint:
            qn, psi_new, _, mode = ik.apply_psi_setpoint_ik(
                Ta, q, params=params, side="left", R_cyl=0.15, phi0=0.0
            )
            # 无 φ=0 临界根 / nearest_ik 失败 → 与 IK 失败同等停仿真
            if qn is None or psi_new is None or mode == "fail":
                break
            q = qn
            psi = psi_new
        else:
            qn = ik.nearest_ik(Ta, psi, q_seed=q, params=params)
            if qn is None:
                break
            q = qn
            psi = ik.compute_arm_angle(q, params)

        phi, dse, dew, act, S, E, W = clearance(q, params, Rm, tm)
        Terr = ik.fk_poe(q, params)
        pos_err = float(np.linalg.norm(Rm @ Terr[:3, 3] + tm - Tw[:3, 3]))
        rows.append(
            {
                "s": s,
                "psi": psi,
                "phi": phi,
                "dse": dse,
                "dew": dew,
                "act": act,
                "mode": mode,
                "q": q.copy(),
                "py": float(Tw[1, 3]),
                "pos_err": pos_err,
                "S": S.copy(),
                "E": E.copy(),
                "W": W.copy(),
            }
        )
        s += ds
    return rows


def summarize(name, rows):
    phis = [r["phi"] for r in rows]
    print(f"\n=== {name} ===")
    print(f"steps={len(rows)}  s_last={rows[-1]['s']:.3f} m  py_last={rows[-1]['py']:.3f}")
    print(f"phi: start={phis[0]:.4f}  min={min(phis):.4f}  end={phis[-1]:.4f}")
    print(
        f"psi: {rows[0]['psi']/DEG:.2f} deg -> {rows[-1]['psi']/DEG:.2f} deg  "
        f"dpsi={(rows[-1]['psi']-rows[0]['psi'])/DEG:.2f} deg"
    )
    dq = ik.wrap_vec(rows[-1]["q"] - rows[0]["q"])
    print(f"|dq|_inf={np.max(np.abs(dq))/DEG:.2f} deg  |dq|_2={np.linalg.norm(dq)/DEG:.2f} deg")
    print(f"max pose err={max(r['pos_err'] for r in rows):.2e}")
    pen = next((r["s"] for r in rows if r["phi"] < -1e-9), None)
    print(f"first phi<0 at s={pen}")
    if any(r.get("mode") for r in rows):
        from collections import Counter

        print("modes:", dict(Counter(r["mode"] for r in rows)))


def write_csv(path, rows):
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["s", "py", "phi", "d_SE", "d_EW", "active", "psi_deg", "mode"]
            + [f"q{i+1}_deg" for i in range(7)]
            + ["pos_err"]
        )
        for r in rows:
            w.writerow(
                [
                    r["s"],
                    r["py"],
                    r["phi"],
                    r["dse"],
                    r["dew"],
                    r["act"],
                    r["psi"] / DEG,
                    r.get("mode", ""),
                    *[float(x) / DEG for x in r["q"]],
                    r["pos_err"],
                ]
            )


def write_timeseries_png(path, rows_fix, rows_set):
    s_f = np.array([r["s"] for r in rows_fix])
    s_r = np.array([r["s"] for r in rows_set])
    phi_f = np.array([r["phi"] for r in rows_fix])
    phi_r = np.array([r["phi"] for r in rows_set])
    psi_f = np.array([r["psi"] for r in rows_fix]) / DEG
    psi_r = np.array([r["psi"] for r in rows_set]) / DEG
    Q = np.column_stack(
        [np.unwrap([r["q"][j] for r in rows_set]) / DEG for j in range(7)]
    )

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, constrained_layout=True)
    fig.suptitle(
        r"Left arm EE along world $-Y$ | $R_c=0.15$, $\phi^\star=0$"
        r" | fixed $\psi$ vs critical setpoint"
    )

    ax = axes[0]
    ax.plot(s_f, phi_f, color=C_FIXED, lw=2.0, label=r"fixed $\psi$")
    ax.plot(s_r, phi_r, color=C_SET, lw=2.0, ls="--", label=r"critical setpoint")
    ax.axhline(0.0, color="C3", lw=1.0, ls=":", label=r"$\phi^\star=0$ ($R_c$)")
    ax.set_ylabel(r"clearance $\phi=\rho_{\min}-R_c$ [m]")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", fontsize=9)

    ax = axes[1]
    ax.plot(s_f, psi_f, color=C_FIXED, lw=2.0, label=r"fixed $\psi$")
    ax.plot(s_r, psi_r, color=C_SET, lw=2.0, ls="--", label=r"critical setpoint")
    ax.set_ylabel(r"arm angle $\psi$ [deg]")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", fontsize=9)

    ax = axes[2]
    for j in range(7):
        ax.plot(s_r, Q[:, j], lw=1.6, label=rf"$q_{{{j+1}}}$")
    ax.set_ylabel(r"joint $q$ [deg] (setpoint)")
    ax.set_xlabel(r"$s$ along $-Y$ [m]")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", ncol=4, fontsize=8)

    fig.savefig(path, dpi=150)
    plt.close(fig)


def write_topview_png(path, rows_set):
    pts = np.vstack([[r["S"][:2], r["E"][:2], r["W"][:2]] for r in rows_set])
    xmin, ymin = pts.min(axis=0) - 0.08
    xmax, ymax = pts.max(axis=0) + 0.08
    xmin, ymin = min(xmin, -0.28), min(ymin, -0.08)
    xmax, ymax = max(xmax, 0.28), max(ymax, 0.35)

    fig, ax = plt.subplots(figsize=(6.5, 6.5), constrained_layout=True)
    ax.set_title(r"Top view SE–EW (critical setpoint, $\phi^\star=0$)")
    ax.add_patch(
        Circle((0.0, 0.0), 0.15, facecolor="0.85", edgecolor="0.35", lw=1.5, zorder=0, label=r"$R_c=0.15$")
    )

    idxs = [0, len(rows_set) // 3, 2 * len(rows_set) // 3, len(rows_set) - 1]
    for k, idx in enumerate(idxs):
        r = rows_set[idx]
        xy = np.vstack([r["S"][:2], r["E"][:2], r["W"][:2]])
        ax.plot(
            xy[:, 0],
            xy[:, 1],
            "-o",
            color=f"C{k}",
            ms=4,
            lw=2.0,
            label=rf"$s={r['s']:.2f}$, $\phi={r['phi']:.3f}$",
        )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel(r"$x$ [m]")
    ax.set_ylabel(r"$y$ [m]")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)

    fig.savefig(path, dpi=150)
    plt.close(fig)


def write_dq_png(path, rows_fix, rows_set):
    dq_f = ik.wrap_vec(rows_fix[-1]["q"] - rows_fix[0]["q"]) / DEG
    dq_r = ik.wrap_vec(rows_set[-1]["q"] - rows_set[0]["q"]) / DEG
    x = np.arange(1, 8)
    w = 0.36

    fig, ax = plt.subplots(figsize=(8, 4.2), constrained_layout=True)
    ax.bar(
        x - w / 2,
        dq_f,
        width=w,
        color=C_FIXED,
        label=rf"fixed ($s_{{\max}}={rows_fix[-1]['s']:.2f}$)",
    )
    ax.bar(
        x + w / 2,
        dq_r,
        width=w,
        color=C_SET,
        label=rf"setpoint ($s_{{\max}}={rows_set[-1]['s']:.2f}$)",
    )
    ax.axhline(0.0, color="0.4", lw=0.8)
    ax.set_xticks(x, [rf"$q_{{{j}}}$" for j in range(1, 8)])
    ax.set_ylabel(r"$\Delta q$ [deg]")
    ax.set_title(r"Joint change over $-Y$ motion (end $-$ start)")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend(loc="best", fontsize=9)

    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    params = ik.ArmParams()
    Rm, tm = ik.mount_left_world()
    q0 = np.array([90, 90, -90, 90, 0, 0, 0], dtype=float) * DEG
    T0 = ik.fk_poe(q0, params)
    Tw0 = np.eye(4)
    Tw0[:3, :3] = Rm @ T0[:3, :3]
    Tw0[:3, 3] = Rm @ T0[:3, 3] + tm

    rows_fix = run(False, q0, Tw0, Rm, tm, params)
    rows_set = run(True, q0, Tw0, Rm, tm, params)
    summarize("FIXED psi", rows_fix)
    summarize("CRITICAL SETPOINT", rows_set)

    write_csv(OUT / "cyl_avoid_fixed.csv", rows_fix)
    write_csv(OUT / "cyl_avoid_setpoint.csv", rows_set)
    write_timeseries_png(OUT / "cyl_avoid_negY.png", rows_fix, rows_set)
    write_topview_png(OUT / "cyl_avoid_topview.png", rows_set)
    write_dq_png(OUT / "cyl_avoid_dq.png", rows_fix, rows_set)
    print(f"\nPNG -> {OUT}")
    for name in ("cyl_avoid_negY.png", "cyl_avoid_topview.png", "cyl_avoid_dq.png"):
        print(" ", OUT / name)


if __name__ == "__main__":
    main()
