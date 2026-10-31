#!/usr/bin/env python3
"""Convert torso STEP to STL in the stand base frame.

STEP frame (mm) -> stand frame (mm):
  X = -Y_s
  Y =  X_s
  Z =  Z_s + 980.2628

Stand frame definition:
  origin  — support-plate bottom center
  Z       — along column, upward
  Y       — along the top mount hole axis (STEP +X)
  X       — forward (completes right-handed frame; STEP -Y)

Left/right mount hole midplanes (stand, m):
  left  (STEP Y=+150): (-0.15, 0, 1.150124)
  right (STEP Y=-150): ( +0.15, 0, 1.150124)
Outer mating faces at Y = ±0.025 m.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cadquery as cq
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform


Z_BOTTOM_STEP_MM = 980.2628


def step_to_stand_trsf() -> gp_Trsf:
    """X=-Ys, Y=Xs, Z=Zs+offset  (hole axis -> +Y, column -> +Z)."""
    trsf = gp_Trsf()
    # p' = R p + t, R maps ex->(0,1,0), ey->(-1,0,0), ez->(0,0,1)
    # row-major 3x4: [  0 -1  0 | 0 ]
    #                [  1  0  0 | 0 ]
    #                [  0  0  1 | Z ]
    trsf.SetValues(
        0.0, -1.0, 0.0, 0.0,
        1.0,  0.0, 0.0, 0.0,
        0.0,  0.0, 1.0, Z_BOTTOM_STEP_MM,
    )
    return trsf


def shape_bbox(shape) -> tuple[float, float, float, float, float, float]:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, True)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return xmin, ymin, zmin, xmax, ymax, zmax


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[3]
    default_step = (
        root
        / "docs"
        / "rokea_docs"
        / "rokea"
        / "AR3&amp;amp;5系列躯干整装(4).stp"
    )
    default_out = Path(__file__).resolve().parents[1] / "meshes" / "torso" / "torso.stl"
    parser.add_argument("--step", type=Path, default=default_step)
    parser.add_argument("--out", type=Path, default=default_out)
    parser.add_argument("--linear-deflection", type=float, default=0.5)
    parser.add_argument("--angular-deflection", type=float, default=0.5)
    args = parser.parse_args()

    if not args.step.is_file():
        print(f"STEP not found: {args.step}", file=sys.stderr)
        return 1

    print(f"Loading STEP: {args.step}")
    result = cq.importers.importStep(str(args.step))
    shape = result.val().wrapped

    xmin, ymin, zmin, xmax, ymax, zmax = shape_bbox(shape)
    print(
        f"STEP bbox mm: "
        f"X[{xmin:.2f},{xmax:.2f}] Y[{ymin:.2f},{ymax:.2f}] Z[{zmin:.2f},{zmax:.2f}]"
    )

    trsf = step_to_stand_trsf()
    transformed = BRepBuilderAPI_Transform(shape, trsf, True).Shape()
    xmin, ymin, zmin, xmax, ymax, zmax = shape_bbox(transformed)
    print(
        f"Stand bbox mm: "
        f"X[{xmin:.2f},{xmax:.2f}] Y[{ymin:.2f},{ymax:.2f}] Z[{zmin:.2f},{zmax:.2f}]"
    )
    print(
        "Mount midplanes (mm): "
        f"L={(-150.0, 0.0, 169.8612181134 + Z_BOTTOM_STEP_MM)}, "
        f"R={(150.0, 0.0, 169.8612181134 + Z_BOTTOM_STEP_MM)}"
    )

    wp = cq.Workplane().newObject([cq.Shape.cast(transformed)])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    print(f"Exporting STL: {args.out}")
    cq.exporters.export(
        wp,
        str(args.out),
        cq.exporters.ExportTypes.STL,
        tolerance=args.linear_deflection,
        angularTolerance=args.angular_deflection,
    )
    size_mb = args.out.stat().st_size / (1024 * 1024)
    print(f"Wrote {args.out} ({size_mb:.2f} MiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
