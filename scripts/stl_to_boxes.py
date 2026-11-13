#!/usr/bin/env python3
import argparse
import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rpy_to_mujoco_qpos import rpy_to_quat
from stl_to_hfield import build_index, quat_to_matrix, read_stl, transform

try:
    import numpy as np
except ImportError:
    raise SystemExit("numpy is required for stl_to_boxes.py")


def column_crossings(triangles, candidates, x, y):
    zs = []
    for i in candidates:
        a, b, c = triangles[i]
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        cx, cy = c[0], c[1]
        determinant = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(determinant) < 1e-12:
            continue
        l1 = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / determinant
        l2 = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / determinant
        l3 = 1.0 - l1 - l2
        if l1 < -1e-9 or l2 < -1e-9 or l3 < -1e-9:
            continue
        zs.append(l1 * a[2] + l2 * b[2] + l3 * c[2])
    zs.sort()
    return zs


def solid_intervals(zs):
    intervals = []
    for i in range(0, len(zs) - 1, 2):
        if zs[i + 1] - zs[i] > 1e-6:
            intervals.append((zs[i], zs[i + 1]))
    return intervals


def voxelize(triangles, bounds, resolution, index_cell):
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    nx = max(1, int(round((xmax - xmin) / resolution)))
    ny = max(1, int(round((ymax - ymin) / resolution)))
    nz = max(1, int(round((zmax - zmin) / resolution)))

    occupancy = np.zeros((nx, ny, nz), dtype=bool)
    index = build_index(triangles, xmin, ymin, index_cell)

    for i in range(nx):
        x = xmin + (i + 0.5) * resolution
        cx = int(math.floor((x - xmin) / index_cell))
        for j in range(ny):
            y = ymin + (j + 0.5) * resolution
            cy = int(math.floor((y - ymin) / index_cell))
            intervals = solid_intervals(column_crossings(triangles, index.get((cx, cy), ()), x, y))
            if not intervals:
                continue
            for k in range(nz):
                z = zmin + (k + 0.5) * resolution
                for lo, hi in intervals:
                    if lo <= z <= hi:
                        occupancy[i, j, k] = True
                        break

    return nx, ny, nz, occupancy


def greedy_boxes(nx, ny, nz, occupancy):
    remaining = occupancy.copy()
    boxes = []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                if not remaining[i, j, k]:
                    continue

                i1 = i
                while i1 + 1 < nx and remaining[i1 + 1, j, k]:
                    i1 += 1

                j1 = j
                while j1 + 1 < ny and remaining[i:i1 + 1, j1 + 1, k].all():
                    j1 += 1

                k1 = k
                while k1 + 1 < nz and remaining[i:i1 + 1, j:j1 + 1, k1 + 1].all():
                    k1 += 1

                remaining[i:i1 + 1, j:j1 + 1, k:k1 + 1] = False
                boxes.append((i, i1, j, j1, k, k1))
    return boxes


def box_geom(bounds, resolution, box, index, attrs):
    xmin, _, ymin, _, zmin, _ = bounds
    i0, i1, j0, j1, k0, k1 = box
    half = [0.5 * (i1 - i0 + 1) * resolution, 0.5 * (j1 - j0 + 1) * resolution,
            0.5 * (k1 - k0 + 1) * resolution]
    center = [xmin + (i0 + i1 + 1) * 0.5 * resolution, ymin + (j0 + j1 + 1) * 0.5 * resolution,
              zmin + (k0 + k1 + 1) * 0.5 * resolution]
    return '    <geom name="field_box_{}" type="box" size="{:.6g} {:.6g} {:.6g}" pos="{:.6g} {:.6g} {:.6g}" {} />'.format(
        index, half[0], half[1], half[2], center[0], center[1], center[2], attrs)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Voxelize an STL into merged MuJoCo box collision geoms.")
    parser.add_argument("--stl", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--resolution", type=float, default=0.02, help="voxel edge length in meters")
    parser.add_argument("--index-cell", type=float, default=0.5)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--translate", default="0 0 0")
    parser.add_argument("--rotate-rpy", default="0 0 0")
    parser.add_argument("--min-boxes-z", type=int, default=0, help="drop boxes spanning fewer than N voxels in z")
    parser.add_argument("--group", default="3")
    parser.add_argument("--contype", default="4")
    parser.add_argument("--conaffinity", default="2")
    parser.add_argument("--condim", default="3")
    parser.add_argument("--friction", default="0.1 0.01 0.0001")
    parser.add_argument("--solref", default="0.01 1")
    parser.add_argument("--solimp", default="0.9 0.95 0.001")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)

    offset = [float(v) for v in args.translate.split()]
    roll, pitch, yaw = (float(v) for v in args.rotate_rpy.split())
    matrix = quat_to_matrix(*rpy_to_quat(roll, pitch, yaw))
    triangles = transform(read_stl(args.stl), args.scale, matrix, offset)

    xs = [v[0] for tri in triangles for v in tri]
    ys = [v[1] for tri in triangles for v in tri]
    zs = [v[2] for tri in triangles for v in tri]
    bounds = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    nx, ny, nz, occupancy = voxelize(triangles, bounds, args.resolution, args.index_cell)
    filled = int(occupancy.sum())

    boxes = greedy_boxes(nx, ny, nz, occupancy)
    if args.min_boxes_z > 1:
        boxes = [b for b in boxes if (b[5] - b[4] + 1) >= args.min_boxes_z]

    attrs = 'group="{}" contype="{}" conaffinity="{}" condim="{}" friction="{}" solref="{}" solimp="{}"'.format(
        args.group, args.contype, args.conaffinity, args.condim, args.friction, args.solref, args.solimp)

    lines = ['<mujoco model="{}">'.format(os.path.splitext(os.path.basename(args.out))[0]), "  <worldbody>"]
    for index, box in enumerate(boxes):
        lines.append(box_geom(bounds, args.resolution, box, index, attrs))
    lines.append("  </worldbody>")
    lines.append("</mujoco>")

    with open(args.out, "w") as handle:
        handle.write("\n".join(lines) + "\n")

    print("wrote {} ({} boxes)".format(args.out, len(boxes)))
    print("voxel grid: {} x {} x {} @ {} m  ->  {} filled voxels".format(nx, ny, nz, args.resolution, filled))
    print("footprint: x {:.3f}..{:.3f}  y {:.3f}..{:.3f}  z {:.3f}..{:.3f}".format(*bounds))
    print("compression: {} -> {} boxes".format(filled, len(boxes)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
