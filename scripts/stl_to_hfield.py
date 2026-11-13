#!/usr/bin/env python3
import argparse
import array
import math
import os
import struct
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rpy_to_mujoco_qpos import rpy_to_quat


def read_binary_stl(path):
    with open(path, "rb") as handle:
        handle.read(80)
        count = struct.unpack("<I", handle.read(4))[0]
        if os.path.getsize(path) != 84 + count * 50:
            return None
        triangles = []
        for _ in range(count):
            record = struct.unpack("<12fH", handle.read(50))
            triangles.append((record[3:6], record[6:9], record[9:12]))
        return triangles


def read_ascii_stl(path):
    triangles = []
    vertices = []
    with open(path) as handle:
        for line in handle:
            parts = line.split()
            if parts and parts[0] == "vertex":
                vertices.append(tuple(float(v) for v in parts[1:4]))
                if len(vertices) == 3:
                    triangles.append(tuple(vertices))
                    vertices = []
    return triangles


def read_stl(path):
    triangles = read_binary_stl(path)
    if triangles is None:
        triangles = read_ascii_stl(path)
    if not triangles:
        raise SystemExit("no triangles found in {}".format(path))
    return triangles


def quat_to_matrix(w, x, y, z):
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]


def transform(triangles, scale, matrix, offset):
    result = []
    for triangle in triangles:
        moved = []
        for vertex in triangle:
            local = [vertex[i] * scale for i in range(3)]
            moved.append(tuple(
                sum(matrix[r][c] * local[c] for c in range(3)) + offset[r] for r in range(3)))
        result.append(tuple(moved))
    return result


def build_index(triangles, xmin, ymin, cell):
    index = {}
    for i, (a, b, c) in enumerate(triangles):
        lo_x = min(a[0], b[0], c[0])
        hi_x = max(a[0], b[0], c[0])
        lo_y = min(a[1], b[1], c[1])
        hi_y = max(a[1], b[1], c[1])
        for cy in range(int(math.floor((lo_y - ymin) / cell)), int(math.floor((hi_y - ymin) / cell)) + 1):
            for cx in range(int(math.floor((lo_x - xmin) / cell)), int(math.floor((hi_x - xmin) / cell)) + 1):
                index.setdefault((cx, cy), []).append(i)
    return index


def surface_height(triangles, candidates, x, y):
    best = None
    for i in candidates:
        a, b, c = triangles[i]
        determinant = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(determinant) < 1e-12:
            continue
        l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / determinant
        l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / determinant
        l3 = 1.0 - l1 - l2
        if l1 < -1e-9 or l2 < -1e-9 or l3 < -1e-9:
            continue
        z = l1 * a[2] + l2 * b[2] + l3 * c[2]
        if best is None or z > best:
            best = z
    return best


def sample(triangles, xmin, xmax, ymin, ymax, resolution, index_cell):
    ncol = int(round((xmax - xmin) / resolution)) + 1
    nrow = int(round((ymax - ymin) / resolution)) + 1
    index = build_index(triangles, xmin, ymin, index_cell)

    heights = array.array("f")
    misses = 0
    for r in range(nrow):
        y = ymin + (ymax - ymin) * r / (nrow - 1)
        cy = int(math.floor((y - ymin) / index_cell))
        for c in range(ncol):
            x = xmin + (xmax - xmin) * c / (ncol - 1)
            cx = int(math.floor((x - xmin) / index_cell))
            value = surface_height(triangles, index.get((cx, cy), ()), x, y)
            if value is None:
                misses += 1
                value = 0.0
            heights.append(value)

    if misses:
        print("warning: {} of {} samples hit no triangle, filled with 0".format(misses, nrow * ncol),
              file=sys.stderr)
    return nrow, ncol, heights


def write_hfield(path, nrow, ncol, heights):
    if sys.byteorder != "little":
        heights.byteswap()
    with open(path, "wb") as handle:
        handle.write(struct.pack("<ii", nrow, ncol))
        handle.write(memoryview(heights).cast("B"))


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Rasterize an STL surface into a MuJoCo hfield.")
    parser.add_argument("--stl", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--scale", type=float, default=1.0, help="uniform scale applied to stl vertices")
    parser.add_argument("--translate", default="0 0 0", help="x y z applied after scaling and rotation")
    parser.add_argument("--rotate-rpy", default="0 0 0", help="roll pitch yaw in radians")
    parser.add_argument("--resolution", type=float, default=0.05, help="grid spacing in meters")
    parser.add_argument("--index-cell", type=float, default=0.25, help="spatial index cell size")
    parser.add_argument("--base-depth", type=float, default=0.5, help="hfield size[3], depth below z=0")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)

    scale = args.scale
    offset = [float(v) for v in args.translate.split()]
    roll, pitch, yaw = (float(v) for v in args.rotate_rpy.split())
    matrix = quat_to_matrix(*rpy_to_quat(roll, pitch, yaw))

    triangles = transform(read_stl(args.stl), scale, matrix, offset)

    xs = [v[0] for tri in triangles for v in tri]
    ys = [v[1] for tri in triangles for v in tri]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)

    nrow, ncol, heights = sample(triangles, xmin, xmax, ymin, ymax, args.resolution, args.index_cell)

    zmin, zmax = min(heights), max(heights)
    write_hfield(args.out, nrow, ncol, heights)

    size_x = (xmax - xmin) / 2.0
    size_y = (ymax - ymin) / 2.0
    center_x = (xmin + xmax) / 2.0
    center_y = (ymin + ymax) / 2.0

    print("wrote {} ({} rows x {} cols, {:.1f} KB)".format(
        args.out, nrow, ncol, os.path.getsize(args.out) / 1024.0))
    print("height range: {:.4f} .. {:.4f} m".format(zmin, zmax))
    print("footprint: x {:.3f}..{:.3f}  y {:.3f}..{:.3f}".format(xmin, xmax, ymin, ymax))
    print("")
    print("hfield size=\"{:.6g} {:.6g} {:.6g} {:.6g}\"".format(
        size_x, size_y, zmax - zmin, args.base_depth))
    print("geom     pos=\"{:.6g} {:.6g} {:.6g}\"".format(center_x, center_y, zmin))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
