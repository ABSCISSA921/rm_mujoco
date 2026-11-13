#!/usr/bin/env python3
import math
import sys


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def main():
    if len(sys.argv) != 7:
        print("usage: rpy_to_mujoco_qpos.py x y z roll pitch yaw", file=sys.stderr)
        return 1

    x, y, z, roll, pitch, yaw = (float(value) for value in sys.argv[1:])
    w, qx, qy, qz = rpy_to_quat(roll, pitch, yaw)

    print("{:.6f} {:.6f} {:.6f} {:.9f} {:.9f} {:.9f} {:.9f}".format(x, y, z, w, qx, qy, qz))
    return 0


if __name__ == "__main__":
    sys.exit(main())
