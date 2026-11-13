#!/usr/bin/env python3
import argparse
import atexit
import os
import re
import shlex
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import OrderedDict

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rpy_to_mujoco_qpos import rpy_to_quat

JOINT_TYPES = {"revolute": "hinge", "continuous": "hinge", "prismatic": "slide"}


def parse_floats(text, count):
    values = [float(v) for v in text.split()]
    if len(values) != count:
        raise ValueError("expected {} values, got {}: {!r}".format(count, len(values), text))
    return values


def fmt(values):
    return " ".join("{:.9g}".format(v) for v in values)


def origin_of(element):
    origin = element.find("origin")
    if origin is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    return parse_floats(origin.get("xyz", "0 0 0"), 3), parse_floats(origin.get("rpy", "0 0 0"), 3)


def normalize_rgba(text):
    values = parse_floats(text, 4)
    if max(values[:3]) > 1.0:
        values = [v / 255.0 for v in values]
    return values


def inertia_is_positive_definite(ixx, iyy, izz, ixy, ixz, iyz):
    if ixx <= 0.0:
        return False
    if ixx * iyy - ixy * ixy <= 0.0:
        return False
    determinant = (ixx * iyy * izz + 2.0 * ixy * ixz * iyz - ixx * iyz * iyz - iyy * ixz * ixz -
                   izz * ixy * ixy)
    return determinant > 0.0


def sanitize(name):
    return re.sub(r"[^A-Za-z0-9_]", "_", name)


class MeshLibrary:
    def __init__(self, out_dir, package_roots):
        self.out_dir = os.path.abspath(out_dir)
        self.package_roots = package_roots
        self.assets = OrderedDict()

    def resolve(self, uri):
        if uri.startswith("package://"):
            package, _, sub = uri[len("package://"):].partition("/")
            root = self.package_roots.get(package)
            if root is None:
                raise ValueError("no root known for package '{}' referenced by '{}'".format(package, uri))
            target = os.path.join(root, sub)
        elif uri.startswith("file://"):
            target = uri[len("file://"):]
        else:
            target = os.path.join(self.out_dir, uri)
        return os.path.relpath(os.path.abspath(target), self.out_dir)

    def add(self, uri, scale):
        path = self.resolve(uri)
        key = (path, tuple(scale))
        if key in self.assets:
            return self.assets[key]

        base = sanitize(os.path.splitext(os.path.basename(path))[0])
        taken = set(self.assets.values())
        name = base
        index = 1
        while name in taken:
            name = "{}_{}".format(base, index)
            index += 1

        self.assets[key] = name
        return name

    def lines(self):
        out = []
        for (path, scale), name in self.assets.items():
            out.append('    <mesh name="{}" file="{}" scale="{}"/>'.format(name, path, fmt(scale)))
        return out


class Converter:
    def __init__(self, args):
        self.args = args
        self.meshes = MeshLibrary(args.out_dir, args.package_roots)
        self.materials = OrderedDict()
        self.body_names = set()
        self.warnings = []
        self.imu_sites = []

    def load_urdf(self):
        root = ET.parse(self.args.urdf).getroot()
        self.root = root
        self.links = OrderedDict((link.get("name"), link) for link in root.findall("link"))
        self.child_joints = {}
        for joint in root.findall("joint"):
            child = joint.find("child").get("link")
            parent = joint.find("parent").get("link")
            if child in self.child_joints:
                raise ValueError("link '{}' has multiple parent joints".format(child))
            if child not in self.links or parent not in self.links:
                raise ValueError("joint '{}' references an unknown link".format(joint.get("name")))
            self.child_joints[child] = joint

        roots = [name for name in self.links if name not in self.child_joints]
        if len(roots) != 1:
            raise ValueError("expected exactly one root link, found {}".format(roots))
        self.root_link = roots[0]

    def collect_materials(self):
        for material in self.root.iter("material"):
            name = material.get("name")
            color = material.find("color")
            if name and color is not None and name not in self.materials:
                self.materials[name] = normalize_rgba(color.get("rgba"))

    def geom_geometry(self, geometry):
        mesh = geometry.find("mesh")
        if mesh is not None:
            scale = parse_floats(mesh.get("scale", "1 1 1"), 3)
            name = self.meshes.add(mesh.get("filename"), scale)
            return 'type="mesh" mesh="{}"'.format(name)

        box = geometry.find("box")
        if box is not None:
            size = parse_floats(box.get("size"), 3)
            return 'type="box" size="{}"'.format(fmt([s / 2.0 for s in size]))

        cylinder = geometry.find("cylinder")
        if cylinder is not None:
            radius = float(cylinder.get("radius"))
            length = float(cylinder.get("length"))
            return 'type="cylinder" size="{:.9g} {:.9g}"'.format(radius, length / 2.0)

        sphere = geometry.find("sphere")
        if sphere is not None:
            return 'type="sphere" size="{:.9g}"'.format(float(sphere.get("radius")))

        capsule = geometry.find("capsule")
        if capsule is not None:
            radius = float(capsule.get("radius"))
            length = float(capsule.get("length"))
            return 'type="capsule" size="{:.9g} {:.9g}"'.format(radius, length / 2.0)

        raise ValueError("unsupported geometry element")

    def geom_lines(self, link_name, element, kind, index, cls):
        geometry = element.find("geometry")
        if geometry is None:
            return []

        xyz, rpy = origin_of(element)
        attrs = [self.geom_geometry(geometry), 'name="{}_{}_{}"'.format(link_name, kind, index)]

        if any(v != 0.0 for v in xyz):
            attrs.append('pos="{}"'.format(fmt(xyz)))
        if any(v != 0.0 for v in rpy):
            attrs.append('quat="{}"'.format(fmt(rpy_to_quat(*rpy))))

        if kind == "visual":
            material = element.find("material")
            material_name = None
            if material is not None:
                if material.find("color") is not None:
                    material_name = material.get("name") or "{}_{}_material".format(link_name, index)
                    self.materials.setdefault(material_name, normalize_rgba(material.find("color").get("rgba")))
                elif material.get("name") in self.materials:
                    material_name = material.get("name")
            if material_name:
                attrs.append('material="{}"'.format(material_name))

        return ['<geom class="{}" {}/>'.format(cls, " ".join(attrs))]

    def inertial_line(self, link):
        inertial = link.find("inertial")
        if inertial is None:
            self.warnings.append("link '{}' has no <inertial>, MuJoCo will derive it from geometry".format(
                link.get("name")))
            return None

        xyz, rpy = origin_of(inertial)
        scale = self.args.body_mass_scale.get(link.get("name"), self.args.mass_scale)
        mass = float(inertial.find("mass").get("value")) * scale
        inertia = inertial.find("inertia")
        full = [
            float(inertia.get("ixx", "0")) * scale, float(inertia.get("iyy", "0")) * scale,
            float(inertia.get("izz", "0")) * scale, float(inertia.get("ixy", "0")) * scale,
            float(inertia.get("ixz", "0")) * scale, float(inertia.get("iyz", "0")) * scale,
        ]

        attrs = ['pos="{}"'.format(fmt(xyz)), 'mass="{:.9g}"'.format(mass)]
        if any(v != 0.0 for v in rpy):
            attrs.append('quat="{}"'.format(fmt(rpy_to_quat(*rpy))))

        if inertia_is_positive_definite(*full):
            attrs.append('fullinertia="{}"'.format(fmt(full)))
        else:
            floor = max(mass * 1e-6, 1e-9)
            diagonal = [max(abs(v), floor) for v in full[:3]]
            self.warnings.append(
                "link '{}' has a non positive definite inertia tensor, falling back to diagonal {}".format(
                    link.get("name"), fmt(diagonal)))
            attrs.append('diaginertia="{}"'.format(fmt(diagonal)))

        return "<inertial {}/>".format(" ".join(attrs))

    def joint_line(self, joint):
        attrs = ['name="{}"'.format(joint.get("name")), 'type="{}"'.format(JOINT_TYPES[joint.get("type")])]

        axis = joint.find("axis")
        if axis is not None:
            attrs.append('axis="{}"'.format(fmt(parse_floats(axis.get("xyz", "1 0 0"), 3))))

        dynamics = joint.find("dynamics")
        if dynamics is not None:
            if dynamics.get("damping") is not None:
                damping = float(dynamics.get("damping")) * self.args.damping_scale
                if damping != 0.0:
                    attrs.append('damping="{:.9g}"'.format(damping))
            if dynamics.get("friction") is not None:
                frictionloss = float(dynamics.get("friction")) * self.args.friction_scale
                if frictionloss != 0.0:
                    attrs.append('frictionloss="{:.9g}"'.format(frictionloss))

        limit = joint.find("limit")
        if joint.get("type") == "revolute" and limit is not None:
            lower, upper = limit.get("lower"), limit.get("upper")
            if lower is not None and upper is not None:
                bounds = [float(lower), float(upper)]
                if max(abs(bounds[0]), abs(bounds[1])) < 1e6:
                    attrs.append('range="{}"'.format(fmt(bounds)))

        return "<joint {}/>".format(" ".join(attrs))

    def body_lines(self, link_name, depth):
        indent = "  " * depth
        link = self.links[link_name]
        self.body_names.add(link_name)

        attrs = ['name="{}"'.format(link_name)]
        joint = self.child_joints.get(link_name)
        if joint is not None:
            xyz, rpy = origin_of(joint)
            if any(v != 0.0 for v in xyz):
                attrs.append('pos="{}"'.format(fmt(xyz)))
            if any(v != 0.0 for v in rpy):
                attrs.append('quat="{}"'.format(fmt(rpy_to_quat(*rpy))))

        lines = ["{}<body {}>".format(indent, " ".join(attrs))]

        if joint is None:
            lines.append('{}  <freejoint name="{}_free_joint"/>'.format(indent, link_name))
        elif joint.get("type") in JOINT_TYPES:
            lines.append("{}{}".format(indent + "  ", self.joint_line(joint)))

        inertial = self.inertial_line(link)
        if inertial:
            lines.append("{}{}".format(indent + "  ", inertial))

        for index, element in enumerate(link.findall("visual")):
            for line in self.geom_lines(link_name, element, "visual", index, "visual"):
                lines.append("{}{}".format(indent + "  ", line))
        if link_name in self.args.visual_collision:
            visuals = link.findall("visual")
            if visuals:
                for line in self.geom_lines(link_name, visuals[0], "collision", 0, "collision"):
                    lines.append("{}{}".format(indent + "  ", line))
            else:
                self.warnings.append("body '{}' configured for visual derived collision but has no visual".format(
                    link_name))
        else:
            for index, element in enumerate(link.findall("collision")):
                is_roller = bool(self.args.roller_pattern.search(link_name))
                cls = "roller" if is_roller else "collision"
                if is_roller and self.args.roller_shapes:
                    geometry = element.find("geometry")
                    if geometry is None or len(geometry) == 0:
                        continue
                    if geometry[0].tag not in self.args.roller_shapes:
                        continue
                for line in self.geom_lines(link_name, element, "collision", index, cls):
                    lines.append("{}{}".format(indent + "  ", line))

        if link_name in self.args.imus:
            site = "{}_site".format(link_name)
            self.imu_sites.append(site)
            lines.append('{}<site name="{}"/>'.format(indent + "  ", site))

        for child in self.links:
            child_joint = self.child_joints.get(child)
            if child_joint is not None and child_joint.find("parent").get("link") == link_name:
                lines.extend(self.body_lines(child, depth + 1))

        lines.append("{}</body>".format(indent))
        return lines

    def build(self):
        self.load_urdf()
        self.collect_materials()

        unknown = sorted(name for name in self.args.imus if name not in self.links)
        for name in unknown:
            self.warnings.append("imu frame '{}' is not a link in the urdf, no sensors will be added".format(name))

        body_lines = self.body_lines(self.root_link, 2)

        lines = ['<mujoco model="{}">'.format(self.args.model_name)]
        lines.append('  <compiler angle="radian" meshdir="."/>')
        lines.append('  <option timestep="{}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic"/>'.format(
            self.args.timestep))
        lines.append("")
        lines.append("  <default>")
        lines.append('    <default class="visual">')
        lines.append('      <geom group="2" contype="0" conaffinity="0" density="0"/>')
        lines.append("    </default>")
        lines.append('    <default class="collision">')
        lines.append('      <geom group="3" contype="2" conaffinity="4" condim="3" friction="{}" solref="{}" '
                     'solimp="{}"/>'.format(self.args.collision_friction, self.args.collision_solref,
                                            self.args.collision_solimp))
        lines.append("    </default>")
        lines.append('    <default class="roller">')
        lines.append('      <geom group="3" contype="2" conaffinity="4" condim="3" friction="{}" solref="{}" '
                     'solimp="{}"/>'.format(self.args.roller_friction, self.args.roller_solref,
                                            self.args.roller_solimp))
        lines.append("    </default>")
        lines.append("  </default>")
        lines.append("")

        asset_lines = []
        for name, rgba in self.materials.items():
            asset_lines.append('    <material name="{}" rgba="{}"/>'.format(sanitize(name), fmt(rgba)))
        asset_lines.extend(self.meshes.lines())
        if asset_lines:
            lines.append("  <asset>")
            lines.extend(asset_lines)
            lines.append("  </asset>")
            lines.append("")

        lines.append("  <worldbody>")
        lines.extend(body_lines)
        lines.append("  </worldbody>")

        if self.imu_sites:
            lines.append("")
            lines.append("  <sensor>")
            for site in self.imu_sites:
                frame = site[:-len("_site")]
                lines.append('    <framequat name="{}_quat" objtype="site" objname="{}"/>'.format(frame, site))
                lines.append('    <gyro name="{}_gyro" site="{}"/>'.format(frame, site))
                lines.append('    <accelerometer name="{}_acc" site="{}"/>'.format(frame, site))
            lines.append("  </sensor>")

        lines.append("</mujoco>")
        return "\n".join(lines) + "\n"


def expand_xacro(path, xacro_args):
    if not xacro_args:
        return path
    descriptor, out = tempfile.mkstemp(suffix=".urdf", prefix="urdf_to_mjcf_")
    os.close(descriptor)
    atexit.register(os.unlink, out)
    command = ["xacro", path] + shlex.split(xacro_args)
    with open(out, "w") as handle:
        subprocess.check_call(command, stdout=handle)
    return out


def resolve_package_roots(packages):
    roots = {}
    for package in packages:
        try:
            roots[package] = subprocess.check_output(["rospack", "find", package]).decode().strip()
        except (OSError, subprocess.CalledProcessError):
            raise SystemExit("could not locate ros package '{}', is the workspace sourced?".format(package))
    return roots


def collect_packages(urdf_path):
    packages = set()
    pattern = re.compile(r"package://([^/]+)/")
    with open(urdf_path) as handle:
        for match in pattern.finditer(handle.read()):
            packages.add(match.group(1))
    return sorted(packages)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Convert an expanded URDF robot description into MJCF.")
    parser.add_argument("--urdf", required=True, help="input urdf file")
    parser.add_argument("--out", required=True, help="output mjcf file")
    parser.add_argument("--model-name", default=None, help="model name, defaults to the output file stem")
    parser.add_argument("--imu", action="append", default=[], dest="imu",
                        help="link carrying an imu, may be given multiple times")
    parser.add_argument("--roller-pattern", default=r"roller",
                        help="regex matching links whose collision geoms get the roller material class")
    parser.add_argument("--roller-shapes", default="", dest="roller_shapes_raw",
                        help="comma separated urdf collision shapes kept on roller links, empty keeps all")
    parser.add_argument("--collision-from-visual", default="", dest="visual_collision_raw",
                        help="comma separated body names whose collision geoms are replaced by their visual mesh")
    parser.add_argument("--timestep", default="0.001")
    parser.add_argument("--friction-scale", type=float, default=0.0,
                        help="multiplier applied to urdf joint dynamics friction (frictionloss)")
    parser.add_argument("--damping-scale", type=float, default=1.0,
                        help="multiplier applied to urdf joint dynamics damping")
    parser.add_argument("--mass-scale", type=float, default=1.0,
                        help="multiplier applied to body mass and inertia tensor")
    parser.add_argument("--body-mass-scale", action="append", default=[], dest="body_mass_scale_raw",
                        metavar="BODY=FACTOR",
                        help="per-body mass and inertia multiplier, takes precedence over --mass-scale")
    parser.add_argument("--collision-friction", default="0.1 0.005 0.0001")
    parser.add_argument("--collision-solref", default="0.02 1")
    parser.add_argument("--collision-solimp", default="0.9 0.95 0.001")
    parser.add_argument("--roller-friction", default="2.0 0.01 0.0001")
    parser.add_argument("--roller-solref", default="0.01 1")
    parser.add_argument("--roller-solimp", default="0.9 0.95 0.001")
    parser.add_argument("--xacro-args", default="", help="arguments passed to xacro when --urdf is a .xacro file")
    args = parser.parse_args(argv)

    args.out_dir = os.path.dirname(os.path.abspath(args.out))
    args.model_name = args.model_name or os.path.splitext(os.path.basename(args.out))[0]
    args.imus = set(args.imu)
    args.roller_pattern = re.compile(args.roller_pattern)
    args.roller_shapes = set(s.strip() for s in args.roller_shapes_raw.split(",") if s.strip())
    args.visual_collision = set(s.strip() for s in args.visual_collision_raw.split(",") if s.strip())

    if args.mass_scale <= 0.0:
        parser.error("--mass-scale must be positive")

    args.body_mass_scale = {}
    for item in args.body_mass_scale_raw:
        name, _, factor = item.partition("=")
        if not name or not factor:
            parser.error("--body-mass-scale expects BODY=FACTOR, got '{}'".format(item))
        value = float(factor)
        if value <= 0.0:
            parser.error("--body-mass-scale factor must be positive")
        args.body_mass_scale[name] = value

    return args


def main(argv):
    args = parse_args(argv)
    args.urdf = expand_xacro(args.urdf, args.xacro_args)
    args.package_roots = resolve_package_roots(collect_packages(args.urdf))

    converter = Converter(args)
    document = converter.build()

    with open(args.out, "w") as handle:
        handle.write(document)

    for warning in converter.warnings:
        print("warning: {}".format(warning), file=sys.stderr)
    print("wrote {} ({} bodies, {} meshes)".format(args.out, len(converter.body_names), len(converter.meshes.assets)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
