#!/usr/bin/env python3
"""Build the agreed ROS/MuJoCo model without modifying its three source descriptions.

Run after sourcing mujoco_env.sh. Geometry/closed chains: series_legged;
IMU positions, gimbal, transmissions and leg limits: series_legged2;
IMU rotations retain nominal mounting directions without small installation offsets;
individual rigid-body mass, COM, inertia and meshes: serial_leg_complete.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def numbers(text):
    return np.fromstring(text, sep=' ')


def fmt(values):
    return ' '.join(format(float(x), '.16g') for x in values)


def rotation_matrix(rotation):
    # Ubuntu 20.04 scipy uses as_dcm; newer scipy calls it as_matrix.
    return rotation.as_matrix() if hasattr(rotation, 'as_matrix') else rotation.as_dcm()


def rotation_from_matrix(matrix):
    return Rotation.from_matrix(matrix) if hasattr(Rotation, 'from_matrix') else Rotation.from_dcm(matrix)


def transform(pos=(0, 0, 0), rotation=None):
    result = np.eye(4)
    result[:3, 3] = pos
    if rotation is not None:
        result[:3, :3] = rotation_matrix(rotation)
    return result


def urdf_pose(element):
    if element is None:
        return np.eye(4)
    return transform(numbers(element.get('xyz', '0 0 0')),
                     Rotation.from_euler('xyz', numbers(element.get('rpy', '0 0 0'))))


def mj_pose(element):
    quat = numbers(element.get('quat', '1 0 0 0'))
    return transform(numbers(element.get('pos', '0 0 0')), Rotation.from_quat(quat[[1, 2, 3, 0]]))


def set_mj_pose(element, pose):
    quat = rotation_from_matrix(pose[:3, :3]).as_quat()
    element.set('pos', fmt(pose[:3, 3]))
    element.set('quat', fmt(quat[[3, 0, 1, 2]]))


def set_urdf_pose(element, pose):
    element.set('xyz', fmt(pose[:3, 3]))
    element.set('rpy', fmt(rotation_from_matrix(pose[:3, :3]).as_euler('xyz')))


def full_inertia(values):
    xx, yy, zz, xy, xz, yz = values
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


def inertia_values(matrix):
    return matrix[0, 0], matrix[1, 1], matrix[2, 2], matrix[0, 1], matrix[0, 2], matrix[1, 2]


def align_anchors(old_a, old_b, new_a, new_b):
    """Map a CAD link's pivot pair to the ROS link's pivot pair.

    CAD's left is -Y. A proper Z half-turn puts it at ROS +Y and reverses
    its hinge axis; the extra Y rotation aligns the longitudinal link axis.
    Never reflect or scale an inertia tensor to accomplish this conversion.
    """
    half_turn = rotation_matrix(Rotation.from_euler('z', math.pi))
    old_vector = half_turn @ (old_b - old_a)
    new_vector = new_b - new_a
    angle = math.atan2(new_vector[0], new_vector[2]) - math.atan2(old_vector[0], old_vector[2])
    rotation = rotation_matrix(Rotation.from_euler('y', angle)) @ half_turn
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = new_a - rotation @ old_a
    return pose, float(np.linalg.norm(rotation @ (old_b - old_a) - new_vector))


def write_xml(path, root):
    # ElementTree.indent is not available in Ubuntu 20.04 Python 3.8.
    def indent(node, level=0):
        if len(node):
            node.text = '\n' + '  ' * (level + 1)
            for child in node:
                indent(child, level + 1)
            node[-1].tail = '\n' + '  ' * level
        node.tail = '\n' + '  ' * level
    indent(root)
    ET.ElementTree(root).write(str(path), encoding='utf-8', xml_declaration=True)


def prepare(package, description):
    destination = package / 'models/series_legged2'
    destination.mkdir(parents=True, exist_ok=True)
    cad_path = package / 'models/serial_leg_complete/robot_passive_limits.xml'
    cad = ET.parse(str(cad_path)).getroot()
    sources = {}
    for name in ('series_legged', 'series_legged2'):
        path = description / 'urdf' / name / (name + '.urdf.xacro')
        sources[name] = ET.fromstring(subprocess.check_output(
            ['xacro', str(path), 'use_simulation:=false'], text=True))
    original, reference = sources['series_legged'], sources['series_legged2']
    joints2 = {j.get('name'): j for j in reference.findall('joint')}
    cad_bodies = {b.get('name'): b for b in cad.findall('.//body')}
    robot = copy.deepcopy(original)
    robot.set('name', 'series_legged2_mujoco_complete')
    active = {side + '_' + joint + '_joint' for side in ('left', 'right') for joint in ('hip', 'knee', 'wheel')}
    active.update(('yaw_joint', 'pitch_joint'))
    passive = {side + '_' + joint + '_joint' for side in ('left', 'right')
               for joint in ('calf', 'mid', 'long', 'short')}

    # Use the detailed linkage tree, not series_legged2's simplified serial leg.
    for joint in list(robot.findall('joint')):
        name = joint.get('name')
        leg = name in active - {'yaw_joint', 'pitch_joint'} or name in passive
        if not leg and name in joints2:
            robot.remove(joint)
            joint = copy.deepcopy(joints2[name])
            robot.append(joint)
        if leg:
            source_name = name.replace('_calf_joint', '_knee_joint')
            source_joint = joints2.get(source_name)
            if source_joint is not None:
                for tag in ('limit', 'safety_controller', 'dynamics'):
                    for old in list(joint.findall(tag)):
                        joint.remove(old)
                    if source_joint.find(tag) is not None:
                        joint.append(copy.deepcopy(source_joint.find(tag)))
        if name not in active | passive:
            joint.set('type', 'fixed')
            for tag in ('limit', 'axis', 'dynamics', 'safety_controller', 'mimic', 'calibration'):
                for old in list(joint.findall(tag)):
                    joint.remove(old)
    for element in list(robot):
        if element.tag in ('transmission', 'gazebo'):
            robot.remove(element)
    for transmission in reference.findall('transmission'):
        if transmission.find('joint').get('name') in active:
            robot.append(copy.deepcopy(transmission))

    joints = {j.get('name'): j for j in robot.findall('joint')}
    # Ideal mounting for simulation; gimbal_imu is already nominal at (0, 0, -pi/2).
    joints['base_imu_joint'].find('origin').set('rpy', f'0 {math.pi} 0')
    links = {link.get('name'): link for link in robot.findall('link')}
    children = {}
    for joint in joints.values():
        children.setdefault(joint.find('parent').get('link'), []).append(joint)

    def forward(q):
        poses = {'base_link': np.eye(4)}
        def visit(parent):
            for joint in children.get(parent, []):
                name = joint.get('name')
                pose = urdf_pose(joint.find('origin'))
                if name in active | passive:
                    axis = numbers(joint.find('axis').get('xyz'))
                    pose = pose @ transform(rotation=Rotation.from_rotvec(axis * q.get(name, 0.0)))
                child = joint.find('child').get('link')
                poses[child] = poses[parent] @ pose
                visit(child)
        visit('base_link')
        return poses

    zero_poses = forward({})
    base_pose = transform(rotation=Rotation.from_euler('z', math.pi))
    cad_hips = np.mean([numbers(cad_bodies[s + '_thigh'].get('pos')) for s in ('left', 'right')], axis=0)
    ros_hips = np.mean([urdf_pose(joints[s + '_hip_joint'].find('origin'))[:3, 3]
                       for s in ('left', 'right')], axis=0)
    base_pose[:3, 3] = ros_hips - base_pose[:3, :3] @ cad_hips
    frames = {'base_link': ('base', base_pose),
              'yaw': ('yaw', transform(rotation=Rotation.from_euler('z', math.pi))),
              'pitch': ('pitch', transform(rotation=Rotation.from_euler('z', math.pi)))}
    anchor_errors = {}
    closure_joints = {j.get('name'): j for g in original.findall('gazebo') for j in g.findall('joint')}
    for side in ('left', 'right'):
        def point(body, tag, name=None):
            element = cad_bodies[side + '_' + body].find(tag if name is None else tag + "[@name='" + side + '_' + name + "']")
            return numbers(element.get('pos', '0 0 0'))
        def offset(joint):
            return urdf_pose(joints[side + '_' + joint + '_joint'].find('origin'))[:3, 3]
        z = np.zeros(3)
        wheel_planar = point('calf', 'body', 'wheel').copy()
        wheel_planar[1] = 0
        target_wheel = offset('wheel').copy()
        target_wheel[1] = 0
        definitions = [
            ('thigh', 'thigh', z, point('thigh', 'body', 'calf'), z, offset('calf')),
            ('calf', 'calf', z, wheel_planar, z, target_wheel),
            ('knee_link', 'output', z, point('output', 'site', 'output_mid_1'), z, offset('mid')),
            ('mid_link', 'midbar', point('midbar', 'site', 'output_mid_2'), z, z, offset('long')),
            ('long_link', 'longbar', point('longbar', 'body', 'midbar'),
             point('longbar', 'body', 'shortbar'), z, offset('short')),
            ('short_link', 'shortbar', z, point('shortbar', 'site', 'calf_short_2'), z,
             numbers(closure_joints[side + '_connect_joint2'].find('pose').text)[:3]),
        ]
        for target, source, old_a, old_b, new_a, new_b in definitions:
            pose, error = align_anchors(old_a, old_b, new_a, new_b)
            frames[side + '_' + target] = (side + '_' + source, pose)
            anchor_errors[side + '_' + target] = error
        # Wheel origin in ROS is the tire centre, not the CAD shaft shoulder.
        wheel_rotation = frames[side + '_calf'][1][:3, :3]
        wheel = np.eye(4)
        wheel[:3, :3] = wheel_rotation
        center = point('wheel', 'geom', 'wheel_collision')
        wheel[:3, 3] = -wheel_rotation @ center
        frames[side + '_wheel'] = (side + '_wheel', wheel)

    model = ET.Element('mujoco', model='series_legged2_complete_ros')
    ET.SubElement(model, 'compiler', angle='radian', inertiafromgeom='false', fusestatic='false',
                  meshdir='../serial_leg_complete/meshes')
    ET.SubElement(model, 'option', timestep='0.001', gravity='0 0 -9.81', integrator='implicitfast',
                  solver='Newton', iterations='100', tolerance='1e-10', cone='elliptic')
    default = ET.SubElement(model, 'default')
    ET.SubElement(default, 'geom', solref='0.004 1', solimp='0.95 0.99 0.001', density='0')
    model.append(copy.deepcopy(cad.find('asset')))
    assets = {a.get('name'): a for a in model.find('asset')}
    custom = ET.SubElement(model, 'custom')
    ET.SubElement(custom, 'numeric', name='leg_spring_coefficients', data='-80.91 1408.375 -3091.28 2094.45')
    world = ET.SubElement(model, 'worldbody')
    ET.SubElement(world, 'light', name='light', pos='0 -2 4', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='20 20 .1', contype='2', conaffinity='1',
                  friction='1 .005 .0001', rgba='.8 .8 .8 1')
    bodies = {}
    def make_body(name, parent, joint=None):
        body = ET.SubElement(parent, 'body', name=name)
        bodies[name] = body
        if joint is None:
            body.set('pos', '0 0 .5')
            ET.SubElement(body, 'freejoint', name='base_free')
        else:
            set_mj_pose(body, urdf_pose(joint.find('origin')))
            if joint.get('type') != 'fixed':
                hinge = ET.SubElement(body, 'joint', name=joint.get('name'), type='hinge',
                                      axis=joint.find('axis').get('xyz'))
                limit = joint.find('limit')
                bounds = [float(limit.get(key)) for key in ('lower', 'upper')]
                limited = max(abs(x) for x in bounds) < 1e8
                hinge.set('limited', 'true' if limited else 'false')
                if limited:
                    hinge.set('range', fmt(bounds))
                dynamics = joint.find('dynamics')
                if dynamics is not None:
                    hinge.set('damping', dynamics.get('damping', '0'))
                    hinge.set('frictionloss', dynamics.get('friction', '0'))
        # Helper frames and fixed shooter frames carry no extra mass: CAD already
        # includes those assemblies. Do not double-count old URDF inertials.
        link = links[name]
        for tag in ('inertial', 'visual', 'collision'):
            for element in list(link.findall(tag)):
                link.remove(element)
        if name in frames:
            source_name, pose = frames[name]
            source = cad_bodies[source_name]
            source_inertia = source.find('inertial')
            com = pose[:3, :3] @ numbers(source_inertia.get('pos')) + pose[:3, 3]
            inertia = pose[:3, :3] @ full_inertia(numbers(source_inertia.get('fullinertia'))) @ pose[:3, :3].T
            ET.SubElement(body, 'inertial', mass=source_inertia.get('mass'), pos=fmt(com),
                          fullinertia=fmt(inertia_values(inertia)))
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'mass', value=source_inertia.get('mass'))
            ET.SubElement(inertial, 'origin', xyz=fmt(com), rpy='0 0 0')
            ET.SubElement(inertial, 'inertia', **dict(zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'),
                                                       (format(v, '.16g') for v in inertia_values(inertia)))))
            for source_geom in source.findall('geom'):
                geom = copy.deepcopy(source_geom)
                set_mj_pose(geom, pose @ mj_pose(source_geom))
                if name.endswith('_wheel') and geom.get('type') == 'cylinder':
                    # Geometry follows the specified ROS wheel radius/centre.
                    collision = original.find("link[@name='" + name + "']/collision/geometry/cylinder")
                    geom.set('size', fmt([float(collision.get('radius')), float(collision.get('length')) / 2]))
                    geom.set('pos', '0 0 0')
                body.append(geom)
                visual = ET.SubElement(link, 'visual' if geom.get('contype') == '0' else 'collision')
                set_urdf_pose(ET.SubElement(visual, 'origin'), mj_pose(geom))
                geometry = ET.SubElement(visual, 'geometry')
                if geom.get('type') == 'mesh':
                    asset = assets[geom.get('mesh')]
                    ET.SubElement(geometry, 'mesh', filename='package://rm_mujoco/models/serial_leg_complete/meshes/' +
                                  asset.get('file'), scale=asset.get('scale', '1 1 1'))
                elif geom.get('type') == 'box':
                    ET.SubElement(geometry, 'box', size=fmt(numbers(geom.get('size')) * 2))
                elif geom.get('type') == 'cylinder':
                    radius, half_length = numbers(geom.get('size'))
                    ET.SubElement(geometry, 'cylinder', radius=str(radius), length=str(2 * half_length))
        for child_joint in children.get(name, []):
            make_body(child_joint.find('child').get('link'), body, child_joint)
    make_body('base_link', world)

    equality = ET.SubElement(model, 'equality')
    closures = []
    for gazebo in original.findall('gazebo'):
        for joint in gazebo.findall('joint'):
            name = joint.get('name')
            parent, child = joint.find('parent').text, joint.find('child').text
            anchor = numbers(joint.find('pose').text)[:3]
            other = np.linalg.inv(zero_poses[child]) @ zero_poses[parent] @ np.r_[anchor, 1]
            ET.SubElement(bodies[parent], 'site', name=name + '_a', pos=fmt(anchor), size='.003', group='5')
            ET.SubElement(bodies[child], 'site', name=name + '_b', pos=fmt(other[:3]), size='.003', group='5')
            ET.SubElement(equality, 'connect', name=name, site1=name + '_a', site2=name + '_b',
                          solref='0.002 1', solimp='0.999 0.9999 0.0001')
            closures.append((parent, anchor, child, other[:3]))
    assert len(closures) == 4
    tendons = ET.SubElement(model, 'tendon')
    for side in ('left', 'right'):
        hip = urdf_pose(joints[side + '_hip_joint'].find('origin'))[:3, 3].copy()
        wheel = urdf_pose(joints[side + '_wheel_joint'].find('origin'))[:3, 3]
        hip[1] += wheel[1]
        ET.SubElement(bodies['base_link'], 'site', name=side + '_spring_hip', pos=fmt(hip), size='.003', group='5')
        ET.SubElement(bodies[side + '_calf'], 'site', name=side + '_spring_wheel', pos=fmt(wheel), size='.003', group='5')
        # Same planar hip-axis-to-wheel-centre distance as the passive spring.
        limit = ET.SubElement(tendons, 'spatial', name=side + '_leg_length_limit',
                              limited='true', range='0 0.40', group='5',
                              solreflimit='0.002 1', solimplimit='0.999 0.9999 0.0001')
        ET.SubElement(limit, 'site', site=side + '_spring_hip')
        ET.SubElement(limit, 'site', site=side + '_spring_wheel')
    sensors = ET.SubElement(model, 'sensor')
    for name in ('base_imu', 'gimbal_imu'):
        ET.SubElement(bodies[name], 'site', name=name + '_site', size='.004', group='5')
        ET.SubElement(sensors, 'gyro', name=name + '_gyro', site=name + '_site')
        ET.SubElement(sensors, 'accelerometer', name=name + '_acc', site=name + '_site')
        ET.SubElement(sensors, 'framequat', name=name + '_quat', objtype='site', objname=name + '_site')

    # Solve the actual URDF linkage, including its rounded link angles. Do not
    # manufacture a closed pose by moving bodies or disabling constraints.
    initial = {'yaw_joint': math.pi / 2, 'pitch_joint': 0.0}
    for side in ('left', 'right'):
        names = [side + '_' + n + '_joint' for n in ('hip', 'calf', 'knee', 'mid', 'long', 'short')]
        def residual(values):
            poses = forward(dict(zip(names, values)))
            result = []
            for parent, p, child, c in closures:
                if parent.startswith(side + '_'):
                    delta = (poses[parent] @ np.r_[p, 1] - poses[child] @ np.r_[c, 1])[:3]
                    result.extend(delta[[0, 2]])
            center = poses[side + '_wheel'][:3, 3]
            result.extend([center[0], center[2] + .20])
            return result
        fit = least_squares(residual, [-.27, .32, .26, -.32, .10, -.32],
                            xtol=1e-14, ftol=1e-14, gtol=1e-14)
        if not fit.success or np.linalg.norm(residual(fit.x)) > 1e-10:
            raise RuntimeError('Could not close the ' + side + ' linkage')
        initial.update(zip(names, map(float, fit.x)))
        initial[side + '_wheel_joint'] = 0.0
    wheel_radius = float(original.find("link[@name='left_wheel']/collision/geometry/cylinder").get('radius'))
    initial_path = package / 'config/series_legged2_initial.yaml'
    with initial_path.open('w') as out:
        out.write('# Closed linkage at 0.20 m; yaw faces base +X. This is an initial pose, not a standing test.\n')
        for heading, velocity in (('initial_joint_positions', False), ('initial_joint_velocities', True)):
            out.write(heading + ':\n  joint_map:\n')
            out.write("    base_free: '" + ('0 0 0 0 0 0' if velocity else fmt([0, 0, .20 + wheel_radius + .0002, 1, 0, 0, 0])) + "'\n")
            for name, value in initial.items():
                out.write("    " + name + ": '" + format(0.0 if velocity else value, '.16g') + "'\n")
    write_xml(destination / 'scene.xml', model)
    write_xml(destination / 'robot.urdf', robot)
    inputs = [cad_path, Path(__file__).resolve()]
    for directory in ('series_legged', 'series_legged2', 'common'):
        inputs.extend(sorted((description / 'urdf' / directory).glob('*.xacro')))
    manifest = {
        'sources_sha256': {str(p.relative_to(package.parent)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
        'cad_to_ros_body_transforms': {name: {'cad_body': old, 'transform': pose.tolist()} for name, (old, pose) in frames.items()},
        'link_pivot_length_difference_m': anchor_errors,
        'mass_kg': sum(float(b.find('inertial').get('mass')) for b in cad_bodies.values()),
        'active_joints': sorted(active),
        'spring_force_N': '((2094.45 * L0 - 3091.28) * L0 + 1408.375) * L0 - 80.91',
        'controller_parameters': 'Unchanged: rm_config/config/rm_controllers/series_legged2.yaml',
    }
    (destination / 'source_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'mass_kg': manifest['mass_kg'], 'active_joints': len(active),
                      'max_link_pivot_length_difference_m': max(anchor_errors.values()),
                      'output': str(destination)}, indent=2))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--description', type=Path)
    args = parser.parse_args()
    prepare(args.package, args.description or args.package.parent / 'rm_description')
