#!/usr/bin/env python3
"""Derive ROS descriptions from the preserved mechanical source, without rebuilding geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(directory):
    source = directory / 'source'
    scene = ET.parse(str(source / 'series_legged.xml'))
    model = scene.getroot()
    # noetic-devel DefaultRobotHWSim writes qfrc_applied. The same effort limits
    # live in the source URDF and are enforced by its EffortJointSaturationInterface.
    motors = model.find('actuator')
    active = {motor.attrib['joint'] for motor in motors}
    model.remove(motors)
    for sensor in model.find('sensor'):
        mapping = {'imu_quat': 'base_imu_quat', 'imu_gyro': 'base_imu_gyro', 'imu_accel': 'base_imu_acc'}
        if sensor.get('name') in mapping:
            sensor.set('name', mapping[sensor.get('name')])
    scene.write(str(directory / 'scene.xml'), encoding='utf-8', xml_declaration=True)

    description = ET.parse(str(source / 'expanded_source.urdf'))
    robot = description.getroot()
    moving = {joint.attrib['name'] for joint in model.findall('worldbody//joint')}
    for joint in robot.findall('joint'):
        if joint.attrib['name'] not in moving:
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics', 'mimic', 'safety_controller', 'calibration'):
                for child in joint.findall(tag):
                    joint.remove(child)
    for transmission in robot.findall('transmission'):
        if transmission.find('joint').get('name') not in active:
            robot.remove(transmission)
    for gazebo in robot.findall('gazebo'):
        robot.remove(gazebo)
    description.write(str(directory / 'robot.urdf'), encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_directory', type=Path, help='rm_mujoco/models/series_legged')
    prepare(parser.parse_args().model_directory)
