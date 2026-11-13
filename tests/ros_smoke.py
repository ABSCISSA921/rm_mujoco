#!/usr/bin/env python3
"""Check a freshly launched, paused series_legged ROS simulation on localhost.

Uses ordinary command topics, controller_manager and upstream step actions.
Never sets poses, contacts or controller flags. Records asynchronous streams
separately; this is an integration check, not a balance-performance test.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import time
from urllib.parse import urlparse

import actionlib
from actionlib.simple_action_client import SimpleGoalState
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Twist
from mujoco_ros_msgs.msg import StepAction, StepGoal
from mujoco_ros_msgs.srv import GetBodyState
from rm_msgs.msg import ChassisCmd, GimbalCmd, LegCmd, LeggedChassisMode, LeggedChassisStatus
import rospy
from sensor_msgs.msg import JointState


def wait_until(predicate, description, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not rospy.is_shutdown():
        if predicate():
            return
        time.sleep(0.01)
    raise RuntimeError('Timeout: ' + description)


def run(output, quick=False):
    if urlparse(os.environ.get('ROS_MASTER_URI', '')).hostname not in ('127.0.0.1', 'localhost'):
        raise RuntimeError('Use mujoco_env.sh and an isolated localhost master')
    # rospy reads use_sim_time only during init; wait for roslaunch to set it.
    wait_until(lambda: rospy.get_param('/use_sim_time', False), 'use_sim_time before node initialization')
    rospy.init_node('rm_mujoco_integration_check')
    records = {'commands': [], 'mode_events': [], 'joint_states': [], 'chassis_status': [], 'body_states': []}

    def mode_cb(msg):
        if not records['mode_events'] or records['mode_events'][-1]['name'] != msg.mode_name:
            records['mode_events'].append({'receipt_sim_time': rospy.get_time(), 'mode': msg.mode, 'name': msg.mode_name})

    def joint_cb(msg):
        records['joint_states'].append({'stamp': msg.header.stamp.to_sec(), 'names': list(msg.name),
                                       'position': list(msg.position), 'velocity': list(msg.velocity),
                                       'applied_effort_readback': list(msg.effort)})

    def chassis_cb(msg):
        records['chassis_status'].append({'stamp': msg.header.stamp.to_sec(), 'cycle_seq': msg.cycle_seq,
                                         'pitch': msg.pitch, 'roll': msg.roll, 'x_dot': msg.x_dot,
                                         'left_length': msg.left_leg_length, 'right_length': msg.right_leg_length})

    subscribers = [rospy.Subscriber('/joint_states', JointState, joint_cb, queue_size=1000),
                   rospy.Subscriber('/controllers/chassis_controller/legged_chassis_mode', LeggedChassisMode,
                                    mode_cb, queue_size=1000),
                   rospy.Subscriber('/controllers/chassis_controller/legged_chassis_status', LeggedChassisStatus,
                                    chassis_cb, queue_size=1000)]
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    chassis_pub = rospy.Publisher('/cmd_chassis', ChassisCmd, queue_size=1)
    leg_pub = rospy.Publisher('/leg_cmd', LegCmd, queue_size=1)
    gimbal_pub = rospy.Publisher('/controllers/gimbal_controller/command', GimbalCmd, queue_size=1)
    rospy.wait_for_service('/controller_manager/list_controllers', timeout=20)
    list_ctrl = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
    switch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    body = rospy.ServiceProxy('/mujoco_server/get_body_state', GetBodyState)
    client = actionlib.SimpleActionClient('/mujoco_server/step', StepAction)
    client.wait_for_server()

    def step(count=20):
        client.send_goal(StepGoal(num_steps=count))
        # actionlib status can report SUCCEEDED before the result message arrives.
        wait_until(lambda: client.simple_state == SimpleGoalState.DONE and client.get_result() is not None,
                   'step result and completion callback', timeout=10)
        if not client.get_result().success:
            raise RuntimeError('MuJoCo step failed (state %s); launch with unpause:=false' % client.get_state())

    def state_map():
        return {c.name: c.state for c in list_ctrl().controller}

    try:
        # A pending spawner switch holds the manager service mutex until an update.
        # Wait for actual controller feedback before querying manager services;
        # loading/gain setup may still be running when the first step completes.
        deadline = time.monotonic() + 20.0
        while not records['chassis_status']:
            if time.monotonic() >= deadline:
                raise RuntimeError('No chassis updates during startup stepping')
            step(1)
            time.sleep(0.01)
        # The gimbal may still be loading when the first chassis sample arrives.
        # Keep stepping while the manager serves the startup/switch requests.
        with ThreadPoolExecutor(max_workers=1) as pool:
            deadline = time.monotonic() + 20.0
            while True:
                future = pool.submit(state_map)
                while not future.done():
                    if time.monotonic() > deadline:
                        raise RuntimeError('Controller manager startup timeout')
                    step(2)
                states = future.result()
                if len(states) == 4 and all(s == 'running' for s in states.values()):
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('Expected four running controllers: ' + str(states))
                step(2)
        wait_until(lambda: all(p.get_num_connections() for p in (vel_pub, chassis_pub, leg_pub, gimbal_pub)),
                   'command subscribers')
        records['controllers_initial'] = state_map()
        command = ChassisCmd(mode=ChassisCmd.RAW, command_source_frame='base_link', follow_source_frame='yaw',
                             power_limit=100.0)
        command.accel.linear.x = command.accel.linear.y = 5.0
        command.accel.angular.z = 10.0
        leg = LegCmd(jump=False, leg_length=0.20)
        velocity = Twist()
        gimbal = GimbalCmd(mode=GimbalCmd.RATE)

        def phase(name, seconds, mode, speed=0.0, yaw_rate=0.0, pitch_rate=0.0):
            command.mode = mode
            velocity.linear.x = speed
            gimbal.rate_yaw, gimbal.rate_pitch = yaw_rate, pitch_rate
            records['commands'].append({'phase': name, 'sim_time': rospy.get_time(), 'mode': mode,
                                        'vx': speed, 'leg_length': leg.leg_length, 'duration': seconds,
                                        'gimbal_yaw_rate': yaw_rate, 'gimbal_pitch_rate': pitch_rate})
            for _ in range(round(seconds / 0.02)):
                command.stamp = rospy.Time.now()
                chassis_pub.publish(command)
                leg_pub.publish(leg)
                vel_pub.publish(velocity)
                gimbal.stamp = command.stamp
                gimbal_pub.publish(gimbal)
                time.sleep(0.003)
                step()
                result = body(name='base_link', admin_hash='')
                if not result.success:
                    raise RuntimeError(result.status_message)
                p, q = result.state.pose.pose.position, result.state.pose.pose.orientation
                record = {'service_stamp': result.state.pose.header.stamp.to_sec(),
                          'query_after_step_sim_time': rospy.get_time(), 'position': [p.x, p.y, p.z],
                          'quaternion_xyzw': [q.x, q.y, q.z, q.w]}
                if not all(math.isfinite(x) for x in record['position'] + record['quaternion_xyzw']):
                    raise RuntimeError('Nonfinite MuJoCo body state')
                records['body_states'].append(record)

        phase('raw_zero', 0.10 if quick else 3.0, ChassisCmd.RAW)
        phase('raw_forward', 0.06 if quick else 0.5, ChassisCmd.RAW, 0.1)
        phase('gimbal_yaw', 0.08 if quick else 0.2, ChassisCmd.RAW, yaw_rate=0.1)
        phase('gimbal_pitch', 0.08 if quick else 0.2, ChassisCmd.RAW, pitch_rate=0.05)
        phase('fallen', 0.04 if quick else 0.2, ChassisCmd.FALLEN)
        with ThreadPoolExecutor(max_workers=1) as pool:
            for start in (False, True):
                names = ['controllers/chassis_controller']
                future = pool.submit(switch, names if start else [], [] if start else names, 2, False, 1.0)
                for _ in range(50):
                    step(2)
                    if future.done():
                        break
                if not future.result(timeout=2).ok:
                    raise RuntimeError('controller stop/start failed')
                records['controllers_after_' + ('start' if start else 'stop')] = state_map()
        phase('restart_fallen', 0.04 if quick else 0.1, ChassisCmd.FALLEN)
        if not records['chassis_status'] or not records['joint_states']:
            raise RuntimeError('Missing controller or joint feedback')
        if len(records['joint_states'][-1]['names']) != 16:
            raise RuntimeError('Expected 16 measured joint states')
        expected = {'yaw_joint', 'pitch_joint', 'left_hip_joint', 'right_hip_joint',
                    'left_knee_joint', 'right_knee_joint', 'left_wheel_joint', 'right_wheel_joint'}
        resources = {r for c in list_ctrl().controller for group in c.claimed_resources for r in group.resources}
        if not expected <= resources:
            raise RuntimeError('Missing controller resources: ' + str(expected - resources))
        records['claimed_joints'] = sorted(resources)
        records['integration_passed'] = True
    except Exception as exc:
        records['integration_passed'] = False
        records['error'] = str(exc)
        raise
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(records, indent=2) + '\n')
    print(json.dumps({k: v for k, v in records.items() if k.startswith('controllers') or k in
                      ('integration_passed', 'mode_events', 'commands')}, indent=2))
    # Keep subscriber objects alive until all records have been collected.
    for sub in subscribers:
        sub.unregister()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--quick', action='store_true', help='Short interface check, not a balance/standing test')
    args = parser.parse_args()
    run(args.output, args.quick)
