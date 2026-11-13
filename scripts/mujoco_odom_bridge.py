#!/usr/bin/env python3
import argparse

import rospy
from mujoco_ros_msgs.srv import GetBodyState
from nav_msgs.msg import Odometry


def quaternion_matrix(x, y, z, w):
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]


def world_to_body(matrix, vector):
    return [sum(matrix[r][c] * vector[r] for r in range(3)) for c in range(3)]


def parse_args():
    parser = argparse.ArgumentParser(description="Publish MuJoCo body state as nav_msgs/Odometry.")
    parser.add_argument("--body", default="base_link")
    parser.add_argument("--topic", default="/gazebo_odom")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--child-frame", default="base_link")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--service", default="/mujoco_server/get_body_state")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("mujoco_odom_bridge", anonymous=False)

    publisher = rospy.Publisher(args.topic, Odometry, queue_size=1)
    proxy = None
    rate = rospy.Rate(args.rate)

    rospy.loginfo("mujoco_odom_bridge: %s -> %s at %.1f Hz", args.body, args.topic, args.rate)

    while not rospy.is_shutdown():
        if proxy is None:
            try:
                rospy.wait_for_service(args.service, timeout=1.0)
            except rospy.ROSException:
                rospy.loginfo_throttle(5.0, "waiting for %s", args.service)
                continue
            proxy = rospy.ServiceProxy(args.service, GetBodyState)

        try:
            response = proxy(args.body, "")
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "get_body_state failed: %s", exc)
            proxy = None
            continue

        if not response.success:
            rospy.logwarn_throttle(2.0, "get_body_state rejected: %s", response.status_message)
            rate.sleep()
            continue

        pose = response.state.pose.pose
        twist = response.state.twist.twist
        matrix = quaternion_matrix(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                   pose.orientation.w)
        linear = world_to_body(matrix, [twist.linear.x, twist.linear.y, twist.linear.z])

        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = args.frame_id
        message.child_frame_id = args.child_frame
        message.pose.pose = pose
        message.twist.twist.linear.x = linear[0]
        message.twist.twist.linear.y = linear[1]
        message.twist.twist.linear.z = linear[2]
        message.twist.twist.angular = twist.angular

        publisher.publish(message)
        rate.sleep()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
