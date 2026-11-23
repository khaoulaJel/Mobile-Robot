#!/usr/bin/env python3
"""Expose the simulated base pose as PoseStamped, preserving odometry time/frame."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomToPose(Node):
    def __init__(self):
        super().__init__('odom_to_pose')
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.publish_pose, qos_profile_sensor_data)

    def publish_pose(self, odometry):
        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.pose_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
