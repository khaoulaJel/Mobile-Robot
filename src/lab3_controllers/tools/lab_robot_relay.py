#!/usr/bin/env python3
"""Make Gazebo behave like the professor's robot: command filter + slow identified velocity response.

    controller --(Twist)--> /lab/cmd_vel --> [input_filter -> identified order-4 plant @ 20 Hz] -->
    /diff_cont/cmd_vel_unstamped --> Gazebo drive plugin --> /odom --> controller

The filter (feasible diamond, deadband) and the plant are exactly tools/plant_models.py, which was checked
against the professor's Simulink blocks (tools/simulink_replay). Gazebo's own drive plugin adds its (small)
dynamics on top. Start the controller with `-p cmd_topic:=/lab/cmd_vel`. A stale command (no message for
`cmd_timeout` s, e.g. controller killed) is treated as zero.

Usage: python3 lab_robot_relay.py --ros-args -p use_sim_time:=true
"""
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plant_models as pm  # noqa: E402


class Relay(Node):
    def __init__(self):
        super().__init__('lab_robot_relay')
        self.declare_parameter('in_topic', '/lab/cmd_vel')
        self.declare_parameter('out_topic', '/diff_cont/cmd_vel_unstamped')
        self.declare_parameter('use_filter', True)
        self.declare_parameter('cmd_timeout', 0.5)
        self.plant = pm.IdentifiedPlant(use_filter=bool(self.get_parameter('use_filter').value))
        self.timeout = float(self.get_parameter('cmd_timeout').value)
        self.cmd = (0.0, 0.0)
        self.last = None
        self.create_subscription(Twist, self.get_parameter('in_topic').value, self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('out_topic').value, 10)
        self.create_timer(self.plant.Ts, self.tick)
        self.get_logger().info('lab_robot_relay: input_filter + identified plant active')

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last = time.monotonic()

    def tick(self):
        if self.last is None or time.monotonic() - self.last > self.timeout:
            self.cmd = (0.0, 0.0)
        v, w = self.plant.step(*self.cmd)
        out = Twist()
        out.linear.x, out.angular.z = float(v), float(w)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
