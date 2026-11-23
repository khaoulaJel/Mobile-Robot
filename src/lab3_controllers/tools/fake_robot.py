#!/usr/bin/env python3
"""Stand-in for Gazebo: unicycle with first-order velocity lag publishing /odom.
Used to test the controllers when Gazebo is unavailable."""
import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Fake(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.cv = self.cw = 0.0
        self.tau = 0.15
        self.dt = 0.02
        self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(self.dt, self.step)

    def cb(self, m):
        self.cv, self.cw = m.linear.x, m.angular.z

    def step(self):
        a = self.dt / (self.tau + self.dt)
        self.v += a * (self.cv - self.v)
        self.w += a * (self.cw - self.w)
        self.th += self.w * self.dt
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'odom'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z = math.sin(self.th / 2)
        o.pose.pose.orientation.w = math.cos(self.th / 2)
        self.pub.publish(o)


rclpy.init()
rclpy.spin(Fake())
