#!/usr/bin/env python3
"""Drive the running yaw_controller through several references via /yaw_ref and log yaw(t).

Usage: validate_yaw.py out.csv [hold_seconds=12]   (Gazebo + yaw_controller must be running)
"""
import csv
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

REFS_DEG = [90, -90, 170, -170, 0]   # 170 -> -170 tests the wrap-around (short way = 20 deg)
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0


class V(Node):
    def __init__(self):
        super().__init__('validate_yaw')
        self.yaw = None
        self.create_subscription(Odometry, '/odom', self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float64, '/yaw_ref', 10)

    def cb(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def main():
    rclpy.init()
    n = V()
    while n.yaw is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    rows, t0 = [], time.time()
    for ref in REFS_DEG:
        n.pub.publish(Float64(data=math.radians(ref)))
        t_ref = time.time()
        end = t_ref + HOLD
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
            rows.append((time.time() - t0, math.degrees(n.yaw), ref))
        err = (ref - math.degrees(n.yaw) + 180) % 360 - 180
        print(f'ref {ref:5d} deg -> yaw {math.degrees(n.yaw):8.2f} deg, error {err:6.2f} deg')
    with open(sys.argv[1], 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'yaw_deg', 'ref_deg'])
        w.writerows(rows)
    rclpy.shutdown()


main()
