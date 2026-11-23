#!/usr/bin/env python3
"""Strict check of a finished motion_controller run against Gazebo + lab_robot_relay.

Reads the controller's CSV (written with -p log_csv:=...), waits for the robot to come to rest, reads the final
pose from /odom and prints/judges: path completed, final position error, final heading error and the maximum
distance of the driven path from the ideal path.  Exit code 0 only if every limit is met.

Usage: lab_robot_gazebo_check.py run.csv --shape square [--settle 15]
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

SHAPES = {   # ideal path in the start frame
    'square': [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)],
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def dist_to_polyline(px, py, pts):
    best = 1e9
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        L2 = (bx - ax) ** 2 + (by - ay) ** 2
        t = max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / L2)) if L2 else 0.0
        best = min(best, math.hypot(px - ax - t * (bx - ax), py - ay - t * (by - ay)))
    return best


class Odom(Node):
    def __init__(self):
        super().__init__('lab_robot_gazebo_check')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--shape', default='square', choices=list(SHAPES))
    ap.add_argument('--settle', type=float, default=6.0,
                    help='wall seconds to wait for the plant to coast to rest. Keep it short: Gazebo itself drifts about '
                         '1.3 mm/s with a zero command (measured, no controller involved), so a long wait adds error '
                         'that does not belong to the controller')
    # Limits include Gazebo's own drift, measured at 1.3 mm/s with a ZERO command and no controller (2.6 cm in 20 s):
    # the controller waits at rest at every corner, so the drift adds to its error. The same square on the model
    # without that drift gives 0.62 cm cross-track and 0.50 cm final position (results/lab_robot_results.md).
    ap.add_argument('--max-xtrk', type=float, default=3.5, help='cm')
    ap.add_argument('--max-pos', type=float, default=2.0, help='cm')
    ap.add_argument('--max-yaw', type=float, default=1.0, help='deg')
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    if not rows or rows[-1]['mode'] != 'DONE':
        print('FAIL: the controller log does not end in DONE')
        return 1
    x0, y0, th0 = float(rows[0]['x']), float(rows[0]['y']), float(rows[0]['yaw'])
    c, s = math.cos(th0), math.sin(th0)
    xs = [(float(r['x']) - x0) * c + (float(r['y']) - y0) * s for r in rows]
    ys = [-(float(r['x']) - x0) * s + (float(r['y']) - y0) * c for r in rows]
    rclpy.init()
    node = Odom()
    end = time.time() + a.settle
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.pose is not None:           # keep the driven path including the coasting after DONE
            px, py = node.pose[0] - x0, node.pose[1] - y0
            xs.append(px * c + py * s)
            ys.append(-px * s + py * c)
    if node.pose is None:
        print('FAIL: no /odom received')
        return 1
    gx, gy, gth = float(rows[-1]['gx']), float(rows[-1]['gy']), float(rows[-1]['gth'])
    pos_err = 100.0 * math.hypot(gx - node.pose[0], gy - node.pose[1])
    yaw_err = math.degrees(abs(wrap(gth - node.pose[2])))
    xtrk = 100.0 * max(dist_to_polyline(x, y, SHAPES[a.shape]) for x, y in zip(xs, ys))
    t_done = float(rows[-1]['t'])
    print(f'{a.shape}: DONE after {t_done:.1f} s (sim); at rest: final position error {pos_err:.2f} cm, '
          f'final heading error {yaw_err:.2f} deg, max cross-track {xtrk:.2f} cm')
    ok = pos_err <= a.max_pos and yaw_err <= a.max_yaw and xtrk <= a.max_xtrk
    print(f'limits: position {a.max_pos} cm, heading {a.max_yaw} deg, cross-track {a.max_xtrk} cm -> '
          f"{'PASS' if ok else 'FAIL'}")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
