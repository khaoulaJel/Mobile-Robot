#!/usr/bin/env python3
"""Inspect a candidate robot state topic BEFORE running the controllers on the real robot.

  python3 check_state_source.py --topic /odometry/filtered [--seconds 10]
  python3 check_state_source.py --topic /robot_pose --rotate --forward     # these MOVE the robot slowly

Reports message type, frame id, rate, position jumps, noise at rest, header latency, and (only with
--rotate / --forward, which publish small velocity commands) whether positive angular.z increases
yaw and whether +x moves the robot along its heading. Prints PASS / WARN / FAIL per check; exit code 1
if any FAIL. The robot must be stationary (and on blocks or in a clear area) during the first phase.
"""
import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

sys.path.insert(0, __file__.rsplit('tools', 1)[0])
from lab3_controllers.state_input import StateInput, StateInputError  # noqa: E402

fails = 0


def check(level, name, detail):
    global fails
    fails += level == 'FAIL'
    print(f'{level}: {name}  ({detail})', flush=True)


def yaw_of(p):
    q = p.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--topic', required=True)
    ap.add_argument('--type', default='auto', help='auto | odometry | pose_stamped | pose | pose_cov')
    ap.add_argument('--seconds', type=float, default=10.0, help='stationary observation time')
    ap.add_argument('--rotate', action='store_true', help='turn slowly CCW for 3 s (moves the robot)')
    ap.add_argument('--forward', action='store_true', help='drive 0.05 m/s for 2 s (moves the robot)')
    ap.add_argument('--cmd-topic', default='/diff_cont/cmd_vel_unstamped')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('check_state_source')
    samples = []          # (wall time, x, y, yaw)
    lat = []

    def cb(pose):
        samples.append((time.time(), pose.position.x, pose.position.y, yaw_of(pose)))
        if si.stamp is not None:
            now = node.get_clock().now().nanoseconds * 1e-9
            lat.append(now - (si.stamp.sec + si.stamp.nanosec * 1e-9))

    try:
        si = StateInput(node, a.topic, a.type, cb, wait_timeout=20.0)
        end = time.time() + 25.0
        while si.subscription is None and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except StateInputError as e:
        check('FAIL', 'state topic usable', str(e)); sys.exit(1)

    t_end = time.time() + a.seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    n = len(samples)
    check('PASS' if n else 'FAIL', 'messages received', f'{n} in {a.seconds:.0f} s, type {si.type_name}')
    if not n:
        sys.exit(1)
    rate = n / a.seconds
    check('PASS' if rate >= 5 else 'WARN', 'update rate', f'{rate:.1f} Hz (want >= 5, ideally 10-50)')
    check('PASS' if si.frame_id else 'WARN', 'frame id', repr(si.frame_id) if si.frame_id is not None
          else 'message type has no header; frame unknown, check it in the stack that publishes it')
    xs, ys = [p[1] for p in samples], [p[2] for p in samples]
    jump = max(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) for i in range(n - 1)) if n > 1 else 0.0
    check('PASS' if jump <= 0.05 else 'WARN', 'position continuity at rest',
          f'largest step between messages {jump * 100:.1f} cm (jumps suggest localization corrections)')
    sx, sy = statistics.pstdev(xs) * 1000, statistics.pstdev(ys) * 1000
    check('PASS' if max(sx, sy) < 5 else 'WARN', 'position noise at rest', f'std x {sx:.1f} mm, y {sy:.1f} mm')
    if lat:
        check('PASS' if abs(statistics.median(lat)) < 0.2 else 'WARN', 'header latency',
              f'median {statistics.median(lat) * 1000:.0f} ms (needs synchronized clocks)')
    pub = node.create_publisher(Twist, a.cmd_topic, 10)

    def drive(v, w, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            m = Twist(); m.linear.x, m.angular.z = v, w
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.05)
        for _ in range(5):
            pub.publish(Twist()); rclpy.spin_once(node, timeout_sec=0.05)
        t1 = time.time()
        while time.time() - t1 < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        if a.rotate:
            y0 = samples[-1][3]
            drive(0.0, 0.3, 3.0)
            dy = math.atan2(math.sin(samples[-1][3] - y0), math.cos(samples[-1][3] - y0))
            check('PASS' if dy > 0.3 else 'FAIL', 'yaw convention: +angular.z turns CCW, yaw increases',
                  f'yaw changed {math.degrees(dy):.1f} deg for about +51 deg commanded')
        if a.forward:
            x0, y0, yw = samples[-1][1], samples[-1][2], samples[-1][3]
            drive(0.05, 0.0, 2.0)
            along = (samples[-1][1] - x0) * math.cos(yw) + (samples[-1][2] - y0) * math.sin(yw)
            check('PASS' if along > 0.04 else 'FAIL', '+x drives along the heading',
                  f'moved {along * 100:.1f} cm along the heading for about 10 cm commanded')
    finally:
        pub.publish(Twist())
    print('exit code 1 if any FAIL' if fails else 'no FAIL')
    sys.exit(1 if fails else 0)


main()
