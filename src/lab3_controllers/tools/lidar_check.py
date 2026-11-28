#!/usr/bin/env python3
"""Is the LiDAR chain ready for the path tests? Read-only, nothing moves.   (bash ~/robot.sh lidar)
Checks, with numbers: /scan arrives, its frame and delay, /robot_pose arrives, its rate, delay and steadiness at rest."""
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SECONDS = 20.0


def stats_of(arrivals):
    ages = sorted(a for a, _ in arrivals)
    return ages[len(ages) // 2]


def main():
    rclpy.init()
    node = rclpy.create_node('lidar_check')
    scans, poses = [], []

    def age(h):
        return time.time() - (h.stamp.sec + h.stamp.nanosec * 1e-9)

    node.create_subscription(LaserScan, '/scan', lambda m: scans.append((age(m.header), m)), qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/robot_pose', lambda m: poses.append((age(m.header), m)), qos_profile_sensor_data)
    print('listening for 20 s; keep the robot STILL ...', flush=True)
    t0 = time.time()
    first_scan = first_pose = None
    while time.time() - t0 < SECONDS + 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if scans and first_scan is None:
            first_scan = time.time()
        if poses and first_pose is None:
            first_pose = time.time()
        if first_pose and time.time() - first_pose > SECONDS:
            break
    ok = []

    def check(name, good, detail, hint=''):
        ok.append(good)
        print(f'  {"OK  " if good else "FAIL"} {name}: {detail}' + ('' if good or not hint else f'\n       -> {hint}'))

    if scans:
        span = time.time() - first_scan
        last = scans[-1][1]
        finite = [r for r in last.ranges if math.isfinite(r) and last.range_min <= r <= last.range_max]
        check('/scan (LiDAR)', len(scans) / span >= 5, f'{len(scans) / span:.1f} per second, frame "{last.header.frame_id}", '
              f'{len(finite)} of {len(last.ranges)} directions see something', 'fewer than 5 per second: LiDAR cable or Wi-Fi')
        check('scan delay', abs(stats_of(scans)) <= 0.3, f'{1000 * stats_of(scans):.0f} ms (needed: within 300 ms)',
              'the clocks differ: guide step "Clocks"')
    else:
        check('/scan (LiDAR)', False, 'nothing received', 'LiDAR USB in the Pi (/dev/ttyUSB0)? Plugged in AFTER the driver '
              'started: run "bash ~/robot.sh stop" then "bash ~/robot.sh start"')
    if poses:
        span = max(1.0, time.time() - first_pose)
        xs = [m.pose.position.x for _, m in poses]
        ys = [m.pose.position.y for _, m in poses]
        yaws = [2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w) for _, m in poses]
        ref = yaws[0]
        turn = [math.degrees(math.atan2(math.sin(a - ref), math.cos(a - ref))) for a in yaws]
        check('/robot_pose (map position)', len(poses) / span >= 5, f'{len(poses) / span:.1f} per second',
              'slow: see ~/laptop_stack.log')
        check('map position delay', abs(stats_of(poses)) <= 0.3, f'{1000 * stats_of(poses):.0f} ms', 'guide step "Clocks"')
        wobble_xy = 100 * math.hypot(statistics.pstdev(xs), statistics.pstdev(ys))
        wobble_th = statistics.pstdev(turn)
        check('map position steady at rest', wobble_xy <= 2.0 and wobble_th <= 1.0,
              f'noise {wobble_xy:.2f} cm, {wobble_th:.2f} deg (x={xs[-1]:.2f} y={ys[-1]:.2f} heading={math.degrees(yaws[-1]):.1f} deg)',
              'the map is still settling or sees too little: wait, move people away, repeat')
    else:
        check('/robot_pose (map position)', False, 'nothing received', 'needs /scan first, correct clocks, and the laptop programs '
              '(~/laptop_stack.log)')
    node.destroy_node()
    rclpy.shutdown()
    good = len(ok) == 5 and all(ok)
    print('LiDAR chain READY for the map tests.' if good else 'LiDAR chain NOT ready (odometry tests still work).')
    return 0 if good else 1


if __name__ == '__main__':
    sys.exit(main())
