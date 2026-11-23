#!/usr/bin/env python3
"""Strict check of reference_controller against the simulated robot (tools/fake_robot.py: unicycle with a velocity
lag, odometry on /odom). No Gazebo needed. Starts both processes itself.

    source install/setup.bash && python3 src/lab3_controllers/tools/check_reference.py

Checks (explicit limits, exit code 1 if any fails):
  1  no reference yet                       -> commands stay exactly zero for 3 s
  2  pose reference (0.40, 0.20, 90 deg)    -> final position error <= 2 cm, heading error <= 1 deg
  3  yaw reference 180 deg                  -> final heading error <= 1 deg, position unchanged (<= 1 cm)
  4  coefficient changed while running      -> parameter accepted and the next pose reference still reached (<= 2 cm)
  5  invalid coefficients (k_beta > 0, polar) -> rejected: the robot stays stopped (|v|, |w| = 0 for 2 s)
  6  coefficients repaired                  -> motion resumes and the reference is reached (<= 2 cm)
  7  node restarted with the initial target heading pi/2 (no topic message) -> heading pi/2 within 1 deg
"""
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = 'reference_controller'


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Probe(Node):
    def __init__(self):
        super().__init__('check_reference')
        self.pose = None
        self.cmd = (0.0, 0.0)
        self.cmd_seen = 0
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self.on_cmd, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/reference', 10)
        self.yaw_pub = self.create_publisher(Float64, '/reference_yaw', 10)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_seen += 1

    def spin(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_pose(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = 'odom'
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        for _ in range(3):
            self.pose_pub.publish(m)
            self.spin(0.1)

    def send_yaw(self, yaw):
        m = Float64()
        m.data = yaw
        for _ in range(3):
            self.yaw_pub.publish(m)
            self.spin(0.1)

    def settle(self, timeout):
        """Wait until the commands have been ~0 for 2 s (the controller finished)."""
        quiet = None
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.1)
            if abs(self.cmd[0]) < 1e-3 and abs(self.cmd[1]) < 1e-3:
                quiet = quiet or time.monotonic()
                if time.monotonic() - quiet > 2.0:
                    return True
            else:
                quiet = None
        return False

    def max_cmd(self, sec):
        v = w = 0.0
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.spin(0.02)
            v, w = max(v, abs(self.cmd[0])), max(w, abs(self.cmd[1]))
        return v, w


def ros2(*args):
    return subprocess.run(['ros2', *args], capture_output=True, text=True, timeout=30)


def main():
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f'{"PASS" if ok else "FAIL"}: {name}  ({detail})', flush=True)

    procs = [subprocess.Popen([sys.executable, os.path.join(HERE, 'fake_robot.py')],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)]
    log = open('/tmp/check_reference_node.log', 'w')
    procs.append(subprocess.Popen(
        ['ros2', 'run', 'lab3_controllers', 'reference_controller', '--ros-args', '-r', f'__node:={NODE}',
         '-p', 'state_topic:=/odom', '-p', 'cmd_topic:=/diff_cont/cmd_vel_unstamped', '-p', 'start_delay:=1.0',
         '-p', 'relative_to_start:=false', '-p', 'max_linear_speed:=0.2', '-p', 'max_angular_speed:=1.0',
         '-p', 'pos_tolerance:=0.01', '-p', 'yaw_tolerance:=0.01', '-p', 'tolerance:=0.005', '-p', 'kp:=1.0'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    rclpy.init()
    probe = Probe()
    try:
        t0 = time.monotonic()
        while (probe.pose is None or probe.cmd_seen == 0) and time.monotonic() - t0 < 20:
            probe.spin(0.1)
        if probe.pose is None or probe.cmd_seen == 0:
            check('start', False, 'no odometry or no command stream after 20 s')
            return 1
        probe.spin(2.0)                                           # start_delay

        v, w = probe.max_cmd(3.0)                                 # 1
        check('1 no reference: commands stay zero', v == 0.0 and w == 0.0, f'max |v|={v}, max |w|={w}')

        probe.send_pose(0.40, 0.20, math.pi / 2)                  # 2
        done = probe.settle(60)
        x, y, th = probe.pose
        e_pos, e_yaw = math.hypot(0.40 - x, 0.20 - y), abs(math.degrees(wrap(math.pi / 2 - th)))
        check('2 pose reference reached', done and e_pos <= 0.02 and e_yaw <= 1.0,
              f'position error {e_pos * 100:.2f} cm (limit 2), heading error {e_yaw:.2f} deg (limit 1), settled={done}')

        before = probe.pose
        probe.send_yaw(math.pi)                                   # 3
        done = probe.settle(40)
        x, y, th = probe.pose
        e_yaw, drift = abs(math.degrees(wrap(math.pi - th))), math.hypot(x - before[0], y - before[1])
        check('3 yaw reference reached', done and e_yaw <= 1.0 and drift <= 0.01,
              f'heading error {e_yaw:.2f} deg (limit 1), position drift {drift * 100:.2f} cm (limit 1), settled={done}')

        r = ros2('param', 'set', f'/{NODE}', 'kv', '0.5')         # 4
        probe.spin(0.5)
        accepted = 'Set parameter successful' in r.stdout
        probe.send_pose(0.10, 0.20, 0.0)
        done = probe.settle(60)
        x, y, th = probe.pose
        e_pos = math.hypot(0.10 - x, 0.20 - y)
        check('4 coefficient changed while running', accepted and done and e_pos <= 0.02,
              f'accepted={accepted}, position error {e_pos * 100:.2f} cm (limit 2), settled={done}')

        ros2('param', 'set', f'/{NODE}', 'strategy', 'polar')     # 5
        ros2('param', 'set', f'/{NODE}', 'k_beta', '2.0')
        probe.send_pose(0.30, 0.20, 0.0)
        probe.spin(1.0)
        v, w = probe.max_cmd(2.0)
        check('5 invalid coefficients rejected, robot stays stopped', v == 0.0 and w == 0.0,
              f'max |v|={v}, max |w|={w} (limit 0)')

        ros2('param', 'set', f'/{NODE}', 'k_beta', '-2.0')        # 6
        ros2('param', 'set', f'/{NODE}', 'strategy', 'stop_turn')
        probe.spin(0.5)
        probe.send_pose(0.30, 0.20, 0.0)
        done = probe.settle(60)
        x, y, th = probe.pose
        e_pos = math.hypot(0.30 - x, 0.20 - y)
        check('6 coefficients repaired, motion resumes', done and e_pos <= 0.02,
              f'position error {e_pos * 100:.2f} cm (limit 2), settled={done}')
        os.killpg(procs[1].pid, signal.SIGTERM)                   # 7: the target heading pi/2 is built in
        procs[1].wait(timeout=10)
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'lab3_controllers', 'reference_controller', '--ros-args', '-r', f'__node:={NODE}',
             '-p', 'state_topic:=/odom', '-p', 'cmd_topic:=/diff_cont/cmd_vel_unstamped', '-p', 'start_delay:=1.0',
             '-p', 'relative_to_start:=false', '-p', 'max_angular_speed:=1.0', '-p', 'tolerance:=0.005', '-p', 'kp:=1.0',
             '-p', 'use_initial_yaw_reference:=true', '-p', f'initial_yaw_reference:={math.pi / 2}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        probe.spin(2.0)
        done = probe.settle(60)
        e_yaw = abs(math.degrees(wrap(math.pi / 2 - probe.pose[2])))
        check('7 initial target heading pi/2 (no message sent)', done and e_yaw <= 1.0,
              f'heading error {e_yaw:.2f} deg (limit 1), settled={done}')
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        log.close()
    print(f'passed: {sum(results)}   failed: {len(results) - sum(results)}')
    return 0 if all(results) and len(results) == 7 else 1


if __name__ == '__main__':
    sys.exit(main())
