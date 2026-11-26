#!/usr/bin/env python3
"""Strict end-to-end check of the p_controller NODE over real ROS topics, against a simulated robot it knows nothing about
(command deadband 0.26 rad/s and 0.05 m/s, gain 1.2, lag 0.6 s, command delay 0.2 s), publishing wheel odometry in its OWN frame and a
SLAM-like map pose in another frame (rotated and shifted), as on the real robot.

    source install/setup.bash && ROS_DOMAIN_ID=<unused> python3 src/lab3_controllers/tools/check_p_controller.py

Checks (judged on the TRUE map pose; exit code 1 if any fails):
  1  no reference                          -> commands stay exactly zero
  2  heading pi/2 in the map frame         -> error <= 2 deg
  3  a second heading                      -> error <= 2 deg
  4  drive +0.40 m along the heading       -> error <= 3 cm, sideways <= 3 cm
  5  drive -0.25 m (backwards)             -> error <= 3 cm
  6  go to a pose                          -> position <= 4 cm, heading <= 3 deg
  7  a coefficient changed while running   -> accepted; an invalid one is refused
  8  odometry stops                        -> zero command within 0.8 s
"""
import math
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

import servo_sim as ss  # noqa: E402
from lab3_controllers.common import wrap_angle  # noqa: E402

CASE = dict(plant='random', seed=3, db_w=0.26, db_v=0.05, gw=1.2, gv=1.15, tau_w=0.6, tau_v=0.5, cmd_delay=0.2)
MAP = (1.3, -0.7, 2.0)                    # where the odometry frame sits in the map


class SimRobot(Node):
    """Runs the robot in real time; odometry 30 Hz in its own frame, map pose 15 Hz about 0.1 s late."""

    def __init__(self):
        super().__init__('sim_robot')
        self.robot = ss.Robot(CASE)
        self.cmd = (0.0, 0.0)
        self.cmd_log = []
        self.odom_on = True
        self.delayed_map = []
        self.k = 0
        self.odom_pub = self.create_publisher(Odometry, '/diff_cont/odom', 10)
        self.map_pub = self.create_publisher(PoseStamped, '/robot_pose', 1)
        self.create_subscription(Twist, '/cmd_vel_tracker', self.on_cmd, 10)
        self.create_timer(ss.DT, self.step)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_log.append((time.monotonic(), m.linear.x, m.angular.z))

    def truth(self):
        from lab3_controllers.servo import compose
        return compose(MAP, (self.robot.x, self.robot.y, self.robot.th))

    def step(self):
        self.robot.step(*self.cmd)
        self.k += 1
        stamp = self.get_clock().now().to_msg()
        if self.odom_on:
            o = Odometry()
            o.header.stamp, o.header.frame_id = stamp, 'odom'
            o.pose.pose.position.x, o.pose.pose.position.y = self.robot.x, self.robot.y
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.robot.th / 2), math.cos(self.robot.th / 2)
            o.twist.twist.linear.x, o.twist.twist.angular.z = self.robot.v, self.robot.w
            self.odom_pub.publish(o)
        if self.k % 2 == 0:
            x, y, th = self.truth()
            p = PoseStamped()
            p.header.stamp, p.header.frame_id = stamp, 'map'
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
            self.delayed_map.append((time.monotonic() + 0.1, p))
        while self.delayed_map and self.delayed_map[0][0] <= time.monotonic():
            self.map_pub.publish(self.delayed_map.pop(0)[1])


class Probe(Node):
    def __init__(self):
        super().__init__('probe')
        self.yaw = self.create_publisher(Float64, '/reference_yaw', 10)
        self.dist = self.create_publisher(Float64, '/reference_x', 10)
        self.pose = self.create_publisher(PoseStamped, '/reference', 10)
        self.params = self.create_client(SetParameters, '/p_controller/set_parameters')

    def set_double(self, name, value):
        """-> (accepted, reason) from the node's own parameter service (the executor thread delivers the answer)."""
        if not self.params.wait_for_service(timeout_sec=20.0):
            return None, 'parameter service not found'
        request = SetParameters.Request(parameters=[Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))])
        future = self.params.call_async(request)
        end = time.monotonic() + 20.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not future.done():
            return None, 'no answer'
        result = future.result().results[0]
        return result.successful, result.reason


def main():
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f'{"PASS" if ok else "FAIL"}: {name}  ({detail})', flush=True)

    rclpy.init()
    sim, probe = SimRobot(), Probe()
    ex = SingleThreadedExecutor()
    ex.add_node(sim)
    ex.add_node(probe)
    threading.Thread(target=ex.spin, daemon=True).start()
    log = open('/tmp/check_p_controller_node.log', 'w')
    node = subprocess.Popen(['ros2', 'run', 'lab3_controllers', 'p_controller', '--ros-args', '-p', 'log_csv:=/tmp/check_p_controller.csv'],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def finished(timeout):
        """True once the commands have been zero for 4 s (the controller is done)."""
        end, quiet_since = time.monotonic() + timeout, None
        time.sleep(2.0)
        sim.cmd_log_start = len(sim.cmd_log)
        while time.monotonic() < end:
            time.sleep(0.1)
            if sim.cmd == (0.0, 0.0) and abs(sim.robot.v) < 0.005 and abs(sim.robot.w) < 0.02:
                quiet_since = quiet_since or time.monotonic()
                if time.monotonic() - quiet_since > 4.5:
                    return True
            else:
                quiet_since = None
        return False

    def send(pub, msg):
        for _ in range(2):
            pub.publish(msg)
            time.sleep(0.15)

    try:
        # nothing is tested before BOTH directions are really connected (discovery can be slow); otherwise fail here
        t0 = time.monotonic()
        connected = False
        while time.monotonic() - t0 < 150 and not connected:
            time.sleep(0.2)
            connected = bool(sim.cmd_log) and sim.odom_pub.get_subscription_count() > 0 and sim.map_pub.get_subscription_count() > 0                 and all(pub.get_subscription_count() > 0 for pub in (probe.yaw, probe.dist, probe.pose))
        print(f'node connected after {time.monotonic() - t0:.1f} s: {connected}', flush=True)
        if not connected:
            check('0 node connected to the robot and the reference topics', False, 'no connection within 150 s')
            return 1
        time.sleep(2.0)
        sim.cmd_log.clear()
        time.sleep(3.0)
        mx = max((max(abs(v), abs(w)) for _, v, w in sim.cmd_log), default=None)
        check('1 no reference: commands stay zero', mx == 0.0, f'largest command {mx}, {len(sim.cmd_log)} messages in 3 s')

        for name, target in (('2 heading pi/2 in the map frame', math.pi / 2), ('3 a second heading', -2.4)):
            send(probe.yaw, Float64(data=target))
            done = finished(60)
            e = abs(math.degrees(wrap_angle(target - sim.truth()[2])))
            check(name, done and e <= 2.0, f'true error {e:.2f} deg (limit 2), finished={done}')

        for name, d, side_limit in (('4 drive +0.40 m', 0.40, 3.0), ('5 drive -0.25 m', -0.25, 3.0)):
            x0, y0, th0 = sim.truth()
            send(probe.dist, Float64(data=d))
            done = finished(60)
            x, y, _ = sim.truth()
            along = (x - x0) * math.cos(th0) + (y - y0) * math.sin(th0)
            side = -(x - x0) * math.sin(th0) + (y - y0) * math.cos(th0)
            check(name, done and abs(d - along) <= 0.03 and abs(side) <= side_limit / 100,
                  f'travelled {along:.3f} m: error {100 * (d - along):+.1f} cm (limit 3), sideways {100 * side:+.1f} cm (limit 3), finished={done}')

        x0, y0, th0 = sim.truth()
        goal = (x0 + 0.35, y0 + 0.25, 0.8)
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = goal[0], goal[1]
        m.pose.orientation.z, m.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
        send(probe.pose, m)
        done = finished(120)
        x, y, th = sim.truth()
        ep, eh = math.hypot(goal[0] - x, goal[1] - y), abs(math.degrees(wrap_angle(goal[2] - th)))
        check('6 go to a pose', done and ep <= 0.04 and eh <= 3.0, f'position {100 * ep:.1f} cm (limit 4), heading {eh:.2f} deg (limit 3), finished={done}')

        good = probe.set_double('heading.kp', 1.5)
        bad = probe.set_double('heading.kp', -1.0)
        check('7 coefficient changed while running, invalid one refused', good[0] is True and bad[0] is False,
              f'valid: accepted={good[0]}; invalid: accepted={bad[0]}, reason={bad[1]!r}')

        send(probe.yaw, Float64(data=wrap_angle(sim.truth()[2] + 2.5)))
        t_wait = time.monotonic()
        while abs(sim.cmd[1]) < 0.5 and time.monotonic() - t_wait < 15.0:     # cut the odometry WHILE it is commanding at speed
            time.sleep(0.02)
        moving = abs(sim.cmd[1]) >= 0.5
        sim.cmd_log.clear()
        stopped_at = time.monotonic()
        sim.odom_on = False
        time.sleep(3.0)
        nonzero = [t - stopped_at for t, v, w in sim.cmd_log if v != 0.0 or w != 0.0]
        last = max(nonzero) if nonzero else 0.0
        check('8 odometry stops: zero command', moving and last <= 0.8 and sim.cmd == (0.0, 0.0),
              f'was moving={moving}, last non-zero command {last:.2f} s after the odometry stopped (limit 0.8)')
    finally:
        import signal
        try:
            os.killpg(node.pid, signal.SIGTERM)
            node.wait(timeout=10)
        except Exception:                    # noqa: BLE001
            pass
        ex.shutdown()
        rclpy.shutdown()
        log.close()
    print(f'passed: {sum(results)}   failed: {len(results) - sum(results)}')
    return 0 if all(results) and len(results) == 8 else 1


if __name__ == '__main__':
    sys.exit(main())
