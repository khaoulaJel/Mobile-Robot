#!/usr/bin/env python3
"""Proportional yaw controller.

Subscribes : /odom (nav_msgs/Odometry)            robot state
Publishes  : /diff_cont/cmd_vel_unstamped (Twist) angular.z = Kp * (yaw_ref - yaw)
Reference  : parameter `yaw_ref` (rad, or deg if `use_degrees`), changeable at run
             time with `ros2 param set /yaw_controller yaw_ref 1.57`, or by
             publishing a std_msgs/Float64 (radians) on `yaw_ref_topic`.
"""

import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from lab3_controllers.state_input import StateInput, StateInputError
from lab3_controllers.common import slew, wrap_angle, yaw_from_quaternion
from lab3_controllers.tracker import COMPENSATION_KEYS, YAW_DEFAULTS, YawTracker


class YawController(Node):
    def __init__(self):
        super().__init__('yaw_controller')
        self.declare_parameter('state_topic', '/odom')   # real robot: /diff_cont/odom (see config/hardware.yaml)
        self.declare_parameter('state_type', 'auto')     # auto | odometry | pose_stamped | pose | pose_cov
        self.declare_parameter('state_wait_timeout', 60.0)  # s to wait for the state topic, 0 = forever (then exit code 2)
        self.declare_parameter('state_jump_warn', 0.3)     # m, warn when the state position jumps
        self.declare_parameter('cmd_topic', '/diff_cont/cmd_vel_unstamped')
        self.declare_parameter('yaw_ref_topic', '/yaw_ref')
        self.declare_parameter('yaw_ref', 1.5708)
        self.declare_parameter('use_degrees', False)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)   # rad/s
        self.declare_parameter('min_angular_speed', 0.0)   # rad/s, deadband compensation
        self.declare_parameter('max_angular_accel', 3.0)   # rad/s^2, 0 = off
        self.declare_parameter('tolerance', 0.005)         # rad (~0.3 deg)
        self.declare_parameter('rate', 20.0)               # Hz
        self.declare_parameter('state_timeout', 0.5)       # s without state -> stop
        # slow-plant / robot-filter compensation (all off by default; see tracker.py and config/lab_robot.yaml)
        for key in COMPENSATION_KEYS:
            self.declare_parameter(key, YAW_DEFAULTS[key])

        self.yaw = None
        self.last_state_time = None
        self.last_state_wall = None
        self.reached = False
        self._load_params()

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.state_input = StateInput(self, self.get_parameter('state_topic').value,
                                      self.get_parameter('state_type').value, self.on_state,
                                      float(self.get_parameter('state_wait_timeout').value),
                                      float(self.get_parameter('state_jump_warn').value))
        self.create_subscription(
            Float64, self.get_parameter('yaw_ref_topic').value, self.on_ref_topic, 10)
        self.add_on_set_parameters_callback(self.on_params)
        self._last_tick = None
        self.create_timer(1.0 / self.rate, self.control_step)
        self.get_logger().info(
            f'yaw_controller: ref={math.degrees(self.ref):.1f} deg kp={self.kp} '
            f'wmax={self.w_max} tol={math.degrees(self.tol):.2f} rad-deg')

    def _load_params(self):
        def g(n):
            return self.get_parameter(n).value
        self.kp = float(g('kp'))
        self.w_max = float(g('max_angular_speed'))
        self.w_min = float(g('min_angular_speed'))
        self.tol = float(g('tolerance'))
        self.w_acc = float(g('max_angular_accel'))
        self.w_cmd = 0.0
        self.rate = float(g('rate'))
        self.timeout = float(g('state_timeout'))
        ref = float(g('yaw_ref'))
        self.ref = math.radians(ref) if g('use_degrees') else ref
        self.tracker = YawTracker(dict(
            kp=self.kp, max_angular_speed=self.w_max, min_angular_speed=self.w_min, tolerance=self.tol,
            **{k: g(k) for k in COMPENSATION_KEYS}))

    def on_params(self, params):
        for p in params:
            if p.name == 'yaw_ref':
                deg = self.get_parameter('use_degrees').value
                self.ref = math.radians(p.value) if deg else float(p.value)
                self.reached = False
            elif p.name == 'kp':
                self.kp = float(p.value)
                self.tracker.p['kp'] = self.kp
            elif p.name == 'max_angular_speed':
                self.w_max = float(p.value)
                self.tracker.p['max_angular_speed'] = self.w_max
            elif p.name == 'min_angular_speed':
                self.w_min = float(p.value)
                self.tracker.p['min_angular_speed'] = self.w_min
            elif p.name == 'max_angular_accel':
                self.w_acc = float(p.value)
            elif p.name == 'tolerance':
                self.tol = float(p.value)
                self.tracker.p['tolerance'] = self.tol
            elif p.name in COMPENSATION_KEYS:
                self.tracker.p[p.name] = p.value
        return SetParametersResult(successful=True)

    def on_ref_topic(self, msg):
        self.ref = msg.data
        self.reached = False

    def on_state(self, pose):
        q = pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.last_state_time = self.get_clock().now()
        self.last_state_wall = time.monotonic()      # wall clock: still works if sim time freezes

    def control_step(self):
        t_m = time.monotonic()
        if self._last_tick is not None and t_m - self._last_tick > 0.4:
            self.get_logger().warn(f'control loop stalled for {t_m - self._last_tick:.2f} s '
                                   '(machine paused or an executor callback blocked)')
        self._last_tick = t_m
        cmd = Twist()
        if self.yaw is None:
            self.w_cmd = 0.0
            self.cmd_pub.publish(cmd)
            return
        age = max((self.get_clock().now() - self.last_state_time).nanoseconds * 1e-9,
                  time.monotonic() - self.last_state_wall)
        if age > self.timeout:
            self.get_logger().warn('state is stale, stopping', throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            return
        w = self.tracker.step(self.yaw, self.ref, 1.0 / self.rate)
        self.w_cmd = slew(self.w_cmd, w, self.w_acc, 1.0 / self.rate)
        cmd.angular.z = self.w_cmd
        self.cmd_pub.publish(cmd)
        for event in self.tracker.pop_events():
            self.get_logger().info(event)
        if self.tracker.reached and not self.reached:
            self.reached = True
            self.get_logger().info(
                f'yaw reached: yaw={math.degrees(self.yaw):.2f} deg, '
                f'error={math.degrees(wrap_angle(self.ref - self.yaw)):.2f} deg')
        elif not self.tracker.reached:
            self.reached = False

    def stop(self):
        try:
            for _ in range(3):      # repeat so the zero command is not lost on shutdown
                self.cmd_pub.publish(Twist())
                time.sleep(0.02)
        except Exception:
            pass


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves: rclpy's default handler shuts the context down first,
    # which would make the final zero-velocity command impossible to publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        node = YawController()
    except StateInputError as e:      # bad state_type, or an unsupported / ambiguous state topic
        rclpy.logging.get_logger('yaw_controller').error(str(e))
        rclpy.shutdown()
        sys.exit(2)
    exit_code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except StateInputError as e:
        node.get_logger().error(str(e))
        exit_code = 2
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
