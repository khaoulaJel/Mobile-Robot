#!/usr/bin/env python3
"""Reference controller: the reference comes from a TOPIC, the robot state comes from a TOPIC, the coefficients are
ROS parameters that can be loaded from a file or changed while the robot runs.

    /reference        geometry_msgs/PoseStamped   go to (x, y) and finish with the heading of the pose
    /reference_yaw    std_msgs/Float64            turn in place to this heading (rad, or degrees if use_degrees)
    <state_topic>     Odometry / PoseStamped / Pose / PoseWithCovarianceStamped (auto-detected)
    -> <cmd_topic>    geometry_msgs/Twist         v (m/s) and w (rad/s)

The newest reference wins. Until the first reference arrives the node commands zero.

Coefficients (kv, kw, k_yaw, kp, tolerances, limits, ...) are the parameters listed in tracker.py (DEFAULTS and
YAW_DEFAULTS). Give them in a file when you start the node, e.g. the numbers you receive on the robot:

    ros2 launch lab3_controllers reference.launch.py config:=reference_controller.yaml
    ros2 param set /reference_controller kv 0.6          # changes the controller while it runs

A parameter change restarts the current reference from the robot's present pose (the controller state is rebuilt).

`relative_to_start: true` reads the references in the frame of the robot's first measured pose instead of the
frame of the state topic.
"""

import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from lab3_controllers.common import slew, wrap_angle, yaw_from_quaternion
from lab3_controllers.state_input import StateInput, StateInputError
from lab3_controllers.tracker import DEFAULTS as PATH_DEFAULTS, DONE, YAW_DEFAULTS, PathTracker, YawTracker

# Coefficients that only the yaw controller has (the others are shared with the path controller).
YAW_ONLY_KEYS = [k for k in YAW_DEFAULTS if k not in PATH_DEFAULTS]
ALL_COEFFICIENTS = {**PATH_DEFAULTS, **{k: YAW_DEFAULTS[k] for k in YAW_ONLY_KEYS}}


class ReferenceController(Node):
    def __init__(self):
        super().__init__('reference_controller')
        d = self.declare_parameter
        d('state_topic', '/diff_cont/odom')
        d('state_type', 'auto')
        d('state_wait_timeout', 60.0)
        d('state_jump_warn', 0.3)
        d('cmd_topic', '/cmd_vel_tracker')
        d('reference_topic', '/reference')
        d('reference_yaw_topic', '/reference_yaw')
        d('use_degrees', False)               # unit of the /reference_yaw number
        d('relative_to_start', False)         # references are relative to the first measured pose
        d('max_linear_accel', 0.4)            # m/s^2, 0 = off
        d('max_angular_accel', 3.0)           # rad/s^2, 0 = off
        d('rate', 20.0)                       # Hz
        d('state_timeout', 0.5)               # s without state -> stop
        d('start_delay', 1.0)                 # s
        for key, val in ALL_COEFFICIENTS.items():     # the coefficients: gains, limits, tolerances, compensation
            d(key, val)

        def g(n):
            return self.get_parameter(n).value
        self.coef = {k: g(k) for k in ALL_COEFFICIENTS}
        self.dt = 1.0 / float(g('rate'))
        self.timeout = float(g('state_timeout'))
        self.start_delay = float(g('start_delay'))
        self.v_acc, self.w_acc = float(g('max_linear_accel')), float(g('max_angular_accel'))
        self.relative = bool(g('relative_to_start'))
        self.degrees = bool(g('use_degrees'))
        self.v_cmd = self.w_cmd = 0.0

        self.pose = None                      # latest (x, y, yaw)
        self.last_state_time = None
        self.last_state_wall = None
        self.origin = None                    # first pose, for relative references
        self.t_start = None
        self.kind = None                      # None | 'pose' | 'yaw'
        self.ref = None                       # the current reference (as received, before the frame change)
        self.tracker = None
        self.rebuild = False
        self._last_tick = None

        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.state_input = StateInput(self, g('state_topic'), g('state_type'), self.on_state,
                                      float(g('state_wait_timeout')), float(g('state_jump_warn')))
        self.create_subscription(PoseStamped, g('reference_topic'), self.on_pose_reference, 10)
        self.create_subscription(Float64, g('reference_yaw_topic'), self.on_yaw_reference, 10)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            f'reference_controller: reference {g("reference_topic")} (pose) / {g("reference_yaw_topic")} (yaw), '
            f'state {g("state_topic")}, command {g("cmd_topic")}')

    # ---- inputs ------------------------------------------------------------------------------
    def on_state(self, pose):
        p, q = pose.position, pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.last_state_time = self.get_clock().now()
        self.last_state_wall = time.monotonic()
        if self.origin is None:
            self.origin = self.pose

    def on_pose_reference(self, msg):
        q = msg.pose.orientation
        self.kind = 'pose'
        self.ref = (msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.rebuild = True
        self.get_logger().info(f'new pose reference x={self.ref[0]:.3f} y={self.ref[1]:.3f} '
                               f'yaw={math.degrees(self.ref[2]):.1f} deg')

    def on_yaw_reference(self, msg):
        self.kind = 'yaw'
        self.ref = (0.0, 0.0, math.radians(msg.data) if self.degrees else float(msg.data))
        self.rebuild = True
        self.get_logger().info(f'new yaw reference {math.degrees(self.ref[2]):.1f} deg')

    def on_parameters(self, params):
        """Coefficients can be changed while running: the controller is rebuilt with the new numbers."""
        for p in params:
            if p.name in self.coef:
                self.coef[p.name] = p.value
                self.rebuild = True
                self.get_logger().info(f'coefficient {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ---- controller construction ---------------------------------------------------------------
    def world_reference(self):
        """The reference in the frame of the state topic."""
        x, y, th = self.ref
        if not self.relative:
            return x, y, th
        x0, y0, th0 = self.origin
        c, s = math.cos(th0), math.sin(th0)
        return x0 + c * x - s * y, y0 + s * x + c * y, wrap_angle(th0 + th)

    def build_tracker(self):
        """(Re)create the controller for the current reference and coefficients, started at the present pose."""
        self.rebuild = False
        gx, gy, gth = self.world_reference()
        try:
            if self.kind == 'pose':
                self.tracker = PathTracker([(gx, gy, gth)], {k: self.coef[k] for k in PATH_DEFAULTS})
                self.tracker.start(self.pose)
            else:
                self.tracker = YawTracker({k: self.coef[k] for k in YAW_DEFAULTS})
        except ValueError as e:               # e.g. a gain with the wrong sign: keep stopped, tell the user
            self.tracker = None
            self.get_logger().error(f'coefficients rejected: {e}')

    # ---- output -------------------------------------------------------------------------------
    def publish(self, v, w, limit=True):
        if limit:
            v = slew(self.v_cmd, v, self.v_acc, self.dt)
            w = slew(self.w_cmd, w, self.w_acc, self.dt)
        self.v_cmd, self.w_cmd = v, w
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    def control_step(self):
        t_m = time.monotonic()
        if self._last_tick is not None and t_m - self._last_tick > 0.4:
            self.get_logger().warn(f'control loop stalled for {t_m - self._last_tick:.2f} s '
                                   '(machine paused or an executor callback blocked)')
        self._last_tick = t_m
        now = self.get_clock().now()
        if self.pose is None:
            self.publish(0.0, 0.0)
            return
        if self.t_start is None:
            self.t_start = now
            self.get_logger().info(f'first state x={self.pose[0]:.3f} y={self.pose[1]:.3f} '
                                   f'yaw={math.degrees(self.pose[2]):.1f} deg')
        if (now - self.t_start).nanoseconds * 1e-9 < self.start_delay:
            self.publish(0.0, 0.0)
            return
        age = max((now - self.last_state_time).nanoseconds * 1e-9, time.monotonic() - self.last_state_wall)
        if age > self.timeout:
            self.get_logger().warn('state is stale, stopping', throttle_duration_sec=2.0)
            self.publish(0.0, 0.0, limit=False)
            return
        if self.kind is None:                 # no reference yet
            self.publish(0.0, 0.0)
            return
        if self.rebuild:
            self.build_tracker()
        if self.tracker is None:
            self.publish(0.0, 0.0)
            return

        if self.kind == 'pose':
            if self.tracker.mode == DONE:
                self.publish(0.0, 0.0)
                return
            v, w = self.tracker.step(self.pose, self.dt)
        else:
            v, w = 0.0, self.tracker.step(self.pose[2], self.world_reference()[2], self.dt)
        for event in self.tracker.pop_events():
            self.get_logger().info(event)
        self.publish(v, w)

    def stop(self):
        try:
            for _ in range(3):                # repeat so the zero command is not lost on shutdown
                self.publish(0.0, 0.0, limit=False)
                time.sleep(0.02)
        except Exception:                     # noqa: BLE001
            pass


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves so the final zero command can still be published.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        node = ReferenceController()
    except StateInputError as e:
        rclpy.logging.get_logger('reference_controller').error(str(e))
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
