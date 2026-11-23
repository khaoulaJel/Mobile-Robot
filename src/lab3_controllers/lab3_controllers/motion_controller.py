#!/usr/bin/env python3
"""Proportional waypoint / path-following controller.

The path is a list of reference points (x, y, theta), given either as the flat parameter
`waypoints` = [x0, y0, th0, x1, y1, th1, ...] (m, m, rad; theta in deg if `use_degrees`)
or generated with `path_shape: circle`. Points are tracked in order. `strategy` selects:

  stop_turn     (default, safest) at every point: drive there with a P law
                v = kv*d*cos(alpha), w = kw*alpha (turning in place first if the bearing error
                is large), STOP, rotate to the point's heading with w = k_yaw*(theta - yaw),
                then go to the next point. Every point is reached and aligned exactly.
  polar         Lyapunov polar-coordinate pose controller (rho, alpha, beta): reaches each pose
                in one smooth motion and hands over to the next point within `blend_radius`
                (corners are rounded). Fast.
  pure_pursuit  follows the polyline through the points with a look-ahead point; best for
                continuous paths such as a circle. Fast and accurate on curves.

`relative_to_start: true` interprets the path in the frame of the robot's initial pose.

Subscribes : /odom (nav_msgs/Odometry)
Publishes  : /diff_cont/cmd_vel_unstamped (geometry_msgs/Twist)
"""

import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from lab3_controllers.state_input import StateInput, StateInputError

from lab3_controllers.common import (
    circle_waypoints, slew, wrap_angle, yaw_from_quaternion)
from lab3_controllers.tracker import DEFAULTS as TRACKER_DEFAULTS, DONE, PathTracker

# Default path: 0.5 m square, counter-clockwise, ending at the start heading.
DEFAULT_WAYPOINTS = [
    0.5, 0.0, 0.0,
    0.5, 0.5, 1.5708,
    0.0, 0.5, 3.1416,
    0.0, 0.0, 0.0,
]


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        d = self.declare_parameter
        d('state_topic', '/odom')           # real robot: /diff_cont/odom (see config/hardware.yaml)
        d('state_type', 'auto')             # auto | odometry | pose_stamped | pose | pose_cov
        d('state_wait_timeout', 60.0)       # s to wait for the state topic, 0 = forever (then exit code 2)
        d('state_jump_warn', 0.3)           # m, warn when the state position jumps by more than this
        d('cmd_topic', '/diff_cont/cmd_vel_unstamped')
        for key, val in TRACKER_DEFAULTS.items():     # strategy, gains, limits, tolerances (see tracker.py)
            d(key, val)
        d('waypoints', DEFAULT_WAYPOINTS)
        d('use_degrees', False)
        d('relative_to_start', False)
        d('path_shape', 'waypoints')          # waypoints | circle
        d('circle_radius', 0.4)               # m
        d('circle_points', 24)                # waypoints per lap
        d('circle_direction', 1)              # +1 CCW, -1 CW
        d('circle_laps', 1)
        # acceleration limits are applied here (slew), the rest of the control law is in tracker.py
        d('max_linear_accel', 0.4)            # m/s^2, 0 = off
        d('max_angular_accel', 3.0)           # rad/s^2, 0 = off
        d('rate', 20.0)                       # Hz
        d('state_timeout', 0.5)               # s
        d('start_delay', 1.0)                 # s, wait before moving
        d('log_csv', '')                      # optional path for a data log

        def g(n):
            return self.get_parameter(n).value
        self.tparams = {k: g(k) for k in TRACKER_DEFAULTS}
        self.v_acc, self.w_acc = float(g('max_linear_accel')), float(g('max_angular_accel'))
        self.timeout = float(g('state_timeout'))
        self.start_delay = float(g('start_delay'))
        self.relative = bool(g('relative_to_start'))
        self.dt = 1.0 / float(g('rate'))
        self.v_cmd = self.w_cmd = 0.0

        if g('path_shape') == 'circle':
            wp = circle_waypoints(float(g('circle_radius')), int(g('circle_points')),
                                  int(g('circle_direction')), int(g('circle_laps')))
            self.relative = True      # a circle is always defined from the start pose
            scale = 1.0
        elif g('path_shape') == 'waypoints':
            wp = [float(v) for v in g('waypoints')]
            scale = math.pi / 180.0 if g('use_degrees') else 1.0
        else:
            raise ValueError("path_shape must be 'waypoints' or 'circle'")
        if len(wp) == 0 or len(wp) % 3 != 0:
            raise ValueError('waypoints must be a non-empty flat list [x,y,theta,...]')
        self.local_path = [(wp[i], wp[i + 1], wp[i + 2] * scale) for i in range(0, len(wp), 3)]
        self.path = list(self.local_path)       # world-frame path, fixed when motion starts
        # constructing the tracker validates strategy and polar gains (ValueError -> node does not start)
        self.tracker = PathTracker(self.path, self.tparams)
        self.strategy = self.tracker.strategy

        self.pose = None            # (x, y, yaw)
        self.last_state_time = None
        self.t_start = None
        self.last_state_wall = None
        self.log = None
        if g('log_csv'):
            self.log = open(g('log_csv'), 'w', buffering=1)
            self.log.write('t,x,y,yaw,gx,gy,gth,idx,mode,v,w\n')

        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.state_input = StateInput(self, g('state_topic'), g('state_type'), self.on_state,
                                      float(g('state_wait_timeout')), float(g('state_jump_warn')))
        self._last_tick = None
        self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            f'motion_controller: strategy={self.strategy}, {len(self.path)} points')

    # ---- helpers -----------------------------------------------------------
    def on_state(self, pose):
        p, q = pose.position, pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.last_state_time = self.get_clock().now()
        self.last_state_wall = time.monotonic()      # wall clock: still works if sim time freezes

    def fix_world_path(self):
        """Transform the path to the odom frame (relative mode) once, at the start pose."""
        if not self.relative:
            return
        x0, y0, th0 = self.pose
        c, s = math.cos(th0), math.sin(th0)
        self.path = [(x0 + c * gx - s * gy, y0 + s * gx + c * gy, wrap_angle(th0 + gth))
                     for gx, gy, gth in self.local_path]

    def publish(self, v, w, limit=True):
        if limit:
            v = slew(self.v_cmd, v, self.v_acc, self.dt)
            w = slew(self.w_cmd, w, self.w_acc, self.dt)
        self.v_cmd, self.w_cmd = v, w
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    # ---- control loop ------------------------------------------------------
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
            self.fix_world_path()
            self.tracker.set_path(self.path)
            self.tracker.start(self.pose)
            self.get_logger().info(
                f'start pose x={self.pose[0]:.3f} y={self.pose[1]:.3f} '
                f'yaw={math.degrees(self.pose[2]):.1f} deg')
        if (now - self.t_start).nanoseconds * 1e-9 < self.start_delay:
            self.publish(0.0, 0.0)
            return
        age_ros = (now - self.last_state_time).nanoseconds * 1e-9
        age_wall = time.monotonic() - self.last_state_wall
        if max(age_ros, age_wall) > self.timeout:
            self.get_logger().warn('state is stale, stopping', throttle_duration_sec=2.0)
            self.publish(0.0, 0.0, limit=False)     # immediate stop, no deceleration ramp
            return
        if self.tracker.mode == DONE:
            self.publish(0.0, 0.0)
            return

        x, y, yaw = self.pose
        v, w = self.tracker.step(self.pose, self.dt)
        for event in self.tracker.pop_events():
            self.get_logger().info(event)

        self.publish(v, w)
        if self.log:
            t = (now - self.t_start).nanoseconds * 1e-9
            tr = self.tracker
            self.log.write(f'{t:.3f},{x:.4f},{y:.4f},{yaw:.4f},{tr.gx:.4f},{tr.gy:.4f},'
                           f'{tr.gth:.4f},{tr.idx},{tr.mode},{self.v_cmd:.4f},'
                           f'{self.w_cmd:.4f}\n')

    def stop(self):
        try:
            for _ in range(3):      # repeat so the zero command is not lost on shutdown
                self.publish(0.0, 0.0, limit=False)
                time.sleep(0.02)
        except Exception:
            pass
        if self.log:
            self.log.close()


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves: rclpy's default handler shuts the context down first,
    # which would make the final zero-velocity command impossible to publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        node = MotionController()
    except StateInputError as e:      # bad state_type, or an unsupported / ambiguous state topic
        rclpy.logging.get_logger('motion_controller').error(str(e))
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
