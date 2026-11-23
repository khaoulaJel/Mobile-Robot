#!/usr/bin/env python3
"""Symbolic controller node: drives a 1 m-diameter circle (tangent start) with a controller synthesized from a
finite abstraction of the robot (see symbolic.py).

Every `tau` seconds the node measures the state, finds its cell, checks whether the current goal arc is reached
(then it moves on to the next one), looks up the input (v, w) that the synthesized policy prescribes for that
cell and holds it for `tau` (zero-order hold, as in the abstraction). If the state leaves the modelled tube or no
winning input exists it stops and reports it: the guarantee only holds inside the tube.

Subscribes : state topic (Odometry / PoseStamped / ...), auto-detected like the other nodes
Publishes  : /diff_cont/cmd_vel_unstamped (geometry_msgs/Twist)
"""

import math
import os
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from lab3_controllers.common import slew, wrap_angle, yaw_from_quaternion
from lab3_controllers.state_input import StateInput, StateInputError
from lab3_controllers.symbolic import Policy


def default_policy_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('lab3_controllers'), 'config', 'symbolic_circle.npz')
    except Exception:       # noqa: BLE001  (running from a source tree)
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'symbolic_circle.npz')


class SymbolicController(Node):
    def __init__(self):
        super().__init__('symbolic_controller')
        d = self.declare_parameter
        d('state_topic', '/odom')
        d('state_type', 'auto')
        d('state_wait_timeout', 60.0)
        d('state_jump_warn', 0.3)
        d('cmd_topic', '/diff_cont/cmd_vel_unstamped')
        d('policy_file', '')                 # '' = the one installed with the package
        d('rate', 20.0)                      # Hz, publishing rate (the policy itself updates every tau)
        d('state_timeout', 0.5)              # s without state -> stop
        d('start_delay', 1.0)                # s
        d('max_linear_accel', 0.0)           # m/s^2, 0 = off (the abstraction assumes the input is applied at once)
        d('max_angular_accel', 0.0)          # rad/s^2, 0 = off
        d('robust_lookup', True)             # neighbour-cell fallback when the measured cell has no input
                                             # (noise, un-modelled lag): OUTSIDE the formal guarantee, logged
        d('log_csv', '')

        def g(n):
            return self.get_parameter(n).value
        path = g('policy_file') or default_policy_path()
        t0 = time.time()
        self.policy = Policy.load(path)
        self.tau = self.policy.tau
        self.n_goals = len(self.policy.goals)
        self.get_logger().info(
            f'symbolic_controller: {self.policy.N} cells, {len(self.policy.inputs)} inputs, tau={self.tau} s, '
            f'{self.n_goals} goal arcs (loaded {path} in {time.time() - t0:.1f} s)')
        self.dt = 1.0 / float(g('rate'))
        self.timeout = float(g('state_timeout'))
        self.start_delay = float(g('start_delay'))
        self.v_acc, self.w_acc = float(g('max_linear_accel')), float(g('max_angular_accel'))
        self.robust = bool(g('robust_lookup'))
        self.n_robust = 0
        self.pose = None
        self.last_state_time = None
        self.last_state_wall = None
        self.origin = None               # (x0, y0, yaw0): the circle starts tangentially at this pose
        self.t_start = None
        self.k = 0                       # current goal arc
        self.u = (0.0, 0.0)
        self.next_update = None
        self.mode = 'RUN'                # RUN | DONE | FAILED
        self.v_cmd = self.w_cmd = 0.0
        self.log = None
        if g('log_csv'):
            self.log = open(g('log_csv'), 'w', buffering=1)
            self.log.write('t,x,y,yaw,radial_err,goal,cell,v,w,mode\n')
        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.state_input = StateInput(self, g('state_topic'), g('state_type'), self.on_state,
                                      float(g('state_wait_timeout')), float(g('state_jump_warn')))
        self._last_tick = None
        self.create_timer(self.dt, self.control_step)

    def on_state(self, pose):
        p, q = pose.position, pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.last_state_time = self.get_clock().now()
        self.last_state_wall = time.monotonic()

    def to_circle_frame(self):
        x0, y0, th0 = self.origin
        c, s = math.cos(th0), math.sin(th0)
        x, y, th = self.pose
        return (c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0), wrap_angle(th - th0))

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
            self.origin = self.pose
            self.next_update = now
            self.get_logger().info(
                f'circle starts tangentially at x={self.pose[0]:.3f} y={self.pose[1]:.3f} '
                f'yaw={math.degrees(self.pose[2]):.1f} deg')
        if (now - self.t_start).nanoseconds * 1e-9 < self.start_delay:
            self.publish(0.0, 0.0)
            return
        age = max((now - self.last_state_time).nanoseconds * 1e-9, time.monotonic() - self.last_state_wall)
        if age > self.timeout:
            self.get_logger().warn('state is stale, stopping', throttle_duration_sec=2.0)
            self.publish(0.0, 0.0, limit=False)
            return
        if self.mode != 'RUN':
            self.publish(0.0, 0.0)
            return
        x, y, th = self.to_circle_frame()
        radial = self.radial_error(x, y)
        q = self.policy.state_index(x, y, th)            # the cell the robot is in (-1 = outside the tube)
        if now >= self.next_update:                      # once per tau: choose a new input
            self.next_update = self.next_update + Duration(seconds=self.tau)
            if self.next_update < now:                   # fell behind (e.g. paused): do not catch up in a burst
                self.next_update = now + Duration(seconds=self.tau)
            self.update_input(q, x, y, th, radial)
        if self.mode == 'RUN':
            self.publish(*self.u)
        else:
            self.publish(0.0, 0.0)
        if self.log:
            t = (now - self.t_start).nanoseconds * 1e-9
            self.log.write(f'{t:.3f},{x:.4f},{y:.4f},{th:.4f},{radial:.4f},{self.k},{q},{self.v_cmd:.4f},'
                           f'{self.w_cmd:.4f},{self.mode}\n')

    def radial_error(self, x, y):
        """Distance from the circle (positive outside), for logging."""
        radius = float(self.policy.meta['radius'])
        cx, cy = float(self.policy.meta.get('centre_x', 0.0)), float(self.policy.meta.get('centre_y', radius))
        return math.hypot(x - cx, y - cy) - radius

    def update_input(self, q, x, y, th, radial):
        """The symbolic controller proper: cell -> goal check -> input from the synthesized policy."""
        # 1. goal arcs reached? (several can be passed at once)
        if q >= 0:
            while self.k < self.n_goals and self.policy.goal_reached(self.k, q):
                self.get_logger().info(f'goal arc {self.k + 1}/{self.n_goals} reached '
                                       f'(radial error {radial * 100:.1f} cm)')
                self.k += 1
        if self.k >= self.n_goals:
            self.mode = 'DONE'
            self.get_logger().info(f'lap complete ({self.n_robust} neighbour-cell fallbacks used)')
            return
        # 2. the policy's input for this cell and goal
        u = self.policy.action(self.k, q) if q >= 0 else None
        # 3. no input: optionally use the best neighbouring cell (outside the formal guarantee)
        if u is None and self.robust:
            u = self.policy.robust_action(self.k, x, y, th)
            if u is not None:
                self.n_robust += 1
                self.get_logger().warn(
                    f'no input for the measured cell (radial error {radial * 100:.1f} cm): using the best '
                    f'neighbouring cell - outside the formal guarantee', throttle_duration_sec=5.0)
        # 4. still nothing: the robot left the tube, stop
        if u is None:
            self.mode = 'FAILED'
            self.get_logger().error(
                f'left the modelled tube or no winning input (radial error {radial * 100:.1f} cm, heading '
                f'{math.degrees(th):.0f} deg): the guarantee no longer holds, stopping')
        else:
            self.u = u

    def stop(self):
        try:
            for _ in range(3):
                self.publish(0.0, 0.0, limit=False)
                time.sleep(0.02)
        except Exception:          # noqa: BLE001
            pass
        if self.log:
            self.log.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        node = SymbolicController()
    except StateInputError as e:
        rclpy.logging.get_logger('symbolic_controller').error(str(e))
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
