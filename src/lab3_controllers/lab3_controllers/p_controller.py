#!/usr/bin/env python3
"""The P controller for the real robot (servo.py): fed by what the robot MEASURES.

    <odom_topic>     nav_msgs/Odometry           wheel odometry: pose, measured speed and turn rate     (/diff_cont/odom)
    <map_topic>      geometry_msgs/PoseStamped   SLAM pose, the professor's frame ('' = use the odometry frame)  (/robot_pose)
    /reference_yaw   std_msgs/Float64            turn to this heading in the map frame (rad, or degrees with use_degrees)
    /reference_x     std_msgs/Float64            drive this distance along the present heading (m, negative = backwards)
    /reference       geometry_msgs/PoseStamped   go to (x, y) in the map frame and finish with that heading
    -> <cmd_topic>   geometry_msgs/Twist         (/cmd_vel_tracker: the joystick overrides it in the professor's twist_mux)

The newest reference wins; before the first one the node sends zero. It also sends zero when the odometry is older than
state_timeout, until the first map pose has been paired with the odometry, and when it is shut down.
Coefficients are the parameters heading.<name> and linear.<name> (names and meaning: SERVO_DEFAULTS in servo.py); they can be
changed while it runs. log_csv writes every control step (state, command, reference, learned values) to a file.
"""
import csv
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from lab3_controllers.common import wrap_angle, yaw_from_quaternion
from lab3_controllers.servo import LINEAR_SERVO_DEFAULTS, SERVO_DEFAULTS, MapOdomFusion, PoseServo

AXES = {'heading': SERVO_DEFAULTS, 'linear': LINEAR_SERVO_DEFAULTS}


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class PController(Node):
    def __init__(self):
        super().__init__('p_controller')
        d = self.declare_parameter
        d('odom_topic', '/diff_cont/odom')
        d('map_topic', '/robot_pose')
        d('cmd_topic', '/cmd_vel_tracker')
        d('reference_yaw_topic', '/reference_yaw')
        d('reference_x_topic', '/reference_x')
        d('reference_topic', '/reference')
        d('use_degrees', False)
        d('rate', 20.0)
        d('state_timeout', 0.5)               # s without odometry -> stop
        d('map_warn_after', 3.0)              # s without a map pose -> warn (the odometry carries on)
        d('map_blend', 0.5)                   # share of a SLAM correction applied at once
        d('steer_gain', 1.5)                  # 1/s   heading correction while driving
        d('face_tolerance', 0.12)             # rad   turn to the point this well before driving
        d('position_tolerance', 0.02)         # m
        d('log_csv', '')
        for axis, defaults in AXES.items():
            for key, val in defaults.items():
                d(f'{axis}.{key}', val)

        def g(n):
            return self.get_parameter(n).value
        self.dt = 1.0 / float(g('rate'))
        self.timeout = float(g('state_timeout'))
        self.map_warn_after = float(g('map_warn_after'))
        self.degrees = bool(g('use_degrees'))
        use_map = bool(g('map_topic'))
        self.fusion = MapOdomFusion(use_map=use_map, blend=float(g('map_blend')))
        self.servo = PoseServo({k: g(f'heading.{k}') for k in SERVO_DEFAULTS},
                               {k: g(f'linear.{k}') for k in LINEAR_SERVO_DEFAULTS},
                               steer_gain=float(g('steer_gain')), face_tol=float(g('face_tolerance')),
                               pos_tol=float(g('position_tolerance')))
        self.kind = None                      # None | 'yaw' | 'x' | 'pose'
        self.ref = None
        self.new_ref = False
        self.done = True
        self.odom_wall = None
        self.map_wall = None
        self.log = None
        if g('log_csv'):
            self.log_file = open(g('log_csv'), 'w', newline='', buffering=1)
            self.log = csv.writer(self.log_file)
            self.log.writerow(['t', 'x', 'y', 'yaw', 'v', 'w', 'cmd_v', 'cmd_w', 'kind', 'ref', 'error', 'heading_state',
                               'u_min', 'lead', 'gain', 'linear_u_min', 'linear_lead'])
        self.t0 = time.monotonic()

        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic'), 10)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, qos_profile_sensor_data)
        if use_map:
            self.create_subscription(PoseStamped, g('map_topic'), self.on_map, qos_profile_sensor_data)
        self.create_subscription(Float64, g('reference_yaw_topic'), self.on_ref_yaw, 10)
        self.create_subscription(Float64, g('reference_x_topic'), self.on_ref_x, 10)
        self.create_subscription(PoseStamped, g('reference_topic'), self.on_ref_pose, 10)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f'p_controller: odom {g("odom_topic")}, map {g("map_topic") or "(none: odometry frame)"}, '
                               f'command {g("cmd_topic")}')

    # ---- inputs ----------------------------------------------------------------------------
    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.fusion.add_odom(stamp_seconds(m.header), (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)),
                             m.twist.twist.linear.x, m.twist.twist.angular.z)
        if self.odom_wall is None:
            self.get_logger().info('wheel odometry received')
        self.odom_wall = time.monotonic()

    def on_map(self, m):
        p, q = m.pose.position, m.pose.orientation
        was_ready = self.fusion.offset is not None
        if self.fusion.add_map(stamp_seconds(m.header), (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))):
            self.map_wall = time.monotonic()
            if not was_ready:
                x, y, th = self.fusion.state()[0]
                self.get_logger().info(f'map frame locked: x={x:.3f} y={y:.3f} heading={math.degrees(th):.1f} deg')
        elif self.fusion.rejected % 50 == 1:
            self.get_logger().warn('map pose cannot be matched to the odometry by time stamp (are both stamped with the same clock?)')

    def _accept(self, kind, ref, text):
        self.kind, self.ref, self.new_ref = kind, ref, True
        self.get_logger().info(f'new reference: {text}')

    def on_ref_yaw(self, m):
        yaw = math.radians(m.data) if self.degrees else float(m.data)
        if math.isfinite(yaw):
            self._accept('yaw', wrap_angle(yaw), f'heading {math.degrees(wrap_angle(yaw)):.1f} deg')

    def on_ref_x(self, m):
        if math.isfinite(m.data) and abs(m.data) <= 5.0:
            self._accept('x', float(m.data), f'drive {m.data:+.3f} m along the present heading')
        else:
            self.get_logger().error('reference_x must be a finite distance of at most 5 m')

    def on_ref_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        goal = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if all(math.isfinite(v) for v in goal):
            self._accept('pose', goal, f'pose x={goal[0]:.3f} y={goal[1]:.3f} heading={math.degrees(goal[2]):.1f} deg')

    def on_parameters(self, params):
        for p in params:
            axis, _, key = p.name.partition('.')
            if axis in AXES and key in AXES[axis]:
                servo = self.servo.heading if axis == 'heading' else self.servo.linear
                trial = dict(servo.p, **{key: p.value})
                if trial['kp'] <= 0 or trial['u_max'] <= 0 or trial['tol'] <= 0 or trial['u_min_floor'] > trial['u_min_cap']:
                    return SetParametersResult(successful=False, reason='needs kp > 0, u_max > 0, tol > 0, u_min_floor <= u_min_cap')
                servo.p[key] = p.value
                if key == 'u_min':
                    servo.u_min, servo.ignored = p.value, 0.0
                if key == 'lead':
                    servo.lead = p.value
                if key == 'pulse_below':
                    servo.near = p.value
                self.get_logger().info(f'coefficient {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ---- control ---------------------------------------------------------------------------
    def publish(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    def start_reference(self, pose):
        x, y, th = pose
        self.new_ref, self.done = False, False
        self.servo.set_target(None)
        self.servo.heading.reset()
        self.servo.linear.reset()
        if self.kind == 'pose':
            self.servo.set_target(self.ref)
        elif self.kind == 'x':
            self.servo.set_distance(pose, self.ref)

    def control(self, pose, v, w):
        """-> (v command, w command, current error for the log)"""
        x, y, th = pose
        if self.kind == 'yaw':
            err = wrap_angle(self.ref - th)
            wc = self.servo.heading.update(err, w, self.dt)
            return 0.0, wc, err
        if self.kind == 'x':
            err = self.servo.remaining(pose)
            vc, wc = self.servo.update(pose, v, w, self.dt)
            return vc, wc, err
        vc, wc = self.servo.update(pose, v, w, self.dt)
        return vc, wc, math.hypot(self.ref[0] - x, self.ref[1] - y)

    def finished(self):
        if self.kind == 'yaw':
            return self.servo.heading.arrived
        return self.servo.phase == 'DONE'

    def report(self, pose):
        x, y, th = pose
        if self.kind == 'yaw':
            result = f'heading {math.degrees(th):.2f} deg, error {math.degrees(wrap_angle(self.ref - th)):+.2f} deg'
        elif self.kind == 'x':
            x0, y0, th0, dist = self.servo.line
            along = (x - x0) * math.cos(th0) + (y - y0) * math.sin(th0)
            side = -(x - x0) * math.sin(th0) + (y - y0) * math.cos(th0)
            result = f'travelled {along:.3f} m of {dist:.3f} m (error {100 * (dist - along):+.1f} cm, sideways {100 * side:+.1f} cm)'
        else:
            result = (f'position error {100 * math.hypot(self.ref[0] - x, self.ref[1] - y):.1f} cm, heading error '
                      f'{math.degrees(wrap_angle(self.ref[2] - th)):+.2f} deg')
        h, l = self.servo.heading, self.servo.linear
        self.get_logger().info(f'ARRIVED: {result} | learned: turn minimum {h.u_min:.3f} rad/s, coasting {h.lead:.2f} s, gain {h.gain:.2f}; '
                               f'drive minimum {l.u_min:.3f} m/s, coasting {l.lead:.2f} s')

    def step(self):
        now = time.monotonic()
        if self.odom_wall is None or now - self.odom_wall > self.timeout:
            if self.odom_wall is not None:
                self.get_logger().warn('odometry is stale, stopping', throttle_duration_sec=2.0)
            self.servo.heading.pause()
            self.servo.linear.pause()
            self.publish(0.0, 0.0)
            return
        if not self.fusion.ready():
            self.get_logger().warn('waiting for the first map pose (is the LiDAR running?)', throttle_duration_sec=5.0)
            self.publish(0.0, 0.0)
            return
        if self.fusion.use_map and now - self.map_wall > self.map_warn_after:
            self.get_logger().warn('no recent map pose: continuing on wheel odometry (drift is not corrected)', throttle_duration_sec=5.0)
        pose, v, w = self.fusion.state()
        if self.kind is None:
            self.publish(0.0, 0.0)
            return
        if self.new_ref:
            self.start_reference(pose)
        vc, wc, err = self.control(pose, v, w)
        self.publish(vc, wc)
        if self.finished() and not self.done:
            self.done = True
            self.report(pose)
        elif not self.finished():
            self.done = False
        if self.log:
            h, l = self.servo.heading, self.servo.linear
            self.log.writerow([f'{now - self.t0:.3f}', f'{pose[0]:.4f}', f'{pose[1]:.4f}', f'{pose[2]:.4f}', f'{v:.4f}', f'{w:.4f}',
                               f'{vc:.4f}', f'{wc:.4f}', self.kind, self.ref, f'{err:.4f}', h.state, f'{h.u_min:.4f}', f'{h.lead:.3f}',
                               f'{h.gain:.3f}', f'{l.u_min:.4f}', f'{l.lead:.3f}'])

    def stop(self):
        try:
            for _ in range(3):
                self.publish(0.0, 0.0)
                time.sleep(0.02)
        except Exception:                     # noqa: BLE001
            pass
        if self.log:
            self.log_file.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        node = PController()
    except ValueError as e:
        rclpy.logging.get_logger('p_controller').error(str(e))
        rclpy.shutdown()
        sys.exit(2)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
