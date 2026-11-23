#!/usr/bin/env python3
"""Odometry calibration helper for the real robot (also runs in Gazebo as a self-check).

  straight : drive a measured distance at low speed using /odom, then stop.
             Measure the REAL distance with a tape and run `calc-radius`.
  rotate   : turn in place by N degrees using /odom (accumulated, so 360+ works), then stop.
             Mark the floor/robot, measure the REAL angle, and run `calc-separation`.
  calc-radius      --odom D_odom --real D_real [--radius 0.034]
  calc-separation  --odom A_odom --real A_real [--separation 0.177046]

Order matters: calibrate the wheel radius first (straight), then the separation (rotation).
r_new = r * D_real / D_odom ;  b_new = b * A_odom / A_real  (first-order estimates).
Apply the result in the robot's driver / odometry configuration, NOT in controller gains.
Usage examples:
  python3 calibrate_odometry.py straight --distance 1.0 --speed 0.1 [--reverse]
  python3 calibrate_odometry.py rotate --angle 360 [--cw]
  python3 calibrate_odometry.py calc-radius --odom 1.00 --real 0.96
"""
import argparse
import math
import sys
import time


def calc(args):
    if args.mode == 'calc-radius':
        m = args.real / args.odom
        print(f'radius multiplier = {m:.4f}  ->  new wheel radius = {args.radius * m:.5f} m '
              f'(was {args.radius} m)')
    else:
        m = args.odom / args.real
        print(f'separation multiplier = {m:.4f}  ->  new wheel separation = '
              f'{args.separation * m:.6f} m (was {args.separation} m)')


def drive(args):
    import rclpy
    from geometry_msgs.msg import Twist
    sys.path.insert(0, __file__.rsplit('tools', 1)[0])
    from lab3_controllers.state_input import StateInput

    rclpy.init()
    node = rclpy.create_node('calibrate_odometry', parameter_overrides=[])
    node.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, args.sim_time)])
    st = {}

    def cb(pose):
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        st['p'] = (pose.position.x, pose.position.y, yaw)
        st['t'] = time.time()

    StateInput(node, args.state_topic, args.state_type, cb)     # Odometry / PoseStamped / Pose / ...
    pub = node.create_publisher(Twist, args.cmd_topic, 10)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def send(v, w):
        t = Twist(); t.linear.x, t.angular.z = float(v), float(w); pub.publish(t)

    try:
        while 'p' not in st:
            rclpy.spin_once(node, timeout_sec=0.1)
        x0, y0, yaw0 = st['p']
        last_yaw, turned = yaw0, 0.0
        sign = -1.0 if (args.reverse if args.mode == 'straight' else args.cw) else 1.0
        target = args.distance if args.mode == 'straight' else math.radians(args.angle)
        t_end = time.time() + 120
        done = 0.0
        aborted = False
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.time() - st['t'] > args.stale:
                print(f'odometry is stale (> {args.stale} s), aborting'); aborted = True; break
            x, y, yaw = st['p']
            turned += math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
            last_yaw = yaw
            if args.mode == 'straight':
                done = math.hypot(x - x0, y - y0)
                rem = target - done
                v = min(args.speed, max(0.03, 1.0 * rem)) if rem > 0.0 else 0.0
                send(sign * v, 0.0)
            else:
                done = abs(turned)
                rem = target - done
                w = min(args.w_speed, max(0.15, 1.5 * rem)) if rem > 0.0 else 0.0
                send(0.0, sign * w)
            if rem <= 0.0:
                break
        for _ in range(5):
            send(0.0, 0.0); spin(0.05)
        spin(1.5)                                   # let it come to rest
        if aborted:
            print('ABORTED: no valid measurement'); sys.exit(2)
        x, y, yaw = st['p']
        if args.mode == 'straight':
            res = math.hypot(x - x0, y - y0)
            print(f'ODOM distance travelled: {res:.4f} m (commanded {target:.3f} m)')
            print('Now measure the REAL distance, then: calc-radius --odom '
                  f'{res:.4f} --real <measured>')
        else:
            res = math.degrees(abs(turned))
            print(f'ODOM rotation: {res:.2f} deg (commanded {args.angle:.1f} deg)')
            print('Now measure the REAL angle (marks on floor), then: calc-separation --odom '
                  f'{res:.2f} --real <measured>')
    finally:
        try:
            for _ in range(3):
                send(0.0, 0.0); spin(0.02)
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('mode', choices=['straight', 'rotate', 'calc-radius', 'calc-separation'])
    ap.add_argument('--distance', type=float, default=1.0)
    ap.add_argument('--speed', type=float, default=0.1)
    ap.add_argument('--angle', type=float, default=360.0)
    ap.add_argument('--w-speed', type=float, default=0.4)
    ap.add_argument('--reverse', action='store_true')
    ap.add_argument('--cw', action='store_true')
    ap.add_argument('--state-topic', default='/odom', help='real robot: /diff_cont/odom (wheel odometry from the Pi)')
    ap.add_argument('--state-type', default='auto', help='auto | odometry | pose_stamped | pose | pose_cov')
    ap.add_argument('--cmd-topic', default='/diff_cont/cmd_vel_unstamped')
    ap.add_argument('--stale', type=float, default=1.5, help='abort if /odom is silent for this long [s]')
    ap.add_argument('--sim-time', action='store_true', help='use in Gazebo')
    ap.add_argument('--odom', type=float)
    ap.add_argument('--real', type=float)
    ap.add_argument('--radius', type=float, default=0.034)
    ap.add_argument('--separation', type=float, default=0.177046)
    a = ap.parse_args()
    if a.mode.startswith('calc'):
        if a.odom is None or a.real is None:
            sys.exit('--odom and --real are required')
        calc(a)
    else:
        drive(a)
