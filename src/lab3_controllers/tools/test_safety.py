#!/usr/bin/env python3
"""Safety tests for the controllers. Needs ROS 2 only (no Gazebo). Exit code 0 = all passed.

A  stale odometry, wall clock      : odom stops -> zero velocity within 0.8 s and stays zero
B  stale odometry, sim clock runs  : same with use_sim_time:=true while /clock keeps ticking
C  shutdown zeros                  : SIGINT -> at least 3 zero commands are really received
D  invalid parameters              : bad strategy / bad polar gains -> process exits non-zero
E  no leftover controller process  : nothing of ours is still running afterwards
Each test prints PASS/FAIL with the measured numbers.
"""
import math
import os
import signal
import subprocess
import tempfile
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

NODE = ['ros2', 'run', 'lab3_controllers', 'motion_controller', '--ros-args']
results = []


def report(name, ok, detail):
    results.append(ok)
    print(f'{"PASS" if ok else "FAIL"}: {name}  ({detail})', flush=True)


class Harness:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('safety_test')
        self.cmds = []                       # (wall time, v, w)
        self.n.create_subscription(Twist, '/test_cmd', self.on_cmd, 10)
        self.odom = self.n.create_publisher(Odometry, '/test_odom', 10)
        self.pubs = {'odom': (self.odom, '/test_odom'),
                     'pose_stamped': (self.n.create_publisher(PoseStamped, '/test_pose_stamped', 10), '/test_pose_stamped'),
                     'pose': (self.n.create_publisher(Pose, '/test_pose', 10), '/test_pose'),
                     'pose_cov': (self.n.create_publisher(PoseWithCovarianceStamped, '/test_pose_cov', 10), '/test_pose_cov')}
        self.kind = 'odom'
        self.pos_x, self.frame = 0.0, 'odom'
        self.bad = self.n.create_publisher(String, '/test_badtype', 10)
        self.amb1 = self.n.create_publisher(PoseStamped, '/test_ambig', 10)
        # a second type on the same topic must come from another process (one node cannot mix types)
        self.amb2 = subprocess.Popen(['ros2', 'topic', 'pub', '--rate', '5', '/test_ambig', 'geometry_msgs/msg/Pose', '{}'],
                                     start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)
        self.clock = self.n.create_publisher(Clock, '/clock', 10)
        self.sim_t = 0.0
        self.last_clock = time.time()

    def on_cmd(self, m):
        self.cmds.append((time.time(), m.linear.x, m.angular.z))

    def publish_state(self):
        pub = self.pubs[self.kind][0]
        stamp = self.n.get_clock().now().to_msg()
        if self.kind == 'odom':
            m = Odometry(); m.header.stamp = stamp; m.pose.pose.orientation.w = 1.0
        elif self.kind == 'pose_stamped':
            m = PoseStamped(); m.header.stamp = stamp; m.pose.orientation.w = 1.0
            m.header.frame_id = self.frame; m.pose.position.x = self.pos_x
        elif self.kind == 'pose':
            m = Pose(); m.orientation.w = 1.0
        else:
            m = PoseWithCovarianceStamped(); m.header.stamp = stamp; m.pose.pose.orientation.w = 1.0
        pub.publish(m)

    def spin(self, sec, odom=True, clock=False):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.01)
            if odom:
                self.publish_state()
            if clock and time.time() - self.last_clock >= 0.01:
                self.sim_t += time.time() - self.last_clock
                self.last_clock = time.time()
                c = Clock(); c.clock.sec = int(self.sim_t)
                c.clock.nanosec = int((self.sim_t % 1) * 1e9)
                self.clock.publish(c)
            time.sleep(0.01)


def launch(extra, yaw=False, kind='odom'):
    topic = {'odom': '/test_odom', 'pose_stamped': '/test_pose_stamped', 'pose': '/test_pose',
             'pose_cov': '/test_pose_cov'}[kind]
    if yaw:
        args = ['ros2', 'run', 'lab3_controllers', 'yaw_controller', '--ros-args',
                '-p', f'state_topic:={topic}', '-p', 'cmd_topic:=/test_cmd',
                '-p', 'yaw_ref:=90.0', '-p', 'use_degrees:=true'] + extra
    else:
        args = NODE + ['-p', f'state_topic:={topic}', '-p', 'cmd_topic:=/test_cmd',
                       '-p', 'start_delay:=0.2', '-p', 'path_shape:=circle', '-p', 'strategy:=pure_pursuit',
                       '-p', 'relative_to_start:=true'] + extra
    # the node's output goes to a file so that a failing test can show WHY (crash, traceback, exit code)
    log = tempfile.NamedTemporaryFile(prefix='node_', suffix='.log', delete=False)
    p = subprocess.Popen(args, start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
    p.logfile = log.name
    log.close()
    return p


def node_diagnostics(p):
    """Exit status and the tail of the node's output, for failure messages."""
    try:
        tail = ' | '.join(open(p.logfile, errors='replace').read().strip().splitlines()[-4:])
    except Exception:       # noqa: BLE001
        tail = '(no log)'
    return f'node exit status {p.poll()} (None = still running); output tail: {tail[-400:]}'


def kill_group(p):
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=5)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass


def stale_once(h, sim, kind):
    h.cmds.clear()
    h.kind = kind
    p = launch(['-p', f'use_sim_time:={"true" if sim else "false"}'], kind=kind)
    try:
        t0 = time.time()                                  # odometry flowing; wait until the node commands motion
        while time.time() - t0 < 25.0 and not any(c[1] > 0.01 for c in h.cmds):
            h.spin(0.2, odom=True, clock=sim)
        h.spin(2.0, odom=True, clock=sim)
        moving = [c for c in h.cmds if c[1] > 0.01]
        t_stop = time.time()
        h.spin(2.0, odom=False, clock=sim)                # odometry silent, clock (if any) keeps going
        after = [c for c in h.cmds if c[0] > t_stop]
        first_zero = next((c[0] - t_stop for c in after if abs(c[1]) < 1e-9 and abs(c[2]) < 1e-9), None)
        last_move = max((c[0] - t_stop for c in after if abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9),
                        default=-1.0)
        ok = bool(moving) and first_zero is not None and first_zero <= 0.8 and last_move <= 0.9
        silent = bool(moving) and first_zero is None and last_move < 0.5     # node went silent: no stop command at all
        detail = (f'moving cmds before={len(moving)}, first zero {first_zero if first_zero is None else round(first_zero, 2)} s, '
                  f'last nonzero {round(last_move, 2)} s after odom stopped')
        return ok, silent, (detail if ok else detail + '; ' + node_diagnostics(p))
    finally:
        kill_group(p)




def stale_test(h, name, sim, kind='odom'):
    """The stale-state stop must happen within 0.8 s. A node that goes completely silent (no command at all, not even
    zero) was seen in about 3 of 110 runs under the default Fast DDS in WSL (executor lost wake-up, the harness itself
    never stalled). That is retried ONCE and reported as a warning; a real logic fault (late or missing stop, motion
    after the data stopped) fails on the first attempt, and a second silent run fails the test."""
    ok, silent, detail = stale_once(h, sim, kind)
    if ok:
        report(name, True, detail)
        return
    if silent:
        ok2, _silent2, detail2 = stale_once(h, sim, kind)
        report(name, ok2, ('RETRIED after the node went silent once (known intermittent middleware stall): ' + detail2)
               if ok2 else ('failed twice, second attempt: ' + detail2))
        return
    report(name, False, detail)


def shutdown_test(h, yaw=False):
    h.cmds.clear()
    p = launch(['-p', 'use_sim_time:=false'], yaw=yaw)
    try:
        t0 = time.time()                       # wait until the node really commands motion (slow start-up under load)
        while time.time() - t0 < 25.0 and not any(abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9 for c in h.cmds):
            h.spin(0.2, odom=True)
        h.spin(1.0, odom=True)
        t_int = time.time()
        os.killpg(p.pid, signal.SIGINT)
        h.spin(1.5, odom=True)
        try:
            p.wait(timeout=5)
        except Exception:
            pass
        zeros = [c for c in h.cmds if c[0] >= t_int - 0.05 and abs(c[1]) < 1e-9 and abs(c[2]) < 1e-9]
        report('C shutdown zero commands (' + ('yaw' if yaw else 'motion') + ')', len(zeros) >= 3 and p.poll() is not None,
               f'{len(zeros)} zero commands received after SIGINT, exited={p.poll() is not None}')
    finally:
        kill_group(p)


def invalid_test(name, extra):
    p = subprocess.Popen(NODE + extra, start_new_session=True, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    try:
        rc = p.wait(timeout=15)
        report(name, rc != 0, f'exit code {rc}')
    except subprocess.TimeoutExpired:
        report(name, False, 'did not exit within 15 s (accepted invalid parameter)')
    finally:
        kill_group(p)


def exit_code_test(name, extra, yaw=False, limit=15.0):
    """The node must fail clearly: exit code 2 (not hang, not start silently)."""
    args = (['ros2', 'run', 'lab3_controllers', 'yaw_controller', '--ros-args'] if yaw else NODE) + extra
    t0 = time.time()
    p = subprocess.Popen(args, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        rc = p.wait(timeout=limit)
        report(name, rc == 2, f'exit code {rc} after {time.time() - t0:.1f} s (expected 2)')
    except subprocess.TimeoutExpired:
        report(name, False, f'still running after {limit} s (no clear failure)')
    finally:
        kill_group(p)


def warning_test(h):
    """A frame change and a position jump on the state topic must be reported in the log."""
    h.kind, h.frame, h.pos_x = 'pose_stamped', 'map', 0.0
    p = subprocess.Popen(NODE + ['-p', 'state_topic:=/test_pose_stamped', '-p', 'cmd_topic:=/test_cmd',
                                 '-p', 'start_delay:=0.2', '-p', 'path_shape:=circle', '-p', 'strategy:=pure_pursuit',
                                 '-p', 'relative_to_start:=true', '-p', 'use_sim_time:=false'],
                         start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        # wait until the node has really subscribed (start-up time varies with machine load and a cold ROS
        # daemon; a fixed delay made this test flaky), then let it log the first frame
        pub = h.pubs[h.kind][0]
        give_up = time.time() + 30.0
        while pub.get_subscription_count() == 0 and time.time() < give_up:
            h.spin(0.2, odom=True)
        h.spin(2.0, odom=True)
        h.frame, h.pos_x = 'odom', 1.0
        h.spin(2.0, odom=True)
        os.killpg(p.pid, signal.SIGINT)
        out = p.communicate(timeout=8)[0]
    except Exception as e:                      # noqa: BLE001
        out = f'error: {e}'
    finally:
        kill_group(p)
    h.kind, h.frame, h.pos_x = 'odom', 'odom', 0.0
    logged = 'frame_id = "map"' in out
    changed = 'frame_id changed' in out
    jumped = 'jumped' in out
    report('G state source: frame id logged, frame change and position jump warned',
           logged and changed and jumped,
           f'frame logged={logged}, change warned={changed}, jump warned={jumped}')


def leftover_test():
    out = subprocess.run(['pgrep', '-af', r'lab3_controllers/(motion|yaw)_controller|motion_controller --ros-args'],
                         capture_output=True, text=True).stdout
    lines = [l for l in out.splitlines() if 'pgrep' not in l and 'test_safety' not in l]
    report('E no leftover controller processes', not lines, f'{len(lines)} found')


def main():
    h = Harness()
    stale_test(h, 'A stale odometry (wall clock)', sim=False)
    stale_test(h, 'B stale odometry (sim clock running)', sim=True)
    shutdown_test(h)
    shutdown_test(h, yaw=True)
    for kind in ('pose_stamped', 'pose', 'pose_cov'):
        stale_test(h, f'F state type {kind}: moves, then stops when data stops', sim=False, kind=kind)
    h.kind = 'odom'
    invalid_test('D1 invalid strategy rejected', ['-p', 'strategy:=bogus'])
    invalid_test('D2 invalid polar gains rejected', ['-p', 'k_alpha:=0.5'])
    exit_code_test('H1 unsupported state type fails clearly (std_msgs/String)',
                   ['-p', 'state_topic:=/test_badtype', '-p', 'cmd_topic:=/test_cmd'])
    exit_code_test('H2 ambiguous state type fails clearly (Pose and PoseStamped on one topic)',
                   ['-p', 'state_topic:=/test_ambig', '-p', 'cmd_topic:=/test_cmd'])
    exit_code_test('H3 missing state topic fails clearly after the timeout (motion)',
                   ['-p', 'state_topic:=/test_never_published', '-p', 'cmd_topic:=/test_cmd',
                    '-p', 'state_wait_timeout:=3.0'])
    exit_code_test('H4 missing state topic fails clearly after the timeout (yaw)',
                   ['-p', 'state_topic:=/test_never_published', '-p', 'cmd_topic:=/test_cmd',
                    '-p', 'state_wait_timeout:=3.0'], yaw=True)
    exit_code_test('H5 invalid state_type rejected',
                   ['-p', 'state_topic:=/test_odom', '-p', 'cmd_topic:=/test_cmd', '-p', 'state_type:=bogus'])
    warning_test(h)
    time.sleep(1.0)
    kill_group(h.amb2)
    leftover_test()
    rclpy.shutdown()
    print(f'{sum(results)}/{len(results)} safety tests passed')
    sys.exit(0 if all(results) else 1)


main()
