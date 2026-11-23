#!/usr/bin/env python3
"""Record a run and tell WHERE an unexpected position change comes from. Read-only: it only subscribes, it never publishes.

Records at the same time
    wheel odometry   (default /diff_cont/odom)   what the wheels say
    map pose         (default /robot_pose)       what SLAM says (the professor's frame)
    commands         (default /diff_cont/cmd_vel_unstamped)  what was sent to the robot

    python3 tools/log_run.py --seconds 60                 # start it, then run your test; it stops after 60 s (or Ctrl+C)
    python3 tools/log_run.py --analyze runs/run_xxx.csv   # analyse an existing file

The verdict compares the two pose sources while the robot was only turning:
    * wheel odometry position moved  -> the ROBOT did not turn in place (slip, unequal wheels, caster drag)
    * map position moved but the odometry did not -> SLAM corrections (the map pose jumped, the robot did not move)
    * position moved only while a forward command was sent -> our commands
"""
import argparse
import csv
import math
import os
import sys
import time


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def record(a):
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('log_run')
    rows = []
    last = {'v': 0.0, 'w': 0.0}
    t0 = time.monotonic()

    def add(src, x, y, yaw):
        rows.append((time.monotonic() - t0, src, x, y, yaw, last['v'], last['w']))

    node.create_subscription(Twist, a.cmd_topic, lambda m: last.update(v=m.linear.x, w=m.angular.z), 10)
    node.create_subscription(Odometry, a.odom_topic, lambda m: add('odom', m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)), qos_profile_sensor_data)
    if a.map_topic:
        node.create_subscription(PoseStamped, a.map_topic, lambda m: add('map', m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)), qos_profile_sensor_data)
    print(f'recording {a.seconds:.0f} s ... run your test now (Ctrl+C ends early)', flush=True)
    try:
        while time.monotonic() - t0 < a.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or '.', exist_ok=True)
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'source', 'x', 'y', 'yaw', 'cmd_v', 'cmd_w'])
        w.writerows(rows)
    print('saved', a.out, f'({len(rows)} samples)')
    return a.out


def load(path):
    out = {'odom': [], 'map': []}
    with open(path) as f:
        for r in csv.DictReader(f):
            out[r['source']].append(tuple(float(r[k]) for k in ('t', 'x', 'y', 'yaw', 'cmd_v', 'cmd_w')))
    return out


def moving_spans(rows, thresh=0.01):
    """Index ranges where a command was being sent (|v| or |w| above thresh)."""
    return [i for i, r in enumerate(rows) if abs(r[4]) > thresh or abs(r[5]) > thresh]


def excursion(rows, idx):
    """(net displacement, largest distance from the first point, total path length) over the given sample indices, in cm."""
    pts = [(rows[i][1], rows[i][2]) for i in idx]
    if len(pts) < 2:
        return 0.0, 0.0, 0.0
    net = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    far = max(math.hypot(p[0] - pts[0][0], p[1] - pts[0][1]) for p in pts)
    path = sum(math.hypot(b[0] - a_[0], b[1] - a_[1]) for a_, b in zip(pts, pts[1:]))
    return net * 100, far * 100, path * 100


def analyze(path):
    d = load(path)
    print(f'--- {path}: odom samples {len(d["odom"])}, map samples {len(d["map"])}')
    if not d['odom']:
        print('no wheel odometry recorded: nothing to analyse')
        return None
    odom = d['odom']
    active = moving_spans(odom)
    if not active:
        print('no command was sent during the recording (v and w ~ 0): start the controller / send the reference while recording')
        return None
    lo, hi = active[0], active[-1]
    seg = range(lo, hi + 1)
    turn_only = [i for i in seg if abs(odom[i][4]) < 0.01]               # samples with no forward command
    net, far, path_len = excursion(odom, seg)
    dyaw = math.degrees(wrap(odom[hi][3] - odom[lo][3]))
    print(f'commanded interval {odom[lo][0]:.1f}-{odom[hi][0]:.1f} s, max forward command {max(abs(odom[i][4]) for i in seg):.3f} m/s, '
          f'max turn command {max(abs(odom[i][5]) for i in seg):.2f} rad/s')
    print(f'WHEEL ODOMETRY: heading changed {dyaw:+.1f} deg; position net {net:.1f} cm, farthest {far:.1f} cm, path length {path_len:.1f} cm')
    verdict = []
    if turn_only and len(turn_only) == len(list(seg)) and far > 3.0:
        verdict.append(f'the ROBOT moved {far:.1f} cm although only a turn was commanded: it does not rotate on the spot '
                       f'(wheel slip, unequal wheels, caster drag)')
    if d['map']:
        m = d['map']
        mt = [i for i, r in enumerate(m) if odom[lo][0] <= r[0] <= odom[hi][0]]
        if len(mt) >= 2:
            mnet, mfar, mpath = excursion(m, mt)
            mdyaw = math.degrees(wrap(m[mt[-1]][3] - m[mt[0]][3]))
            print(f'MAP POSE (/robot_pose): heading changed {mdyaw:+.1f} deg; position net {mnet:.1f} cm, farthest {mfar:.1f} cm, path length {mpath:.1f} cm; '
                  f'{len(mt)} samples')
            jumps = [(m[i][0], 100 * math.hypot(m[i][1] - m[i - 1][1], m[i][2] - m[i - 1][2])) for i in mt[1:]
                     if 100 * math.hypot(m[i][1] - m[i - 1][1], m[i][2] - m[i - 1][2]) > 2.0]
            print(f'map position jumps above 2 cm between samples: {len(jumps)}' + (f' (largest {max(j[1] for j in jumps):.1f} cm)' if jumps else ''))
            print(f'heading: odometry {dyaw:+.1f} deg vs map {mdyaw:+.1f} deg: difference {math.degrees(wrap(math.radians(mdyaw - dyaw))):+.1f} deg')
            if mfar > far + 3.0:
                verdict.append(f'the MAP position moved {mfar:.1f} cm but the wheels only {far:.1f} cm: SLAM corrections, not the robot')
    fwd = [i for i in seg if abs(odom[i][4]) > 0.01]
    if fwd:
        verdict.append(f'{len(fwd)} samples had a forward command (max {max(abs(odom[i][4]) for i in fwd):.3f} m/s): position change can come from our commands')
    print('VERDICT:' if verdict else 'VERDICT: nothing abnormal: no unexpected position change in this recording')
    for v in verdict:
        print('  -', v)
    return verdict


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--odom-topic', default='/diff_cont/odom')
    ap.add_argument('--map-topic', default='/robot_pose', help="'' to skip")
    ap.add_argument('--cmd-topic', default='/diff_cont/cmd_vel_unstamped')
    ap.add_argument('--out', default=os.path.join('runs', time.strftime('run_%Y%m%d_%H%M%S.csv')))
    ap.add_argument('--analyze', metavar='CSV', help='analyse a recorded file instead of recording')
    a = ap.parse_args()
    analyze(a.analyze if a.analyze else record(a))


if __name__ == '__main__':
    sys.exit(main())
