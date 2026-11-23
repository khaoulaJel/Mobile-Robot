#!/usr/bin/env python3
"""Measure the robot's real response in about 85 s: latency, gain, coasting time and command deadband.

Total time about 85 s.
Why: the controller presets (`lab_robot.yaml`: lead_v, lead_w, deadband_v, deadband_w) depend on how the robot
answers speed commands THROUGH the state topic the controller will use. The professor's identified model (coasting
0.61 s / 1.0 s, deadband |v| <= 0.04 and |w| <= 0.2) may or may not describe the real robot, and a different state topic
(raw /diff_cont/odom vs the EKF's /odometry/filtered) has a different lag. This tool measures it and prints the YAML lines.

What it does (all commands are small and short; ABORTS and sends zeros if the state stops, the robot leaves a
`--max-distance` circle, or on Ctrl+C):
  1. 2 s at rest                      -> creep / noise floor
  2. forward step v = --v-step for 3 s, then 0 for 5 s     -> latency, gain, 90 % rise, coasting time
  3. turn step  w = --w-step for 3 s, then 0 for 5 s       -> same for the heading
  4. staircase v = 0.01 ... 0.08 (alternating +/-), 2 s each, w = 0   -> smallest v that moves the robot
  5. staircase w = 0.05 ... 0.40 (alternating +/-), 2 s each, v = 0   -> smallest w that moves the robot
Needs about 1 m x 1 m of clear floor. Velocities are taken from the state POSE (finite differences, the same
signal the controllers use) or from the message twist (--source twist).

Usage (robot / Gazebo running, ONE controller at a time):
  python3 measure_response.py --state-topic /diff_cont/odom --cmd-topic /cmd_vel_tracker --out response.json
The analysis is a pure function (analyse), tested on synthetic data in test/test_measure_response.py.
"""
import argparse
import json
import math
import signal
import sys
import time


# ---- schedule ------------------------------------------------------------------------------------------------
def build_schedule(v_step=0.1, w_step=0.5, hold=2.0):
    """List of segments (name, v, w, duration). Staircase signs alternate so the robot returns to where it was."""
    seg = [('rest', 0.0, 0.0, 2.0),
           ('fwd_step', v_step, 0.0, 3.0), ('fwd_rest', 0.0, 0.0, 5.0),
           ('rot_step', 0.0, w_step, 3.0), ('rot_rest', 0.0, 0.0, 5.0)]
    for k in range(1, 9):
        seg += [(f'v_level_{k}', (0.01 * k) * (1 if k % 2 else -1), 0.0, hold), (f'v_gap_{k}', 0.0, 0.0, hold)]
    for k in range(1, 9):
        seg += [(f'w_level_{k}', 0.0, (0.05 * k) * (1 if k % 2 else -1), hold), (f'w_gap_{k}', 0.0, 0.0, hold)]
    return seg


def segment_windows(schedule):
    """name -> (t_start, t_end) on the schedule clock."""
    t, out = 0.0, {}
    for name, _v, _w, dur in schedule:
        out[name] = (t, t + dur)
        t += dur
    return out


# ---- analysis (pure) -----------------------------------------------------------------------------------------
def _window(ts, xs, a, b):
    return [(t, x) for t, x in zip(ts, xs) if a <= t < b]


def _mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def _integral(ts, xs, a, b):
    """Trapezoid integral of the sampled signal over [a, b] with linear interpolation at both ends, so the result
    does not depend on whether a sample happens to fall just before or just after a boundary."""
    def at(t):
        for i in range(1, len(ts)):
            if ts[i] >= t:
                t0, t1 = ts[i - 1], ts[i]
                return xs[i] if t1 == t0 else xs[i - 1] + (xs[i] - xs[i - 1]) * (t - t0) / (t1 - t0)
        return xs[-1]
    pts = [(a, at(a))] + [(t, x) for t, x in zip(ts, xs) if a < t < b] + [(b, at(b))]
    return sum(0.5 * (p[1] + q[1]) * (q[0] - p[0]) for p, q in zip(pts, pts[1:]))


def _step_metrics(ts, sig, t_on, t_off, t_end, level):
    """Latency, gain, 90 % rise and coasting-equivalent lag of one step response (signal `sig` sampled at ts)."""
    on = _window(ts, sig, t_on, t_off)
    tail = [x for t, x in on if t >= t_off - 1.0]
    if len(tail) < 3 or level == 0.0:
        return None
    steady = _mean(tail)
    if abs(steady) < 1e-9:
        return dict(steady=steady, gain=0.0, latency=None, t90=None, coast_lag=None)
    sgn = 1.0 if steady > 0 else -1.0
    lat = next((t - t_on for t, x in on if sgn * x >= 0.1 * abs(steady)), None)
    t90 = next((t - t_on for t, x in on if sgn * x >= 0.9 * abs(steady)), None)
    # coasting: integral of the velocity after the command went to zero, divided by the steady velocity
    area = _integral(ts, sig, t_off, t_end)
    return dict(steady=steady, gain=steady / level, latency=lat, t90=t90, coast_lag=area / steady)


def analyse(ts, v, w, schedule, moves_ratio=0.3, moves_floor_v=0.004, moves_floor_w=0.02):
    """Return the measured response. ts: time on the schedule clock; v, w: measured forward / turn velocity.
    A staircase level 'moves' the robot when the mean velocity over the second half of the hold exceeds
    max(floor, moves_ratio * command); floors keep the creep noise floor from counting as motion."""
    win = segment_windows(schedule)
    res = {}
    rest = win['rest']
    res['creep_v'] = abs(_mean([x for t, x in _window(ts, v, *rest)]))
    res['creep_w'] = abs(_mean([x for t, x in _window(ts, w, *rest)]))
    fwd, rot = dict((s[0], s) for s in schedule)['fwd_step'], dict((s[0], s) for s in schedule)['rot_step']
    res['v'] = _step_metrics(ts, v, win['fwd_step'][0], win['fwd_step'][1], win['fwd_rest'][1], fwd[1])
    res['w'] = _step_metrics(ts, w, win['rot_step'][0], win['rot_step'][1], win['rot_rest'][1], rot[2])
    for key, sig, floor, idx in (('v', v, moves_floor_v, 1), ('w', w, moves_floor_w, 2)):
        levels = []
        for name, *rest_ in schedule:
            if not name.startswith(f'{key}_level_'):
                continue
            cmd = dict((s[0], s) for s in schedule)[name][idx]
            a, b = win[name]
            m = abs(_mean([x for t, x in _window(ts, sig, (a + b) / 2, b)]))
            levels.append(dict(command=abs(cmd), measured=m, moves=m > max(floor, moves_ratio * abs(cmd))))
        res[f'{key}_levels'] = levels
        moving = [lv['command'] for lv in levels if lv['moves']]
        res[f'{key}_smallest_moving_command'] = min(moving) if moving else None
        # a deadband exists if a smaller level is dropped while a larger one moves
        res[f'{key}_deadband'] = (None if not moving else
                                  max([lv['command'] for lv in levels if lv['command'] < min(moving)] or [0.0]))
    return res


def suggestions(res):
    """Text with the YAML lines for lab_robot.yaml, and the warnings a human should read."""
    lines, warn = [], []
    for k, name in (('v', 'lead_v'), ('w', 'lead_w')):
        m = res.get(k)
        if m and m['coast_lag'] is not None:
            lines.append(f"{name}: {max(0.0, m['coast_lag']):.2f}      # measured coasting time [s]"
                         f" (gain {m['gain']:.2f}, latency {m['latency']:.2f} s, 90% rise {m['t90']:.2f} s)")
        else:
            warn.append(f'could not measure the {k} step (the robot did not move enough)')
    for k, name in (('v', 'deadband_v'), ('w', 'deadband_w')):
        db, first = res[f'{k}_deadband'], res[f'{k}_smallest_moving_command']
        if first is None:
            warn.append(f'no staircase level moved the robot in {k}: deadband larger than the tested range')
        elif db and db > 0:
            lines.append(f'{name}: {db:.2f}      # largest command that did NOT move the robot; smallest that moved: {first:.2f}')
        else:
            lines.append(f'# no {k} deadband: even the smallest tested command ({first:.2f}) moved the robot -> {name}: 0.0')
    if res['creep_v'] > 0.002 or res['creep_w'] > 0.01:
        warn.append(f"the robot drifts at rest: {res['creep_v'] * 1000:.1f} mm/s, {res['creep_w']:.3f} rad/s")
    return lines, warn


# ---- synthetic robot (for tests and for checking the analysis) -----------------------------------------------
def simulate(plant, schedule, dt=0.05, noise=0.0, seed=0):
    """Drive a model of the robot (anything with step(v, w) -> (v, w)) through the schedule; returns ts, v, w."""
    import random
    rng = random.Random(seed)
    ts, vs, ws, t = [], [], [], 0.0
    for _name, cv, cw, dur in schedule:
        for _ in range(int(round(dur / dt))):
            v, w = plant.step(cv, cw)
            ts.append(t)
            vs.append(v + rng.gauss(0, noise))
            ws.append(w + rng.gauss(0, 5 * noise))
            t += dt
    return ts, vs, ws


# ---- ROS runner ------------------------------------------------------------------------------------------------
def run_ros(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    sys.path.insert(0, __file__.rsplit('tools', 1)[0])
    from lab3_controllers.common import yaw_from_quaternion

    class Runner(Node):
        def __init__(self):
            super().__init__('measure_response')
            self.pub = self.create_publisher(Twist, args.cmd_topic, 10)
            self.sub = self.create_subscription(Odometry, args.state_topic, self.on_state, qos_profile_sensor_data)
            self.samples = []          # (wall time, x, y, yaw, twist v, twist w)
            self.last = None

        def on_state(self, m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            self.samples.append((time.monotonic(), p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w),
                                 m.twist.twist.linear.x, m.twist.twist.angular.z))
            self.last = time.monotonic()

        def cmd(self, v, w):
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            self.pub.publish(msg)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = Runner()
    schedule = build_schedule(args.v_step, args.w_step, args.hold)
    total = sum(s[3] for s in schedule)
    print(f'waiting for the state on {args.state_topic} ...')
    t_wait = time.monotonic()
    while node.last is None and time.monotonic() - t_wait < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.last is None:
        print('FAIL: no state message received')
        return 2
    x0, y0 = node.samples[-1][1], node.samples[-1][2]
    print(f'measuring for {total:.0f} s: keep a 1 m x 1 m area clear and the emergency stop at hand (Ctrl+C stops)')
    t0, idx, status = time.monotonic(), 0, 'ok'
    clock = [(s, sum(x[3] for x in schedule[:i]), sum(x[3] for x in schedule[:i + 1]))
             for i, s in enumerate(schedule)]
    try:
        while True:
            t = time.monotonic() - t0
            if t >= total:
                break
            cur = next(c for c in clock if c[1] <= t < c[2])[0]
            node.cmd(cur[1], cur[2])
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - node.last > 0.5:
                status = 'state lost'
                break
            s = node.samples[-1]
            if math.hypot(s[1] - x0, s[2] - y0) > args.max_distance:
                status = f'left the {args.max_distance} m safety circle'
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        status = 'interrupted'
    finally:
        for _ in range(5):
            node.cmd(0.0, 0.0)
            time.sleep(0.02)
    if status != 'ok':
        print(f'ABORTED: {status}. Zero commands sent.')
        node.destroy_node()
        rclpy.shutdown()
        return 3
    ts, vs, ws = [], [], []
    S = node.samples
    k = max(1, int(round(args.smooth / 0.033)))                # finite difference over ~args.smooth seconds
    for i in range(k, len(S)):
        dt = S[i][0] - S[i - k][0]
        if dt <= 0:
            continue
        if args.source == 'twist':
            v, w = S[i][4], S[i][5]
        else:
            yaw = S[i][3]
            v = ((S[i][1] - S[i - k][1]) * math.cos(yaw) + (S[i][2] - S[i - k][2]) * math.sin(yaw)) / dt
            w = math.atan2(math.sin(S[i][3] - S[i - k][3]), math.cos(S[i][3] - S[i - k][3])) / dt
        ts.append(S[i][0] - t0)
        vs.append(v)
        ws.append(w)
    node.destroy_node()
    rclpy.shutdown()
    return ts, vs, ws, schedule


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--state-topic', default='/diff_cont/odom')
    ap.add_argument('--cmd-topic', default='/cmd_vel_tracker')
    ap.add_argument('--source', choices=['pose', 'twist'], default='pose')
    ap.add_argument('--v-step', type=float, default=0.1)
    ap.add_argument('--w-step', type=float, default=0.5)
    ap.add_argument('--hold', type=float, default=2.0, help='seconds per staircase level (slow robots need >= 2)')
    ap.add_argument('--smooth', type=float, default=0.2, help='finite-difference window [s] for --source pose')
    ap.add_argument('--max-distance', type=float, default=0.8, help='abort if the robot moves further than this [m]')
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    out = run_ros(args)
    if isinstance(out, int):
        return out
    ts, vs, ws, schedule = out
    res = analyse(ts, vs, ws, schedule)
    lines, warn = suggestions(res)
    print('\nMEASURED RESPONSE (state topic %s, source %s)' % (args.state_topic, args.source))
    for k, name in (('v', 'forward speed'), ('w', 'turn rate')):
        m = res[k]
        if m:
            print(f"  {name}: gain {m['gain']:.2f}, latency {m['latency']}, 90% rise {m['t90']}, coasting {m['coast_lag']}")
    for k in ('v', 'w'):
        print(f"  {k} deadband: largest non-moving command {res[k + '_deadband']}, smallest moving {res[k + '_smallest_moving_command']}")
    print('\nSuggested lines for the controller YAML:')
    for line in lines:
        print('  ' + line)
    for wmsg in warn:
        print('  WARNING: ' + wmsg)
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(dict(result=res, suggestions=lines, warnings=warn, topic=args.state_topic), f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
