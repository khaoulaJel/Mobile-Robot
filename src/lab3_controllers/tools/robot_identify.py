#!/usr/bin/env python3
"""How far does the robot really move for a short command? Sends single pulses of known strength and length, and measures
each one from the wheel odometry once the robot is at rest again.

    python3 robot_identify.py            turn pulses and drive pulses (about 2 minutes)
    python3 robot_identify.py turn       turn pulses only (the robot stays on the spot)
    python3 robot_identify.py drive      drive pulses only

Every pulse is followed by the same pulse the other way, so the robot stays about where it started. The largest single move is
about 25 deg or 11 cm. It sends the commands itself (no controller may be running). It stops if the odometry goes silent or
the robot gets 35 cm or 60 deg from where it started; if this program dies, the robot's own 0.5 s command timeouts stop it.
Results: $ROBOT_LOGBOOK/identify/<time>_pulses.csv (one line per pulse) and _trace.csv (every odometry sample).
"""
import csv
import datetime
import math
import os
import sys
import threading
import time

PLAN = {'turn': ([0.3, 0.45, 0.6], [0.2, 0.3, 0.4, 0.5, 0.7]),            # rad/s, s
        'drive': ([0.06, 0.10, 0.15], [0.2, 0.3, 0.4, 0.5, 0.7])}          # m/s, s
LIMIT_DISTANCE, LIMIT_ANGLE, SILENCE = 0.35, math.radians(60), 1.0


def schedule(axes):
    """-> [(axis, signed command, seconds)], every pulse followed by its opposite; short and weak ones first."""
    out = []
    for axis in axes:
        strengths, lengths = PLAN[axis]
        for length in lengths:
            for u in strengths:
                out += [(axis, u, length), (axis, -u, length)]
    return out


def table(results):
    """-> text: mean distance moved (in the commanded direction) for each strength and length."""
    lines = []
    for axis, unit in (('turn', 'deg'), ('drive', 'cm')):
        mine = [r for r in results if r['axis'] == axis]
        if not mine:
            continue
        strengths = sorted({abs(r['command']) for r in mine})
        lengths = sorted({r['length'] for r in mine})
        lines.append(f'\n{axis.upper()}: moved ({unit}), mean of both directions   [ideal = strength x length]')
        lines.append('length \\ strength ' + ''.join(f'{u:>16g}' for u in strengths))
        for length in lengths:
            cells = []
            for u in strengths:
                got = [r['moved'] for r in mine if abs(r['command']) == u and r['length'] == length]
                ideal = math.degrees(u * length) if axis == 'turn' else 100 * u * length
                cells.append(f'{sum(got) / len(got):6.2f} / {ideal:5.1f}' if got else '-')
            lines.append(f'{length:>14.2f} s   ' + ''.join(f'{c:>16}' for c in cells))
    return '\n'.join(lines)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    yes = '--yes' in argv
    words = [a for a in argv if a != '--yes']
    if any(w not in PLAN for w in words):
        print(__doc__)
        return 2
    axes = words or ['turn', 'drive']

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('robot_identify')
    state = {'at': None, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 0.0, 'w': 0.0, 'turned': 0.0}
    samples = []

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if state['at'] is not None:                          # heading without the +-180 wrap
            state['turned'] += math.atan2(math.sin(yaw - state['yaw']), math.cos(yaw - state['yaw']))
        state.update(at=time.monotonic(), x=p.x, y=p.y, yaw=yaw, v=m.twist.twist.linear.x, w=m.twist.twist.angular.z)
        samples.append((state['at'], p.x, p.y, state['turned'], state['v'], state['w']))

    node.create_subscription(Odometry, '/diff_cont/odom', on_odom, qos_profile_sensor_data)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()
    publisher, results, status = None, [], 'finished'
    try:
        print('waiting for the robot ...', flush=True)
        t0 = time.monotonic()
        while state['at'] is None and time.monotonic() - t0 < 60:
            time.sleep(0.1)
        if state['at'] is None:
            print('NOT STARTED: no odometry', file=sys.stderr)
            return 2
        time.sleep(4.0)
        problem = None
        if node.count_publishers('/diff_cont/odom') != 1:
            problem = f'{node.count_publishers("/diff_cont/odom")} odometry publishers: there must be exactly 1'
        elif node.count_publishers('/cmd_vel_tracker') != 0:
            problem = 'another controller is publishing /cmd_vel_tracker: stop it first'
        elif node.count_subscribers('/cmd_vel_tracker') == 0:
            problem = 'nothing listens to /cmd_vel_tracker: are the laptop programs running?'
        if problem:
            print(f'NOT STARTED: {problem}', file=sys.stderr)
            return 2
        plan = schedule(axes)
        print(f'{len(plan)} short pulses ({" and ".join(axes)}), about {int(sum(t + 2.2 for _, _, t in plan) // 60) + 1} minutes. '
              'Largest single move about 25 deg or 11 cm; each is undone by the next one.')
        print('NEEDED: robot on the floor, 50 cm free all around. STOP: Ctrl+C, or the motor battery switch.')
        if not yes and input('Press Enter to start, or type n to cancel: ').strip().lower().startswith('n'):
            print('cancelled, nothing moved')
            return 0
        publisher = node.create_publisher(Twist, '/cmd_vel_tracker', 10)
        time.sleep(1.5)

        def send(v, w):
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            publisher.publish(m)

        home = (state['x'], state['y'], state['turned'])

        def check():
            if time.monotonic() - state['at'] > SILENCE:
                raise RuntimeError('the odometry stopped')
            if math.hypot(state['x'] - home[0], state['y'] - home[1]) > LIMIT_DISTANCE:
                raise RuntimeError('the robot is more than 35 cm from where it started')
            if abs(state['turned'] - home[2]) > LIMIT_ANGLE:
                raise RuntimeError('the robot has turned more than 60 deg from where it started')

        def run_for(seconds, v, w, until_still=False):
            begin, quiet = time.monotonic(), None
            while True:
                now = time.monotonic()
                if now - begin >= seconds and not until_still:
                    return
                if until_still:
                    still = abs(state['v']) < 0.005 and abs(state['w']) < 0.02
                    quiet = (quiet or now) if still else None
                    if now - begin >= seconds and ((quiet and now - quiet >= 0.5) or now - begin >= seconds + 3.0):
                        return
                send(v, w)
                check()
                time.sleep(max(0.0, min(0.05, begin + seconds - time.monotonic())) if not until_still else 0.05)

        run_for(1.0, 0, 0)
        for k, (axis, u, length) in enumerate(plan, 1):
            x0, y0, a0, th0 = state['x'], state['y'], state['turned'], state['yaw']
            first = len(samples)
            begin = time.monotonic()
            run_for(length, u if axis == 'drive' else 0.0, u if axis == 'turn' else 0.0)
            real = time.monotonic() - begin
            send(0, 0)
            run_for(1.2, 0, 0, until_still=True)
            if axis == 'turn':
                moved = math.degrees(state['turned'] - a0) * math.copysign(1, u)
            else:
                moved = 100 * ((state['x'] - x0) * math.cos(th0) + (state['y'] - y0) * math.sin(th0)) * math.copysign(1, u)
            speed_index, threshold = (5, 0.05) if axis == 'turn' else (4, 0.008)
            onset = next((s[0] - begin for s in samples[first:] if abs(s[speed_index]) > threshold), None)
            results.append({'axis': axis, 'command': u, 'length': length, 'real_length': round(real, 3),
                            'moved': round(moved, 3), 'unit': 'deg' if axis == 'turn' else 'cm',
                            'first_motion_s': '' if onset is None else round(onset, 3)})
            print(f'{k:3d}/{len(plan)}  {axis:5s} {u:+.2f} for {length:.2f} s -> moved {moved:+6.2f} {results[-1]["unit"]}'
                  + ('' if onset is None else f'   (first motion after {onset:.2f} s)'), flush=True)
    except KeyboardInterrupt:
        status = 'stopped by the operator'
    except RuntimeError as e:
        status = f'STOPPED for safety: {e}'
    finally:
        if publisher is not None:
            for _ in range(5):
                m = Twist()
                publisher.publish(m)
                time.sleep(0.03)
        ex.shutdown()
        spinner.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if results:
        folder = os.path.join(os.environ.get('ROBOT_LOGBOOK') or os.path.expanduser('~/robot_logbook'), 'identify')
        os.makedirs(folder, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%m%d-%H%M%S')
        with open(os.path.join(folder, f'{stamp}_pulses.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(results[0]))
            w.writeheader()
            w.writerows(results)
        with open(os.path.join(folder, f'{stamp}_trace.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['monotonic_s', 'x', 'y', 'turned_rad', 'v', 'w'])
            w.writerows(samples)
        text = table(results)
        with open(os.path.join(folder, f'{stamp}_table.txt'), 'w') as f:
            f.write(f'{status}, {len(results)} pulses\n{text}\n')
        print(text)
        print(f'\n{status}: {len(results)} pulses saved in {folder} ({stamp})')
    else:
        print(status)
    return 0 if status == 'finished' else 1


if __name__ == '__main__':
    sys.exit(main())
