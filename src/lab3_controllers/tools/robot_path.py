#!/usr/bin/env python3
"""Drive a path on the robot with one of our controllers, measure it, and log it.

    python3 robot_path.py square --controller stop_turn
    python3 robot_path.py series --shapes square,circle_r50 --controllers stop_turn,polar,pure_pursuit,p_servo,symbolic
    python3 robot_path.py shapes                       list the shapes and the floor space each one needs

    --state auto|odom|fused   what the controller steers by. auto: fused (wheel odometry placed in the LiDAR map) when
                              /robot_pose is being received, else wheel odometry. "odom" with the LiDAR running is the
                              INDEPENDENT CHECK: the map pose is recorded but not used, so it can disagree with the controller.
    --yes   --note "..."   --settings <path_robot.json>
Controllers: stop_turn, polar, pure_pursuit (motion_controller), p_servo (p_controller, point by point), symbolic (circle_r50).
The shape is laid out from where the robot stands and the way it faces. A run is aborted if the robot gets 25 cm from the
path, the odometry stops, or it stands still for 25 s. Results: $ROBOT_LOGBOOK/paths.csv, runs/pathNNN_*, report.html.
"""
import argparse
import copy
import csv
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import paths  # noqa: E402
import robot_test as rt  # noqa: E402
from lab3_controllers.common import wrap_angle, yaw_from_quaternion  # noqa: E402

FIELDS = ['run', 'session', 'date', 'time', 'shape', 'controller', 'state', 'status', 'max_cm', 'rms_cm', 'end_cm', 'end_deg',
          'time_s', 'covered', 'map_max_cm', 'map_rms_cm', 'map_end_cm', 'map_end_deg', 'estimates_apart_cm', 'fallbacks',
          'settings', 'code', 'note']
STUCK_AFTER, ODOM_SILENCE = 25.0, 1.5
COMPLETED = ('COMPLETE', 'STUCK', 'TIMEOUT', 'FAILED')        # a series goes on after these; it stops after ABORTED


def relative(pose, start):
    """A pose expressed in the frame of the start pose."""
    dx, dy = pose[0] - start[0], pose[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap_angle(pose[2] - start[2])


def time_limit(controller, shape):
    length = paths.length_of(paths.shape(shape)[1])
    if controller == 'symbolic':
        return 330.0
    if controller == 'p_servo':
        return 90.0 + 12.0 * len(paths.waypoints(shape, paths.settings()['p_servo']['ds'])) + 20.0 * length
    return 90.0 + 35.0 * length


def read_rows(folder):
    path = os.path.join(folder, 'paths.csv')
    if not os.path.exists(path):
        return []
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def write_rows(folder, rows):
    path = os.path.join(folder, 'paths.csv')
    with open(path + '.tmp', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    os.replace(path + '.tmp', path)


def ros_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    return repr(float(v))


def controller_command(controller, shape, s, state, have_map, log_csv):
    """-> (command line, settings actually given, the list of poses to send for p_servo, in the START frame)"""
    from ament_index_python.packages import get_package_prefix
    programs = os.path.join(get_package_prefix('lab3_controllers'), 'lib', 'lab3_controllers')
    topic = '/fused_odom' if state == 'fused' else '/diff_cont/odom'
    given, goals = {}, []
    if controller in paths.TRACKERS:
        from lab3_controllers.tracker import DEFAULTS
        given = {k: (type(DEFAULTS[k])(v) if k != 'strategy' else v) for k, v in paths.tracker_params(controller, s).items()}
        wp = [v for point in paths.waypoints(shape, s['ds']) for v in point]
        command = [os.path.join(programs, 'motion_controller'), '--ros-args', '-p', f'state_topic:={topic}', '-p',
                   'cmd_topic:=/cmd_vel_tracker', '-p', 'relative_to_start:=true', '-p', 'path_shape:=waypoints', '-p',
                   'waypoints:=[' + ', '.join(repr(float(v)) for v in wp) + ']', '-p', 'start_delay:=1.0', '-p', f'log_csv:={log_csv}']
        for k, v in given.items():
            command += ['-p', f'{k}:={v if k == "strategy" else ros_value(v)}']
        given['ds'] = s['ds']
        command += ['-p', 'final_heading_servo:=true']
        for k, v in paths.heading_profile().items():
            command += ['-p', f'heading.{k}:={float(v)!r}']
        given.update({f'final heading.{k}': v for k, v in paths.heading_profile().items()})
    elif controller == 'symbolic':
        if shape != paths.SYMBOLIC_SHAPE:
            raise ValueError(f'the symbolic controller exists only for {paths.SYMBOLIC_SHAPE}')
        command = [os.path.join(programs, 'symbolic_controller'), '--ros-args', '-p', f'state_topic:={topic}', '-p',
                   'cmd_topic:=/cmd_vel_tracker', '-p', 'start_delay:=1.0', '-p', f'log_csv:={log_csv}']
    else:
        command = [os.path.join(programs, 'p_controller'), '--ros-args', '-p', f'log_csv:={log_csv}']
        if state != 'fused':
            command += ['-p', "map_topic:=''"]
        given = {f'heading.{k}': v for k, v in paths.heading_profile().items()}
        given.update({f'linear.{k}': v for k, v in s['linear'].items()})
        given.update({k: s[k] for k in ('steer_gain', 'face_tolerance', 'position_tolerance')})
        for k, v in given.items():
            command += ['-p', f'{k}:={float(v)!r}']
        goals = paths.waypoints(shape, s['ds'])
        given['ds'] = s['ds']
    return command, given, goals


def run_path(a, folder, controller, shape):
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    from lab3_controllers.servo import MapOdomFusion

    s = paths.settings(a.settings or paths.SETTINGS_FILE)[controller]
    paths.shape(shape)
    if controller == 'symbolic' and shape != paths.SYMBOLIC_SHAPE:
        raise ValueError(f'the symbolic controller exists only for {paths.SYMBOLIC_SHAPE}')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('robot_path')
    seen = {'odom_at': None, 'odom': None, 'maps': 0, 'map_at': None, 'cmd': (0.0, 0.0)}
    odom_only, in_map = MapOdomFusion(use_map=False), MapOdomFusion(use_map=True)
    relay = {'pub': None}
    lock = threading.Lock()

    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        item = (stamp(m.header), (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)), m.twist.twist.linear.x,
                m.twist.twist.angular.z)
        with lock:
            odom_only.add_odom(*item)
            in_map.add_odom(*item)
            seen['odom_at'], seen['odom'] = time.monotonic(), item
            if relay['pub'] is not None and in_map.ready():
                x, y, th = in_map.state()[0]
                out = Odometry()
                out.header = m.header
                out.header.frame_id = 'map'
                out.pose.pose.position.x, out.pose.pose.position.y = float(x), float(y)
                out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
                out.twist = m.twist
                relay['pub'].publish(out)

    def on_map(m):
        p, q = m.pose.position, m.pose.orientation
        seen['maps'] += 1
        with lock:
            if in_map.add_map(stamp(m.header), (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))):
                seen['map_at'] = time.monotonic()

    node.create_subscription(Odometry, '/diff_cont/odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/robot_pose', on_map, qos_profile_sensor_data)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()

    def close():
        ex.shutdown()
        spinner.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    controller_proc = None
    try:
        print('waiting for the robot ...', flush=True)
        t0 = time.monotonic()
        while seen['odom_at'] is None and time.monotonic() - t0 < 60:
            time.sleep(0.1)
        if seen['odom_at'] is None:
            raise RuntimeError('no odometry: is the Pi driver running, and the same ROS domain here?')
        time.sleep(5.0)
        n = node.count_publishers('/diff_cont/odom')
        if n != 1:
            raise RuntimeError(f'{n} odometry publishers: there must be exactly 1 (another robot on this domain?)')
        if node.count_publishers('/cmd_vel_tracker') != 0:
            raise RuntimeError('another controller is already publishing /cmd_vel_tracker: stop it first')
        if node.count_subscribers('/cmd_vel_tracker') == 0:
            raise RuntimeError('nothing listens to /cmd_vel_tracker: are the laptop programs running?')
        have_map = seen['map_at'] is not None and time.monotonic() - seen['map_at'] < 1.5 and seen['maps'] > 5
        state = a.state if a.state != 'auto' else ('fused' if have_map else 'odom')
        if state == 'fused' and not have_map:
            raise RuntimeError('fused needs the LiDAR map pose (/robot_pose), and none is being received (run: robot.sh lidar)')
        node.create_subscription(Twist, '/cmd_vel_tracker', lambda m: seen.update(cmd=(m.linear.x, m.angular.z)), 10)
        if state == 'fused':
            relay['pub'] = node.create_publisher(Odometry, '/fused_odom', 10)

        os.makedirs(os.path.join(folder, 'runs'), exist_ok=True)
        rows = read_rows(folder)
        run = 1 + max([int(r['run']) for r in rows], default=0)
        base = os.path.join(folder, 'runs', f'path{run:03d}')
        command, given, goals = controller_command(controller, shape, s, state, have_map, base + '_controller.csv')
        room = paths.room_needed(shape)
        print(f'path run {run}: {shape} with {controller}, steering by {state}'
              + ('   (LiDAR map recorded as an independent check)' if have_map and state == 'odom' else ''))
        print('FREE FLOOR needed from where the robot stands: ' + ', '.join(f'{k} {v:.2f} m' for k, v in room.items()))
        if not a.yes and input('Press Enter to drive, or type n to cancel: ').strip().lower().startswith('n'):
            print('cancelled, nothing moved')
            close()
            return 'CANCELLED'

        log = open(base + '.log', 'w')
        with lock:
            start_odom = odom_only.state()[0]
            start_map = in_map.state()[0] if have_map else None
        controller_proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                           preexec_fn=rt.die_with_parent)

        def text():
            with open(base + '.log', errors='replace') as f:
                return f.read()

        reference = None
        if controller == 'p_servo':
            reference = node.create_publisher(PoseStamped, '/reference', 10)
            t0 = time.monotonic()
            while not (reference.get_subscription_count() > 0 and 'wheel odometry received' in text()
                       and (state != 'fused' or 'map frame locked' in text())):
                if controller_proc.poll() is not None or time.monotonic() - t0 > 60:
                    raise RuntimeError(f'p_controller did not become ready; see {base}.log')
                time.sleep(0.1)
        control_start = start_map if state == 'fused' else start_odom
        judge = paths.Judge(shape, (0.0, 0.0, 0.0))
        judge_map = paths.Judge(shape, (0.0, 0.0, 0.0)) if have_map else None
        limit = time_limit(controller, shape)
        trace_file = open(base + '_trace.csv', 'w', newline='')
        trace = csv.writer(trace_file)
        trace.writerow(['t', 'x', 'y', 'yaw', 'map_x', 'map_y', 'map_yaw', 'cmd_v', 'cmd_w', 'off_path_cm'])
        sent = time.monotonic()
        last_motion, status, finished_at, goal_index, apart = sent, 'TIMEOUT', None, -1, 0.0
        pose_c = (0.0, 0.0, 0.0)
        while time.monotonic() - sent < limit:
            time.sleep(0.05)
            now = time.monotonic()
            with lock:
                o = odom_only.state()[0]
                mp = in_map.state()[0] if have_map else None
                v, w = seen['odom'][2], seen['odom'][3]
            ro = relative(o, start_odom)
            rmap = relative(mp, start_map) if mp is not None else None
            pose_c = relative(mp, control_start) if state == 'fused' else ro
            off = judge.add(pose_c[0], pose_c[1])
            if judge_map is not None:
                judge_map.add(rmap[0], rmap[1])
                apart = max(apart, math.hypot(rmap[0] - ro[0], rmap[1] - ro[1]))
            trace.writerow([f'{now - sent:.3f}', *(f'{q:.4f}' for q in ro), *((f'{q:.4f}' for q in rmap) if rmap else ('', '', '')),
                            f'{seen["cmd"][0]:.4f}', f'{seen["cmd"][1]:.4f}', f'{100 * off:.2f}'])
            if abs(v) >= rt.STILL[0] or abs(w) >= rt.STILL[1]:
                last_motion = now
            if now - seen['odom_at'] > ODOM_SILENCE:
                status = 'ABORTED: odometry stopped'
                break
            if off > paths.ABORT_OFF_PATH:
                status = f'ABORTED: more than {100 * paths.ABORT_OFF_PATH:.0f} cm from the path'
                break
            if controller_proc.poll() is not None:
                status = 'ABORTED: controller exited'
                break
            said = text()
            if controller == 'p_servo':
                if said.count('ARRIVED:') > goal_index and finished_at is None:
                    goal_index += 1
                    if goal_index >= len(goals):
                        finished_at = now
                    else:
                        gx, gy, gth = paths.place([goals[goal_index]], control_start)[0]
                        msg = PoseStamped()
                        msg.header.frame_id = 'map' if state == 'fused' else 'odom'
                        msg.pose.position.x, msg.pose.position.y = float(gx), float(gy)
                        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(gth / 2), math.cos(gth / 2)
                        reference.publish(msg)
                        last_motion = now
            elif finished_at is None and ('final heading done' in said or 'lap complete' in said):
                finished_at = now
            if 'left the modelled tube' in said:
                status = 'FAILED'
                break
            if finished_at is not None and now - finished_at > 2.0:
                status = 'COMPLETE'
                break
            if finished_at is None and now - last_motion > STUCK_AFTER:
                status = 'STUCK'
                break
        trace_file.close()
        duration = (finished_at or time.monotonic()) - sent
    except KeyboardInterrupt:
        if controller_proc is None or 'trace_file' not in locals():
            print('\nstopped before any movement')
            close()
            return 'CANCELLED'
        trace_file.close()
        status, duration = 'ABORTED: stopped by the operator', time.monotonic() - sent
    except BaseException:
        if controller_proc is not None and controller_proc.poll() is None:
            os.killpg(controller_proc.pid, signal.SIGINT)
        close()
        raise
    if controller_proc.poll() is None:
        os.killpg(controller_proc.pid, signal.SIGINT)
        try:
            controller_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(controller_proc.pid, signal.SIGKILL)
    time.sleep(1.0)
    with lock:
        final_odom = relative(odom_only.state()[0], start_odom)
        final_map = relative(in_map.state()[0], start_map) if have_map else None
    final_control = relative(in_map.state()[0], control_start) if state == 'fused' else final_odom
    r = judge.result(final_control)
    row = {'run': run, 'session': a.session, 'date': datetime.date.today().isoformat(),
           'time': datetime.datetime.now().strftime('%H:%M:%S'), 'shape': shape, 'controller': controller, 'state': state,
           'status': status, 'max_cm': f'{r["max_cm"]:.2f}', 'rms_cm': f'{r["rms_cm"]:.2f}', 'end_cm': f'{r["end_cm"]:.2f}',
           'end_deg': f'{r["end_deg"]:.2f}', 'time_s': f'{duration:.1f}', 'covered': f'{r["covered"]:.2f}',
           'map_max_cm': '', 'map_rms_cm': '', 'map_end_cm': '', 'map_end_deg': '', 'estimates_apart_cm': '',
           'fallbacks': text().count('neighbouring cell') if controller == 'symbolic' else '',
           'settings': json.dumps(given, sort_keys=True), 'code': rt.code_version(), 'note': a.note}
    if judge_map is not None:
        m = judge_map.result(final_map)
        row.update(map_max_cm=f'{m["max_cm"]:.2f}', map_rms_cm=f'{m["rms_cm"]:.2f}', map_end_cm=f'{m["end_cm"]:.2f}',
                   map_end_deg=f'{m["end_deg"]:.2f}', estimates_apart_cm=f'{100 * apart:.2f}')
    rows.append(row)
    write_rows(folder, rows)
    print(f'{status}  by the {"fused" if state == "fused" else "odometry"} position the controller used: off the path rms {row["rms_cm"]} cm, '
          f'max {row["max_cm"]} cm; end {row["end_cm"]} cm, {row["end_deg"]} deg; {row["time_s"]} s')
    if judge_map is not None:
        print(f'      by the LiDAR map{" (independent: not used for steering)" if state == "odom" else ""}: rms {row["map_rms_cm"]} cm, '
              f'max {row["map_max_cm"]} cm; end {row["map_end_cm"]} cm, {row["map_end_deg"]} deg; odometry and map differ by up to '
              f'{row["estimates_apart_cm"]} cm')
    close()
    return status


def series(a, folder):
    shapes_wanted = [s for s in a.shapes.split(',') if s]
    controllers = [c for c in a.controllers.split(',') if c]
    for c in controllers:
        if c not in paths.CONTROLLERS:
            raise ValueError(f'unknown controller "{c}"')
    plan = [(sh, c) for sh in shapes_wanted for c in controllers if c != 'symbolic' or sh == paths.SYMBOLIC_SHAPE]
    for sh in shapes_wanted:
        paths.shape(sh)
    if not plan:
        raise ValueError('nothing to run')
    a.session = a.session or datetime.datetime.now().strftime('%m%d-%H%M%S')
    print(f'PATH SERIES {a.session}: {len(plan)} runs: ' + ', '.join(f'{sh}/{c}' for sh, c in plan))
    print('Before EACH run you can put the robot back on its start mark; it asks every time (Enter = drive, s = skip, q = stop).')
    code, done = 0, 0
    for k, (shape, controller) in enumerate(plan, 1):
        room = paths.room_needed(shape)
        print(f'\n----- {k}/{len(plan)}: {shape} with {controller} -----')
        print('free floor: ' + ', '.join(f'{n} {v:.2f} m' for n, v in room.items()))
        if not a.yes:
            answer = input('Robot on its start mark and area clear? [Enter = drive, s = skip, q = stop] ').strip().lower()
            if answer.startswith('q'):
                break
            if answer.startswith('s'):
                continue
        one = copy.copy(a)
        one.yes = True
        try:
            status = run_path(one, folder, controller, shape)
        except (ValueError, RuntimeError) as e:
            print(f'SERIES STOPPED: {e}', file=sys.stderr)
            code = 2
            break
        except KeyboardInterrupt:
            print('\nSERIES STOPPED by the operator')
            code = 1
            break
        if status.split(':')[0] not in COMPLETED:
            code = 1
            if a.yes or input(f'That run ended "{status}". Check the robot. Continue the series? [y/N] ').strip().lower() != 'y':
                break
        done += 1
    print(f'\nseries finished: {done}/{len(plan)} runs driven')
    rt.try_report(folder)
    return code


def main(argv=None):
    for name in ('SIGTERM', 'SIGHUP'):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), signal.default_int_handler)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('what', help='a shape name, "series", or "shapes"')
    ap.add_argument('--controller', default='pure_pursuit', choices=paths.CONTROLLERS)
    ap.add_argument('--shapes', default='square,circle_r50')
    ap.add_argument('--controllers', default=','.join(paths.CONTROLLERS))
    ap.add_argument('--state', default='auto', choices=['auto', 'odom', 'fused'])
    ap.add_argument('--settings', default='')
    ap.add_argument('--session', default='')
    ap.add_argument('--note', default='')
    ap.add_argument('--yes', action='store_true')
    a = ap.parse_args(argv)
    folder = os.environ.get('ROBOT_LOGBOOK') or os.path.expanduser('~/robot_logbook')
    if a.what == 'shapes':
        for name in paths.NAMES:
            room = paths.room_needed(name)
            print(f'{name:12} length {paths.length_of(paths.shape(name)[1]):.2f} m   free floor: '
                  + ', '.join(f'{k} {v:.2f}' for k, v in room.items()))
        return 0
    try:
        if a.what == 'series':
            return series(a, folder)
        status = run_path(a, folder, a.controller, a.what)
        if status != 'CANCELLED':
            rt.try_report(folder)
        return 0 if status in ('COMPLETE', 'CANCELLED') else 1
    except (ValueError, RuntimeError) as e:
        print(f'NOT STARTED / STOPPED: {e}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
