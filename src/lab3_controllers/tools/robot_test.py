#!/usr/bin/env python3
"""One command per robot test, and one logbook that keeps every result with the coefficients that produced it.

    python3 robot_test.py turn 30          turn by 30 deg from where the robot is (+ = left)
    python3 robot_test.py heading 90       turn to the heading 90 deg of the present frame
    python3 robot_test.py drive 0.2        drive 0.2 m along the present heading (negative = backwards)
    python3 robot_test.py suite            a whole series of turns and drives, both controllers (add --quick for a short one)
    python3 robot_test.py pi2              turn to the heading 90 deg (pi/2) six times, from different angles and both sides
                                           (--compare adds the plain P controller; --repeat 2 does it twice)
    python3 robot_test.py report           rebuild report.html (tables and one plot per run) from the logbook
    python3 robot_test.py show             print the logbook
    python3 robot_test.py hand 3 19.5      add a hand measurement to run 3 (deg turned, or cm travelled; same sign as asked)

    --set heading.kp=2.5     change a coefficient for this run (repeatable). Change ONE thing between runs when tuning.
    --note "on carpet"       free text kept in the logbook
    --frame auto|odom|map    auto: the professor's /robot_pose if it is being received, else wheel odometry
    --controller p|basic     p = p_controller (ours, default). basic = the plain P law of reference_controller
                             (config/reference_controller.yaml: no derivative, no pulses, no learning); forward drives only
    --profile v1|<file>|none robot settings given to p_controller (default: config/robot_measured.yaml, our robot's measurements)
    --yes                    do not ask before moving (the operator has already confirmed the area is clear)

It starts the controller itself (do NOT start another one), sends one reference, waits until the robot has finished, stops the
controller (which sends zero) and writes the run. Both controllers are recorded and judged the same way by this tool:
error = what the robot's sensors say after it has come to rest; time_s = last moment it was moving.
Status: ARRIVED (p says so) | SETTLED (basic: at rest, command zero) | STUCK (at rest although still commanded: the command is
too small to move the motors) | TIMEOUT | ABORTED. It aborts if the robot goes outside the limits below.
The controller starts fresh each run, so what it learns during one run does not leak into the next: runs are comparable.
Logbook folder: $ROBOT_LOGBOOK, else ~/robot_logbook  (logbook.csv, LOGBOOK.md, runs/runNNN.csv|.log|_coefficients.txt)
"""
import argparse
import csv
import datetime
import math
import os
import signal
import subprocess
import sys
import threading
import time

LIMITS = {'turn_deg': 170.0, 'drive_m': 1.0,               # largest request accepted
          'turn_drift_m': 0.15,                            # a turn on the spot may not move this far
          'turn_extra_deg': 60.0,                          # nor get this much further from the target than at the start
          'drive_extra_m': 0.15, 'drive_side_m': 0.20,     # nor drive this far past the target or off the line
          'odom_silence_s': 1.5}

CONTROLLERS = {'p': 'p_controller', 'basic': 'plain P'}
STILL = (0.005, 0.02)                                      # m/s, rad/s: below both the robot counts as at rest

FIELDS = ['run', 'session', 'controller', 'date', 'time', 'test', 'asked', 'unit', 'topic', 'sent', 'frame', 'status', 'error', 'overshoot', 'sideways_cm', 'time_s',
          'hand_value', 'hand_error', 'changed', 'profile', 'controller_version', 'kp', 'lead', 'u_max', 'tol', 'u_min_start', 'pulse_share',
          'learned_u_min', 'learned_lead', 'learned_gain', 'code', 'note']


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_changes(items, axes):
    """['heading.kp=2.5'] -> {'heading.kp': 2.5}; unknown names and non-numbers are refused."""
    out = {}
    for item in items:
        name, sep, value = item.partition('=')
        axis, _, key = name.partition('.')
        if not sep or axis not in axes or key not in axes[axis]:
            raise ValueError(f'unknown coefficient "{item}": use heading.<name>=<number> or linear.<name>=<number>')
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f'"{item}" is not a finite number')
        out[name] = number
    return out


def progress(test, start, pose, goal):
    """-> (remaining, sideways): remaining is deg for turns and cm for a drive; negative once past the target on a drive."""
    if test == 'drive':
        x0, y0, th0 = start
        along = (pose[0] - x0) * math.cos(th0) + (pose[1] - y0) * math.sin(th0)
        side = -(pose[0] - x0) * math.sin(th0) + (pose[1] - y0) * math.cos(th0)
        return 100 * (goal - along), 100 * side
    return math.degrees(wrap(goal - pose[2])), 0.0


def limit_broken(test, start, pose, goal):
    """-> reason to abort, or None."""
    remaining, side = progress(test, start, pose, goal)
    if test == 'drive':
        if remaining * math.copysign(1, goal) < -100 * LIMITS['drive_extra_m']:
            return 'drove too far past the target'
        if abs(remaining) > 100 * (abs(goal) + LIMITS['drive_extra_m']):
            return 'is driving away from the target'
        if abs(side) > 100 * LIMITS['drive_side_m']:
            return 'is too far off the line'
        return None
    if math.hypot(pose[0] - start[0], pose[1] - start[1]) > LIMITS['turn_drift_m']:
        return 'moved away during a turn on the spot'
    first = abs(math.degrees(wrap(goal - start[2])))
    if abs(remaining) > min(first + LIMITS['turn_extra_deg'], 179.0):
        return 'is turning away from the target'
    return None


def overshoot(first_remaining, history):
    """Largest travel PAST the target (same unit as the history), 0 if it never passed. Turns: ignores the +-180 wrap."""
    direction = 1.0 if first_remaining >= 0 else -1.0
    return max([0.0] + [-direction * r for r in history if abs(r) < 90.0])


def suite_plan(quick=False):
    """-> [(controller, test, value)]. Moves come in opposite pairs so that the robot stays where it started."""
    turns = [15, -15, 45, -45] if quick else [10, -10, 20, -20, 45, -45, 90, -90, 150, -150]
    drives = [0.2] if quick else [0.1, 0.2, 0.4]
    plan = [(c, 'turn', float(v)) for v in turns for c in ('p', 'basic')]
    for d in drives:                       # plain P cannot reverse: it drives forward, p_controller brings the robot back
        plan += [('p', 'drive', d), ('p', 'drive', -d), ('basic', 'drive', d), ('p', 'drive', -d)]
    return plan


def at_rest_status(since_sent, since_motion, command):
    """For a controller that does not announce arrival: None while it may still be working, else SETTLED or STUCK."""
    if since_sent < 5.0 or since_motion < 4.0:
        return None
    return 'SETTLED' if max(abs(command[0]), abs(command[1])) < 1e-3 else 'STUCK'


def completed(status):
    return status in ('ARRIVED', 'SETTLED', 'STUCK', 'TIMEOUT')


DEFAULT_PROFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'robot_measured.yaml')


def read_profile(path):
    """'heading.name: number' lines of a settings file -> {name: float}."""
    out = {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            text = line.split('#')[0].strip()
            if text.startswith(('heading.', 'linear.')):
                name, value = text.split(':')
                out[name.strip()] = float(value)
    return out


PI2_STARTS = [0.0, 180.0, 45.0, 135.0, -30.0, 150.0]       # 90 deg is approached from both sides, by 45 to 120 deg


def pi2_plan(compare=False):
    """-> [(controller, test, value, note)]: before each pi/2 run p_controller turns to a different starting angle."""
    plan = []
    for kind in (('p', 'basic') if compare else ('p',)):
        for start in PI2_STARTS:
            plan += [('p', 'heading', start, 'setup'), (kind, 'heading', 90.0, 'pi/2')]
    return plan


def controller_version():
    """A short fingerprint of the control law that is actually installed and run."""
    import hashlib
    from lab3_controllers import servo
    with open(servo.__file__, 'rb') as f:
        return hashlib.sha1(f.read().replace(b'\r', b'')).hexdigest()[:7]


def read_book(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def write_book(folder, rows):
    path = os.path.join(folder, 'logbook.csv')
    with open(path + '.tmp', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    os.replace(path + '.tmp', path)
    cols = ['run', 'session', 'controller', 'time', 'test', 'asked', 'unit', 'topic', 'sent', 'frame', 'status', 'error', 'overshoot', 'time_s', 'hand_error', 'changed', 'note']
    lines = ['# Robot test logbook', '',
             'error, overshoot and hand_error are in deg for turns and cm for drives. "error" is what the robot\'s own sensors say;',
             '"hand_error" is from the floor marks or tape. All coefficients of each run: logbook.csv and runs/.', '',
             '| ' + ' | '.join(cols) + ' |', '|' + '---|' * len(cols)]
    lines += ['| ' + ' | '.join(str(r.get(c, '')).replace('|', '/') for c in cols) + ' |' for r in rows]
    with open(os.path.join(folder, 'LOGBOOK.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def hand_target(row):
    """What the hand measurement is compared with: the turn actually requested (deg) or the distance (cm)."""
    if row['test'] == 'drive':
        return 100 * float(row['asked'])
    if row['test'] == 'turn':
        return float(row['asked'])
    return None                               # an absolute heading cannot be compared with a turned angle


def set_hand(row, value):
    target = hand_target(row)
    row['hand_value'] = f'{value:g}'
    row['hand_error'] = '' if target is None else f'{target - value:+.2f}'


def code_version():
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        out = subprocess.run(['git', '-C', here, 'log', '-1', '--format=%h'], capture_output=True, text=True, timeout=10)
        return out.stdout.strip() or 'unknown'
    except Exception:                          # noqa: BLE001
        return 'unknown'


def die_with_parent():
    """Run in the controller process before it starts: the kernel ends it if this tool dies in ANY way (closed terminal,
    kill -9, crash), so a controller can never be left driving the robot. SIGTERM makes the controller send zero first."""
    import ctypes
    ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGTERM)          # 1 = PR_SET_PDEATHSIG
    if os.getppid() == 1:
        os._exit(1)


def run_test(a, folder):
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float64

    from lab3_controllers.common import yaw_from_quaternion
    from lab3_controllers.servo import LINEAR_SERVO_DEFAULTS, SERVO_DEFAULTS, MapOdomFusion

    axes = {'heading': SERVO_DEFAULTS, 'linear': LINEAR_SERVO_DEFAULTS}
    changes = parse_changes(a.set, axes)
    test, asked, kind = a.test, a.value, a.controller
    if kind == 'basic' and changes:
        raise ValueError('--set changes p_controller coefficients: it does not apply to --controller basic')
    wanted = DEFAULT_PROFILE.replace('.yaml', '_v1.yaml') if a.profile == 'v1' else (a.profile or DEFAULT_PROFILE)
    profile_path = None if a.profile == 'none' or kind != 'p' else wanted
    robot_settings = {}
    if profile_path:
        robot_settings = parse_changes([f'{k}={v!r}' for k, v in read_profile(profile_path).items()], axes)
        if test == 'drive' and not a.profile:              # straight-line settings: the tuned ones of tools/path_tune.py
            import paths
            robot_settings.update(parse_changes([f'linear.{k}={float(v)!r}' for k, v in
                                                 paths.settings()['p_servo']['linear'].items()], axes))
    if kind == 'basic' and test == 'drive' and asked < 0:
        raise ValueError('the plain P controller cannot drive backwards (it would turn around)')
    if test == 'turn' and not 0 < abs(asked) <= LIMITS['turn_deg']:
        raise ValueError(f'a turn must be between -{LIMITS["turn_deg"]:g} and {LIMITS["turn_deg"]:g} deg, not 0')
    if test == 'heading' and abs(asked) > 360:
        raise ValueError('give the heading in degrees')
    if test == 'drive' and not 0.03 <= abs(asked) <= LIMITS['drive_m']:
        raise ValueError(f'a drive must be 0.03 to {LIMITS["drive_m"]:g} m (give metres, not cm)')

    # Ctrl+C must reach this tool, not shut ROS down under it: the final reading is taken AFTER the controller is stopped
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('robot_test')
    seen = {'odom_at': None, 'odom': None, 'map_at': None, 'maps': 0}
    state = {'fusion': None}

    def pose_of(q, p):
        return p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        item = (stamp(m.header), pose_of(q, p), m.twist.twist.linear.x, m.twist.twist.angular.z)
        seen['odom_at'], seen['odom'] = time.monotonic(), item
        if state['fusion'] is not None:
            state['fusion'].add_odom(*item)

    def on_map(m):
        seen['maps'] += 1
        if state['fusion'] is not None and state['fusion'].add_map(stamp(m.header), pose_of(m.pose.orientation, m.pose.position)):
            seen['map_at'] = time.monotonic()

    node.create_subscription(Odometry, '/diff_cont/odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/robot_pose', on_map, qos_profile_sensor_data)
    if test == 'drive':
        topic = '/reference_x' if kind == 'p' else '/reference'      # plain P takes a goal pose, not a distance
    else:
        topic = '/reference_yaw'
    message_type = PoseStamped if topic == '/reference' else Float64
    reference = node.create_publisher(message_type, topic, 10)
    command_seen = {'last': (0.0, 0.0)}
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()

    def close():
        """Stop the ROS thread BEFORE the library shuts down (otherwise the process can crash while exiting)."""
        ex.shutdown()
        spinner.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    controller = None
    moving_done = False
    try:
        print('waiting for the robot ...', flush=True)
        t0 = time.monotonic()
        while seen['odom_at'] is None and time.monotonic() - t0 < 60:
            time.sleep(0.1)
        if seen['odom_at'] is None:
            raise RuntimeError('no odometry: is ~/start_robot.sh running on the Pi, and the same ROS domain here?')
        time.sleep(4.0)
        odom_publishers = node.count_publishers('/diff_cont/odom')
        if odom_publishers != 1:
            raise RuntimeError(f'{odom_publishers} odometry publishers: there must be exactly 1 (another robot on this domain?)')
        if node.count_publishers('/cmd_vel_tracker') != 0:
            raise RuntimeError('another controller is already publishing /cmd_vel_tracker: stop it first')
        if node.count_subscribers('/cmd_vel_tracker') == 0:
            raise RuntimeError('nothing listens to /cmd_vel_tracker: is ~/start_laptop_stack.sh running?')
        frame = a.frame if a.frame != 'auto' else ('map' if seen['maps'] > 3 else 'odom')
        node.create_subscription(Twist, '/cmd_vel_tracker',
                                 lambda m: command_seen.update(last=(m.linear.x, m.angular.z)), 10)

        os.makedirs(os.path.join(folder, 'runs'), exist_ok=True)
        rows = read_book(os.path.join(folder, 'logbook.csv'))
        run = 1 + max([int(r['run']) for r in rows], default=0)
        base = os.path.join(folder, 'runs', f'run{run:03d}')
        data = base + '.csv'
        config = None
        programs = os.path.join(get_package_prefix('lab3_controllers'), 'lib', 'lab3_controllers')
        if kind == 'p':
            command = [os.path.join(programs, 'p_controller'), '--ros-args', '-p', f'log_csv:={data}']
            if frame == 'odom':
                command += ['-p', "map_topic:=''"]
        else:
            config = os.path.join(get_package_share_directory('lab3_controllers'), 'config', 'reference_controller.yaml')
            command = [os.path.join(programs, 'reference_controller'), '--ros-args', '--params-file', config,
                       '-p', f'state_topic:={"/robot_pose" if frame == "map" else "/diff_cont/odom"}',
                       '-p', 'cmd_topic:=/cmd_vel_tracker', '-p', 'start_delay:=0.5']
        for name, value in dict(robot_settings, **changes).items():
            command += ['-p', f'{name}:={float(value)!r}']

        what = {'turn': f'turn {asked:+g} deg on the spot', 'heading': f'turn to heading {asked:g} deg',
                'drive': f'drive {100 * asked:+g} cm'}[test]
        print(f'run {run}: {CONTROLLERS[kind]}: {what}   frame: {"professor map (/robot_pose)" if frame == "map" else "wheel odometry"}')
        print(f'robot settings: {os.path.basename(profile_path) if profile_path else "none (code defaults)"}   '
              f'changed for this run: {", ".join(f"{k}={v:g}" for k, v in changes.items()) or "nothing"}')
        print('topics: state in    /diff_cont/odom (nav_msgs/Odometry)' + (' + /robot_pose (geometry_msgs/PoseStamped)' if frame == 'map' else ''))
        meaning = ('geometry_msgs/PoseStamped, goal pose' if message_type is PoseStamped
                   else 'std_msgs/Float64, ' + ('metres ahead' if test == 'drive' else 'target heading in rad'))
        print(f'        reference   {topic} ({meaning})')
        print('        command out /cmd_vel_tracker -> twist_mux -> /diff_cont/cmd_vel_unstamped (geometry_msgs/Twist)')
        if not a.yes:
            print('Mark the start position on the floor. Check the area is clear. Stop at any time with Ctrl+C or the motor switch.')
            if input('Press Enter to move, or type n to cancel: ').strip().lower().startswith('n'):
                print('cancelled, nothing moved')
                return 0

        log = open(base + '.log', 'w')
        controller = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                      preexec_fn=die_with_parent)

        def said(text):
            with open(base + '.log', errors='replace') as f:
                return text in f.read()

        if frame == 'map':
            state['fusion'] = MapOdomFusion(use_map=True)
        else:
            state['fusion'] = MapOdomFusion(use_map=False)
        t0 = time.monotonic()
        started = 'wheel odometry received' if kind == 'p' else 'first state'
        while not (reference.get_subscription_count() > 0 and said(started) and state['fusion'].ready()
                   and (frame == 'odom' or kind != 'p' or said('map frame locked'))):
            if controller.poll() is not None or time.monotonic() - t0 > 60:
                raise RuntimeError(f'the controller did not become ready; see {base}.log')
            time.sleep(0.1)
        time.sleep(1.0)
        start = state['fusion'].state()[0]
        goal = {'turn': wrap(start[2] + math.radians(asked)), 'heading': wrap(math.radians(asked)), 'drive': asked}[test]
        first = progress(test, start, start, goal)[0]

        if message_type is PoseStamped:
            message = PoseStamped()
            message.header.frame_id = frame
            message.pose.position.x = start[0] + asked * math.cos(start[2])
            message.pose.position.y = start[1] + asked * math.sin(start[2])
            message.pose.orientation.z, message.pose.orientation.w = math.sin(start[2] / 2), math.cos(start[2] / 2)
            sent_text = f'{message.pose.position.x:.4f} {message.pose.position.y:.4f} {start[2]:.4f}'
        else:
            message = Float64(data=float(goal))
            sent_text = f'{goal:.4f}'
        print(f'sending {topic}: {sent_text}' + ('' if test == 'drive' else f' rad ({math.degrees(goal):.1f} deg)'), flush=True)
        trace_file = open(base + '_trace.csv', 'w', newline='')
        trace = csv.writer(trace_file)
        trace.writerow(['t', 'x', 'y', 'yaw', 'v', 'w', 'cmd_v', 'cmd_w', 'error', 'sideways'])
        reference.publish(message)
        sent = time.monotonic()
        history, arrived_at, status, resent = [], None, 'TIMEOUT', False
        last_motion, moved = sent, False
        accepted = 'new reference' if kind == 'p' else ('new pose reference' if message_type is PoseStamped else 'new yaw reference')
        while time.monotonic() - sent < a.timeout:
            time.sleep(0.05)
            now = time.monotonic()
            pose = state['fusion'].state()[0]
            v, w = seen['odom'][2], seen['odom'][3]
            if abs(v) >= STILL[0] or abs(w) >= STILL[1]:
                last_motion, moved = now, True
            remaining, off = progress(test, start, pose, goal)
            history.append(remaining)
            trace.writerow([f'{now - sent:.3f}', f'{pose[0]:.4f}', f'{pose[1]:.4f}', f'{pose[2]:.4f}', f'{v:.4f}', f'{w:.4f}',
                            f'{command_seen["last"][0]:.4f}', f'{command_seen["last"][1]:.4f}', f'{remaining:.3f}', f'{off:.2f}'])
            if now - seen['odom_at'] > LIMITS['odom_silence_s']:
                status = 'ABORTED: odometry stopped'
                break
            reason = limit_broken(test, start, pose, goal)
            if reason:
                status = f'ABORTED: robot {reason}'
                break
            if controller.poll() is not None:
                status = 'ABORTED: controller exited'
                break
            if not resent and now - sent > 2.0 and not said(accepted):
                reference.publish(message)
                resent = True
            if kind == 'p':
                if arrived_at is None and said('ARRIVED:'):
                    arrived_at = now
                if arrived_at is not None and now - arrived_at > 3.0:
                    status = 'ARRIVED'
                    break
            else:
                ending = at_rest_status(now - sent, now - last_motion, command_seen['last'])
                if ending:
                    status = ending
                    break
        interrupted = False
        moving_done = True
    except KeyboardInterrupt:
        interrupted = True
        if controller is None or 'history' not in locals():
            print('\nstopped before any movement')
            return 1
        status = 'ABORTED: stopped by the operator'
        moving_done = True
    finally:
        if controller is not None and controller.poll() is None:
            os.killpg(controller.pid, signal.SIGINT)           # the controller sends zero as it shuts down
            try:
                controller.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(controller.pid, signal.SIGKILL)
        if not moving_done:                                    # refused, cancelled or failed: nothing to record
            close()

    trace_file.close()
    time.sleep(1.0)                                            # let the robot come to rest before the final reading
    pose = state['fusion'].state()[0]
    error, side = progress(test, start, pose, goal)
    unit = 'cm' if test == 'drive' else 'deg'
    last = None
    if kind == 'p' and os.path.exists(data):
        with open(data, newline='') as f:
            for last in csv.DictReader(f):
                pass
    axis = 'linear' if test == 'drive' else 'heading'
    used = {f'{ax}.{k}': v for ax, d in axes.items() for k, v in d.items()}
    used.update(robot_settings)
    used.update(changes)
    if kind != 'p':                                            # the plain P coefficients, read from its own file
        import yaml
        with open(config) as f:
            text = f.read()
        c = yaml.safe_load(text)['reference_controller']['ros__parameters']
        drive = test == 'drive'
        used.update({f'{axis}.kp': c['kv' if drive else 'kp'], f'{axis}.lead': c['lead_v' if drive else 'lead_w'],
                     f'{axis}.u_max': c['max_linear_speed' if drive else 'max_angular_speed'],
                     f'{axis}.tol': c['pos_tolerance' if drive else 'tolerance'],
                     f'{axis}.u_min': c['deadband_v' if drive else 'deadband_w'], f'{axis}.pulse_share': ''})
    with open(base + '_coefficients.txt', 'w') as f:
        f.write(''.join(f'{k} = {v}\n' for k, v in used.items()) if kind == 'p' else text)
    learned = ('linear_u_min', 'linear_lead', None) if axis == 'linear' else ('u_min', 'lead', 'gain')
    row = {'run': run, 'session': a.session, 'controller': CONTROLLERS[kind], 'date': datetime.date.today().isoformat(), 'time': datetime.datetime.now().strftime('%H:%M:%S'),
           'test': test, 'asked': f'{asked:g}', 'unit': 'm' if test == 'drive' else 'deg', 'topic': topic, 'sent': sent_text,
           'frame': frame, 'status': status,
           'error': f'{error:+.2f}', 'overshoot': f'{overshoot(first, history):.2f}',
           'sideways_cm': f'{side:+.1f}' if test == 'drive' else '',
           'time_s': f'{last_motion - sent:.1f}' if moved else '', 'hand_value': '', 'hand_error': '',
           'changed': '; '.join(f'{k}={v:g}' for k, v in changes.items()),
           'profile': os.path.basename(profile_path) if profile_path else '', 'controller_version': controller_version(),
           'kp': used[f'{axis}.kp'], 'lead': used[f'{axis}.lead'], 'u_max': used[f'{axis}.u_max'], 'tol': used[f'{axis}.tol'],
           'u_min_start': used[f'{axis}.u_min'], 'pulse_share': used[f'{axis}.pulse_share'],
           'learned_u_min': last[learned[0]] if last else '', 'learned_lead': last[learned[1]] if last else '',
           'learned_gain': last[learned[2]] if last and learned[2] else '', 'code': code_version(), 'note': a.note}
    rows.append(row)
    write_book(folder, rows)                                   # saved before any question, so nothing is lost
    print(f'\n{status}   robot says: error {error:+.2f} {unit}' + (f', sideways {side:+.1f} cm' if test == 'drive' else '')
          + f', overshoot {float(row["overshoot"]):.2f} {unit}' + (f', time {row["time_s"]} s' if row['time_s'] else ''))
    if not a.yes and not interrupted and not a.session:
        quantity = 'distance travelled in cm (negative = backwards)' if test == 'drive' else 'angle turned in deg (+ = left)'
        try:
            text = input(f'Your own measurement, {quantity} (Enter to skip): ').strip()
            if text:
                set_hand(row, float(text.replace(',', '.')))
                write_book(folder, rows)
        except (ValueError, EOFError, KeyboardInterrupt):
            print(f'not recorded; add it later with: robot_test.py hand {run} <number>')
    print(f'saved as run {run} in {folder}')
    close()
    a.last_status = status
    return 0 if status in ('ARRIVED', 'SETTLED', 'STUCK') else 1


def try_report(folder):
    try:
        import robot_report
        robot_report.make_report(folder)
    except Exception as e:                       # noqa: BLE001  (the data is already saved; the report can be redone)
        print(f'report not made ({e}); the logbook is complete. Retry: robot_test.py report')


def run_suite(a, folder):
    import copy
    if a.test == 'pi2':
        plan = pi2_plan(a.compare) * max(1, a.repeat)
    else:
        plan = [(c, t, v, a.note) for c, t, v in suite_plan(a.quick)] * max(1, a.repeat)
    session = datetime.datetime.now().strftime('%m%d-%H%M%S')
    reach = 100 * max([v for _, t, v, _ in plan if t == 'drive'], default=0)
    print(f'SESSION {session}: {len(plan)} runs, roughly {len(plan) * 25 // 60 + 1} minutes.')
    if reach:
        print(f'The robot turns on the spot, drives up to {reach:.0f} cm ahead of where it starts and comes back. '
              'It does NOT see obstacles.')
        print('NEEDED: robot on the floor, 1 m free all around it and 1.2 m in front. STOP: Ctrl+C here, or the motor battery switch.')
    else:
        print('The robot only turns on the spot. Every second run is a turn TO 90 deg (pi/2) of the odometry frame, '
              'from a different angle.')
        print('NEEDED: robot on the floor, 50 cm free all around, no cable that can wrap. STOP: Ctrl+C, or the motor switch.')
    if not a.yes and input('Press Enter to start the series, or type n to cancel: ').strip().lower().startswith('n'):
        print('cancelled, nothing moved')
        return 0
    done, code = 0, 0
    for kind, test, value, note in plan:
        one = copy.copy(a)
        one.controller, one.test, one.value, one.session, one.yes, one.note = kind, test, value, session, True, note
        one.set = a.set if kind == 'p' else []
        one.last_status = None
        print(f'\n----- {done + 1}/{len(plan)} -----', flush=True)
        try:
            run_test(one, folder)
        except (ValueError, RuntimeError) as e:
            print(f'SERIES STOPPED: {e}', file=sys.stderr)
            code = 2
            break
        except KeyboardInterrupt:
            print('\nSERIES STOPPED by the operator')
            code = 1
            break
        if one.last_status is None or not completed(one.last_status):
            print(f'SERIES STOPPED for safety after: {one.last_status}. Look at the robot before running again.')
            code = 1
            break
        done += 1
    print(f'\nseries finished: {done}/{len(plan)} runs completed')
    try_report(folder)
    return code


def main(argv=None):
    for name in ('SIGTERM', 'SIGHUP'):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), signal.default_int_handler)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('test', choices=['turn', 'heading', 'drive', 'suite', 'pi2', 'report', 'show', 'hand'])
    ap.add_argument('value', nargs='?', type=float)
    ap.add_argument('extra', nargs='?', type=float)
    ap.add_argument('--set', action='append', default=[], metavar='axis.name=value')
    ap.add_argument('--note', default='')
    ap.add_argument('--frame', choices=['auto', 'odom', 'map'], default='auto')
    ap.add_argument('--timeout', type=float, default=45.0)
    ap.add_argument('--yes', action='store_true')
    ap.add_argument('--controller', choices=sorted(CONTROLLERS), default='p')
    ap.add_argument('--quick', action='store_true', help='suite: a short series')
    ap.add_argument('--repeat', type=int, default=1, help='suite: run the whole series this many times')
    ap.add_argument('--session', default='')
    ap.add_argument('--profile', default='')
    ap.add_argument('--compare', action='store_true', help='pi2: also run the plain P controller')
    a = ap.parse_args(argv)
    folder = os.environ.get('ROBOT_LOGBOOK') or os.path.expanduser('~/robot_logbook')
    if a.test == 'show':
        path = os.path.join(folder, 'LOGBOOK.md')
        print(open(path, encoding='utf-8').read() if os.path.exists(path) else 'the logbook is empty')
        return 0
    if a.test == 'hand':
        rows = read_book(os.path.join(folder, 'logbook.csv'))
        match = [r for r in rows if a.value is not None and int(r['run']) == int(a.value)]
        if not match or a.extra is None:
            print('usage: robot_test.py hand <run number> <measured value>', file=sys.stderr)
            return 2
        set_hand(match[0], a.extra)
        write_book(folder, rows)
        print(f'run {int(a.value)}: hand value {a.extra:g}, hand error {match[0]["hand_error"] or "(not comparable for a heading test)"}')
        return 0
    if a.test == 'report':
        import robot_report
        robot_report.make_report(folder)
        return 0
    if a.test in ('suite', 'pi2'):
        try:
            from lab3_controllers.servo import LINEAR_SERVO_DEFAULTS, SERVO_DEFAULTS
            parse_changes(a.set, {'heading': SERVO_DEFAULTS, 'linear': LINEAR_SERVO_DEFAULTS})
        except ValueError as e:
            print(f'NOT STARTED: {e}', file=sys.stderr)
            return 2
        return run_suite(a, folder)
    if a.value is None or not math.isfinite(a.value):
        ap.error(f'{a.test} needs a number')
    try:
        return run_test(a, folder)
    except (ValueError, RuntimeError) as e:
        print(f'NOT STARTED / STOPPED: {e}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
