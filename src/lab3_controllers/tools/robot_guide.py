#!/usr/bin/env python3
"""The guided test day:  bash ~/robot.sh guide

Walks through every step in order. For each step it says what it is for and what to prepare, then:
    Enter = do it      s = skip      b = back      a number = jump to that step      q = quit (progress is remembered)
Nothing moves without a further confirmation from the step itself. Progress: $ROBOT_LOGBOOK/guide_state.json
"""
import datetime
import json
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HOME = os.path.expanduser('~')
ROBOT = os.path.join(HERE, 'robot.sh')


def book():
    return os.environ.get('ROBOT_LOGBOOK') or os.path.expanduser('~/robot_logbook')


def say(text=''):
    print(text, flush=True)


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return 'q'


def sh(command, **kw):
    return subprocess.call(command, **kw)


def robot(*args):
    """One robot.sh command; Ctrl+C stops that command, not the guide."""
    say(f'    > bash ~/robot.sh {" ".join(args)}')
    proc = subprocess.Popen(['bash', ROBOT, *args])
    while True:
        try:
            return proc.wait()
        except KeyboardInterrupt:
            say('\n    (stopping that command ...)')


def addresses():
    """-> (pi, laptop as configured) from ~/cdds.xml"""
    try:
        with open(os.path.join(HOME, 'cdds.xml')) as f:
            text = f.read()
    except OSError:
        return None, None
    peers = re.findall(r'Peer address="([0-9.]+)"', text)
    own = re.findall(r'NetworkInterface address="([0-9.]+)"', text)
    return (peers[0] if peers else None), (own[0] if own else None)


def my_ips():
    return subprocess.run(['hostname', '-I'], capture_output=True, text=True).stdout.split()


def on_pi(pi, command, timeout=40):
    return subprocess.run(['ssh', '-n', '-i', os.path.join(HOME, '.ssh', 'robot_key'), '-o', 'BatchMode=yes', '-o',
                           'ConnectTimeout=8', f'ubuntu@{pi}', command], capture_output=True, text=True, timeout=timeout)


# ------------------------------------------------------------------ clocks
NTP_CODE = r'''
import socket, struct, time
for host in ('pool.ntp.org', 'time.windows.com', 'time.google.com'):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(3)
        t0 = time.time(); s.sendto(b'\x1b' + 47 * b'\0', (host, 123)); d = s.recv(48); t3 = time.time()
        f = lambda o: struct.unpack('!I', d[o:o+4])[0] - 2208988800 + struct.unpack('!I', d[o+4:o+8])[0] / 2**32
        print(((f(32) - t0) + (f(40) - t3)) / 2); break
    except Exception:
        pass
'''


def clock_offset(pi=None):
    """Seconds to ADD to that computer's clock to get true time; None if no time server answered."""
    try:
        if pi is None:
            out = subprocess.run([sys.executable, '-c', NTP_CODE], capture_output=True, text=True, timeout=20).stdout
        else:
            out = subprocess.run(['ssh', '-i', os.path.join(HOME, '.ssh', 'robot_key'), '-o', 'BatchMode=yes', '-o',
                                  'ConnectTimeout=8', f'ubuntu@{pi}', 'python3 -'], input=NTP_CODE, capture_output=True, text=True,
                                 timeout=30).stdout
        return float(out.split()[0])
    except Exception:                          # noqa: BLE001
        return None


# ------------------------------------------------------------------ steps
def setup_script():
    """laptop_setup.sh of the workspace whose controllers are INSTALLED (it writes that workspace into ~/robot_env.sh)."""
    try:
        from ament_index_python.packages import get_package_prefix
        workspace = os.path.dirname(os.path.dirname(get_package_prefix('lab3_controllers')))
        candidate = os.path.join(workspace, 'src', 'lab3_controllers', 'tools', 'laptop_setup.sh')
        if os.path.exists(candidate):
            return candidate
    except Exception:                          # noqa: BLE001
        pass
    return os.path.join(HERE, 'laptop_setup.sh')


def step_network():
    pi, laptop = addresses()
    say(f'    configured now: Pi {pi}, laptop {laptop}. This laptop has: {", ".join(my_ips())}')
    say('    Network: a private hotspot (keep its settings screen open on the phone); laptop on it, VPN OFF.')
    say('    The Pi address: on the Pi "hostname -I", or ask Claude to find it on the hotspot.')
    new = ask(f'    Pi address [Enter = keep {pi}]: ') or pi
    mine = next((ip for ip in my_ips() if ip.rsplit('.', 1)[0] == (new or '').rsplit('.', 1)[0]), None)
    if mine is None:
        say(f'    PROBLEM: this laptop has no address in the same network as {new}. Join the hotspot, then repeat this step.')
        return False
    test = on_pi(new, 'hostname')
    if test.returncode != 0:
        say('    PROBLEM: cannot log in to the Pi with the key. ' + test.stderr.strip()[-200:])
        say(f'    If the Pi is new to this key:  ssh-copy-id -i ~/.ssh/robot_key.pub ubuntu@{new}')
        return False
    say(f'    logged in to "{test.stdout.strip()}"')
    if new != pi or mine != laptop:
        say('    addresses changed: stopping old programs and writing the new settings on both computers')
        robot('stop')
        sh(['bash', setup_script(), new, '12'])
        r = on_pi(new, f'cd ~/Mobile-Robot && bash src/lab3_controllers/tools/pi_setup.sh {mine} 12', timeout=90)
        say(r.stdout.strip())
        if r.returncode != 0:
            say(r.stderr.strip()[-300:])
            return False
        say('    NOTE: a new terminal is needed only if you type ROS commands yourself; this guide does not need it.')
    return True


def step_clocks():
    pi, _ = addresses()
    laptop, remote = clock_offset(), clock_offset(pi)
    if laptop is None or remote is None:
        say(f'    could not measure (laptop {laptop}, Pi {remote}): no time server reachable. The LiDAR map needs the two clocks'
            ' within 0.3 s; if /robot_pose is missing later, come back to this step.')
        return False
    say(f'    true time minus clock:  laptop {laptop:+.3f} s,  Pi {remote:+.3f} s   (needed: both within 0.15 s of each other)')
    if abs(laptop - remote) <= 0.15:
        say('    clocks are fine')
        return True
    if ask('    set the laptop (WSL) clock now? It asks for your WSL password. [Y/n] ').lower() != 'n':
        wanted = clock_offset()
        if wanted is not None:
            sh(['sudo', 'date', '-s', f'@{time.time() + wanted:.3f}'])
        laptop = clock_offset()
        say(f'    laptop now {laptop:+.3f} s, Pi {remote:+.3f} s')
    ok = laptop is not None and abs(laptop - remote) <= 0.3
    if not ok:
        say('    still apart. If the Pi is the wrong one:  ssh to it and run  sudo timedatectl set-ntp true')
    return ok


def step_hardware():
    say('    Pi on a charger that gives 5 V 3 A (it dropped out on a weak one). Arduino USB in the Pi. LiDAR (LD19) USB in the')
    say('    Pi, mounted facing forward. Motor battery charged and ON only when a test is about to move.')
    if ask('    all connected? [Y/n] ').lower() == 'n':
        return False
    pi, _ = addresses()
    r = on_pi(pi, 'ls /dev/ttyACM0 /dev/ttyUSB0 2>/dev/null;for f in /sys/class/hwmon/hwmon*; do [ "$(cat $f/name)" = rpi_volt ] && '
                  'echo lowvoltage=$(cat $f/in0_lcrit_alarm); done; '
                  'echo detections=$(journalctl -k -b --no-pager | grep -ci "undervoltage detected")')
    say('    ' + r.stdout.strip().replace('\n', '\n    '))
    ok = '/dev/ttyACM0' in r.stdout and 'lowvoltage=1' not in r.stdout
    if '/dev/ttyUSB0' not in r.stdout.split('\n'):
        say('    NOTE: no LiDAR on /dev/ttyUSB0. Wheel-odometry tests still run; the LiDAR check and "fused" paths need it.')
    say('    restarting the Pi driver so that it sees what is plugged in now')
    robot('stop')
    return ok and robot('start') == 0


def step_status():
    return robot('status') == 0


def step_lidar():
    return robot('lidar') == 0


def step_pi2():
    say('    6 turns to pi/2 from different angles (+6 set-up turns), about 6 minutes. Motor battery ON, 50 cm free around.')
    return robot('pi2') == 0


def step_ab():
    say('    The settings that passed 12/12 on the robot (v1) against the new direction-aware ones (v2): twice each, ~25 min.')
    a = robot('pi2', '--repeat', '2', '--profile', 'v1', '--note', 'A v1')
    return a == 0 and robot('pi2', '--repeat', '2', '--note', 'B v2') == 0


def step_compare_plain():
    say('    The same pi/2 test with the plain P controller beside p_controller (about 11 minutes).')
    return robot('pi2', '--compare') == 0


def step_identify():
    say('    Measures how far short pulses really move the robot, turning AND driving (about 4 minutes).')
    say('    This is the measurement the driving settings still need. 50 cm free all around.')
    return robot('identify') == 0


def step_fit():
    say('    No movement. Fits the robot model to every identification so far, re-tunes the path controllers on it')
    say('    (about 6 minutes); the next tests use the result. The previous settings are kept with a time stamp.')
    stamp = datetime.datetime.now().strftime('%m%d-%H%M')
    keep = os.path.join(book(), 'settings_history')
    os.makedirs(keep, exist_ok=True)
    for name in ('robot_model.json', 'path_robot.json'):
        source = os.path.join(HERE, '..', 'config', name)
        if os.path.exists(source):
            shutil.copy(source, os.path.join(keep, f'{stamp}_before_{name}'))
    if sh([sys.executable, os.path.join(HERE, 'robot_model.py'), 'fit', os.path.join(book(), 'identify')]) != 0:
        return False
    if sh([sys.executable, os.path.join(HERE, 'path_tune.py'), '--report', os.path.join(book(), f'{stamp}_path_tuning.json')]) != 0:
        return False
    for name in ('robot_model.json', 'path_robot.json'):
        shutil.copy(os.path.join(HERE, '..', 'config', name), os.path.join(keep, f'{stamp}_after_{name}'))
    return True


def step_drive():
    say('    Straight drives forward and back with p_controller. 70 cm free ahead, 30 cm around.')
    code = 0
    for distance in ('0.2', '-0.2', '0.4', '-0.4'):
        code |= robot('drive', distance, '--yes') if ask(f'    drive {distance} m now? [Y/n] ').lower() != 'n' else 0
    return code == 0


def step_paths_core():
    say('    SQUARE (0.5 m) with every path controller. Tape or a tile corner as a start mark helps: put the robot back each')
    say('    time. Needs about 1.0 m x 1.0 m (ahead and to the left).')
    return robot('path', 'series', '--shapes', 'square', '--controllers', 'stop_turn,polar,pure_pursuit,p_servo',
                 '--state', 'odom') == 0


def step_paths_circle():
    say('    The 1 m CIRCLE (the lab note task) with every controller, including the symbolic one.')
    say('    Needs 1.0 m ahead/behind-to-ahead and 1.5 m to the LEFT. The symbolic one may take more than one lap.')
    return robot('path', 'series', '--shapes', 'circle_r50', '--controllers', 'pure_pursuit,stop_turn,polar,symbolic,p_servo',
                 '--state', 'odom') == 0


def step_paths_lidar():
    say('    The same two paths steering by the LiDAR-corrected position ("fused"). Run step "LiDAR check" first.')
    return robot('path', 'series', '--shapes', 'square,circle_r50', '--controllers', 'stop_turn,pure_pursuit,symbolic',
                 '--state', 'fused') == 0


def step_paths_more():
    robot('path', 'shapes')
    names = ask('    which shapes? (comma separated, Enter = triangle,eight,rectangle) ') or 'triangle,eight,rectangle'
    controllers = ask('    which controllers? (Enter = stop_turn,pure_pursuit) ') or 'stop_turn,pure_pursuit'
    return robot('path', 'series', '--shapes', names, '--controllers', controllers) == 0


def step_results():
    robot('results')
    target = os.path.join(book(), '..', 'logbook_backup_' + datetime.datetime.now().strftime('%m%d-%H%M'))
    shutil.make_archive(target, 'zip', book())
    say(f'    backup written: {os.path.abspath(target)}.zip')
    return True


def step_stop():
    say('    Switch the motor battery OFF.')
    return robot('stop') == 0


STEPS = [
    ('Connect: hotspot, Pi address, settings on both computers', step_network),
    ('Hardware checklist, Pi power, restart the driver', step_hardware),
    ('Clocks of the laptop and the Pi (the LiDAR map needs them)', step_clocks),
    ('Status: Arduino, driver, exactly one robot  (no movement)', step_status),
    ('LiDAR check: scan, map position, delay  (no movement)', step_lidar),
    ('Heading pi/2 test (the main result)', step_pi2),
    ('A/B: the settings proven on the robot against the new ones (optional)', step_ab),
    ('pi/2 against the plain P controller (optional)', step_compare_plain),
    ('Identify turning and DRIVING (needed before drives and paths)', step_identify),
    ('Fit the model and re-tune the path controllers  (no movement)', step_fit),
    ('Straight drives', step_drive),
    ('Paths on wheel odometry: square, every controller', step_paths_core),
    ('Paths on wheel odometry: 1 m circle, incl. symbolic', step_paths_circle),
    ('Paths steering with the LiDAR map', step_paths_lidar),
    ('More shapes (your choice)', step_paths_more),
    ('Results: report with plots, backup', step_results),
    ('Stop everything', step_stop),
]


def main():
    os.makedirs(book(), exist_ok=True)
    state_file = os.path.join(book(), 'guide_state.json')
    try:
        with open(state_file) as f:
            state = json.load(f)
    except (OSError, ValueError):
        state = {}
    i = next((k for k in range(len(STEPS)) if state.get(str(k)) != 'done'), 0)
    if state and ask(f'Continue at step {i + 1}? [Y = yes, r = start again] ').lower() == 'r':
        state, i = {}, 0
    while 0 <= i < len(STEPS):
        say('\n' + '=' * 78)
        for k, (title, _) in enumerate(STEPS):
            mark = {'done': 'x', 'problem': '!', 'skipped': '-'}.get(state.get(str(k)), ' ')
            say(f'  {">" if k == i else " "} [{mark}] {k + 1:2d}. {title}')
        title, action = STEPS[i]
        say(f'\nSTEP {i + 1}: {title}')
        answer = ask('Enter = do it, s = skip, b = back, number = jump, q = quit: ').lower()
        if answer == 'q':
            break
        if answer == 's':
            state[str(i)] = 'skipped'
            i += 1
        elif answer == 'b':
            i = max(0, i - 1)
            continue
        elif answer.isdigit() and 1 <= int(answer) <= len(STEPS):
            i = int(answer) - 1
            continue
        else:
            try:
                ok = action()
            except KeyboardInterrupt:
                say('\n    interrupted')
                ok = False
            except Exception as e:             # noqa: BLE001  (a failed step must not end the guide)
                say(f'    this step failed: {type(e).__name__}: {e}')
                ok = False
            state[str(i)] = 'done' if ok else 'problem'
            say(f'\nstep {i + 1}: {"OK" if ok else "NOT OK - read the lines above (copy them to Claude if unclear)"}')
            if ok:
                i += 1
            else:
                choice = ask('Enter = try this step again, s = go on, q = quit: ').lower()
                if choice == 'q':
                    break
                if choice == 's':
                    i += 1
        with open(state_file, 'w') as f:
            json.dump(state, f)
    with open(state_file, 'w') as f:
        json.dump(state, f)
    say(f'\nProgress saved. Continue any time with:  bash ~/robot.sh guide     Logbook: {book()}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
