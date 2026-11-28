#!/usr/bin/env python3
"""A model of OUR robot at the level of its two wheels, and the fit of its numbers from robot_identify.py measurements.

    python3 robot_model.py                 show the present model
    python3 robot_model.py fit             fit it to every pulse file in $ROBOT_LOGBOOK/identify and save it

Each wheel (the Arduino runs a speed loop per wheel) behaves like this, with s = the commanded wheel speed (m/s):
    * a wheel that starts from rest (or reverses) does nothing for   dead_fix + dead_len / |s|   seconds
    * ... plus dead_same seconds when it starts the same way it last turned
    * |s| <= deadband is ignored
    * then it follows gain * s with a short first-order lag tau
Turning and driving give separate (dead_fix, dead_len, gain), because the robot is loaded differently.

WHAT IS MEASURED AND WHAT IS NOT is recorded in the model file ('measured'): turning comes from 30 identification pulses and
47 correcting pulses of 2026-11-28. Driving is a PREDICTION from the same wheel numbers until a drive identification is fitted.
"""
import csv
import glob
import json
import math
import os
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_FILE = os.path.join(HERE, '..', 'config', 'robot_model.json')
SEPARATION = 0.177046                              # m, wheel separation used by the robot's own odometry

# turning: the wheel model fitted to the 29 usable identification pulses matches them with 0.64 deg rms
DEFAULT = {
    'turn': {'dead_fix': 0.0775, 'dead_len': 0.00825, 'gain': 1.04},
    'drive': {'dead_fix': 0.0775, 'dead_len': 0.00825, 'gain': 1.0},
    'dead_same': 0.11,
    'deadband': 0.0050,                            # m/s of wheel speed (plain P: stalled while commanding up to 0.052 rad/s)
    'tau': 0.04,
    'measured': {'turn': True, 'drive': False},
}


def load(path=MODEL_FILE):
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    return json.loads(json.dumps(DEFAULT))


def vary(model, rng, spread, drive_spread=None):
    """A neighbouring robot: every number varies by up to +-spread (the unmeasured driving numbers by drive_spread)."""
    out = json.loads(json.dumps(model))
    if drive_spread is None:
        drive_spread = spread if model['measured']['drive'] else max(spread, 0.5)

    def near(value, s):
        return value * (1.0 + rng.uniform(-s, s))
    for axis, s in (('turn', spread), ('drive', drive_spread)):
        out[axis] = {'dead_fix': near(model[axis]['dead_fix'], s), 'dead_len': near(model[axis]['dead_len'], s),
                     'gain': near(model[axis]['gain'], min(s, 0.15))}
    out['dead_same'] = near(model['dead_same'], max(spread, 0.3) if spread else 0.0)
    out['deadband'] = near(model['deadband'], spread)
    return out


class Wheel:
    def __init__(self):
        self.sign, self.on, self.moved_sign, self.engaged, self.extra, self.speed = 0, 0.0, 0, False, 0.0, 0.0


class WheelRobot:
    """x, y, th: the pose its wheel odometry reports (this model has no wheel slip). step(v, w) advances one period."""

    def __init__(self, model, pose=(0.0, 0.0, 0.0), cmd_delay=0.0, rng=None, jitter=0.0):
        self.m = model
        self.x, self.y, self.th = pose
        self.v = self.w = 0.0
        self.wheels = (Wheel(), Wheel())
        self.rng, self.jitter = rng, jitter
        self.delay = cmd_delay
        self._queue = None

    def _wheel(self, wheel, s, axis, h):
        m = self.m
        sign = (s > 0) - (s < 0)
        if abs(s) <= m['deadband']:
            sign = 0
        if sign == 0 or sign != wheel.sign:
            wheel.sign, wheel.on, wheel.engaged = sign, 0.0, False
            wheel.extra = 0.0
            if sign:
                if sign == wheel.moved_sign:
                    wheel.extra = m['dead_same']
                if self.rng and self.jitter:
                    wheel.extra += self.rng.uniform(-self.jitter, self.jitter)
        target = 0.0
        if sign:
            wheel.on += h
            if wheel.engaged or wheel.on > m[axis]['dead_fix'] + m[axis]['dead_len'] / abs(s) + wheel.extra:
                wheel.engaged, wheel.moved_sign = True, sign
                target = m[axis]['gain'] * s
        wheel.speed += (target - wheel.speed) * (1.0 - math.exp(-h / m['tau']))
        return wheel.speed

    def step(self, v_cmd, w_cmd, dt=0.05, sub=5):
        if self._queue is None:
            self._queue = deque([(0.0, 0.0)] * int(round(self.delay / dt)))
        self._queue.append((v_cmd, w_cmd))
        v_cmd, w_cmd = self._queue.popleft()
        left, right = v_cmd - w_cmd * SEPARATION / 2, v_cmd + w_cmd * SEPARATION / 2
        axis = 'drive' if abs(v_cmd) >= abs(w_cmd) * SEPARATION / 2 else 'turn'
        h = dt / sub
        for _ in range(sub):
            sl = self._wheel(self.wheels[0], left, axis, h)
            sr = self._wheel(self.wheels[1], right, axis, h)
            self.v, self.w = (sl + sr) / 2, (sr - sl) / SEPARATION
            mid = self.th + self.w * h / 2
            self.x += self.v * math.cos(mid) * h
            self.y += self.v * math.sin(mid) * h
            self.th = math.atan2(math.sin(self.th + self.w * h), math.cos(self.th + self.w * h))


def pulse_move(model, axis, u, length, previous=-1):
    """What the model says one pulse moves (deg or cm, in the commanded direction) after a pulse the other way."""
    robot = WheelRobot(model)
    sign = 1.0
    for wheel, s in zip(robot.wheels, ((-sign, sign) if axis == 'turn' else (sign, sign))):
        wheel.moved_sign = int(s * previous)
    for k in range(int(round(length / 0.01))):
        robot.step(u if axis == 'drive' else 0.0, u if axis == 'turn' else 0.0, dt=0.01, sub=1)
    for _ in range(100):
        robot.step(0.0, 0.0, dt=0.01, sub=1)
    return math.degrees(robot.th) if axis == 'turn' else 100 * robot.x


def read_pulses(folder):
    """Every identification pulse file -> [(axis, |command|, length, moved)]. The first pulse of a file is left out (its
    previous direction is unknown); the others follow a pulse the other way."""
    out = []
    for path in sorted(glob.glob(os.path.join(folder, '*_pulses.csv'))):
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
        previous_axis = None
        for r in rows:
            if r['axis'] == previous_axis:
                out.append((r['axis'], abs(float(r['command'])), float(r['real_length']), float(r['moved'])))
            previous_axis = r['axis']
    return out


def fit_axis(model, axis, pulses):
    """Least-squares grid fit of (dead_fix, dead_len, gain) of one axis. -> (numbers, rms, n) in deg or cm."""
    data = [(u, t, m) for a, u, t, m in pulses if a == axis]
    if len(data) < 8:
        raise ValueError(f'{len(data)} {axis} pulses: at least 8 are needed')
    to_wheel = SEPARATION / 2 if axis == 'turn' else 1.0           # command -> wheel speed

    def predict(a, b, g, u, t):                                    # the same law as WheelRobot, in closed form
        useful = max(0.0, t - (a + b / (u * to_wheel)))
        return math.degrees(g * u * useful) if axis == 'turn' else 100 * g * u * useful

    def error(numbers):
        a, b, g = numbers
        return sum((m - predict(a, b, g, u, t)) ** 2 for u, t, m in data)

    grid = [(a / 100, b / 2000, g / 100) for a in range(0, 31, 2) for b in range(0, 41, 2) for g in range(80, 121, 4)]
    a0, b0, g0 = min(grid, key=error)
    fine = [(max(0.0, a0 + da / 400), max(0.0, b0 + db / 8000), g0 + dg / 100)
            for da in range(-4, 5) for db in range(-4, 5) for dg in range(-3, 4)]
    best = min(fine, key=error)
    numbers = {'dead_fix': round(best[0], 4), 'dead_len': round(best[1], 5), 'gain': round(best[2], 3)}
    return numbers, math.sqrt(error(best) / len(data)), len(data)


def fit(folder, path=MODEL_FILE):
    model = load(path)
    pulses = read_pulses(folder)
    report = []
    for axis, unit in (('turn', 'deg'), ('drive', 'cm')):
        try:
            numbers, rms, n = fit_axis(model, axis, pulses)
        except ValueError as e:
            report.append(f'{axis}: not fitted ({e}); keeping {model[axis]}')
            continue
        if axis == 'turn':
            # the identification alternates direction: it measures the change-of-direction case, the model's base case
            pass
        report.append(f'{axis}: before {model[axis]}')
        model[axis] = numbers
        model['measured'][axis] = True
        report.append(f'{axis}: fitted to {n} pulses: {numbers}  (rms {rms:.2f} {unit})')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(model, f, indent=2)
        f.write('\n')
    return model, report


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['fit']:
        folder = argv[1] if len(argv) > 1 else os.path.join(os.environ.get('ROBOT_LOGBOOK') or os.path.expanduser('~/robot_logbook'),
                                                            'identify')
        _, report = fit(folder)
        print('\n'.join(report))
        print(f'saved {os.path.abspath(MODEL_FILE)}')
        return 0
    model = load()
    print(json.dumps(model, indent=2))
    for axis in ('turn', 'drive'):
        if not model['measured'][axis]:
            print(f'NOTE: {axis} is NOT measured yet: these numbers are a prediction')
    return 0


if __name__ == '__main__':
    sys.exit(main())
