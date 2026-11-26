#!/usr/bin/env python3
"""Test bench for controllers that must work on a robot we have NOT measured (no ROS).

Every case draws a different robot and a different network, none of which the controller is told about:
    robot    command deadband 0-0.30 rad/s (0-0.06 m/s), gain +-30 %, lag 0.15-1.0 s, command delay 0.03-0.35 s
             plant='identified': instead, the professor's 4th-order identified model (a different STRUCTURE, used only for judging)
    odometry (wheel encoders) 20-40 Hz, 0.02-0.15 s late, noisy speed, slow drift (wheel scale error)
    map pose (/robot_pose, SLAM) 8-30 Hz, 0.05-0.4 s late, noisy, SLAM corrections that jump
    network  freezes of 0.3-4 s that stop both streams
The robot is judged in the MAP frame, as the professor does.
"""
import math
import os
import random
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import plant_models as pm  # noqa: E402
from lab3_controllers.common import slew, wrap_angle  # noqa: E402
from lab3_controllers.servo import AxisServo, MapOdomFusion, PoseServo, compose, offset_between  # noqa: E402
from lab3_controllers.tracker import YawTracker  # noqa: E402

DT = 0.05            # controller period (20 Hz)
SUB = 5              # plant sub-steps per controller period


class Robot:
    """True robot in the map frame."""

    def __init__(self, c, pose=(0.0, 0.0, 0.0)):
        self.c = c
        self.x, self.y, self.th = pose
        self.v = self.w = 0.0
        self.queue = deque([(0.0, 0.0)] * max(0, int(round(c['cmd_delay'] / DT))))
        self.ident = pm.IdentifiedPlant(use_filter=False, gain=(c['gv'], c['gw'])) if c['plant'] == 'identified' else None

    def step(self, v_cmd, w_cmd):
        """One controller period with the command held."""
        self.queue.append((v_cmd, w_cmd))
        v_cmd, w_cmd = self.queue.popleft()
        c = self.c
        if abs(v_cmd) <= c['db_v'] and abs(w_cmd) <= c['db_w']:      # the robot ignores small commands
            v_cmd = w_cmd = 0.0
        if self.ident:
            v_new, w_new = self.ident.step(v_cmd, w_cmd)
        moved = []
        h = DT / SUB
        for _ in range(SUB):
            if self.ident:
                self.v, self.w = v_new, w_new
            else:
                self.v += (c['gv'] * v_cmd - self.v) * (1.0 - math.exp(-h / c['tau_v']))
                self.w += (c['gw'] * w_cmd - self.w) * (1.0 - math.exp(-h / c['tau_w']))
            self.x += self.v * math.cos(self.th) * h
            self.y += self.v * math.sin(self.th) * h
            self.th = wrap_angle(self.th + self.w * h)
            moved.append((self.v * h, self.w * h))
        return moved


class Sensors:
    """The two pose streams as they arrive at the laptop."""

    def __init__(self, c, robot):
        self.c, self.r = c, robot
        self.rng = random.Random(c['seed'])
        # the wheel-odometry frame starts somewhere else and drifts through the wheel scale errors
        self.odom = [self.rng.uniform(-1, 1), self.rng.uniform(-1, 1), self.rng.uniform(-math.pi, math.pi)]
        self.bias = [0.0, 0.0, 0.0]          # SLAM estimate = truth + bias (changes when SLAM corrects)
        self.next_odom = self.next_map = 0.0
        self.q_odom, self.q_map = [], []
        self.jumps = sorted(c['jumps'])

    def move(self, moved):
        for dv, dw in moved:
            dw_o = dw * self.c['odom_scale_w']
            self.odom[2] = wrap_angle(self.odom[2] + dw_o)
            self.odom[0] += dv * self.c['odom_scale_v'] * math.cos(self.odom[2])
            self.odom[1] += dv * self.c['odom_scale_v'] * math.sin(self.odom[2])

    def truth(self):
        """The pose in the professor's frame: what SLAM says without noise."""
        return (self.r.x + self.bias[0], self.r.y + self.bias[1], wrap_angle(self.r.th + self.bias[2]))

    def frozen(self, t):
        c = self.c
        return c['freeze_period'] > 0 and ((t - c['phase']) % c['freeze_period']) < c['freeze_len']

    def update(self, t):
        c, rng = self.c, self.rng
        while self.jumps and self.jumps[0][0] <= t:
            _, dx, dy, dth = self.jumps.pop(0)
            self.bias = [self.bias[0] + dx, self.bias[1] + dy, self.bias[2] + dth]
        while self.next_odom <= t:
            ts = self.next_odom
            self.next_odom += (1.0 / c['odom_rate']) * rng.uniform(0.85, 1.15)
            if not self.frozen(ts):
                pose = (self.odom[0], self.odom[1], wrap_angle(self.odom[2] + rng.gauss(0, math.radians(0.05))))
                self.q_odom.append((ts + c['odom_delay'], ts, pose, self.r.v * c['odom_scale_v'] + rng.gauss(0, c['noise_v']),
                                    self.r.w * c['odom_scale_w'] + rng.gauss(0, c['noise_w'])))
        while self.next_map <= t:
            ts = self.next_map
            self.next_map += (1.0 / c['map_rate']) * rng.uniform(0.7, 1.3)
            if not self.frozen(ts):
                x, y, th = self.truth()
                pose = (x + rng.gauss(0, c['map_noise_xy']), y + rng.gauss(0, c['map_noise_xy']),
                        wrap_angle(th + rng.gauss(0, c['map_noise_th'])))
                self.q_map.append((ts + c['map_delay'], ts, pose))

    @staticmethod
    def _pop(queue, t):
        out = [m for m in queue if m[0] <= t]
        queue[:] = [m for m in queue if m[0] > t]
        return out

    def arrivals(self, t):
        return self._pop(self.q_odom, t), self._pop(self.q_map, t)


# ------------------------------------------------------------------ controller adapters (what the ROS node would do)
class ServoHeading:
    """The new design: fused odometry + map, AxisServo on the measured turn rate."""

    def __init__(self, params, use_odom=True):
        self.servo = AxisServo(params)
        self.fusion = MapOdomFusion()
        self.use_odom = use_odom
        self.last = None
        self.map_pose = self.map_prev = None
        self.w_map = 0.0

    def tick(self, t, odom, maps, ref):
        for _, ts, pose, v, w in odom:
            self.fusion.add_odom(ts, pose, v, w)
            self.last = t
        for _, ts, pose in maps:
            if self.use_odom:
                self.fusion.add_map(ts, pose)
            else:                                        # ablation: the map pose only, speed from differences
                if self.map_pose is not None and ts > self.map_pose[0]:
                    raw = wrap_angle(pose[2] - self.map_pose[1][2]) / (ts - self.map_pose[0])
                    self.w_map += 0.4 * (raw - self.w_map)
                self.map_pose = (ts, pose)
                self.last = t
        ready = self.fusion.ready() if self.use_odom else self.map_pose is not None
        if not ready:
            return 0.0, 0.0
        if self.last is None or t - self.last > 0.5:
            self.servo.pause()
            return 0.0, 0.0
        pose, _, w = self.fusion.state() if self.use_odom else (self.map_pose[1], 0.0, self.w_map)
        return 0.0, self.servo.update(wrap_angle(ref - pose[2]), w, DT)


class OldPresetHeading:
    """The previously shipped design (reference_lab_robot_tuned: observer on the map pose only), as the node runs it."""

    def __init__(self, params):
        self.tr = YawTracker(params)
        self.yaw = self.stamp = self.last = None
        self.cmd = 0.0

    def tick(self, t, odom, maps, ref):
        for _, ts, pose in maps:
            self.yaw, self.stamp, self.last = pose[2], ts, t
        if self.yaw is None:
            return 0.0, 0.0
        if t - self.last > 0.5:
            self.cmd = 0.0
            self.tr.observe_command(0.0)
            return 0.0, 0.0
        w = self.tr.step(self.yaw, ref, DT, stamp=self.stamp, age=t - self.last, now=t)
        self.cmd = slew(self.cmd, w, 1.5, DT)
        self.tr.observe_command(self.cmd)
        return 0.0, self.cmd


class ServoPose:
    """Go to (x, y, theta) with the PoseServo on the fused state."""

    def __init__(self, heading, linear=None, **kw):
        self.servo = PoseServo(heading, linear, **kw)
        self.fusion = MapOdomFusion()
        self.last = None
        self.sent = False

    def tick(self, t, odom, maps, ref):
        for _, ts, pose, v, w in odom:
            self.fusion.add_odom(ts, pose, v, w)
            self.last = t
        for _, ts, pose in maps:
            self.fusion.add_map(ts, pose)
        if not self.fusion.ready():
            return 0.0, 0.0
        if t - self.last > 0.5:
            self.servo.heading.pause()
            self.servo.linear.pause()
            return 0.0, 0.0
        if not self.sent:
            self.servo.set_target(ref)
            self.sent = True
        pose, v, w = self.fusion.state()
        return self.servo.update(pose, v, w, DT)


class ServoDistance(ServoPose):
    """/reference_x as the node runs it: drive `ref` metres along the present heading."""

    def tick(self, t, odom, maps, ref):
        for _, ts, pose, v, w in odom:
            self.fusion.add_odom(ts, pose, v, w)
            self.last = t
        for _, ts, pose in maps:
            self.fusion.add_map(ts, pose)
        if not self.fusion.ready():
            return 0.0, 0.0
        if t - self.last > 0.5:
            self.servo.heading.pause()
            self.servo.linear.pause()
            return 0.0, 0.0
        pose, v, w = self.fusion.state()
        if not self.sent:
            self.servo.set_distance(pose, ref)
            self.sent = True
        return self.servo.update(pose, v, w, DT)


# ------------------------------------------------------------------ cases and runs
def make_case(rng, plant='random'):
    case = dict(
        plant=plant, seed=rng.randrange(10 ** 6),
        db_w=rng.uniform(0.0, 0.30), db_v=rng.uniform(0.0, 0.06), gw=rng.uniform(0.7, 1.3), gv=rng.uniform(0.75, 1.25),
        tau_w=rng.uniform(0.15, 1.0), tau_v=rng.uniform(0.15, 0.8), cmd_delay=rng.uniform(0.03, 0.35),
        odom_rate=rng.uniform(20.0, 40.0), odom_delay=rng.uniform(0.02, 0.15), noise_w=rng.uniform(0.005, 0.04),
        noise_v=rng.uniform(0.002, 0.01), odom_scale_w=rng.uniform(0.96, 1.04), odom_scale_v=rng.uniform(0.97, 1.03),
        map_rate=rng.uniform(8.0, 30.0), map_delay=rng.uniform(0.05, 0.4), map_noise_th=math.radians(rng.uniform(0.2, 1.0)),
        map_noise_xy=rng.uniform(0.002, 0.012), freeze_period=rng.choice([0.0, 20.0, 35.0, 35.0]), freeze_len=rng.uniform(0.3, 4.0),
        phase=rng.uniform(0, 35),
        jumps=[(rng.uniform(3.0, 12.0), rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), math.radians(rng.uniform(-3.0, 3.0)))
               for _ in range(rng.choice([0, 0, 1, 2]))])
    if plant == 'identified':
        case['gw'], case['gv'] = rng.uniform(0.8, 1.2), rng.uniform(0.85, 1.15)
    return case


def make_cases(n, seed=1, plant='random'):
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        c = make_case(rng, plant)
        c['y0'] = rng.uniform(-math.pi, math.pi)
        c['turn'] = rng.choice([-1, 1]) * math.radians(rng.uniform(20, 160))
        r, a = rng.uniform(0.2, 0.7), rng.uniform(-math.pi, math.pi)
        c['goal'] = (r * math.cos(a), r * math.sin(a), rng.uniform(-math.pi, math.pi))
        out.append(c)
    return out


def run_heading(make_ctrl, c, hold=25.0, trace=False):
    robot = Robot(c, (0.0, 0.0, c['y0']))
    sens = Sensors(c, robot)
    ctrl = make_ctrl()
    ref = wrap_angle(c['y0'] + c['turn'])
    errs, cmds = [], []
    for k in range(int(hold / DT)):
        t = k * DT
        sens.update(t)
        v, w = ctrl.tick(t, *sens.arrivals(t), ref)
        sens.move(robot.step(v, w))
        errs.append(math.degrees(wrap_angle(ref - sens.truth()[2])))
        cmds.append(w)
    settle = next((i * DT for i in range(len(errs) - 1, -1, -1) if abs(errs[i]) > 2.0), -DT) + DT
    direction = 1.0 if c['turn'] > 0 else -1.0
    over = max(0.0, max(-direction * e for e in errs))
    tail = int(5.0 / DT)
    chatter = sum(1 for i in range(len(cmds) - tail, len(cmds)) if cmds[i] != 0.0 and cmds[i - 1] == 0.0)
    out = dict(final=errs[-1], settle=settle, over=over, chatter=chatter)
    if trace:
        out['errs'] = errs
    return out


def run_pose(make_ctrl, c, hold=60.0):
    robot = Robot(c, (0.0, 0.0, c['y0']))
    sens = Sensors(c, robot)
    ctrl = make_ctrl()
    gx, gy, gth = c['goal']
    done_at = None
    for k in range(int(hold / DT)):
        t = k * DT
        sens.update(t)
        v, w = ctrl.tick(t, *sens.arrivals(t), c['goal'])
        sens.move(robot.step(v, w))
        if done_at is None and getattr(getattr(ctrl, 'servo', None), 'phase', '') == 'DONE' and ctrl.sent:
            done_at = t
    x, y, th = sens.truth()
    return dict(pos=100 * math.hypot(gx - x, gy - y), head=abs(math.degrees(wrap_angle(gth - th))),
                done=done_at is not None, time=done_at if done_at is not None else hold)


def run_distance(make_ctrl, c, dist, hold=30.0):
    """Drive `dist` metres; judged in the map frame along the heading at the start. Errors in cm."""
    robot = Robot(c, (0.0, 0.0, c['y0']))
    sens = Sensors(c, robot)
    ctrl = make_ctrl()
    start, errs, cmds = None, [], []
    for k in range(int(hold / DT)):
        t = k * DT
        sens.update(t)
        v, w = ctrl.tick(t, *sens.arrivals(t), dist)
        if start is None and ctrl.sent:
            start = sens.truth()
        sens.move(robot.step(v, w))
        cmds.append(v)
        if start is not None:
            x, y, _ = sens.truth()
            along = (x - start[0]) * math.cos(start[2]) + (y - start[1]) * math.sin(start[2])
            side = -(x - start[0]) * math.sin(start[2]) + (y - start[1]) * math.cos(start[2])
            errs.append((100 * (dist - along), 100 * side))
        else:
            errs.append((100 * dist, 0.0))
    settle = next((i * DT for i in range(len(errs) - 1, -1, -1) if abs(errs[i][0]) > 3.0), -DT) + DT
    over = max(0.0, max(-math.copysign(1.0, dist) * e[0] for e in errs))
    tail = int(5.0 / DT)
    chatter = sum(1 for i in range(len(cmds) - tail, len(cmds)) if cmds[i] != 0.0 and cmds[i - 1] == 0.0)
    return dict(final=errs[-1][0], side=abs(errs[-1][1]), settle=settle, over=over, chatter=chatter)


def distances(n, seed=9):
    rng = random.Random(seed)
    return [rng.choice([-1, 1]) * rng.uniform(0.15, 0.8) for _ in range(n)]


def evaluate_distance(make_ctrl, cases):
    res = [run_distance(make_ctrl, c, d) for c, d in zip(cases, distances(len(cases)))]
    s = summarize([abs(r['final']) for r in res])
    s.update(within2=sum(abs(r['final']) <= 3.0 for r in res) / len(res), over=sum(r['over'] for r in res) / len(res),
             settle=sum(r['settle'] for r in res) / len(res), chatter=sum(r['chatter'] for r in res) / len(res),
             side=sum(r['side'] for r in res) / len(res))
    return s


def summarize(values):
    v = sorted(values)
    n = len(v)
    mean = sum(v) / n
    se = (sum((x - mean) ** 2 for x in v) / max(n - 1, 1)) ** 0.5 / n ** 0.5
    return dict(mean=mean, se=se, median=v[n // 2], p90=v[int(n * 0.9)], p95=v[min(n - 1, int(n * 0.95))], worst=v[-1])


def evaluate_heading(make_ctrl, cases):
    res = [run_heading(make_ctrl, c) for c in cases]
    s = summarize([abs(r['final']) for r in res])
    s.update(within2=sum(abs(r['final']) <= 2.0 for r in res) / len(res), over=sum(r['over'] for r in res) / len(res),
             settle=sum(r['settle'] for r in res) / len(res), chatter=sum(r['chatter'] for r in res) / len(res))
    return s


def evaluate_pose(make_ctrl, cases):
    res = [run_pose(make_ctrl, c) for c in cases]
    s = summarize([r['pos'] for r in res])
    h = summarize([r['head'] for r in res])
    s.update(head_mean=h['mean'], head_p95=h['p95'], done=sum(r['done'] for r in res) / len(res), time=sum(r['time'] for r in res) / len(res),
             ok=sum(r['pos'] <= 4.0 and r['head'] <= 4.0 for r in res) / len(res))
    return s


def describe(name, s):
    print(f'{name:<34} |error| mean {s["mean"]:5.2f} +-{s["se"]:.2f}  median {s["median"]:5.2f}  p95 {s["p95"]:6.2f}  worst {s["worst"]:6.1f} deg'
          f' | within 2 deg {s["within2"] * 100:3.0f}% | overshoot {s["over"]:5.1f} | settle {s["settle"]:4.1f} s')
