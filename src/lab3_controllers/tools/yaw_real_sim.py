#!/usr/bin/env python3
"""Heading control under the REAL conditions of the professor's robot (no ROS).

    robot         identified velocity model behind the command filter (deadband 0.2 rad/s, diamond), tools/plant_models.py
    pose source   the map pose /robot_pose: low jittery rate, latency, noise, and FREEZES (no data for a few seconds, periodically)
    controller    the reference controller exactly as the node runs it: a 20 Hz timer that reuses the latest pose, a stale stop
                  when the data is older than 0.5 s (zero command, no controller step), acceleration limit 1.5 rad/s^2

The node replica is `NodeLoop`; `run_case` drives one turn and reports the final error, the settling time and the overshoot.
Usage:  python3 tools/yaw_real_sim.py [--cases 40]
"""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import plant_models as pm  # noqa: E402
from lab3_controllers.common import slew, wrap_angle  # noqa: E402
from lab3_controllers.tracker import YawTracker  # noqa: E402

DT = 0.05            # world step and node timer (20 Hz)


class PoseSource:
    """Heading samples as the node receives them from /robot_pose."""

    def __init__(self, rate=12.0, latency=0.15, noise_deg=0.5, freeze_period=35.0, freeze_len=3.0, phase=0.0, seed=0):
        self.rng = random.Random(seed)
        self.rate, self.latency, self.noise = rate, latency, math.radians(noise_deg)
        self.period, self.flen, self.phase = freeze_period, freeze_len, phase
        self.next_t = 0.0
        self.pending = []                    # (arrival time, measured yaw)

    def update(self, t, true_yaw):
        while self.next_t <= t:
            ts = self.next_t
            self.next_t += (1.0 / self.rate) * self.rng.uniform(0.7, 1.3)
            if self.period > 0 and ((ts - self.phase) % self.period) < self.flen:
                continue                     # frozen: this sample never arrives
            self.pending.append((ts + self.latency, wrap_angle(true_yaw + self.rng.gauss(0.0, self.noise)), ts))

    def arrivals(self, t):
        out = [p for p in self.pending if p[0] <= t]
        self.pending = [p for p in self.pending if p[0] > t]
        return out


class NodeLoop:
    """What reference_controller.control_step does for a heading reference. timed=True passes the sample stamp and the
    age of the sample to the controller (the node does this with use_state_stamps: true)."""

    def __init__(self, params, state_timeout=0.5, accel=1.5, tracker=None, timed=False):
        self.tracker = tracker or YawTracker(params)
        self.timeout, self.accel, self.timed = state_timeout, accel, timed
        self.yaw = None
        self.stamp = None
        self.last_arrival = None
        self.w_cmd = 0.0

    def tick(self, t, arrivals, ref):
        for _, y, ts in arrivals:
            self.yaw, self.last_arrival, self.stamp = y, t, ts
        if self.yaw is None:
            self.w_cmd = slew(self.w_cmd, 0.0, self.accel, DT)
            return self.w_cmd
        if t - self.last_arrival > self.timeout:           # stale: immediate stop, no controller step
            self.w_cmd = 0.0
            if self.timed:
                self.tracker.observe_command(0.0)          # the model must know the robot was told to stop
            return 0.0
        if self.timed:
            w = self.tracker.step(wrap_angle(self.yaw), ref, DT, stamp=self.stamp, age=t - self.last_arrival, now=t)
        else:
            w = self.tracker.step(wrap_angle(self.yaw), ref, DT)
        self.w_cmd = slew(self.w_cmd, w, self.accel, DT)
        if self.timed:
            self.tracker.observe_command(self.w_cmd)           # the model needs the command that was really sent
        return self.w_cmd


def run_case(params, y0_deg, ref_deg, source_kw, hold=30.0, loop_factory=None, plant_gain=1.0, jumps=()):
    """jumps: [(time s, degrees)]: SLAM corrects the map heading, i.e. the (map) heading changes instantly."""
    plant = pm.IdentifiedPlant(use_filter=True, gain=(1.0, plant_gain))
    src = PoseSource(**source_kw)
    timed = bool(params.get('_timed', False))
    loop = (loop_factory or (lambda p: NodeLoop({k: v for k, v in p.items() if k != '_timed'}, timed=timed)))(params)
    yaw, ref = math.radians(y0_deg), math.radians(ref_deg)
    n = int(hold / DT)
    pending = sorted(jumps)
    errs = []
    for k in range(n):
        t = k * DT
        while pending and pending[0][0] <= t:
            yaw = wrap_angle(yaw + math.radians(pending.pop(0)[1]))
        src.update(t, yaw)
        w = loop.tick(t, src.arrivals(t), ref)
        _, ww = plant.step(0.0, w)
        yaw = wrap_angle(yaw + ww * DT)
        errs.append(math.degrees(wrap_angle(ref - yaw)))
    settle = next((i * DT for i in range(n - 1, -1, -1) if abs(errs[i]) > 1.0), -DT) + DT
    first = errs[0]
    direction = 1.0 if first > 0 else -1.0
    over = max(0.0, max(-direction * e for e in errs))
    return dict(final=errs[-1], settle=settle, over=over, worst_late=max(abs(e) for e in errs[int(n * 0.6):]))


def make_cases(n, seed=1, stress=False):
    """Random but reproducible turn problems and pose-source conditions (the unknowns of the real network).
    stress=True: the harsher conditions suggested by the real run (slow source, long freezes, +-20 % robot gain, SLAM jumps)."""
    rng = random.Random(seed)
    cases = []
    for i in range(n):
        y0 = rng.uniform(-180, 180)
        turn = rng.choice([-1, 1]) * rng.uniform(30, 150)
        ref = (y0 + turn + 180) % 360 - 180
        if stress:
            src = dict(rate=rng.uniform(3.0, 15.0), latency=rng.uniform(0.05, 0.60), noise_deg=rng.uniform(0.2, 1.5),
                       freeze_period=rng.choice([20.0, 30.0, 35.0, 40.0]), freeze_len=rng.uniform(2.0, 6.0),
                       phase=rng.uniform(0, 35), seed=rng.randrange(10 ** 6))
            jumps = [(rng.uniform(4.0, 18.0), rng.choice([-1, 1]) * rng.uniform(2.0, 6.0)) for _ in range(rng.choice([0, 1, 2]))]
            gain = rng.uniform(0.8, 1.2)
        else:
            src = dict(rate=rng.uniform(8.0, 25.0), latency=rng.uniform(0.05, 0.30), noise_deg=rng.uniform(0.2, 1.0),
                       freeze_period=rng.choice([0.0, 35.0, 35.0, 20.0]), freeze_len=rng.uniform(2.0, 4.0),
                       phase=rng.uniform(0, 35), seed=rng.randrange(10 ** 6))
            jumps, gain = [], 1.0
        cases.append(dict(y0=y0, ref=ref, source=src, jumps=jumps, plant_gain=gain))
    return cases


def evaluate(params, cases, **kw):
    res = [run_case(params, c['y0'], c['ref'], c['source'], jumps=c['jumps'], plant_gain=c['plant_gain'], **kw) for c in cases]
    finals = sorted(abs(r['final']) for r in res)
    return dict(
        mean_abs=sum(finals) / len(finals), median=finals[len(finals) // 2], p90=finals[int(len(finals) * 0.9)],
        worst=finals[-1], within1=sum(f <= 1.0 for f in finals) / len(finals),
        mean_over=sum(r['over'] for r in res) / len(res),
        mean_settle=sum(r['settle'] for r in res) / len(res))


def describe(name, s):
    print(f'{name:<44} final |error|: mean {s["mean_abs"]:5.2f}  median {s["median"]:5.2f}  p90 {s["p90"]:5.2f}  worst {s["worst"]:6.2f} deg'
          f' | within 1 deg {s["within1"] * 100:4.0f} % | overshoot {s["mean_over"]:5.2f} deg | settle {s["mean_settle"]:5.1f} s')


PRESET = dict(kp=1.0, tolerance=0.01, max_angular_speed=1.0, diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2,
              lead_w=0.8, vel_filter=0.7, vel_window=5, rest_window=1.0)


TIMED = dict(PRESET, _timed=True, pose_latency=0.2, vel_span=1.0, vel_filter=0.5, rest_mean_test=True, rest_yaw=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cases', type=int, default=40)
    a = ap.parse_args()
    for label, stress in (('NORMAL', False), ('STRESS', True)):
        cases = make_cases(a.cases, stress=stress)
        print(f'=== {label} conditions ({a.cases} cases)')
        describe('plain P law (no compensation)', evaluate(dict(kp=1.0, tolerance=0.01, **{k: PRESET[k] for k in ('diamond_v', 'diamond_w', 'deadband_v', 'deadband_w')}), cases))
        describe('preset used in the run (reference_lab_robot)', evaluate(PRESET, cases))
        describe('preset + timed mode (stamps, latency, mean rest test)', evaluate(TIMED, cases))


if __name__ == '__main__':
    main()
