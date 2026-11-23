#!/usr/bin/env python3
"""Forward position (x and x-dot) control under the REAL conditions of the professor's robot (no ROS).

    robot         identified velocity model behind the command filter (forward-speed deadband 0.04 m/s, lag), tools/plant_models.py
    pose source   the map pose: low jittery rate, latency, noise (cm) and FREEZES (no data for seconds, periodically)
    controller    exactly as the node runs it: 20 Hz timer reusing the latest sample, stale stop after 0.5 s, acceleration limit 0.4 m/s^2

Same design as tools/yaw_real_sim.py, for a straight move of the signed distance x along a line.
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
from lab3_controllers.common import slew  # noqa: E402
from lab3_controllers.tracker import LinearTracker  # noqa: E402

DT = 0.05


class PoseSource1D:
    def __init__(self, rate=12.0, latency=0.15, noise_cm=0.5, freeze_period=35.0, freeze_len=3.0, phase=0.0, seed=0):
        self.rng = random.Random(seed)
        self.rate, self.latency, self.noise = rate, latency, noise_cm / 100.0
        self.period, self.flen, self.phase = freeze_period, freeze_len, phase
        self.next_t = 0.0
        self.pending = []

    def update(self, t, true_x):
        while self.next_t <= t:
            ts = self.next_t
            self.next_t += (1.0 / self.rate) * self.rng.uniform(0.7, 1.3)
            if self.period > 0 and ((ts - self.phase) % self.period) < self.flen:
                continue
            self.pending.append((ts + self.latency, true_x + self.rng.gauss(0.0, self.noise), ts))

    def arrivals(self, t):
        out = [p for p in self.pending if p[0] <= t]
        self.pending = [p for p in self.pending if p[0] > t]
        return out


class LinearNodeLoop:
    """What the node does for a position reference (forward speed only)."""

    def __init__(self, params, state_timeout=0.5, accel=0.4, timed=True):
        self.tracker = LinearTracker(params)
        self.timeout, self.accel, self.timed = state_timeout, accel, timed
        self.x = self.stamp = self.last_arrival = None
        self.v_cmd = 0.0

    def tick(self, t, arrivals, ref):
        for _, x, ts in arrivals:
            self.x, self.last_arrival, self.stamp = x, t, ts
        if self.x is None:
            self.v_cmd = slew(self.v_cmd, 0.0, self.accel, DT)
            return self.v_cmd
        if t - self.last_arrival > self.timeout:
            self.v_cmd = 0.0
            self.tracker.observe_command(0.0)
            return 0.0
        if self.timed:
            v = self.tracker.step(self.x, ref, DT, stamp=self.stamp, age=t - self.last_arrival, now=t)
        else:
            v = self.tracker.step(self.x, ref, DT)
        self.v_cmd = slew(self.v_cmd, v, self.accel, DT)
        self.tracker.observe_command(self.v_cmd)
        return self.v_cmd


def run_case(params, ref_m, source_kw, hold=40.0, plant_gain=1.0, jumps=(), true_deadband_v=0.04, extra_lag=0.0):
    """jumps: [(time s, centimetres)]: SLAM corrects the map position (it changes instantly).
    true_deadband_v / extra_lag: the real robot differs from the model the controller knows (forward-speed deadband in m/s, an
    extra first-order lag in seconds)."""
    base = pm.IdentifiedPlant(use_filter=False, gain=(plant_gain, 1.0))
    plant = pm.Chain(base, extra_lag) if extra_lag > 0 else base
    src = PoseSource1D(**source_kw)
    loop = LinearNodeLoop({k: v for k, v in params.items() if k != '_timed'}, timed=bool(params.get('_timed', True)))
    x = 0.0
    n = int(hold / DT)
    pending = sorted(jumps)
    errs = []
    for k in range(n):
        t = k * DT
        while pending and pending[0][0] <= t:
            x += pending.pop(0)[1] / 100.0
        src.update(t, x)
        v = loop.tick(t, src.arrivals(t), ref_m)
        v_in, _ = pm.input_filter(v, 0.0, vdead=true_deadband_v)          # the real robot's command filter
        vv, _ = plant.step(v_in, 0.0)
        x += vv * DT
        errs.append((ref_m - x) * 100.0)                       # cm
    settle = next((i * DT for i in range(n - 1, -1, -1) if abs(errs[i]) > 1.0), -DT) + DT
    direction = 1.0 if errs[0] > 0 else -1.0
    over = max(0.0, max(-direction * e for e in errs))
    return dict(final=errs[-1], settle=settle, over=over)


def make_cases(n, seed=1, stress=False, mismatch=False):
    rng = random.Random(seed)
    cases = []
    for _ in range(n):
        ref = rng.choice([-1, 1]) * rng.uniform(0.15, 0.9)
        if stress or mismatch:
            src = dict(rate=rng.uniform(3.0, 15.0), latency=rng.uniform(0.05, 0.60), noise_cm=rng.uniform(0.3, 2.0),
                       freeze_period=rng.choice([20.0, 30.0, 35.0, 40.0]), freeze_len=rng.uniform(2.0, 6.0),
                       phase=rng.uniform(0, 35), seed=rng.randrange(10 ** 6))
            jumps = [(rng.uniform(4.0, 25.0), rng.choice([-1, 1]) * rng.uniform(1.0, 4.0)) for _ in range(rng.choice([0, 1, 2]))]
            gain = rng.uniform(0.8, 1.2)
        else:
            src = dict(rate=rng.uniform(8.0, 25.0), latency=rng.uniform(0.05, 0.30), noise_cm=rng.uniform(0.2, 1.0),
                       freeze_period=rng.choice([0.0, 35.0, 35.0, 20.0]), freeze_len=rng.uniform(2.0, 4.0),
                       phase=rng.uniform(0, 35), seed=rng.randrange(10 ** 6))
            jumps, gain = [], 1.0
        case = dict(ref=ref, source=src, jumps=jumps, plant_gain=gain)
        if mismatch:       # the real robot's forward deadband is 0.025-0.06 m/s (the controller is told 0.04), extra lag 0-0.4 s
            case.update(true_deadband_v=rng.uniform(0.025, 0.06), extra_lag=rng.uniform(0.0, 0.4))
        cases.append(case)
    return cases


def evaluate(params, cases, **kw):
    res = [run_case(params, c['ref'], c['source'], jumps=c['jumps'], plant_gain=c['plant_gain'],
                    true_deadband_v=c.get('true_deadband_v', 0.04), extra_lag=c.get('extra_lag', 0.0), **kw) for c in cases]
    f = sorted(abs(r['final']) for r in res)
    n = len(f)
    return dict(mean_abs=sum(f) / n, median=f[n // 2], p90=f[int(n * 0.9)], worst=f[-1], within1=sum(x <= 1.0 for x in f) / n,
                mean_over=sum(r['over'] for r in res) / n, mean_settle=sum(r['settle'] for r in res) / n)


def describe(name, s):
    print(f'{name:<40} final |error|: mean {s["mean_abs"]:5.2f}  median {s["median"]:5.2f}  p90 {s["p90"]:5.2f}  worst {s["worst"]:6.2f} cm'
          f' | within 1 cm {s["within1"] * 100:4.0f} % | overshoot {s["mean_over"]:5.2f} cm | settle {s["mean_settle"]:5.1f} s')


FIXED = dict(diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2)
PLAIN_P = dict(FIXED, _timed=True, kv=0.8, min_linear_speed=0.0, vel_filter=0.0, rest_window=0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cases', type=int, default=40)
    a = ap.parse_args()
    for label, stress in (('NORMAL', False), ('STRESS', True)):
        print(f'=== {label} conditions ({a.cases} cases)')
        describe('plain P law (kv 0.8)', evaluate(PLAIN_P, make_cases(a.cases, stress=stress)))


if __name__ == '__main__':
    main()
