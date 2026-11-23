#!/usr/bin/env python3
"""Closed-loop simulation of the symbolic circle controller (no ROS).

Plants
  kinematic   the exact unicycle the abstraction was built for (the guarantee must hold: tube never left)
  identified  the professor's identified velocity model behind his input_filter (the abstraction ignores its lag)
  lag:<tau>   unicycle with a first-order velocity lag (Gazebo-like or slower)
Options: measurement noise on the state the controller sees, extra command latency (samples of 0.05 s).

Usage: python3 symbolic_sim.py [--plant kinematic] [--noise 0.002] [--seeds 5]
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import plant_models as pm  # noqa: E402
from lab3_controllers.symbolic import Policy  # noqa: E402

DT = 0.05


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def make_plant(spec):
    if spec == 'kinematic':
        return None
    if spec == 'identified':
        return pm.IdentifiedPlant(use_filter=True)
    if spec.startswith('lag:'):
        return pm.LagPlant(float(spec[4:]))
    raise ValueError(spec)


def run(policy, plant_spec='kinematic', noise=(0.0, 0.0), seed=0, tmax=1500.0, start=(0.0, 0.0, 0.0), robust=False):
    rng = np.random.default_rng(seed)
    plant = make_plant(plant_spec)
    r = float(policy.meta['radius'])
    cx, cy = 0.0, r
    x, y, th = start
    t, k, mode = 0.0, 0, 'RUN'
    max_rad = 0.0
    steps_per_goal = [0] * len(policy.goals)
    xs, ys = [x], [y]
    n_sub = int(round(policy.tau / DT))
    while t < tmax and mode == 'RUN':
        mx = x + rng.normal(0, noise[0])
        my = y + rng.normal(0, noise[0])
        mth = th + rng.normal(0, noise[1])
        q = policy.state_index(mx, my, mth)
        if q < 0 and not robust:
            mode = 'LEFT_TUBE'
            break
        while k < len(policy.goals) and q >= 0 and policy.goal_reached(k, q):
            k += 1
        if k >= len(policy.goals):
            mode = 'DONE'
            break
        u = policy.action(k, q) if q >= 0 else None
        if u is None and robust:
            u = policy.robust_action(k, mx, my, mth)
        if u is None:
            mode = 'LEFT_TUBE' if q < 0 else 'NO_INPUT'
            break
        steps_per_goal[k] += 1
        v, w = u
        for _ in range(n_sub):
            if plant is None:
                vv, ww = v, w
            else:
                vv, ww = plant.step(v, w)
            th2 = th + ww * DT
            x += vv * math.cos(th + ww * DT / 2) * DT
            y += vv * math.sin(th + ww * DT / 2) * DT
            th = wrap(th2)
            max_rad = max(max_rad, abs(math.hypot(x - cx, y - cy) - r))
            xs.append(x)
            ys.append(y)
            t += DT
    return dict(mode=mode, t=t, goals=k, max_radial=max_rad, steps=steps_per_goal, final=(x, y, th), xs=xs, ys=ys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--policy', default=os.path.join(HERE, '..', 'config', 'symbolic_circle.npz'))
    ap.add_argument('--plant', default='kinematic')
    ap.add_argument('--noise', type=float, default=0.0, help='position noise [m]; heading noise = 1.5 x this [rad]')
    ap.add_argument('--seeds', type=int, default=1)
    ap.add_argument('--robust', action='store_true', help='neighbour-cell fallback (outside the formal guarantee)')
    a = ap.parse_args()
    pol = Policy.load(a.policy)
    tube = float(pol.meta['tube'])
    res = [run(pol, a.plant, (a.noise, 1.5 * a.noise), seed=s, robust=a.robust) for s in range(a.seeds)]
    done = sum(r['mode'] == 'DONE' for r in res)
    print(f"{'robust ' if a.robust else ''}plant={a.plant} noise={a.noise * 1000:.1f} mm: completed {done}/{len(res)}; "
          f"time {np.mean([r['t'] for r in res]):.0f} s; max radial error {100 * max(r['max_radial'] for r in res):.2f} cm "
          f"(tube +-{tube * 100:.0f} cm); outcomes {sorted(set(r['mode'] for r in res))}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
