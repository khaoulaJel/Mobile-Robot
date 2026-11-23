#!/usr/bin/env python3
"""Robustness of one stop_turn parameter set on the identified plant + robot filter (no ROS).

Varies what the real robot will change: odometry noise, calibration (gain) error, waypoint spacing.
Usage: python3 robustness.py [--preset ../config/lab_robot.yaml-like json] ; defaults to LAB_ROBOT below.
"""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shape_bench as sb  # noqa: E402
import tune_lab_plant as tp  # noqa: E402

CANDIDATE = dict(kv=0.8, kw=2.2, k_yaw=0.8, turn_in_place_deg=15.0, lead_v=0.3, lead_w=0.8, vel_filter=0.7,
                 max_linear_speed=0.1)      # = config/lab_robot.yaml (tools/tune_lab_plant.py result)
SHAPES = ['square 0.5', 'triangle', 'circle R0.4', 'sine wave', 'figure-eight', 'star (36 deg tips)']


def case(args):
    name, params, plant, noise, seed, ds = args
    over = dict(tp.FIXED)
    over.update(params)
    rows = sb.run_all(ds, 0.15, ['stop_turn'], over={'stop_turn': over}, plant=plant, only=SHAPES,
                      settle=12.0, noise=noise, seed=seed)
    return name, rows


def main():
    cases = []
    for noise_name, noise in (('no noise', (0.0, 0.0)), ('noise 0.5mm/0.03deg', (0.0005, 0.0005)),
                              ('noise 2mm/0.17deg', (0.002, 0.003))):
        for seed in ((0,) if noise == (0.0, 0.0) else (1, 2, 3)):
            cases.append((noise_name, dict(CANDIDATE), 'identified', noise, seed, 0.1))
    for plant, label in (('identified@0.85,0.85', 'gain -15%'), ('identified@1.15,1.15', 'gain +15%'),
                         ('identified@0.85,1.15', 'v -15% / w +15%'), ('identified@1.15,0.85', 'v +15% / w -15%'),
                         ('identified@1,1,2', 'command latency +0.1 s'), ('identified@1,1,4', 'command latency +0.2 s')):
        cases.append((label, dict(CANDIDATE), plant, (0.0, 0.0), 0, 0.1))
    for ds in (0.05, 0.2):
        cases.append((f'waypoint spacing {ds} m', dict(CANDIDATE), 'identified', (0.0, 0.0), 0, ds))
    with ProcessPoolExecutor(max(1, (os.cpu_count() or 2) - 1)) as ex:
        res = list(ex.map(case, cases))
    agg = {}
    for name, rows in res:
        a = agg.setdefault(name, [])
        a.extend(rows)
    print(f"{'condition':26s} {'runs':>4s} {'done':>5s} {'max xtrk':>9s} {'mean xtrk':>9s} {'max final':>9s} {'max yaw':>8s} {'score':>6s}")
    for name, rows in agg.items():
        n = len(rows)
        print(f"{name:26s} {n:4d} {sum(r['ok'] for r in rows):2d}/{n:<2d} {max(r['max'] for r in rows):9.2f} "
              f"{sum(r['max'] for r in rows) / n:9.2f} {max(r['fin'] for r in rows):9.2f} "
              f"{max(r['yaw'] for r in rows):8.2f} {sum(tp.score(r) for r in rows) / n:6.2f}")


if __name__ == '__main__':
    main()
