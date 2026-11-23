#!/usr/bin/env python3
"""Random-search tuning on the professor's identified plant + input filter (no ROS), with a held-out
validation set of shapes the search never saw (guards against tuning to the square and the circle).

score per run [cm] = max cross-track + final position error + 0.5 * final heading error [deg] + 0.002 * time [s]
                     (200 if the path was not completed). Lower is better.

Usage: python3 tune_lab_plant.py --strategy stop_turn --samples 120 --seed 1 [--out result.json]
"""
import argparse
import json
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shape_bench as sb  # noqa: E402

TRAIN = ['square 0.5', 'triangle', 'hexagon R0.3', 'circle R0.4', 'sine wave', 'figure-eight']
VALID = ['rectangle 0.8x0.3', 'square 0.5 CW', 'star (36 deg tips)', 'circle R0.5 (lab polar)',
         'circle R0.2 (tight)', 'ellipse 0.6x0.3', 'stadium', 'spiral out']

# the professor's robot: feasible diamond + deadband; plant coasting 0.61 s (v) and 1.0 s (w), measured
ROBOT = dict(diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2)
# features that are always on in the lab-robot preset (not searched)
FIXED = dict(ROBOT, rest_window=1.0, smooth_yaw_tolerance=0.21, vel_window=5)
NOISE = (0.0005, 0.0005)       # odometry noise seen by the controller during tuning [m, rad]; 0.5 mm / 0.03 deg

SPACE = {
    'stop_turn': dict(
        kv=[0.4, 0.5, 0.65, 0.8, 1.0], kw=[0.8, 1.0, 1.3, 1.7, 2.2], k_yaw=[0.5, 0.8, 1.0, 1.5],
        turn_in_place_deg=[10.0, 15.0, 20.0, 30.0], lead_v=[0.3, 0.45, 0.6, 0.75], lead_w=[0.6, 0.8, 1.0, 1.2],
        vel_filter=[0.4, 0.5, 0.7], max_linear_speed=[0.1, 0.15, 0.2], decel=[0.4]),
    'pure_pursuit': dict(
        lookahead=[0.1, 0.15, 0.2, 0.3, 0.4], max_linear_speed=[0.05, 0.08, 0.12, 0.2], k_yaw=[0.6, 1.0, 1.5],
        lead_v=[0.0, 0.3, 0.61, 0.8], lead_w=[0.0, 0.5, 1.0], vel_filter=[0.2, 0.3, 0.5], decel=[0.2, 0.4]),
}


def score(row):
    if not row['ok']:
        return 200.0
    return row['max'] + row['fin'] + 0.5 * row['yaw'] + 0.002 * row['t']     # small time penalty


def evaluate(args):
    strategy, params, shapes, ds, settle, noise = args
    over = dict(FIXED)
    over.update(params)
    rows = sb.run_all(ds, 0.15, [strategy], over={strategy: over}, plant='identified', only=shapes,
                      settle=settle, noise=noise, seed=1)
    return rows


def summarize(rows):
    sc = [score(r) for r in rows]
    return dict(mean=sum(sc) / len(sc), worst=max(sc), done=sum(r['ok'] for r in rows), n=len(rows),
                max_xtrk=max(r['max'] for r in rows), max_fin=max(r['fin'] for r in rows),
                max_yaw=max(r['yaw'] for r in rows), time=sum(r['t'] for r in rows) / len(rows))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--strategy', default='stop_turn', choices=list(SPACE))
    ap.add_argument('--samples', type=int, default=100)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--ds', type=float, default=0.1)
    ap.add_argument('--settle', type=float, default=12.0)
    ap.add_argument('--top', type=int, default=6)
    ap.add_argument('--out', default='')
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()
    rng = random.Random(a.seed)
    space = SPACE[a.strategy]
    cands = [{k: rng.choice(v) for k, v in space.items()} for _ in range(a.samples)]
    cands.insert(0, {})        # the untuned defaults (with the robot filter) as reference
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(evaluate, [(a.strategy, c, TRAIN, a.ds, a.settle, NOISE) for c in cands]))
    ranked = sorted(zip(cands, map(summarize, res)), key=lambda cs: (-cs[1]['done'], cs[1]['mean']))
    print(f'{len(cands)} candidates, {time.time() - t0:.0f} s, strategy {a.strategy}, train={TRAIN}')
    base = next(s for c, s in ranked if c == {})
    print('untuned (robot filter only):', {k: round(v, 2) for k, v in base.items()})
    best = ranked[:a.top]
    with ProcessPoolExecutor(a.workers) as ex:
        val = list(ex.map(evaluate, [(a.strategy, c, VALID, a.ds, a.settle, NOISE) for c, _ in best]))
    out = []
    for (c, s), vr in zip(best, val):
        vs = summarize(vr)
        print('\nTRAIN', {k: round(v, 2) for k, v in s.items()})
        print('VALID', {k: round(v, 2) for k, v in vs.items()})
        print('  ', c)
        out.append(dict(params=c, train=s, valid=vs, valid_rows=vr))
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
