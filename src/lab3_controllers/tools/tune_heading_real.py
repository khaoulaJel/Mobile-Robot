#!/usr/bin/env python3
"""Tune the heading controller's coefficients for the real conditions (no ROS): random search, then local refinement, on
the simulator of tools/yaw_real_sim.py (the professor's robot model behind a slow, late, noisy, freezing pose source).

    python3 tools/tune_heading_real.py [--random 300] [--refine 200] [--workers 4] [--out results/real_robot/tuned_heading.json]

Training cases and validation cases are different random draws, so the reported numbers are for conditions the search has
never seen. The score is what we want from a turn: small final error (mean and 90th percentile), little overshoot, quick.
Searches two controller modes: the classic one and the timed one (sample stamps, latency compensation, averaging).
"""
import argparse
import json
import math
import os
import random
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import yaw_real_sim as ys  # noqa: E402

FIXED = dict(diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2)      # the robot (professor's model)
SPACE = {                                  # name: (low, high, kind)
    'kp': (0.25, 1.3, 'f'), 'max_angular_speed': (0.3, 1.0, 'f'), 'tolerance': (0.005, 0.03, 'f'),
    'lead_w': (0.0, 1.0, 'f'), 'pose_latency': (0.0, 0.7, 'f'), 'vel_span': (0.5, 2.0, 'f'),
    'vel_filter': (0.2, 1.0, 'f'), 'rest_window': (0.6, 2.0, 'f'), 'rest_yaw': (0.008, 0.05, 'f'),
    'rest_pos': (0.003, 0.003, 'f'), 'settle_time': (1.0, 4.0, 'f'), 'rest_timeout': (3.0, 7.0, 'f'),
    'pulse_extra': (0, 3, 'i'), 'rest_retries': (3, 8, 'i'),
    'obs_tau': (0.3, 1.5, 'f'), 'obs_gain': (0.7, 1.3, 'f'), 'obs_k1': (0.05, 0.9, 'f'), 'obs_k2': (0.0, 1.0, 'f'),
    'ki': (0.0, 0.2, 'f'),
}
OBSERVER_KEYS = ('obs_tau', 'obs_gain', 'obs_k1', 'obs_k2', 'ki')
CLASSIC_KEYS = ('kp', 'max_angular_speed', 'tolerance', 'lead_w', 'vel_filter', 'rest_window', 'rest_yaw', 'settle_time',
                'rest_timeout', 'pulse_extra', 'rest_retries')


def sample(rng, mode):
    p = dict(FIXED)
    for k, (lo, hi, kind) in SPACE.items():
        if mode == 'classic' and k not in CLASSIC_KEYS:
            continue
        if mode == 'timed' and k in OBSERVER_KEYS:
            continue
        p[k] = rng.randint(int(lo), int(hi)) if kind == 'i' else rng.uniform(lo, hi)
    if mode == 'classic':
        p['vel_window'] = 5
    else:
        p['_timed'] = True
        p['rest_mean_test'] = True
    if mode == 'observer':
        p['use_observer'] = True
    return p


def perturb(rng, params, scale):
    q = dict(params)
    for k, (lo, hi, kind) in SPACE.items():
        if k not in q or lo == hi:
            continue
        if rng.random() < 0.5:
            v = q[k] + rng.gauss(0.0, scale * (hi - lo))
            v = min(hi, max(lo, v))
            q[k] = int(round(v)) if kind == 'i' else v
    return q


def score(params, cases):
    s = ys.evaluate(params, cases)
    return (s['mean_abs'] + 0.3 * s['p90'] + 0.05 * s['mean_over'] + 0.02 * s['mean_settle'] + 0.5 * (1.0 - s['within1'])), s


_CASES = None


def _init(cases):
    global _CASES
    _CASES = cases


def _eval(params):
    return score(params, _CASES)


def search(mode, n_random, n_refine, workers, train, seed):
    rng = random.Random(seed)
    with Pool(workers, initializer=_init, initargs=(train,)) as pool:
        cands = [sample(rng, mode) for _ in range(n_random)]
        res = pool.map(_eval, cands, chunksize=4)
        best_i = min(range(len(cands)), key=lambda i: res[i][0])
        best, best_sc = cands[best_i], res[best_i][0]
        for rnd in range(n_refine // 40):
            scale = 0.15 * (0.6 ** rnd)
            cands = [perturb(rng, best, scale) for _ in range(40)]
            res = pool.map(_eval, cands, chunksize=2)
            i = min(range(len(cands)), key=lambda j: res[j][0])
            if res[i][0] < best_sc:
                best, best_sc = cands[i], res[i][0]
    return best, best_sc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--random', type=int, default=300)
    ap.add_argument('--refine', type=int, default=200)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--train', type=int, default=30, help='cases per condition set (normal and stress) for training')
    ap.add_argument('--val', type=int, default=80)
    ap.add_argument('--modes', default='classic,observer', help='comma list of: classic, timed, observer')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    train = ys.make_cases(a.train, seed=11, stress=False) + ys.make_cases(a.train, seed=12, stress=True)
    val_normal, val_stress = ys.make_cases(a.val, seed=101, stress=False), ys.make_cases(a.val, seed=102, stress=True)
    out = {}
    for name in a.modes.split(','):
        best, sc = search(name, a.random, a.refine, a.workers, train, seed={'classic': 5, 'timed': 7, 'observer': 9}[name])
        out[name] = best
        print(f'--- best {name} mode (training score {sc:.3f})')
        print('   ', {k: (round(v, 4) if isinstance(v, float) else v) for k, v in best.items() if k not in FIXED})
        ys.describe(f'  {name}: validation, normal conditions', ys.evaluate(best, val_normal))
        ys.describe(f'  {name}: validation, STRESS conditions', ys.evaluate(best, val_stress))
    print('--- references on the same validation cases')
    for label, prm in (('plain P law', dict(kp=1.0, tolerance=0.01, **FIXED)), ('preset used in the run', ys.PRESET)):
        ys.describe(f'  {label}: normal', ys.evaluate(prm, val_normal))
        ys.describe(f'  {label}: STRESS', ys.evaluate(prm, val_stress))
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w') as f:
            json.dump(out, f, indent=1)
        print('saved', a.out)


if __name__ == '__main__':
    main()
