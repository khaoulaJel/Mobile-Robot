#!/usr/bin/env python3
"""Random search for the preview-LQR tracker on the identified plant (train / held-out shapes).
Score [cm] = max cross-track + final position + 0.5 * final heading [deg] + 0.002 * time [s]."""
import argparse
import json
import os
import random
import sys
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plant_models as pm  # noqa: E402
import preview_lqr as pl  # noqa: E402

TRAIN = ['circle R0.4', 'ellipse 0.6x0.3', 'figure-eight', 'sine wave']
VALID = ['circle R0.5 (lab polar)', 'circle R0.2 (tight)', 'stadium']
SPACE = dict(v_cruise=[0.06, 0.08, 0.1, 0.12, 0.15], a_lat=[0.08, 0.12, 0.18, 0.3], a_dec=[0.03, 0.06, 0.1],
             horizon=[40, 60, 90], q_e=[1e3, 1e4, 1e5], q_th=[1.0, 10.0, 100.0], r=[0.3, 1.0, 3.0, 10.0])


def score(r):
    return 200.0 if not r['ok'] else r['max'] + r['fin'] + 0.5 * r['yaw'] + 0.002 * r['t']


def evaluate(args):
    params, shapes, noise, seed = args
    return [pl.run(s, params, pm.IdentifiedPlant(), noise=noise, seed=seed) for s in shapes]


def summ(rows):
    sc = [score(r) for r in rows]
    return dict(mean=sum(sc) / len(sc), worst=max(sc), done=sum(r['ok'] for r in rows),
                max_xtrk=max(r['max'] for r in rows), max_fin=max(r['fin'] for r in rows),
                max_yaw=max(r['yaw'] for r in rows), time=sum(r['t'] for r in rows) / len(rows))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=150)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--noise', type=float, default=0.0005)
    ap.add_argument('--top', type=int, default=4)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rng = random.Random(a.seed)
    cands = [dict()] + [{k: rng.choice(v) for k, v in SPACE.items()} for _ in range(a.samples)]
    noise = (a.noise, a.noise)
    with ProcessPoolExecutor(18) as ex:
        res = list(ex.map(evaluate, [(c, TRAIN, noise, 1) for c in cands]))
        ranked = sorted(zip(cands, map(summ, res)), key=lambda cs: (-cs[1]['done'], cs[1]['mean']))
        print('untuned defaults:', {k: round(v, 2) for k, v in next(s for c, s in ranked if c == {}).items()})
        best = ranked[:a.top]
        val = list(ex.map(evaluate, [(c, VALID, noise, 1) for c, _ in best]))
    out = []
    for (c, s), vr in zip(best, val):
        print('\nTRAIN', {k: round(v, 2) for k, v in s.items()})
        print('VALID', {k: round(v, 2) for k, v in summ(vr).items()})
        print('  ', c)
        out.append(dict(params=c, train=s, valid=summ(vr)))
    if a.out:
        json.dump(out, open(a.out, 'w'), indent=1)
