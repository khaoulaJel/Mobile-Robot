#!/usr/bin/env python3
"""The P family on MEASURED feedback, on robots nobody measured (tools/servo_sim.py). Fair comparison: every member is tuned with the
same optimiser, budget and training robots (only its own coefficients free), then judged on robots it never saw:
    unseen       new random robots from the same ranges
    structure    the professor's 4th-order identified model (a different kind of robot; never used for tuning)
    beyond       outside the tuning ranges: deadband 0.30-0.40, gain 0.55-1.5, delay 0.35-0.6 s, lag 1.0-1.4 s, freezes up to 6 s

    python3 tools/servo_study.py [--random 220] [--refine 200] [--workers 16] [--out results/real_robot/servo_study.json]
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
import servo_sim as ss  # noqa: E402

SPACE = {
    'kp': (0.5, 2.5), 'ki': (0.0, 0.4), 'lead': (0.1, 1.2), 'u_max': (0.4, 1.0), 'tol': (0.008, 0.025),
    'u_min': (0.1, 0.35), 'stall_time': (0.5, 1.5), 'grow': (1.1, 1.6), 'shrink': (0.75, 0.97),
    'pulse_below': (0.08, 0.4), 'pulse_share': (0.6, 1.0), 'rest_rate': (0.015, 0.06), 'rest_time': (0.15, 0.6),
    'settle_min': (0.4, 1.0), 'settle_max': (2.5, 5.0), 'rate_filter': (0.3, 1.0),
}
CORE = ('kp', 'u_max', 'tol', 'rate_filter', 'rest_rate')
PULSE = ('pulse_below', 'pulse_share', 'rest_time', 'settle_min', 'settle_max')
LEARN = ('stall_time', 'grow', 'shrink')
OFF = dict(lead=0.0, ki=0.0, u_min=0.0, adapt=False, pulses=False, u_min_floor=0.0)
FAMILY = {                        # name: (free coefficients, fixed settings)
    'P': (CORE, {}),
    'PI': (CORE + ('ki',), {}),
    'PD': (CORE + ('lead',), {}),
    'PD+min': (CORE + ('lead', 'u_min'), {}),
    'PID+min': (CORE + ('lead', 'u_min', 'ki'), {}),
    'PD+pulse': (CORE + ('lead', 'u_min') + PULSE, dict(pulses=True)),
    'PD+pulse+learn': (CORE + ('lead', 'u_min') + PULSE + LEARN, dict(pulses=True, adapt=True, u_min_floor=0.03)),
}


def sample(rng, name):
    keys, fixed = FAMILY[name]
    p = dict(OFF, **fixed)
    p.update({k: rng.uniform(*SPACE[k]) for k in keys})
    return p


def perturb(rng, params, scale, name):
    q = dict(params)
    for k in FAMILY[name][0]:
        if rng.random() < 0.4:
            lo, hi = SPACE[k]
            q[k] = min(hi, max(lo, q[k] + rng.gauss(0.0, scale * (hi - lo))))
    return q


def beyond_cases(n, seed):
    rng = random.Random(seed)
    cases = ss.make_cases(n, seed=seed)
    for c in cases:
        c.update(db_w=rng.uniform(0.30, 0.40), gw=rng.choice([rng.uniform(0.55, 0.7), rng.uniform(1.3, 1.5)]),
                 cmd_delay=rng.uniform(0.35, 0.6), tau_w=rng.uniform(1.0, 1.4), freeze_len=rng.uniform(3.0, 6.0))
    if AXIS == 'distance':           # a separate generator, so the heading cases stay exactly as they were
        r2 = random.Random(seed + 1)
        for c in cases:
            c.update(db_v=r2.uniform(0.06, 0.075), gv=r2.choice([r2.uniform(0.6, 0.72), r2.uniform(1.28, 1.45)]),
                     tau_v=r2.uniform(0.8, 1.1))
    return cases


AXIS = 'heading'                  # 'heading' (errors in degrees) or 'distance' (errors in cm, "within" = 3 cm)
DISTANCE_SPACE = {'kp': (0.4, 2.5), 'ki': (0.0, 0.5), 'lead': (0.1, 1.2), 'u_max': (0.07, 0.2), 'tol': (0.006, 0.02), 'u_min': (0.025, 0.08),
                  'stall_time': (0.5, 1.5), 'grow': (1.1, 1.6), 'shrink': (0.75, 0.97), 'pulse_below': (0.03, 0.15),
                  'pulse_share': (0.6, 1.0), 'rest_rate': (0.004, 0.02), 'rest_time': (0.15, 0.6), 'settle_min': (0.4, 1.0),
                  'settle_max': (2.5, 5.0), 'rate_filter': (0.3, 1.0)}
# fixed on the distance axis: its own limits, and the heading servo keeps its defaults (it only holds the line)
DISTANCE_FIXED = dict(accel=0.4, u_min_cap=0.1, near_max=0.25)


def use_axis(axis):
    global AXIS, SPACE
    AXIS = axis
    if axis == 'distance':
        SPACE = DISTANCE_SPACE


def run_set(params, cases):
    if AXIS == 'distance':
        floor = dict(u_min_floor=0.015 if params.get('adapt') else 0.0)
        return ss.evaluate_distance(lambda: ss.ServoDistance({}, dict(params, **DISTANCE_FIXED, **floor)), cases)
    return ss.evaluate_heading(lambda: ss.ServoHeading(params), cases)


def score(s):
    return s['mean'] + 0.3 * s['p95'] + 0.03 * s['settle'] + 0.02 * s['over'] + 6.0 * (1.0 - s['within2']) + 0.2 * s['chatter']


_CASES = None


def _init(cases):
    global _CASES
    _CASES = cases


def _eval(params):
    return score(run_set(params, _CASES))


def tune(name, n_random, n_refine, workers, train, seed):
    rng = random.Random(seed)
    with Pool(workers, initializer=_init, initargs=(train,)) as pool:
        cands = [sample(rng, name) for _ in range(n_random)]
        res = pool.map(_eval, cands, chunksize=2)
        i = min(range(len(cands)), key=res.__getitem__)
        best, best_sc = cands[i], res[i]
        rounds = max(1, n_refine // 40)
        for rnd in range(rounds):
            cands = [perturb(rng, best, 0.15 * (0.7 ** rnd), name) for _ in range(40)]
            res = pool.map(_eval, cands, chunksize=1)
            i = min(range(len(cands)), key=res.__getitem__)
            if res[i] < best_sc:
                best, best_sc = cands[i], res[i]
    return best, best_sc


def validation_sets(n):
    return {'unseen': ss.make_cases(2 * n, seed=301), 'structure': ss.make_cases(n, seed=302, plant='identified'),
            'beyond': beyond_cases(n, 303)}


def line(name, set_name, s):
    unit, band = ('cm', '3 cm') if AXIS == 'distance' else ('deg', '2 deg')
    return (f'{name:<16}{set_name:<10} mean {s["mean"]:5.2f} +-{s["se"]:.2f}  p95 {s["p95"]:6.2f}  worst {s["worst"]:6.1f} {unit}  within {band} '
            f'{s["within2"] * 100:3.0f}%  overshoot {s["over"]:5.1f}  settle {s["settle"]:4.1f} s')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--random', type=int, default=220)
    ap.add_argument('--refine', type=int, default=200)
    ap.add_argument('--workers', type=int, default=16)
    ap.add_argument('--train', type=int, default=80)
    ap.add_argument('--val', type=int, default=100)
    ap.add_argument('--families', default=','.join(FAMILY))
    ap.add_argument('--axis', default='heading', choices=['heading', 'distance'])
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    use_axis(a.axis)
    train = ss.make_cases(a.train, seed=101)
    sets = validation_sets(a.val)
    result = {}
    for idx, name in enumerate(a.families.split(',')):
        prm, sc = tune(name, a.random, a.refine, a.workers, train, seed=10 + list(FAMILY).index(name))
        table = {k: run_set(prm, v) for k, v in sets.items()}
        result[name] = dict(params=prm, train_score=sc, validation=table)
        for k, s in table.items():
            print(line(name, k, s), flush=True)
        if a.out:
            os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
            with open(a.out, 'w') as f:
                json.dump(result, f, indent=1)
    print('done')


if __name__ == '__main__':
    main()
