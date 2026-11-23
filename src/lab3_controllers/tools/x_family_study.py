#!/usr/bin/env python3
"""The P-controller family compared FAIRLY on the forward position (x and x-dot) of the professor's robot (no ROS).

Same method as tools/p_family_study.py (heading): P, PI, PD, PID and PID + rest/pulse layer, each tuned with the same optimiser, budget
and training cases, judged on unseen validation cases with the standard error, then stress-tested against a different robot.
Simulator: tools/x_real_sim.py (forward-speed deadband 0.04 m/s and lag behind a slow, late, noisy, freezing pose source).

    python3 tools/x_family_study.py [--random 240] [--refine 160] [--workers 12] [--out results/real_robot/x_family_study.json]
"""
import argparse
import json
import os
import random
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import x_real_sim as xs  # noqa: E402

SPACE = {
    'kv': (0.2, 1.5, 'f'), 'max_linear_speed': (0.05, 0.25, 'f'), 'pos_tolerance': (0.003, 0.02, 'f'),
    'lead_v': (0.0, 1.2, 'f'), 'ki_v': (0.0, 0.3, 'f'),
    'x_obs_tau': (0.2, 1.5, 'f'), 'x_obs_gain': (0.7, 1.3, 'f'), 'x_obs_k1': (0.05, 0.9, 'f'), 'x_obs_k2': (0.0, 1.0, 'f'),
    'pose_latency': (0.0, 0.7, 'f'), 'vel_span': (0.5, 2.0, 'f'), 'vel_filter': (0.2, 1.0, 'f'),
    'rest_window': (0.6, 2.0, 'f'), 'rest_pos': (0.002, 0.01, 'f'), 'settle_time': (1.0, 4.0, 'f'), 'rest_timeout': (3.0, 7.0, 'f'),
    'pulse_extra': (0, 3, 'i'), 'rest_retries': (3, 8, 'i'),
}
BASE = dict(xs.FIXED, _timed=True, rest_window=0.0, ki_v=0.0, lead_v=0.0, pose_latency=0.0, x_use_observer=False, vel_filter=0.0)
OBS = ('x_obs_tau', 'x_obs_gain', 'x_obs_k1', 'x_obs_k2', 'pose_latency', 'vel_span', 'vel_filter')
CORE = ('kv', 'max_linear_speed', 'pos_tolerance')
LAYER = ('rest_window', 'rest_pos', 'settle_time', 'rest_timeout', 'pulse_extra', 'rest_retries')
FAMILY = {
    'P': (CORE, {}),
    'PI': (CORE + ('ki_v',), {}),
    'PD': (CORE + ('lead_v',) + OBS, {'x_use_observer': True}),
    'PID': (CORE + ('lead_v', 'ki_v') + OBS, {'x_use_observer': True}),
    'PID+layer': (CORE + ('lead_v', 'ki_v') + OBS + LAYER, {'x_use_observer': True, 'rest_mean_test': True}),
}


def draw(rng, k):
    lo, hi, kind = SPACE[k]
    return rng.randint(int(lo), int(hi)) if kind == 'i' else rng.uniform(lo, hi)


def sample(rng, name):
    keys, fixed = FAMILY[name]
    p = dict(BASE, **fixed)
    for k in keys:
        p[k] = draw(rng, k)
    return p


def perturb(rng, params, scale, name):
    q = dict(params)
    for k in FAMILY[name][0]:
        lo, hi, kind = SPACE[k]
        if rng.random() < 0.5 and lo != hi:
            v = min(hi, max(lo, q[k] + rng.gauss(0.0, scale * (hi - lo))))
            q[k] = int(round(v)) if kind == 'i' else v
    return q


def score(params, cases):
    s = xs.evaluate(params, cases)
    return (s['mean_abs'] + 0.3 * s['p90'] + 0.05 * s['mean_over'] + 0.02 * s['mean_settle'] + 0.5 * (1.0 - s['within1'])), s


_CASES = None


def _init(cases):
    global _CASES
    _CASES = cases


def _eval(params):
    return score(params, _CASES)


def tune(name, n_random, n_refine, workers, train, seed):
    rng = random.Random(seed)
    with Pool(workers, initializer=_init, initargs=(train,)) as pool:
        cands = [sample(rng, name) for _ in range(n_random)]
        res = pool.map(_eval, cands, chunksize=4)
        i = min(range(len(cands)), key=lambda j: res[j][0])
        best, best_sc = cands[i], res[i][0]
        for rnd in range(n_refine // 40):
            cands = [perturb(rng, best, 0.15 * (0.6 ** rnd), name) for _ in range(40)]
            res = pool.map(_eval, cands, chunksize=2)
            i = min(range(len(cands)), key=lambda j: res[j][0])
            if res[i][0] < best_sc:
                best, best_sc = cands[i], res[i][0]
    return best, best_sc


def stats(params, cases):
    res = [xs.run_case(params, c['ref'], c['source'], jumps=c['jumps'], plant_gain=c['plant_gain'],
                       true_deadband_v=c.get('true_deadband_v', 0.04), extra_lag=c.get('extra_lag', 0.0)) for c in cases]
    f = [abs(r['final']) for r in res]
    n = len(f)
    mean = sum(f) / n
    se = (sum((x - mean) ** 2 for x in f) / max(n - 1, 1)) ** 0.5 / n ** 0.5
    fs = sorted(f)
    return dict(mean=mean, se=se, p90=fs[int(n * 0.9)], worst=fs[-1], within1=sum(x <= 1.0 for x in f) / n,
                over=sum(r['over'] for r in res) / n, settle=sum(r['settle'] for r in res) / n)


def perturbed(cases, what):
    out = []
    for c in cases:
        s, g = dict(c['source']), c['plant_gain']
        if what == 'robot gain x0.7':
            g *= 0.7
        elif what == 'robot gain x1.3':
            g *= 1.3
        elif what == 'extra pose delay +0.3 s':
            s['latency'] += 0.3
        elif what == 'pose noise x2':
            s['noise_cm'] *= 2
        elif what == 'freezes x2 longer':
            s['freeze_len'] *= 2
        elif what == 'pose rate x0.5':
            s['rate'] *= 0.5
        elif what == 'true deadband 0.025 (model says 0.04)':
            out.append(dict(c, source=s, plant_gain=g, true_deadband_v=0.025))
            continue
        elif what == 'true deadband 0.06 (model says 0.04)':
            out.append(dict(c, source=s, plant_gain=g, true_deadband_v=0.06))
            continue
        elif what == 'robot lag +0.4 s (model unaware)':
            out.append(dict(c, source=s, plant_gain=g, extra_lag=0.4))
            continue
        out.append(dict(c, source=s, plant_gain=g))
    return out


ROBUSTNESS = ['nominal (wrong robot)', 'robot gain x0.7', 'robot gain x1.3', 'extra pose delay +0.3 s', 'pose noise x2', 'freezes x2 longer',
              'pose rate x0.5', 'true deadband 0.025 (model says 0.04)', 'true deadband 0.06 (model says 0.04)',
              'robot lag +0.4 s (model unaware)']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--random', type=int, default=240)
    ap.add_argument('--refine', type=int, default=160)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--train', type=int, default=30)
    ap.add_argument('--val', type=int, default=100)
    ap.add_argument('--families', default=','.join(FAMILY))
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    train = (xs.make_cases(a.train, seed=11, stress=False) + xs.make_cases(a.train, seed=12, stress=True)
             + xs.make_cases(a.train, seed=13, mismatch=True))
    val_n, val_s = xs.make_cases(a.val, seed=201, stress=False), xs.make_cases(a.val, seed=202, stress=True)
    val_m = xs.make_cases(a.val, seed=203, mismatch=True)
    rob_base = xs.make_cases(60, seed=301, mismatch=True)
    best, table, rob = {}, {}, {}
    for name in a.families.split(','):
        prm, sc = tune(name, a.random, a.refine, a.workers, train, seed={'P': 1, 'PI': 2, 'PD': 3, 'PID': 4, 'PID+layer': 5}[name])
        best[name] = prm
        table[name] = dict(normal=stats(prm, val_n), stress=stats(prm, val_s), mismatch=stats(prm, val_m), train_score=sc)
        rob[name] = {w: stats(prm, perturbed(rob_base, w) if w != 'nominal (wrong robot)' else rob_base)['mean'] for w in ROBUSTNESS}
        n, s, m = table[name]['normal'], table[name]['stress'], table[name]['mismatch']
        print(f'{name:<10} normal {n["mean"]:5.2f} +-{n["se"]:.2f} cm (1cm {n["within1"] * 100:3.0f}%, overshoot {n["over"]:5.1f})   '
              f'STRESS {s["mean"]:5.2f} +-{s["se"]:.2f} (1cm {s["within1"] * 100:3.0f}%, worst {s["worst"]:5.1f})   '
              f'WRONG-ROBOT {m["mean"]:5.2f} +-{m["se"]:.2f} (1cm {m["within1"] * 100:3.0f}%, p90 {m["p90"]:5.2f}, worst {m["worst"]:5.1f})',
              flush=True)
    print('\nROBUSTNESS: mean final error (cm) when the real robot differs from the tuning conditions')
    print(f'{"":<24}' + ''.join(f'{n:>10}' for n in best))
    for w in ROBUSTNESS:
        print(f'{w:<24}' + ''.join(f'{rob[n][w]:10.2f}' for n in best))
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w') as f:
            json.dump(dict(tuned=best, validation=table, robustness=rob), f, indent=1, default=float)
        print('saved', a.out)


if __name__ == '__main__':
    main()
