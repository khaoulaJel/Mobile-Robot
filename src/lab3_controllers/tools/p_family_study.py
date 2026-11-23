#!/usr/bin/env python3
"""The P-controller family compared FAIRLY on the heading of the professor's robot (no ROS).

    P    proportional only                                  u = kp * e
    PI   + integral (anti-windup)                           u = kp * e + ki * int(e)
    PD   + derivative, from the observer's turn-rate        u = kp * (e - lead * w_hat)       (lead = kd / kp)
    PID  both
    PID+layer   PID + the rest-verify / corrective-pulse layer (what the tuned preset uses)

Every member is tuned with the SAME optimiser, the SAME budget and the SAME training cases (only its own coefficients are free), then
judged on validation cases none of them saw, with the standard error of the mean, and finally stress-tested against a robot that is
different from the one it was tuned for (gain, delay, noise, freezes, rate). All run on the simulator of tools/yaw_real_sim.py:
the professor's robot model (deadband, lag) behind a slow, late, noisy, freezing pose source.

    python3 tools/p_family_study.py [--random 240] [--refine 160] [--workers 12] [--out results/real_robot/p_family_study.json]
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
import tune_heading_real as th  # noqa: E402
import yaw_real_sim as ys  # noqa: E402

# vel_filter 0: no speed estimate at all, so P and PI really are P and PI (the observer families free it again)
BASE = dict(th.FIXED, _timed=True, rest_window=0.0, ki=0.0, lead_w=0.0, pose_latency=0.0, use_observer=False, vel_filter=0.0)
OBS = ('obs_tau', 'obs_gain', 'obs_k1', 'obs_k2', 'pose_latency', 'vel_span', 'vel_filter')
CORE = ('kp', 'max_angular_speed', 'tolerance')
LAYER = ('rest_window', 'rest_yaw', 'settle_time', 'rest_timeout', 'pulse_extra', 'rest_retries')
FAMILY = {                     # name: (free keys, fixed overrides)
    'P': (CORE, {}),
    'PI': (CORE + ('ki',), {}),
    'PD': (CORE + ('lead_w',) + OBS, {'use_observer': True}),
    'PID': (CORE + ('lead_w', 'ki') + OBS, {'use_observer': True}),
    'PID+layer': (CORE + ('lead_w', 'ki') + OBS + LAYER, {'use_observer': True, 'rest_mean_test': True}),
}


def sample(rng, name):
    keys, fixed = FAMILY[name]
    p = dict(BASE, **fixed)
    for k in keys:
        lo, hi, kind = th.SPACE[k]
        p[k] = rng.randint(int(lo), int(hi)) if kind == 'i' else rng.uniform(lo, hi)
    if name == 'PID+layer':
        p['rest_pos'] = 0.003
    return p


def perturb(rng, params, scale, name):
    keys = FAMILY[name][0]
    q = dict(params)
    for k in keys:
        lo, hi, kind = th.SPACE[k]
        if rng.random() < 0.5 and lo != hi:
            v = min(hi, max(lo, q[k] + rng.gauss(0.0, scale * (hi - lo))))
            q[k] = int(round(v)) if kind == 'i' else v
    return q


_CASES = None


def _init(cases):
    global _CASES
    _CASES = cases


def _eval(params):
    return th.score(params, _CASES)


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


def stats(params, cases, **kw):
    """Validation statistics with the standard error of the mean final error."""
    res = [ys.run_case(params, c['y0'], c['ref'], c['source'], jumps=c['jumps'], plant_gain=c['plant_gain'],
                       true_deadband_w=c.get('true_deadband_w', 0.2), extra_lag=c.get('extra_lag', 0.0), **kw) for c in cases]
    f = [abs(r['final']) for r in res]
    n = len(f)
    mean = sum(f) / n
    se = (sum((x - mean) ** 2 for x in f) / max(n - 1, 1)) ** 0.5 / n ** 0.5
    fs = sorted(f)
    return dict(mean=mean, se=se, p90=fs[int(n * 0.9)], worst=fs[-1], within1=sum(x <= 1.0 for x in f) / n,
                over=sum(r['over'] for r in res) / n, settle=sum(r['settle'] for r in res) / n)


def perturbed_cases(cases, what):
    out = []
    for c in cases:
        s = dict(c['source'])
        g = c['plant_gain']
        if what == 'robot gain x0.7':
            g *= 0.7
        elif what == 'robot gain x1.3':
            g *= 1.3
        elif what == 'extra pose delay +0.3 s':
            s['latency'] += 0.3
        elif what == 'pose noise x2':
            s['noise_deg'] *= 2
        elif what == 'freezes x2 longer':
            s['freeze_len'] *= 2
        elif what == 'pose rate x0.5':
            s['rate'] *= 0.5
        elif what == 'true deadband 0.12 (model says 0.2)':
            out.append(dict(c, source=s, plant_gain=g, true_deadband_w=0.12))
            continue
        elif what == 'true deadband 0.30 (model says 0.2)':
            out.append(dict(c, source=s, plant_gain=g, true_deadband_w=0.30))
            continue
        elif what == 'robot lag +0.4 s (model unaware)':
            out.append(dict(c, source=s, plant_gain=g, extra_lag=0.4))
            continue
        elif what == 'BEYOND the tuning range':
            s.update(rate=min(s['rate'], 2.5), latency=s['latency'] + 0.5, noise_deg=s['noise_deg'] * 2.5, freeze_len=max(s['freeze_len'], 8.0))
        out.append(dict(c, source=s, plant_gain=g))
    return out


ROBUSTNESS = ['nominal (wrong robot)', 'robot gain x0.7', 'robot gain x1.3', 'extra pose delay +0.3 s', 'pose noise x2', 'freezes x2 longer',
              'pose rate x0.5', 'true deadband 0.12 (model says 0.2)', 'true deadband 0.30 (model says 0.2)',
              'robot lag +0.4 s (model unaware)', 'BEYOND the tuning range']


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
    train = (ys.make_cases(a.train, seed=11, stress=False) + ys.make_cases(a.train, seed=12, stress=True)
             + ys.make_cases(a.train, seed=13, mismatch=True))
    val_n, val_s = ys.make_cases(a.val, seed=201, stress=False), ys.make_cases(a.val, seed=202, stress=True)
    val_m = ys.make_cases(a.val, seed=203, mismatch=True)
    rob_base = ys.make_cases(60, seed=301, mismatch=True)
    best, table, rob = {}, {}, {}
    for name in a.families.split(','):
        prm, sc = tune(name, a.random, a.refine, a.workers, train, seed={'P': 1, 'PI': 2, 'PD': 3, 'PID': 4, 'PID+layer': 5}[name])
        best[name] = prm
        table[name] = dict(normal=stats(prm, val_n), stress=stats(prm, val_s), mismatch=stats(prm, val_m), train_score=sc)
        rob[name] = {w: stats(prm, perturbed_cases(rob_base, w) if w != 'nominal (wrong robot)' else rob_base)['mean'] for w in ROBUSTNESS}
        n, s, m = table[name]['normal'], table[name]['stress'], table[name]['mismatch']
        print(f'{name:<10} normal {n["mean"]:5.2f} +-{n["se"]:.2f} deg (1deg {n["within1"] * 100:3.0f}%, overshoot {n["over"]:5.1f})   '
              f'STRESS {s["mean"]:5.2f} +-{s["se"]:.2f} (1deg {s["within1"] * 100:3.0f}%, worst {s["worst"]:5.1f})   '
              f'WRONG-ROBOT {m["mean"]:5.2f} +-{m["se"]:.2f} (1deg {m["within1"] * 100:3.0f}%, p90 {m["p90"]:5.2f}, worst {m["worst"]:5.1f})',
              flush=True)
    print('\nROBUSTNESS: mean final error (deg) when the real robot differs from the tuning conditions')
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
