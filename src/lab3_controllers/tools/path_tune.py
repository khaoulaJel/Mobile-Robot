#!/usr/bin/env python3
"""Tune every path controller on the model of OUR robot (config/robot_model.json), then judge it on shapes and robot variants
it was NOT tuned on. Writes config/path_robot.json (used by robot_path.py) and path_tuning.json (the evidence).

    python3 path_tune.py [--random 160] [--refine 120] [--workers 14] [--report <json>]      (about 5 minutes)

Run it again after "robot_model.py fit": then the driving numbers are measured instead of predicted.
Training: shapes square, circle_r40, triangle, eight; judged on held-out shapes circle_r50, hexagon, rectangle, square_cw,
stadium, with new variants of the model. A run that does not finish, or strays 25 cm, is a failure.
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
import paths  # noqa: E402
import robot_model as rm  # noqa: E402

TRAIN = ('square', 'circle_r40', 'triangle', 'eight')
TEST = ('circle_r50', 'hexagon', 'rectangle', 'square_cw', 'stadium')

COMMON = dict(ds=(0.03, 0.12), min_linear_speed=(0.0, 0.07), min_angular_speed=(0.0, 0.5), pos_tolerance=(0.01, 0.025),
              yaw_tolerance=(0.012, 0.045), max_linear_speed=(0.07, 0.15), max_angular_speed=(0.4, 1.0))
SPACE = {
    'stop_turn': dict(COMMON, kv=(0.5, 1.8), kw=(1.2, 4.5), k_yaw=(0.8, 4.0), turn_in_place_deg=(8.0, 45.0), lead_v=(0.0, 0.25),
                      lead_w=(0.0, 0.15)),
    'polar': dict(COMMON, k_rho=(0.6, 2.2), k_alpha=(2.3, 6.0), k_beta=(-2.5, -0.1), blend_radius=(0.02, 0.18), lead_v=(0.0, 0.25),
                  lead_w=(0.0, 0.15)),
    'pure_pursuit': dict(COMMON, lookahead=(0.04, 0.2), k_yaw=(0.8, 4.0), lead_v=(0.0, 0.25), lead_w=(0.0, 0.15)),
    'p_servo': dict(ds=(0.08, 0.2), steer_gain=(0.5, 4.0), face_tolerance=(0.05, 0.2), position_tolerance=(0.012, 0.03)),
}
LINEAR_SPACE = dict(u_min=(0.03, 0.07), tol=(0.004, 0.012), pulse_share=(0.5, 1.0), pulse_below=(0.015, 0.06), lead=(0.05, 0.6),
                    settle_min=(0.4, 0.8), kp=(0.5, 1.6), u_max=(0.08, 0.16), dead_same=(0.05, 0.16))


def clip_to(space, values):
    return {k: (min(space[k][1], max(space[k][0], v)) if k in space else v) for k, v in values.items()}


def sample(rng, name, base):
    s = json.loads(json.dumps(base))
    for k, (lo, hi) in SPACE[name].items():
        s[k] = round(rng.uniform(lo, hi), 4)
    if name == 'p_servo':
        for k, (lo, hi) in LINEAR_SPACE.items():
            s['linear'][k] = round(rng.uniform(lo, hi), 4)
    return s


def perturb(rng, name, base, scale):
    s = json.loads(json.dumps(base))
    pairs = [(s, k, v) for k, v in SPACE[name].items()]
    if name == 'p_servo':
        pairs += [(s['linear'], k, v) for k, v in LINEAR_SPACE.items()]
    for target, k, (lo, hi) in pairs:
        if rng.random() < 0.35:
            target[k] = round(min(hi, max(lo, target.get(k, (lo + hi) / 2) + rng.gauss(0, scale * (hi - lo)))), 4)
    return s


def variants(model, n, seed, spread):
    rng = random.Random(seed)
    return [model] + [rm.vary(model, rng, spread) for _ in range(n - 1)]


def run_set(name, s, models, shapes, seed):
    out = []
    for j, m in enumerate(models):
        for i, shape in enumerate(shapes):
            try:
                r = paths.simulate(name, shape, s, m, seed=seed + 31 * j + i)
            except ValueError:
                return None
            r.pop('xs'), r.pop('ys')
            r.update(shape=shape, variant=j, ideal_len=paths.length_of(paths.shape(shape)[1]))
            out.append(r)
    return out


def cost(runs):
    """cm of path error (rms plus a share of the worst), the end position, the end heading, a little for time; failure is dear."""
    total = 0.0
    for r in runs:
        if not r['finished']:
            total += 40.0 + 20.0 * (1.0 - r['covered'])
            continue
        total += r['rms_cm'] + 0.35 * r['max_cm'] + 0.35 * r['end_cm'] + 0.12 * r['end_deg'] + 0.25 * r['time'] / (8.0 * r['ideal_len'])
    return total / len(runs)


def stats(runs):
    ok = [r for r in runs if r['finished']]
    mean = lambda key: sum(r[key] for r in ok) / len(ok) if ok else float('nan')  # noqa: E731
    return dict(runs=len(runs), finished=len(ok), rms_cm=mean('rms_cm'), max_cm=mean('max_cm'),
                worst_cm=max((r['max_cm'] for r in ok), default=float('nan')), end_cm=mean('end_cm'), end_deg=mean('end_deg'),
                time_s=mean('time'), cost=cost(runs))


def _work(job):
    name, s, models, shapes, seed = job
    runs = run_set(name, s, models, shapes, seed)
    return (1e9 if runs is None else cost(runs)), s


def tune(name, model, pool, n_random, n_refine, seed=5):
    rng = random.Random(seed + len(name))
    train = variants(model, 3, 100, 0.15)
    start = paths.FALLBACK[name]
    shapes = TRAIN
    jobs = [(name, start, train, shapes, 1)] + [(name, sample(rng, name, start), train, shapes, 1) for _ in range(n_random)]
    ranked = sorted(pool.map(_work, jobs), key=lambda x: x[0])
    for scale in (0.15, 0.07):
        parents = [s for _, s in ranked[:6]]
        jobs = [(name, perturb(rng, name, parents[i % len(parents)], scale), train, shapes, 1) for i in range(n_refine // 2)]
        ranked = sorted(ranked[:12] + pool.map(_work, jobs), key=lambda x: x[0])
    # choose among the best on a validation set (training shapes, NEW robot variants, new noise)
    check = variants(model, 6, 200, 0.2)[1:]
    jobs = [(name, s, check, TRAIN, 900) for _, s in ranked[:8]]
    scored = pool.map(_work, jobs)
    return min(scored, key=lambda x: x[0])[1]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--random', type=int, default=160)
    ap.add_argument('--refine', type=int, default=120)
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 4) - 2))
    ap.add_argument('--out', default=paths.SETTINGS_FILE)
    ap.add_argument('--report', default=os.path.join(HERE, '..', '..', '..', '..', 'results', 'real_robot', 'path_tuning.json'))
    ap.add_argument('--only', default='stop_turn,polar,pure_pursuit,p_servo')
    a = ap.parse_args(argv)
    model = rm.load()
    print(f'robot model: turning {"MEASURED" if model["measured"]["turn"] else "predicted"}, '
          f'driving {"MEASURED" if model["measured"]["drive"] else "PREDICTED (not measured yet)"}')
    chosen = paths.settings(a.out)
    evidence = {'model': model, 'train_shapes': TRAIN, 'test_shapes': TEST, 'results': {}}
    if os.path.exists(a.report):
        with open(a.report) as f:
            evidence['results'] = json.load(f).get('results', {})
    test = variants(model, 6, 777, 0.2)
    with Pool(a.workers) as pool:
        for name in a.only.split(','):
            best = tune(name, model, pool, a.random, a.refine)
            before = stats(run_set(name, paths.FALLBACK[name], test, TEST, 4000))
            after = stats(run_set(name, best, test, TEST, 4000))
            if (after['finished'], -after['cost']) < (before['finished'], -before['cost']):
                best, after = paths.FALLBACK[name], before              # never ship something worse than the starting point
            chosen[name] = best
            evidence['results'][name] = {'settings': best, 'held_out_before': before, 'held_out_after': after}
            for label, s in (('untuned', before), ('TUNED', after)):
                print(f'{name:13} {label:8} held-out: finished {s["finished"]}/{s["runs"]}  path error rms {s["rms_cm"]:.2f} cm, '
                      f'max {s["max_cm"]:.2f} (worst {s["worst_cm"]:.2f}) cm | end {s["end_cm"]:.2f} cm {s["end_deg"]:.2f} deg | '
                      f'{s["time_s"]:.0f} s', flush=True)
            with open(a.out, 'w', encoding='utf-8') as f:
                json.dump(chosen, f, indent=2)
                f.write('\n')
            os.makedirs(os.path.dirname(a.report), exist_ok=True)
            with open(a.report, 'w') as f:
                json.dump(evidence, f, indent=1)
    print(f'saved {os.path.abspath(a.out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
