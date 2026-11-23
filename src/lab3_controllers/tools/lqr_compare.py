#!/usr/bin/env python3
"""Preview LQR vs the lab_robot stop_turn preset on the identified plant, plus robustness of the LQR."""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plant_models as pm  # noqa: E402
import preview_lqr as pl  # noqa: E402
import robustness as rb  # noqa: E402
import shape_bench as sb  # noqa: E402
import tune_lab_plant as tp  # noqa: E402
import tune_preview as tv  # noqa: E402

BEST = dict(v_cruise=0.06, a_lat=0.18, a_dec=0.03, horizon=60, q_e=1e5, q_th=100.0, r=10.0)
SHAPES = ['circle R0.4', 'circle R0.5 (lab polar)', 'circle R0.2 (tight)', 'ellipse 0.6x0.3', 'figure-eight',
          'sine wave', 'stadium']
LAB = dict(tp.FIXED)
LAB.update(rb.CANDIDATE)


def lqr(args):
    shape, plant_spec, noise, seed = args
    return pl.run(shape, BEST, sb.make_plant(plant_spec, 0.15), noise=noise, seed=seed)


def stop(args):
    shape, plant_spec, noise, seed = args
    return sb.run_all(0.1, 0.15, ['stop_turn'], over={'stop_turn': LAB}, plant=plant_spec, only=[shape],
                      settle=12.0, noise=noise, seed=seed)[0]


if __name__ == '__main__':
    with ProcessPoolExecutor(18) as ex:
        a = list(ex.map(lqr, [(s, 'identified', (0, 0), 0) for s in SHAPES]))
        b = list(ex.map(stop, [(s, 'identified', (0, 0), 0) for s in SHAPES]))
        print(f"{'shape':24s} | {'LQR time':>8s} {'xtrk':>5s} {'rms':>5s} {'pos':>5s} {'yaw':>5s} | "
              f"{'stop time':>9s} {'xtrk':>5s} {'rms':>5s} {'pos':>5s} {'yaw':>5s}")
        for s, x, y in zip(SHAPES, a, b):
            print(f"{s:24s} | {x['t']:8.1f} {x['max']:5.2f} {x['rms']:5.2f} {x['fin']:5.2f} {x['yaw']:5.2f} | "
                  f"{y['t']:9.1f} {y['max']:5.2f} {y['rms']:5.2f} {y['fin']:5.2f} {y['yaw']:5.2f}")
        print('\nrobustness of the LQR (7 shapes each; the controller keeps the NOMINAL model)')
        conds = [('no noise', 'identified', (0, 0), (0,)), ('noise 0.5mm/0.03deg', 'identified', (5e-4, 5e-4), (1, 2, 3)),
                 ('noise 2mm/0.17deg', 'identified', (2e-3, 3e-3), (1, 2, 3)),
                 ('gain -15%', 'identified@0.85,0.85', (0, 0), (0,)), ('gain +15%', 'identified@1.15,1.15', (0, 0), (0,)),
                 ('v-15% w+15%', 'identified@0.85,1.15', (0, 0), (0,)), ('v+15% w-15%', 'identified@1.15,0.85', (0, 0), (0,)),
                 ('latency +0.1 s', 'identified@1,1,2', (0, 0), (0,)), ('latency +0.2 s', 'identified@1,1,4', (0, 0), (0,))]
        print(f"{'condition':22s} {'runs':>4s} {'done':>6s} {'max xtrk':>8s} {'max pos':>7s} {'max yaw':>7s} {'score':>6s}")
        for name, spec, noise, seeds in conds:
            rows = list(ex.map(lqr, [(s, spec, noise, sd) for s in SHAPES for sd in seeds]))
            n = len(rows)
            print(f"{name:22s} {n:4d} {sum(r['ok'] for r in rows):3d}/{n:<2d} {max(r['max'] for r in rows):8.2f} "
                  f"{max(r['fin'] for r in rows):7.2f} {max(r['yaw'] for r in rows):7.2f} "
                  f"{sum(tv.score(r) for r in rows) / n:6.2f}")
