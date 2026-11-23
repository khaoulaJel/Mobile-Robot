#!/usr/bin/env python3
"""Evaluate and plot motion logs recorded with `-p log_csv:=...`.

Usage: evaluate.py RESULTS_DIR   (expects square_<strategy>.csv and circle_<strategy>.csv)
Everything is expressed in the robot's start frame. Writes <shape>_comparison.png and
metrics.md into RESULTS_DIR.
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

STRATS = ['stop_turn', 'polar', 'pure_pursuit']
SQUARE = [(0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)]
CIRCLE_R = 0.4


def load(path):
    rows = list(csv.DictReader(open(path)))
    x0, y0, th0 = float(rows[0]['x']), float(rows[0]['y']), float(rows[0]['yaw'])
    c, s = math.cos(th0), math.sin(th0)
    pts = []
    for r in rows:
        dx, dy = float(r['x']) - x0, float(r['y']) - y0
        pts.append((float(r['t']), c * dx + s * dy, -s * dx + c * dy, float(r['yaw']) - th0))
    return pts


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main(d):
    out = ['| path | strategy | time [s] | final pos err [cm] | final yaw err [deg] | tracking |',
           '| --- | --- | --- | --- | --- | --- |']
    for shape in ('square', 'circle'):
        fig, ax = plt.subplots(figsize=(6, 6))
        for s in STRATS:
            f = os.path.join(d, f'{shape}_{s}.csv')
            if not os.path.exists(f):
                continue
            p = load(f)
            t_end = p[-1][0] - 1.0                    # minus the 1 s start delay
            xs, ys = [q[1] for q in p], [q[2] for q in p]
            if shape == 'square':
                gx, gy = SQUARE[-1]
                miss = max(min(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
                           for cx, cy in SQUARE)
                track = f'closest pass to corners: max {miss * 100:.1f} cm'
            else:
                gx, gy = 0.0, 0.0
                e = [abs(math.hypot(x, y - CIRCLE_R) - CIRCLE_R) for x, y in zip(xs, ys)]
                track = (f'radial err: max {max(e) * 100:.1f} cm, '
                         f'rms {math.sqrt(sum(v * v for v in e) / len(e)) * 100:.1f} cm')
            pe = math.hypot(xs[-1] - gx, ys[-1] - gy) * 100
            ye = abs(math.degrees(wrap(p[-1][3])))
            out.append(f'| {shape} | {s} | {t_end:.1f} | {pe:.1f} | {ye:.1f} | {track} |')
            ax.plot(xs, ys, label=f'{s} ({t_end:.0f} s)')
        if shape == 'square':
            ax.plot([0] + [c[0] for c in SQUARE], [0] + [c[1] for c in SQUARE], 'kx--', ms=9)
        else:
            a = [2 * math.pi * k / 200 for k in range(201)]
            ax.plot([CIRCLE_R * math.sin(v) for v in a], [CIRCLE_R * (1 - math.cos(v)) for v in a],
                    'k--', lw=1, label='ideal')
        ax.set_aspect('equal'); ax.grid(True); ax.legend()
        ax.set_xlabel('x [m] (start frame)'); ax.set_ylabel('y [m]')
        ax.set_title(f'Gazebo: {shape}')
        fig.tight_layout(); fig.savefig(os.path.join(d, f'{shape}_comparison.png'), dpi=120)
    open(os.path.join(d, 'metrics.md'), 'w').write('\n'.join(out) + '\n')
    print('\n'.join(out))


main(sys.argv[1])
