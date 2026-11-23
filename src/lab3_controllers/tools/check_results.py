#!/usr/bin/env python3
"""Strict acceptance check of logged Gazebo runs. Exit 0 only if every check passes.

Usage: check_results.py YAW_CSV RESULT_DIR   (RESULT_DIR has {square,circle}_{stop_turn,polar,pure_pursuit}.csv)
Checks CSV validity (complete, finite, monotonic time, ends in DONE) and numeric thresholds.
"""
import csv
import math
import os
import sys

# thresholds (simulation, lowest-error configuration)
YAW_ERR_DEG = 0.5
FINAL_POS_CM = 1.5
FINAL_YAW_DEG = 1.5
# Gazebo is not deterministic: pure pursuit's corner pass varied 1.9-2.55 cm over repeated runs
SQUARE_PASS_CM = {'stop_turn': 1.5, 'polar': 3.0, 'pure_pursuit': 3.0}
CIRCLE_RAD_CM = {'stop_turn': 1.0, 'polar': 1.0, 'pure_pursuit': 1.5}
MAX_TIME = {'square': 80.0, 'circle': 220.0}
SQUARE = [(0.5, 0.0), (0.5, 0.5), (0.0, 0.5), (0.0, 0.0)]
R = 0.4
fails = 0


def check(name, ok, detail):
    global fails
    fails += 0 if ok else 1
    print(f'{"PASS" if ok else "FAIL"}: {name}  ({detail})')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(path):
    rows = list(csv.DictReader(open(path)))
    vals = [[float(r[k]) for k in ('t', 'x', 'y', 'yaw', 'gx', 'gy', 'gth')] for r in rows]
    return rows, vals


def yaw_checks(path):
    # validate_yaw.py writes t,yaw_deg,ref_deg; the last 2 s of each 9 s hold are the settled value
    rows = list(csv.DictReader(open(path)))
    refs = []
    for r in rows:
        ref = float(r['ref_deg'])
        if not refs or refs[-1][0] != ref:
            refs.append([ref, []])
        refs[-1][1].append((float(r['t']), float(r['yaw_deg'])))
    check('yaw: 5 references present', len(refs) == 5, f'{len(refs)} found')
    for ref, samples in refs:
        t_end = samples[-1][0]
        tail = [y for t, y in samples if t >= t_end - 1.0]
        err = max(abs((ref - y + 180) % 360 - 180) for y in tail)
        check(f'yaw: reference {ref:+.0f} deg', err <= YAW_ERR_DEG, f'worst error in last 1 s {err:.2f} deg (limit {YAW_ERR_DEG})')


def run_checks(shape, strat, path):
    if not os.path.exists(path):
        check(f'{shape} {strat}: log exists', False, path)
        return
    rows, v = load(path)
    name = f'{shape} {strat}'
    finite = all(math.isfinite(x) for row in v for x in row)
    mono = all(b[0] >= a[0] for a, b in zip(v, v[1:]))
    check(f'{name}: CSV valid', finite and mono and len(v) > 100 and rows[-1]['mode'] == 'DONE',
          f'{len(v)} rows, finite={finite}, monotonic={mono}, last mode={rows[-1]["mode"]}')
    t0, x0, y0, th0 = v[0][0], v[0][1], v[0][2], v[0][3]
    c, s = math.cos(th0), math.sin(th0)
    pts = [(c * (r[1] - x0) + s * (r[2] - y0), -s * (r[1] - x0) + c * (r[2] - y0)) for r in v]
    dur = v[-1][0] - 1.0
    check(f'{name}: completes in time', dur <= MAX_TIME[shape], f'{dur:.1f} s (limit {MAX_TIME[shape]})')
    gx, gy = (SQUARE[-1] if shape == 'square' else (0.0, 0.0))
    pe = math.hypot(pts[-1][0] - gx, pts[-1][1] - gy) * 100
    ye = abs(math.degrees(wrap(v[-1][6] - v[-1][3])))
    check(f'{name}: final position', pe <= FINAL_POS_CM, f'{pe:.2f} cm (limit {FINAL_POS_CM})')
    check(f'{name}: final heading', ye <= FINAL_YAW_DEG, f'{ye:.2f} deg (limit {FINAL_YAW_DEG})')
    if shape == 'square':
        miss = max(min(math.hypot(x - cx, y - cy) for x, y in pts) for cx, cy in SQUARE) * 100
        check(f'{name}: corners reached', miss <= SQUARE_PASS_CM[strat], f'worst pass {miss:.2f} cm (limit {SQUARE_PASS_CM[strat]})')
    else:
        rad = max(abs(math.hypot(x, y - R) - R) for x, y in pts) * 100
        check(f'{name}: radial error', rad <= CIRCLE_RAD_CM[strat], f'max {rad:.2f} cm (limit {CIRCLE_RAD_CM[strat]})')


def main(yaw_csv, d):
    yaw_checks(yaw_csv)
    for shape in ('square', 'circle'):
        for strat in ('stop_turn', 'polar', 'pure_pursuit'):
            run_checks(shape, strat, os.path.join(d, f'{shape}_{strat}.csv'))
    print(f'{fails} failed check(s)')
    sys.exit(1 if fails else 0)


main(sys.argv[1], sys.argv[2])
