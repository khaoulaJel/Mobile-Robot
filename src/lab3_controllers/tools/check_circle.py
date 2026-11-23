#!/usr/bin/env python3
"""Strict check of a circle run from a controller CSV (motion_controller log or symbolic_controller log).

motion_controller CSV: t,x,y,yaw,gx,gy,gth,idx,mode,v,w   (world frame)
symbolic_controller CSV: t,x,y,yaw,radial_err,goal,cell,v,w,mode   (already in the circle frame)
The circle has radius R (1 m diameter -> 0.5), starts tangentially at the first logged pose, counter-clockwise.

Usage: check_circle.py run.csv --radius 0.5 --max-radial 0.015 [--max-final 0.015]
Exit code 0 only if the run completed and every limit is met.
"""
import argparse
import csv
import math
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--radius', type=float, default=0.5)
    ap.add_argument('--max-radial', type=float, required=True, help='m, maximum distance from the circle')
    ap.add_argument('--max-final', type=float, default=None, help='m, final distance from the start point')
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    if not rows:
        print('FAIL: empty log')
        return 1
    last = rows[-1]
    symbolic = 'radial_err' in rows[0]
    if last['mode'] != 'DONE':
        print(f"FAIL: run did not complete (last mode {last['mode']})")
        return 1
    if symbolic:
        radial = [abs(float(r['radial_err'])) for r in rows]
        final = math.hypot(float(last['x']), float(last['y']))
    else:
        x0, y0, th0 = float(rows[0]['x']), float(rows[0]['y']), float(rows[0]['yaw'])
        c, s = math.cos(th0), math.sin(th0)
        pts = [((float(r['x']) - x0) * c + (float(r['y']) - y0) * s,
                -(float(r['x']) - x0) * s + (float(r['y']) - y0) * c) for r in rows]
        radial = [abs(math.hypot(px, py - a.radius) - a.radius) for px, py in pts]
        final = math.hypot(*pts[-1])
    mx = max(radial)
    rms = math.sqrt(sum(v * v for v in radial) / len(radial))
    t = float(last['t'])
    print(f'circle R={a.radius} m: completed in {t:.1f} s; max distance from the circle {mx * 100:.2f} cm '
          f'(rms {rms * 100:.2f} cm); final distance from the start {final * 100:.2f} cm')
    ok = mx <= a.max_radial and (a.max_final is None or final <= a.max_final)
    lim = f'limits: max distance {a.max_radial * 100:.1f} cm' + \
        ('' if a.max_final is None else f', final {a.max_final * 100:.1f} cm')
    print(lim, '->', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
