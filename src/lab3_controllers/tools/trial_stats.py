#!/usr/bin/env python3
"""Repeatability statistics over several logged runs of the same path.

Usage: trial_stats.py run1.csv run2.csv ...   (logs written with `-p log_csv:=...`)
Per run: duration, final position error and final heading error against the last goal in the
log (gx, gy, gth); then mean and standard deviation across runs. NOTE: this uses odometry as
the reference. For the report also measure the real final pose with a tape and compare.
"""
import csv
import math
import statistics
import sys


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def one(path):
    rows = list(csv.DictReader(open(path)))
    last = rows[-1]
    pe = math.hypot(float(last['gx']) - float(last['x']), float(last['gy']) - float(last['y']))
    ye = abs(math.degrees(wrap(float(last['gth']) - float(last['yaw']))))
    return float(last['t']) - 1.0, pe * 100.0, ye


def ms(v):
    return f'{statistics.mean(v):.2f} +/- {statistics.pstdev(v):.2f}'


if __name__ == '__main__':
    res = [one(p) for p in sys.argv[1:]]
    print('run, time [s], pos err [cm], yaw err [deg]')
    for p, r in zip(sys.argv[1:], res):
        print(f'{p}, {r[0]:.1f}, {r[1]:.1f}, {r[2]:.1f}')
    if len(res) > 1:
        print(f'mean +/- std over {len(res)} runs: time {ms([r[0] for r in res])} s, '
              f'pos {ms([r[1] for r in res])} cm, yaw {ms([r[2] for r in res])} deg')
