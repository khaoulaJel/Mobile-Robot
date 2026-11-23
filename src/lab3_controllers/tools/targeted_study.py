#!/usr/bin/env python3
"""Final targeted, paired experiments (same lags and seeds for every candidate). No ROS needed.

F  stop_turn linear gain kv around 0.8
G  standalone yaw controller gain kp around 1.0 (90 and 170 deg steps)
H  speed and acceleration limits of stop_turn
I  wide repeatability of the default: 10 seeds x 5 lags (50 runs), mean / std / worst
Usage: python3 targeted_study.py [out.md]
"""
import math
import random
import statistics
import sys

sys.path.insert(0, __file__.rsplit('targeted_study.py', 1)[0])
import sensitivity as z  # noqa: E402
import sim_compare as s  # noqa: E402

OUT = []


def say(x=''):
    print(x, flush=True)
    OUT.append(x)


def yaw_step(kp, step_deg, tol=0.005):
    sys.path.insert(0, z.__file__.rsplit('tools', 1)[0])
    from lab3_controllers.common import slew, wrap_angle, yaw_p_control
    st, fe, ov = [], [], []
    for tau in z.LAGS:
        for sd in z.SEEDS:
            rng = random.Random(sd); th = w = cw = 0.0; ref = math.radians(step_deg); t = 0.0
            settle = None; over = 0.0
            while t < 40:
                cmd, _ = yaw_p_control(ref, th + rng.gauss(0, 0.003), kp, 1.0, 0.0, tol)
                cw = slew(cw, cmd, 3.0, 0.05); w += 0.05 / (tau + 0.05) * (cw - w)
                th = wrap_angle(th + w * 0.05); t += 0.05
                e = wrap_angle(ref - th)
                if e < 0: over = max(over, -e)
                if abs(e) > math.radians(0.5): settle = None
                elif settle is None: settle = t
            st.append(settle if settle is not None else 99.0); fe.append(abs(math.degrees(e))); ov.append(math.degrees(over))
    return max(st), max(fe), max(ov)


def main():
    say('# Targeted paired experiments (lags 0.10-0.50 s, odometry noise, same 3 seeds for every candidate)\n')
    say('## F. stop_turn kv (square)\n'); say(z.HEAD)
    for kv in (0.6, 0.7, 0.8, 0.9, 1.0):
        say(z.row(f'kv {kv}', z.study('stop_turn', 'square', kv=kv)))
    say()
    say('## G. yaw controller kp (worst case over lags and seeds; tolerance 0.005 rad)\n')
    say('| kp | 90 deg: settle [s] | overshoot [deg] | 170 deg: settle [s] | overshoot [deg] | final err [deg] |')
    say('| --- | --- | --- | --- | --- | --- |')
    for kp in (0.8, 0.9, 1.0, 1.1, 1.2, 1.5):
        a = yaw_step(kp, 90); b = yaw_step(kp, 170)
        say(f'| {kp} | {a[0]:.1f} | {a[2]:.2f} | {b[0]:.1f} | {b[2]:.2f} | {max(a[1], b[1]):.2f} |')
    say()
    say('## H. stop_turn speed and acceleration limits (square)\n'); say(z.HEAD)
    for vmax in (0.10, 0.15, 0.20):
        for acc_v in (0.2, 0.4, 0.8):
            say(z.row(f'vmax {vmax} m/s, accel {acc_v} m/s^2', z.study('stop_turn', 'square', vmax=vmax, acc_v=acc_v)))
    say()
    say('## I. wide repeatability of the default stop_turn square (10 seeds x 5 lags = 50 runs)\n')
    fin, yaw, T, cte = [], [], [], []
    done = 0
    for tau in z.LAGS:
        for sd in range(101, 111):
            r = s.run('stop_turn', s.SQUARE, tau=tau, seed=sd, tmax=150.0, **z.NOISE)
            if r['ok']:
                done += 1
                fin.append(r['pos_err'] * 100); yaw.append(r['yaw_err']); T.append(r['t'])
                cte.append(s.cross_track(r, s.SQUARE)[0])
    f = lambda v: f'{statistics.mean(v):.2f} +/- {statistics.pstdev(v):.2f} (worst {max(v):.2f})'
    say(f'- completed: {done}/50')
    say(f'- final position error [cm]: {f(fin)}')
    say(f'- final heading error [deg]: {f(yaw)}')
    say(f'- max cross-track error [cm]: {f(cte)}')
    say(f'- completion time [s]: {f(T)}')
    if len(sys.argv) > 1:
        open(sys.argv[1], 'w', encoding='utf-8').write('\n'.join(OUT) + '\n')


main()
