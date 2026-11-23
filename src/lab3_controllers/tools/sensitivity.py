#!/usr/bin/env python3
"""Parameter sensitivity / robustness study in the Python plant model (no ROS needed).

Every candidate is run over plant lags {0.10, 0.15, 0.25, 0.35, 0.50} s, with Gaussian odometry
noise (sigma 2 mm, 0.17 deg) and 3 seeds. Reported per candidate:
  done      completed runs / total   (a hang or timeout counts as a failure)
  cte_max   worst max cross-track error over all runs [cm]
  cte_rms   mean RMS cross-track error [cm]
  fin_mean  mean final position error [cm],  fin_max worst final position error [cm]
  yaw_max   worst final heading error [deg]
  T         mean completion time [s]
Usage: python3 sensitivity.py [out.md]
"""
import math
import sys

sys.path.insert(0, __file__.rsplit('tools', 1)[0] + 'tools')
import sim_compare as s  # noqa: E402

LAGS = (0.10, 0.15, 0.25, 0.35, 0.50)
SEEDS = (1, 2, 3)
NOISE = dict(noise_p=0.002, noise_y=0.003)
CIRCLE = s.circle()
PATHS = {'square': (s.SQUARE, None), 'circle': (CIRCLE, 0.4)}
OUT = []


def say(line=''):
    print(line, flush=True)
    OUT.append(line)


def study(strategy, shape, tmax=120.0, **over):
    path, R = PATHS[shape]
    n = ok = 0
    ctem, ctes, fin, yaw, T, pas = [], [], [], [], [], []
    for tau in LAGS:
        for sd in SEEDS:
            r = s.run(strategy, path, tau=tau, tmax=tmax, seed=sd, **NOISE, **over)
            n += 1
            if r['ok']:
                ok += 1
                cm, cr = s.cross_track(r, path, R)
                ctem.append(cm); ctes.append(cr)
                if R is None:
                    pas.append(s.metrics(r, path)['pass_max_cm'])
                fin.append(r['pos_err'] * 100); yaw.append(r['yaw_err']); T.append(r['t'])
    if not ok:
        return dict(done=f'0/{n}')
    return dict(done=f'{ok}/{n}', cte_max=max(ctem), cte_rms=sum(ctes) / len(ctes),
                fin_mean=sum(fin) / len(fin), fin_max=max(fin), yaw_max=max(yaw),
                T=sum(T) / len(T), pass_max=max(pas) if pas else float('nan'))


def row(label, d):
    if 'cte_max' not in d:
        return f'| {label} | {d["done"]} | - | - | - | - | - | - | - |'
    pm = '-' if d['pass_max'] != d['pass_max'] else f'{d["pass_max"]:.2f}'
    return (f'| {label} | {d["done"]} | {pm} | {d["cte_max"]:.2f} | {d["cte_rms"]:.2f} | {d["fin_mean"]:.2f} | '
            f'{d["fin_max"]:.2f} | {d["yaw_max"]:.2f} | {d["T"]:.1f} |')


HEAD = ('| setting | done | pass_max [cm] | cte_max [cm] | cte_rms [cm] | fin_mean [cm] | fin_max [cm] | yaw_max [deg] | T [s] |\n'
        '| --- | --- | --- | --- | --- | --- | --- | --- | --- |')


def yaw_study(kp, tol):
    """90 deg step of the standalone yaw controller over all lags/seeds (noise 0.17 deg)."""
    import random
    sys.path.insert(0, __file__.rsplit('tools', 1)[0])
    from lab3_controllers.common import slew, wrap_angle, yaw_p_control
    st, fe, ov = [], [], []
    for tau in LAGS:
        for sd in SEEDS:
            rng = random.Random(sd); th = w = cw = 0.0; ref = math.radians(90); t = 0.0
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
    say('# Sensitivity study (plant model, lags 0.10-0.50 s, odometry noise, 3 seeds)\n')
    say('## A. pure_pursuit look-ahead\n')
    for shape in ('square', 'circle'):
        say(f'**{shape}**\n'); say(HEAD)
        for L in (0.08, 0.10, 0.12, 0.15, 0.20, 0.25):
            say(row(f'lookahead {L:.2f} m', study('pure_pursuit', shape, lookahead=L)))
        say()
    say('## B. polar blend radius (other gains fixed)\n')
    for shape in ('square', 'circle'):
        say(f'**{shape}**\n'); say(HEAD)
        for b in (0.02, 0.03, 0.05, 0.08, 0.10):
            say(row(f'blend {b:.2f} m', study('polar', shape, blend=b)))
        say()
    say('## C. stop_turn stopping tolerances (square)\n')
    say(HEAD)
    for pt in (0.005, 0.0075, 0.01, 0.015):
        for yt in (0.003, 0.005, 0.0075, 0.01):
            say(row(f'pos {pt * 100:.2f} cm, yaw {yt:.4f} rad', study('stop_turn', 'square', ptol=pt, ytol=yt)))
    say()
    say('## D. stop_turn linear gain kv and alignment gain k_yaw (square, tolerances 1 cm / 0.01 rad)')
    say()
    say(HEAD)
    for kv in (0.5, 0.65, 0.8, 1.0, 1.8):
        for ky in (1.0, 1.5, 2.5):
            say(row(f'kv {kv}, k_yaw {ky}', study('stop_turn', 'square', kv=kv, k_yaw=ky)))
    say()
    say('## E. standalone yaw controller, 90 deg step, worst case over lags and seeds')
    say()
    say('| kp | tolerance [rad] | settle to 0.5 deg [s] | final error [deg] | overshoot [deg] |')
    say('| --- | --- | --- | --- | --- |')
    for kp in (0.6, 0.8, 1.0, 1.5, 2.5):
        for tol in (0.005, 0.01):
            a, b, c = yaw_study(kp, tol)
            say(f'| {kp} | {tol} | {a:.1f} | {b:.2f} | {c:.2f} |')
    say()
    if len(sys.argv) > 1:
        open(sys.argv[1], 'w', encoding='utf-8').write('\n'.join(OUT) + '\n')


if __name__ == '__main__':
    main()
