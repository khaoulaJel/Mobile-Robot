#!/usr/bin/env python3
"""Pure-Python comparison of the motion strategies on a lagged unicycle (no ROS needed).

Plant: commands go through acceleration limits then a first-order lag (tau), roughly what
the Gazebo drive plugin does. Usage: python3 sim_compare.py
"""
import math
import sys

sys.path.insert(0, __file__.rsplit('tools', 1)[0])
from lab3_controllers.common import circle_waypoints, slew, wrap_angle  # noqa: E402
from lab3_controllers.tracker import DONE, PathTracker  # noqa: E402

DT = 0.05
SQUARE = [(0.5, 0.0, 0.0), (0.5, 0.5, math.pi / 2), (0.0, 0.5, math.pi), (0.0, 0.0, 0.0)]


def circle(R=0.4, n=24):
    f = circle_waypoints(R, n)
    return [tuple(f[i:i + 3]) for i in range(0, len(f), 3)]


DEFAULTS = dict(
    stop_turn=dict(kv=0.8, kw=3.0, k_yaw=1.0, tip=35.0, vmax=0.2, ptol=0.01, ytol=0.01),
    polar=dict(k_rho=1.5, k_alpha=4.0, k_beta=-2.0, blend=0.03, decel=0.4, k_yaw=1.5, vmax=0.2, ptol=0.01, ytol=0.01),
    pure_pursuit=dict(lookahead=0.12, decel=0.4, k_yaw=1.5, vmax=0.2, ptol=0.01, ytol=0.01),
)


_KEYMAP = dict(tip='turn_in_place_deg', vmax='max_linear_speed', ptol='pos_tolerance',
               ytol='yaw_tolerance', blend='blend_radius')


def tracker_params(strategy, p):
    """Translate this module's short parameter names to PathTracker parameters."""
    out = dict(strategy=strategy)
    for k, v in p.items():
        if k in ('acc_v', 'acc_w'):
            continue
        out[_KEYMAP.get(k, k)] = v
    return out


def run(strategy, path, tau=0.15, tmax=200.0, noise_p=0.0, noise_y=0.0, seed=0, plant=None,
        settle=0.0, odom_hold=1, **over):
    """noise_p [m], noise_y [rad]: Gaussian odometry noise seen by the controller (not by the metrics).
    plant: optional object with step(cmd_v, cmd_w) -> (v, w) (tools/plant_models.py); default is the
    first-order lag with time constant tau. The control logic is lab3_controllers.tracker.PathTracker,
    the same code the ROS node runs. settle: seconds to keep simulating with a zero command after the
    controller reports DONE, so the final error is measured at rest (matters for slow plants); the
    returned time t is the time at which the controller reported DONE. odom_hold=k: the controller sees a new
    pose only every k-th tick (Gazebo's /odom updates slower than the 20 Hz control loop)."""
    import random
    rng = random.Random(seed)
    p = dict(DEFAULTS.get(strategy, {}))
    p.update(over)
    tr = PathTracker(path, tracker_params(strategy, p))
    x = y = th = 0.0
    v = w = cmd_v = cmd_w = 0.0
    t = 0.0
    k_tick = 0
    meas = (x, y, th)
    tr.start((x, y, th))
    xs, ys = [x], [y]
    while t < tmax and tr.mode != DONE:
        if k_tick % odom_hold == 0:
            meas = (x + rng.gauss(0, noise_p), y + rng.gauss(0, noise_p), th + rng.gauss(0, noise_y))
        k_tick += 1
        cv, cw = tr.step(meas, DT)
        cmd_v = slew(cmd_v, cv, p.get('acc_v', 0.4), DT)
        cmd_w = slew(cmd_w, cw, p.get('acc_w', 3.0), DT)
        if plant is not None:
            v, w = plant.step(cmd_v, cmd_w)
        else:
            a = DT / (tau + DT)
            v += a * (cmd_v - v)
            w += a * (cmd_w - w)
        th = wrap_angle(th + w * DT)
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        xs.append(x); ys.append(y)
        t += DT
    t_done, ok = t, tr.mode == DONE
    for _ in range(int(settle / DT)):         # coast to rest with zero command
        cmd_v = slew(cmd_v, 0.0, p.get('acc_v', 0.4), DT)
        cmd_w = slew(cmd_w, 0.0, p.get('acc_w', 3.0), DT)
        if plant is not None:
            v, w = plant.step(cmd_v, cmd_w)
        else:
            a = DT / (tau + DT)
            v += a * (cmd_v - v)
            w += a * (cmd_w - w)
        th = wrap_angle(th + w * DT)
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        xs.append(x); ys.append(y)
    return dict(t=t_done, ok=ok, xs=xs, ys=ys,
                pos_err=math.hypot(path[-1][0] - x, path[-1][1] - y),
                yaw_err=math.degrees(abs(wrap_angle(path[-1][2] - th))))


def cross_track(r, path, R=None):
    """(max, rms) cross-track error [cm] of the TRUE path against the ideal path."""
    if R is not None:
        e = [abs(math.hypot(x, y - R) - R) for x, y in zip(r['xs'], r['ys'])]
    else:
        pts = [(0.0, 0.0)] + [(a, b) for a, b, _ in path]
        e = []
        for x, y in zip(r['xs'], r['ys']):
            best = 1e9
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                L2 = (bx - ax) ** 2 + (by - ay) ** 2
                u = max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / L2))
                best = min(best, math.hypot(x - ax - u * (bx - ax), y - ay - u * (by - ay)))
            e.append(best)
    return 100 * max(e), 100 * math.sqrt(sum(v * v for v in e) / len(e))


def metrics(r, path, R=None):
    """Corner/waypoint closest approach, plus radial error for a circle."""
    out = {}
    if R is None:
        out['pass_max_cm'] = 100 * max(min(math.hypot(x - gx, y - gy) for x, y in zip(r['xs'], r['ys']))
                                       for gx, gy, _ in path)
    else:
        e = [abs(math.hypot(x, y - R) - R) for x, y in zip(r['xs'], r['ys'])]
        out['rad_max_cm'] = 100 * max(e)
        out['rad_rms_cm'] = 100 * math.sqrt(sum(v * v for v in e) / len(e))
    return out


if __name__ == '__main__':
    for name, path, R in [('square', SQUARE, None), ('circle R=0.4', circle(), 0.4)]:
        print(f'--- {name}')
        for s in DEFAULTS:
            r = run(s, path)
            m = ', '.join(f'{k}={v:.1f}' for k, v in metrics(r, path, R).items())
            print(f"{s:13s} t={r['t']:6.1f}s ok={r['ok']} pos={r['pos_err']*100:4.1f}cm "
                  f"yaw={r['yaw_err']:4.1f}deg  {m}")
