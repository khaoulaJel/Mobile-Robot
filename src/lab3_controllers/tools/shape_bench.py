#!/usr/bin/env python3
"""Path-shape benchmark in the plant model (no ROS): how does each strategy do on ANY shape?

For every shape the ideal path is a dense polyline; the controller only gets waypoints
sampled from it (corners for polygons, every `ds` metres for curves, tangent headings).
Reported error is the cross-track distance of the TRUE trajectory to the IDEAL path, so
chord/corner cutting counts against the controller (as a lab "path error" would).

Usage: python3 shape_bench.py [--ds 0.1] [--tau 0.15] [--strategies stop_turn,polar,pure_pursuit]
"""
import argparse
import math
import sys

sys.path.insert(0, __file__.rsplit('tools', 1)[0])
sys.path.insert(0, __file__.rsplit('\\', 1)[0] if '\\' in __file__ else __file__.rsplit('/', 1)[0])
from lab3_controllers.common import wrap_angle  # noqa: E402
import sim_compare as sc  # noqa: E402
import plant_models as pm  # noqa: E402


def make_plant(kind, tau):
    """lag | identified | identified_nofilter | lag_filter (fresh instance per run)."""
    if kind == 'lag':
        return None
    if kind == 'lag_filter':
        return pm.LagPlant(tau, use_filter=True)
    if kind.startswith('identified@'):            # identified@gv,gw[,delay] : gain (calibration) error, latency
        t = kind.split('@')[1].split(',')
        return pm.IdentifiedPlant(use_filter=True, gain=(float(t[0]), float(t[1])),
                                  delay=int(t[2]) if len(t) > 2 else 0)
    if kind.startswith('identified+lag'):          # identified+lag0.15 : extra first-order lag after the plant
        return pm.Chain(pm.IdentifiedPlant(use_filter=True), float(kind[len('identified+lag'):]))
    if kind == 'identified':
        return pm.IdentifiedPlant(use_filter=True)
    if kind == 'identified_nofilter':
        return pm.IdentifiedPlant(use_filter=False)
    raise ValueError(kind)



# ---- shape library: (ideal dense polyline, waypoint corner indices or None for "sample curve") ----
def _poly(corners, closed=True):
    pts = list(corners) + ([corners[0]] if closed else [])
    return pts


def _dense(pts, step=0.005):
    out = [pts[0]]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        out += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(1, n + 1)]
    return out


def _curve(fn, t0, t1, n=1500):
    return [fn(t0 + (t1 - t0) * k / n) for k in range(n + 1)]


def _regular(n, R):
    return _poly([(R * math.cos(2 * math.pi * k / n), R * math.sin(2 * math.pi * k / n)) for k in range(n)])


def _star():
    pts = []
    for k in range(5):
        a = 2 * math.pi * (2 * k) / 5
        pts.append((0.4 * math.cos(a), 0.4 * math.sin(a)))
    return _poly(pts)


def _normalize(pts, ccw_start=True):
    """Translate so the path starts at the origin and rotate so the first segment points along +x."""
    x0, y0 = pts[0]
    j = next(i for i, (x, y) in enumerate(pts) if math.hypot(x - x0, y - y0) > 1e-3)
    a = math.atan2(pts[j][1] - y0, pts[j][0] - x0)
    c, s = math.cos(-a), math.sin(-a)
    return [((x - x0) * c - (y - y0) * s, (x - x0) * s + (y - y0) * c) for x, y in pts]


def shapes():
    S = {}
    S['square 0.5'] = ('poly', _poly([(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)]))
    S['square 0.5 CW'] = ('poly', _poly([(0, 0), (0.5, 0), (0.5, -0.5), (0, -0.5)]))
    S['rectangle 0.8x0.3'] = ('poly', _poly([(0, 0), (0.8, 0), (0.8, 0.3), (0, 0.3)]))
    S['triangle'] = ('poly', _poly([(0, 0), (0.6, 0), (0.3, 0.52)]))
    S['hexagon R0.3'] = ('poly', _normalize(_regular(6, 0.3)))
    S['star (36 deg tips)'] = ('poly', _normalize(_star()))
    S['circle R0.4'] = ('curve', _normalize(_curve(lambda t: (0.4 * math.sin(t), 0.4 * (1 - math.cos(t))), 0, 2 * math.pi)))
    S['circle R0.5 (lab polar)'] = ('curve', _normalize(_curve(lambda t: (0.5 * math.sin(t), 0.5 * (1 - math.cos(t))), 0, 2 * math.pi)))
    S['circle R0.2 (tight)'] = ('curve', _normalize(_curve(lambda t: (0.2 * math.sin(t), 0.2 * (1 - math.cos(t))), 0, 2 * math.pi)))
    S['ellipse 0.6x0.3'] = ('curve', _normalize(_curve(lambda t: (0.6 * math.sin(t), 0.3 * (1 - math.cos(t))), 0, 2 * math.pi)))
    S['figure-eight'] = ('curve', _normalize(_curve(lambda t: (0.4 * math.sin(t), 0.2 * math.sin(2 * t)), 0, 2 * math.pi)))
    S['sine wave'] = ('curve', _curve(lambda t: (t, 0.15 * math.sin(2 * math.pi * t / 0.8)), 0, 1.6))
    S['spiral out'] = ('curve', _normalize(_curve(lambda t: ((0.05 + 0.04 * t) * math.cos(t), (0.05 + 0.04 * t) * math.sin(t)), 0, 3 * math.pi)))
    S['stadium'] = ('curve', _stadium())
    return S


def _stadium():
    r, L = 0.2, 0.5
    pts = [(0, 0)]
    pts += [(L * k / 100, 0) for k in range(1, 101)]
    pts += [(L + r * math.sin(a), r - r * math.cos(a)) for a in [math.pi * k / 200 for k in range(1, 201)]]
    pts += [(L - L * k / 100, 2 * r) for k in range(1, 101)]
    pts += [(-r * math.sin(a), r + r * math.cos(a)) for a in [math.pi * k / 200 for k in range(1, 201)]]
    return pts


# ---- waypoint generation from an ideal path ---------------------------------------------------
def to_waypoints(kind, ideal, ds):
    """Return controller waypoints (x, y, theta). Polygons: its corners; curves: arc-length samples."""
    if kind == 'poly':
        corners = ideal[1:]            # the start is the robot's pose
    else:
        corners, acc = [], 0.0
        for a, b in zip(ideal, ideal[1:]):
            acc += math.hypot(b[0] - a[0], b[1] - a[1])
            if acc >= ds:
                corners.append(b)
                acc = 0.0
        if math.hypot(corners[-1][0] - ideal[-1][0], corners[-1][1] - ideal[-1][1]) > 1e-3:
            corners.append(ideal[-1])
    pts = [ideal[0]] + corners
    wps = []
    for i in range(1, len(pts)):
        if i < len(pts) - 1:
            th = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])   # heading of the NEXT leg
        else:
            th = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0]) if kind != 'poly' else \
                math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])   # closed polygon: start heading
        wps.append((pts[i][0], pts[i][1], th))
    return wps


def cross_track_dense(xs, ys, ideal):
    dense = _dense(ideal, 0.004) if len(ideal) < 400 else ideal
    e = []
    for x, y in zip(xs, ys):
        e.append(min(math.hypot(x - px, y - py) for px, py in dense))
    return max(e) * 100, math.sqrt(sum(v * v for v in e) / len(e)) * 100


def run_all(ds, tau, strategies, over=None, plant='lag', only=None, settle=0.0, noise=(0.0, 0.0), seed=0, odom_hold=1):
    rows = []
    for name, (kind, ideal) in shapes().items():
        if only and name not in only:
            continue
        wps = to_waypoints(kind, ideal, ds)
        for s in strategies:
            r = sc.run(s, wps, tau=tau, tmax=600.0, plant=make_plant(plant, tau), settle=settle,
                        noise_p=noise[0], noise_y=noise[1], seed=seed, odom_hold=odom_hold,
                        **(over or {}).get(s, {}))
            mx, rms = cross_track_dense(r['xs'][::2], r['ys'][::2], ideal)
            end = ideal[-1]
            rows.append(dict(shape=name, strat=s, n=len(wps), ok=r['ok'], t=r['t'], max=mx, rms=rms,
                             fin=100 * math.hypot(end[0] - r['xs'][-1], end[1] - r['ys'][-1]),
                             yaw=r['yaw_err']))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ds', type=float, default=0.10)
    ap.add_argument('--tau', type=float, default=0.15)
    ap.add_argument('--strategies', default='stop_turn,polar,pure_pursuit')
    ap.add_argument('--plant', default='lag', help='lag | lag_filter | identified | identified_nofilter | identified@gv,gw')
    ap.add_argument('--settle', type=float, default=0.0, help='seconds of zero command after DONE (use 8 for slow plants)')
    ap.add_argument('--only', default='', help='comma separated shape names')
    a = ap.parse_args()
    rows = run_all(a.ds, a.tau, a.strategies.split(','), plant=a.plant, settle=a.settle,
                   only=[x for x in a.only.split(',') if x] or None)
    print(f'ds={a.ds} m, plant={a.plant} (lag tau={a.tau} s).  cross-track = true path vs ideal path (cm)')
    print(f"{'shape':24s} {'strategy':13s} {'pts':>4s} {'done':>5s} {'time':>7s} {'xtrk max':>9s} {'xtrk rms':>9s} {'final':>7s}")
    last = None
    for r in rows:
        nm = r['shape'] if r['shape'] != last else ''
        last = r['shape']
        print(f"{nm:24s} {r['strat']:13s} {r['n']:4d} {str(r['ok']):>5s} {r['t']:7.1f} "
              f"{r['max']:9.2f} {r['rms']:9.2f} {r['fin']:7.2f}")


if __name__ == '__main__':
    main()
