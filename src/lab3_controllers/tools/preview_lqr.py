#!/usr/bin/env python3
"""Preview LQR (= unconstrained linear MPC) path tracker that uses the identified plant inside the controller.

Literature basis: MPC with an identified actuator model for differential-drive path tracking (Pacheco & Luo 2015;
event-triggered MPC-PID with first-order actuator dynamics, PLOS ONE 2025; dead-time compensation for path tracking,
arXiv 2507.06935). Design:
  state z = [e, th_e, x1..x4]   e = lateral offset (left +), th_e = heading error, x = identified plant state
  e+    = e + Ts v th_e
  th_e+ = th_e + Ts (w_real - v kappa)             w_real = C2 x   (plant output)
  x+    = A x + B1 v + B2 w                         v is a planned speed profile, w is the only decision
  minimise sum q_e e^2 + q_th th_e^2 + r w^2 over a preview horizon of N samples with the path curvature kappa(s)
  and speed v(s) known along the horizon (backward Riccati recursion with an affine term).
Moving at v >= 0.05 m/s keeps the command out of the professor's deadband by construction (only w is decided).
The plant state x is estimated by running the identified model on the commands sent (open loop observer).
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import plant_models as pm  # noqa: E402
import shape_bench as sb  # noqa: E402
from lab3_controllers.common import slew, wrap_angle  # noqa: E402
from lab3_controllers.tracker import DONE, PathTracker, YawTracker  # noqa: E402
import tune_lab_plant as tp  # noqa: E402
import robustness as rb  # noqa: E402

TS = 0.05
ROBOT = dict(diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2)


class Path:
    def __init__(self, pts):
        p = np.array(pts, float)
        d = np.hypot(*np.diff(p, axis=0).T)
        keep = np.concatenate([[True], d > 1e-9])
        p = p[keep]
        self.p = p
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
        self.L = self.s[-1]
        dx, dy = np.gradient(p[:, 0], self.s), np.gradient(p[:, 1], self.s)
        self.psi = np.unwrap(np.arctan2(dy, dx))
        k = np.gradient(self.psi, self.s)
        w = max(3, int(0.08 / max(np.mean(np.diff(self.s)), 1e-6)) | 1)       # ~8 cm smoothing window
        self.kappa = np.convolve(np.pad(k, w // 2, mode='edge'), np.ones(w) / w, mode='valid')

    def at(self, s):
        s = np.clip(s, 0.0, self.L)
        return (np.interp(s, self.s, self.p[:, 0]), np.interp(s, self.s, self.p[:, 1]),
                np.interp(s, self.s, self.psi), np.interp(s, self.s, self.kappa))


class PreviewLQR:
    def __init__(self, path, v_cruise=0.1, a_lat=0.15, a_dec=0.06, v_floor=0.05, horizon=60,
                 q_e=1e4, q_th=10.0, r=1.0, plant_json=None, align=None):
        self.path = path
        self.vc, self.a_lat, self.a_dec, self.vf, self.N = v_cruise, a_lat, a_dec, v_floor, horizon
        self.q_e, self.q_th, self.r = q_e, q_th, r
        plant = pm.IdentifiedPlant(plant_json)
        self.A = np.array(plant.A)
        self.B = np.array(plant.B)
        self.C2 = np.array(plant.C)[1]
        self.xh = np.zeros(4)                    # estimated plant state
        self.s_idx = 0
        self.s = 0.0
        self.done = False
        self.v_est = 0.0
        self.align = YawTracker(dict(kp=1.0, lead_w=0.8, vel_filter=0.7, vel_window=5, rest_window=1.0, **ROBOT)) \
            if align is None else align
        self.phase = 'align'
        self.last_cmd = (0.0, 0.0)

    def profile(self, s):
        _, _, _, k = self.path.at(s)
        v = min(self.vc, math.sqrt(self.a_lat / max(abs(k), 1e-6)))
        v = min(v, math.sqrt(2.0 * self.a_dec * max(self.path.L - s, 0.0)) + 0.0)
        return max(v, self.vf)

    def observe(self, cmd):
        u = pm.input_filter(*cmd)
        self.xh = self.A @ self.xh + self.B @ np.array(u)

    def step(self, pose, prev_pose):
        x, y, th = pose
        P = self.path
        if self.phase == 'align':
            psi0 = P.psi[0]
            w = self.align.step(th, psi0, TS)
            if self.align.reached:
                self.phase = 'track'
            return 0.0, w
        # projection (forward only)
        lo = self.s_idx
        hi = min(len(P.s) - 1, lo + 120)
        d2 = (P.p[lo:hi + 1, 0] - x) ** 2 + (P.p[lo:hi + 1, 1] - y) ** 2
        self.s_idx = lo + int(np.argmin(d2))
        s0 = P.s[self.s_idx]
        self.s = s0
        px, py, psi, kap = P.at(s0)
        e = -math.sin(psi) * (x - px) + math.cos(psi) * (y - py)
        the = wrap_angle(th - psi)
        # velocity estimate and end rule: stop when the predicted rest point reaches the end
        if prev_pose is not None:
            self.v_est += 0.3 * (((x - prev_pose[0]) * math.cos(th) + (y - prev_pose[1]) * math.sin(th)) / TS - self.v_est)
        if P.L - s0 <= max(0.01, self.v_est * 0.61):
            self.done = True
            return 0.0, 0.0
        # horizon
        N = self.N
        ss = np.empty(N)
        vs = np.empty(N)
        ks = np.empty(N)
        s = s0
        for k in range(N):
            vs[k] = self.profile(s)
            ks[k] = P.at(s)[3]
            ss[k] = s
            s += vs[k] * TS
        n = 6
        Q = np.diag([self.q_e, self.q_th, 0, 0, 0, 0])
        Pm = Q.copy()
        pv = np.zeros(n)
        B = np.zeros((n, 1))
        B[2:, 0] = self.B[:, 1]
        Ks, kffs = [None] * N, [None] * N
        for k in range(N - 1, -1, -1):
            A = np.zeros((n, n))
            A[0, 0] = 1.0
            A[0, 1] = TS * vs[k]
            A[1, 1] = 1.0
            A[1, 2:] = TS * self.C2
            A[2:, 2:] = self.A
            d = np.zeros(n)
            d[1] = -TS * vs[k] * ks[k]
            d[2:] = self.B[:, 0] * vs[k]
            S = self.r + (B.T @ Pm @ B)[0, 0]
            g = Pm @ d + pv
            K = (B.T @ Pm @ A) / S
            kff = (B.T @ g)[0] / S
            Ks[k], kffs[k] = K, kff
            Pn = Q + A.T @ Pm @ A - (A.T @ Pm @ B) @ K
            pv = A.T @ g - (A.T @ Pm @ B)[:, 0] * kff
            Pm = (Pn + Pn.T) / 2
        z = np.concatenate([[e, the], self.xh])
        w = float(-(Ks[0] @ z)[0] - kffs[0])
        v = float(vs[0])
        room = 2.5 * (0.97 - v / 0.3)                 # professor's feasible diamond: |v|/0.3 + |w|/2.5 <= 1
        w = max(-room, min(room, w))
        return v, w


def run(shape, params, plant, noise=(0.0, 0.0), seed=0, settle=12.0, tmax=400.0, finish=True):
    rng = np.random.default_rng(seed)
    kind, ideal = sb.shapes()[shape]
    path = Path(ideal)
    ctl = PreviewLQR(path, **params)
    x = y = th = 0.0
    cv = cw = 0.0
    prev = None
    xs, ys = [x], [y]
    t = 0.0
    while t < tmax and not ctl.done:
        meas = (x + rng.normal(0, noise[0]), y + rng.normal(0, noise[0]), th + rng.normal(0, noise[1]))
        v, w = ctl.step(meas, prev)
        prev = meas
        cv = slew(cv, v, 0.4, TS)
        cw = slew(cw, w, 3.0, TS)
        ctl.observe((cv, cw))
        vv, ww = plant.step(cv, cw)
        th = wrap_angle(th + ww * TS)
        x += vv * math.cos(th) * TS
        y += vv * math.sin(th) * TS
        xs.append(x)
        ys.append(y)
        t += TS
    t_track = t
    if finish and ctl.done:
        # hand over to the lab_robot stop_turn tracker for the last point: wait for rest, then fine-position
        # the robot at the path end with its position check and pulse alignment
        fp = dict(tp.FIXED)
        fp.update(rb.CANDIDATE)
        fp['strategy'] = 'stop_turn'
        ex, ey = ideal[-1]
        fin_tr = PathTracker([(ex, ey, float(path.psi[-1]))], fp)
        fin_tr.start((x, y, th))
        prev_f = None
        n_rest = 0
        while t < tmax and fin_tr.mode != DONE:
            meas = (x + rng.normal(0, noise[0]), y + rng.normal(0, noise[0]), th + rng.normal(0, noise[1]))
            v, w = fin_tr.step(meas, TS) if n_rest >= 80 else (0.0, 0.0)     # 4 s of zero command first
            n_rest += 1
            cv = slew(cv, v, 0.4, TS)
            cw = slew(cw, w, 3.0, TS)
            vv, ww = plant.step(cv, cw)
            th = wrap_angle(th + ww * TS)
            x += vv * math.cos(th) * TS
            y += vv * math.sin(th) * TS
            xs.append(x)
            ys.append(y)
            t += TS
    t_done = t
    for _ in range(int(settle / TS)):
        cv, cw = slew(cv, 0.0, 0.4, TS), slew(cw, 0.0, 3.0, TS)
        vv, ww = plant.step(cv, cw)
        th = wrap_angle(th + ww * TS)
        x += vv * math.cos(th) * TS
        y += vv * math.sin(th) * TS
        xs.append(x)
        ys.append(y)
    mx, rms = sb.cross_track_dense(xs[::2], ys[::2], ideal)
    end = ideal[-1]
    fin = 100 * math.hypot(end[0] - x, end[1] - y)
    psi_end = path.psi[-1]
    return dict(shape=shape, ok=ctl.done, t=t_done, t_track=t_track, max=mx, rms=rms, fin=fin,
                yaw=abs(math.degrees(wrap_angle(psi_end - th))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--shapes', default='circle R0.4,circle R0.5 (lab polar),circle R0.2 (tight),ellipse 0.6x0.3,'
                                        'figure-eight,sine wave,spiral out,stadium')
    ap.add_argument('--v', type=float, default=0.1)
    ap.add_argument('--qe', type=float, default=1e4)
    ap.add_argument('--qth', type=float, default=10.0)
    ap.add_argument('--r', type=float, default=1.0)
    a = ap.parse_args()
    print(f"{'shape':24s} {'done':>5s} {'time':>6s} {'xtrk max':>8s} {'rms':>5s} {'fin pos':>7s} {'fin yaw':>7s}")
    for s in a.shapes.split(','):
        r = run(s, dict(v_cruise=a.v, q_e=a.qe, q_th=a.qth, r=a.r), pm.IdentifiedPlant())
        print(f"{r['shape']:24s} {str(r['ok']):>5s} {r['t']:6.1f} {r['max']:8.2f} {r['rms']:5.2f} {r['fin']:7.2f} {r['yaw']:7.2f}")


if __name__ == '__main__':
    main()
