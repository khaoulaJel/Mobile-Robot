#!/usr/bin/env python3
"""Yaw controller on the identified plant + robot filter (no ROS): references 90, -90, 170, -170, 0 deg.

Per reference: settling time (last time the error left the +-band), overshoot beyond the reference and the
error at the end of the hold. Usage: python3 yaw_bench.py [--hold 45] [--noise 0.0005]
"""
import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import plant_models as pm  # noqa: E402
from lab3_controllers.common import slew, wrap_angle  # noqa: E402
from lab3_controllers.tracker import YawTracker  # noqa: E402

DT = 0.05
REFS_DEG = [90.0, -90.0, 170.0, -170.0, 0.0]
ROBOT = dict(diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2)
LAB = dict(kp=1.0, lead_w=0.8, vel_filter=0.7, vel_window=5, rest_window=1.0, **ROBOT)


def run(params, hold=45.0, plant=None, noise=0.0, seed=0, band_deg=0.5):
    rng = random.Random(seed)
    tr = YawTracker(params)
    plant = plant or pm.IdentifiedPlant(use_filter=True)
    yaw, cw = 0.0, 0.0
    out = []
    for ref_deg in REFS_DEG:
        ref = math.radians(ref_deg)
        n = int(hold / DT)
        errs = []
        for k in range(n):
            meas = yaw + rng.gauss(0.0, noise)
            w = tr.step(wrap_angle(meas), ref, DT)
            cw = slew(cw, w, 3.0, DT)
            _, ww = plant.step(0.0, cw)
            yaw = wrap_angle(yaw + ww * DT)
            errs.append(math.degrees(wrap_angle(ref - yaw)))
        settle = next((i * DT for i in range(n - 1, -1, -1) if abs(errs[i]) > band_deg), -DT) + DT
        # overshoot: the largest excursion beyond the reference, in the direction of travel
        direction = 1.0 if errs[0] > 0 else -1.0
        over = max(0.0, max(-direction * e for e in errs))
        out.append(dict(ref=ref_deg, settle=settle, over=over, final=errs[-1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hold', type=float, default=45.0)
    ap.add_argument('--noise', type=float, default=0.0)
    a = ap.parse_args()
    cases = {'legacy P law (kp 1.0, no compensation)': dict(kp=1.0, **ROBOT),
             'legacy + min speed 0.22 (deadband lift only)': dict(kp=1.0, min_angular_speed=0.22, **ROBOT),
             'lab preset (lead + pulses)': LAB}
    for name, prm in cases.items():
        res = run(prm, a.hold, noise=a.noise)
        print(f'--- {name}')
        for r in res:
            print(f"  ref {r['ref']:7.1f} deg: settle to +-0.5 deg {r['settle']:5.1f} s, overshoot {r['over']:5.2f} deg, "
                  f"final error {r['final']:6.2f} deg")


if __name__ == '__main__':
    main()
