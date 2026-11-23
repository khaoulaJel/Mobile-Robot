#!/usr/bin/env python3
"""Step 1 of the Simulink cross-check: run the lab_robot preset on the Python plant and log the command stream.

Writes replay.csv (t, cmd_v, cmd_w, v_out, w_out) next to this file; replay_check.m then feeds cmd_v/cmd_w
through the professor's own Simulink blocks and compares the output velocities with v_out / w_out.
Usage: python3 make_replay.py [shape name, default 'triangle']
"""
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, '..')
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.join(TOOLS, '..'))
import plant_models as pm  # noqa: E402
import robustness as rb  # noqa: E402
import shape_bench as sb  # noqa: E402
import tune_lab_plant as tp  # noqa: E402
from lab3_controllers.common import slew, wrap_angle  # noqa: E402
from lab3_controllers.tracker import DONE, PathTracker  # noqa: E402

DT = 0.05


def main():
    shape = sys.argv[1] if len(sys.argv) > 1 else 'triangle'
    params = dict(tp.FIXED)
    params.update(rb.CANDIDATE)
    params['strategy'] = 'stop_turn'
    kind, ideal = sb.shapes()[shape]
    tracker = PathTracker(sb.to_waypoints(kind, ideal, 0.1), params)
    tracker.start((0.0, 0.0, 0.0))
    plant = pm.IdentifiedPlant()
    x = y = th = 0.0
    cv = cw = 0.0
    rows, extra = [], 0
    for k in range(4000):
        if tracker.mode == DONE:
            extra += 1
            if extra > 240:          # 12 s of zero command after the path: the robot coasts to rest
                break
        v, w = tracker.step((x, y, th), DT)
        cv = slew(cv, v, 0.4, DT)
        cw = slew(cw, w, 3.0, DT)
        vv, ww = plant.step(cv, cw)
        rows.append((k * DT, cv, cw, vv, ww))      # exact k*Ts time stamps
        th = wrap_angle(th + ww * DT)
        x += vv * math.cos(th) * DT
        y += vv * math.sin(th) * DT
    out = os.path.join(HERE, 'replay.csv')
    with open(out, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['t', 'cmd_v', 'cmd_w', 'v_out', 'w_out'])
        wr.writerows(rows)
    print(f'{shape}: {len(rows)} samples -> {out}; final pose x={x:.5f} y={y:.5f} yaw={math.degrees(th):.3f} deg')


if __name__ == '__main__':
    main()
