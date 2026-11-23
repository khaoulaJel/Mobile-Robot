#!/usr/bin/env python3
"""Plot yaw step responses and the square path. Usage: plot_results.py yaw.csv motion.csv outdir"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(p):
    with open(p) as f:
        return list(csv.DictReader(f))


yaw, mot, out = load(sys.argv[1]), load(sys.argv[2]), sys.argv[3]

t = [float(r['t']) for r in yaw]
fig, ax = plt.subplots(figsize=(8, 4))
ax.plot(t, [float(r['yaw_deg']) for r in yaw], label='yaw (odom)')
ax.step(t, [float(r['ref_deg']) for r in yaw], 'r--', where='post', label='reference')
ax.set_xlabel('time [s]'); ax.set_ylabel('yaw [deg]'); ax.grid(True); ax.legend()
ax.set_title('Yaw P controller (kp=1.5) in Gazebo')
fig.tight_layout(); fig.savefig(out + '/yaw_response.png', dpi=130)

fig, ax = plt.subplots(figsize=(5, 5))
ax.plot([float(r['x']) for r in mot], [float(r['y']) for r in mot], label='robot path')
seen = []
for r in mot:
    g = (round(float(r['gx']), 3), round(float(r['gy']), 3))
    if g not in seen:
        seen.append(g)
ax.plot([g[0] for g in seen], [g[1] for g in seen], 'rx', ms=10, label='waypoints')
ax.set_aspect('equal'); ax.grid(True); ax.legend(); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
ax.set_title('Motion controller: square path')
fig.tight_layout(); fig.savefig(out + '/motion_path.png', dpi=130)
