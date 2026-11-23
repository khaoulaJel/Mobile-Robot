#!/usr/bin/env python3
"""Plant models for the ROS-free benchmarks.

IdentifiedPlant  = the order-4 discrete velocity model identified from the professor's supplied
                   dataset (matlab_input_set/robot_identification_data.mat, exported with MATLAB from
                   matlab/lab02_offline_results/Lab02_supplied_data_results.mat), preceded by the
                   professor's `input_filter` (Simulink chart in robot_mimo_identification.slx):
                       feasible = |v|/0.3 + |w|/2.5 <= 1      (else the command is replaced by 0, 0)
                       deadband = |v| <= 0.04 and |w| <= 0.2  (then the command is replaced by 0, 0)
                   Data provenance (real robot or not) is UNVERIFIED, so treat results as a stress test.
LagPlant         = first-order lag (what tools/sim_compare.py used until now).
Both have step(cmd_v, cmd_w) -> (v, w) at the 0.05 s sample time.
"""
import json
import os

VMAX, WMAX = 0.3, 2.5          # professor's feasible-set limits
V_DEAD, W_DEAD = 0.04, 0.2     # professor's rectangular deadband


def input_filter(v, w, vmax=VMAX, wmax=WMAX, vdead=V_DEAD, wdead=W_DEAD):
    """The professor's command filter: infeasible or deadband commands become (0, 0)."""
    feasible = abs(v) / vmax + abs(w) / wmax <= 1.0
    dead = abs(v) <= vdead and abs(w) <= wdead
    return (v, w) if feasible and not dead else (0.0, 0.0)


class LagPlant:
    def __init__(self, tau=0.15, dt=0.05, use_filter=False):
        self.a = dt / (tau + dt)
        self.v = self.w = 0.0
        self.use_filter = use_filter

    def step(self, cv, cw):
        if self.use_filter:
            cv, cw = input_filter(cv, cw)
        self.v += self.a * (cv - self.v)
        self.w += self.a * (cw - self.w)
        return self.v, self.w


class IdentifiedPlant:
    """gain=(gv, gw) multiplies the output velocities: a wheel-radius / wheel-separation calibration error."""

    def __init__(self, path=None, use_filter=True, gain=(1.0, 1.0), delay=0):
        self.gain = gain
        self.delay = int(delay)                  # extra command latency in samples (communication delay)
        self._queue = [(0.0, 0.0)] * self.delay
        path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data',
                                    'identified_plant.json')
        with open(path) as f:
            p = json.load(f)
        self.A, self.B, self.C, self.D, self.Ts = p['A'], p['B'], p['C'], p['D'], p['Ts']
        self.x = [0.0] * len(self.A)
        self.use_filter = use_filter
        self.last_cmd = (0.0, 0.0)

    def step(self, cv, cw):
        if self.use_filter:
            cv, cw = input_filter(cv, cw)
        if self.delay:
            self._queue.append((cv, cw))
            cv, cw = self._queue.pop(0)
        self.last_cmd = (cv, cw)
        u = (cv, cw)
        y = [sum(self.C[i][j] * self.x[j] for j in range(len(self.x))) +
             sum(self.D[i][k] * u[k] for k in range(2)) for i in range(2)]
        self.x = [sum(self.A[i][j] * self.x[j] for j in range(len(self.x))) +
                  sum(self.B[i][k] * u[k] for k in range(2)) for i in range(len(self.x))]
        return y[0] * self.gain[0], y[1] * self.gain[1]


class Chain:
    """plant followed by an extra first-order lag (e.g. the Gazebo drive plugin's own dynamics)."""

    def __init__(self, plant, tau, dt=0.05):
        self.plant, self.a = plant, dt / (tau + dt)
        self.v = self.w = 0.0

    def step(self, cv, cw):
        v, w = self.plant.step(cv, cw)
        self.v += self.a * (v - self.v)
        self.w += self.a * (w - self.w)
        return self.v, self.w
