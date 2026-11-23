"""Compensation for a robot that drops small commands and responds slowly.

Everything here is OFF by default (all the parameters are 0), so on a simple robot or in Gazebo it does
nothing. `tracker.py` uses it through the class `Compensator`, which gives four helpers:

    _predict   where will the robot come to rest if we stopped commanding now?   (lead prediction)
    _at_rest   has the robot really stopped?                                     (rest detection)
    _mean_pose average of the last poses                                         (noise reduction at rest)
    _shape     keep a command out of the robot's deadband and inside its feasible set
"""

import math
from collections import deque

from lab3_controllers.common import wrap_angle

DIAMOND_MARGIN = 0.97                # stay inside the feasible diamond with some margin
DEADBAND_MARGIN = 1.10               # leave the deadband by 10 % (lift to 1.1 * threshold)


class Compensator:
    """Base class of PathTracker and YawTracker. `p` is the parameter dict."""

    def __init__(self, p):
        self.p = p
        self.v_est = self.w_est = 0.0     # estimated velocities (from finite differences of the pose)
        self._hist = deque()              # recent poses, for the at-rest test
        self._vhist = deque()             # recent poses, for the velocity estimate

    # ---- 1. lead prediction ------------------------------------------------------------------
    def _predict(self, pose, dt):
        """Pose the robot will rest at if we stopped commanding now: pose + velocity * coasting time.
        The velocity is a low-passed finite difference of the measured pose."""
        x, y, yaw = pose
        p = self.p
        n = max(1, int(p['vel_window']))
        if dt > 0.0 and self._vhist:
            k = min(n, len(self._vhist))
            px, py, pyaw = self._vhist[-k]
            vi = ((x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)) / (k * dt)   # forward speed
            wi = wrap_angle(yaw - pyaw) / (k * dt)                                    # turn rate
            a = p['vel_filter']
            self.v_est += a * (vi - self.v_est)
            self.w_est += a * (wi - self.w_est)
        self._vhist.append(pose)
        while len(self._vhist) > n:
            self._vhist.popleft()
        if p['rest_window'] > 0.0 and dt > 0.0:
            self._hist.append(pose)
            while len(self._hist) > int(p['rest_window'] / dt) + 1:
                self._hist.popleft()
        if p['lead_v'] == 0.0 and p['lead_w'] == 0.0:
            return pose
        return (x + self.v_est * p['lead_v'] * math.cos(yaw),
                y + self.v_est * p['lead_v'] * math.sin(yaw),
                wrap_angle(yaw + self.w_est * p['lead_w']))

    # ---- 2. rest detection -------------------------------------------------------------------
    def _mean_pose(self, pose, n=10):
        """Average of the last n poses (noise reduction when the robot is at rest)."""
        h = list(self._hist)[-n:] or [pose]
        return (sum(q[0] for q in h) / len(h), sum(q[1] for q in h) / len(h),
                math.atan2(sum(math.sin(q[2]) for q in h), sum(math.cos(q[2]) for q in h)))

    def _at_rest(self, dt):
        """True when the pose has hardly moved over the last rest_window seconds."""
        p = self.p
        n = int(p['rest_window'] / dt) + 1
        if len(self._hist) < n:
            return False
        x1, y1, th1 = self._hist[-1]
        return all(math.hypot(x1 - x0, y1 - y0) <= p['rest_pos'] and
                   abs(wrap_angle(th1 - th0)) <= p['rest_yaw'] for x0, y0, th0 in self._hist)

    # ---- 3. command shaping ------------------------------------------------------------------
    def _shape(self, v, w):
        """Keep the command out of the robot's deadband and inside its feasible set, so the robot's
        input filter never drops it. Both steps scale (v, w) together, i.e. keep the curvature."""
        p = self.p
        dv, dw = p['deadband_v'], p['deadband_w']
        if (v != 0.0 or w != 0.0) and dv > 0.0 and dw > 0.0 and abs(v) <= dv and abs(w) <= dw:
            # smallest scale factor that leaves the deadband rectangle along either axis
            factors = []
            if v != 0.0:
                factors.append(dv * DEADBAND_MARGIN / abs(v))
            if w != 0.0:
                factors.append(dw * DEADBAND_MARGIN / abs(w))
            f = min(factors)
            v, w = v * f, w * f
        if p['diamond_v'] > 0.0 and p['diamond_w'] > 0.0:
            s = abs(v) / p['diamond_v'] + abs(w) / p['diamond_w']
            if s > DIAMOND_MARGIN:
                f = DIAMOND_MARGIN / s
                v, w = v * f, w * f
        return v, w

    def _pulse_size(self):
        """Angular speed of one corrective heading pulse: the smallest the robot will not drop."""
        p = self.p
        return p['pulse_w'] or (p['deadband_w'] * DEADBAND_MARGIN if p['deadband_w'] > 0.0 else 0.05)
