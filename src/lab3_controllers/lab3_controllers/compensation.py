"""Compensation for a robot that drops small commands and responds slowly.

Everything here is OFF by default (all the parameters are 0), so on a simple robot or in Gazebo it does
nothing. `tracker.py` uses it through the class `Compensator`, which gives four helpers:

    _predict   where will the robot come to rest if we stopped commanding now?   (lead prediction)
    _at_rest   has the robot really stopped?                                     (rest detection)
    _mean_pose average of the last poses                                         (noise reduction at rest)
    _shape     keep a command out of the robot's deadband and inside its feasible set

TIMED MODE (for a slow, late, jittery state such as the SLAM map pose /robot_pose). The caller passes the time stamp of
each measurement and how long ago it arrived; `_predict_timed` then
    * updates the velocity estimate only from NEW samples, with the real time between them (the 20 Hz control loop reuses
      the same pose several times, which would make the speed estimate flicker),
    * predicts the present state from the old sample: pose + estimated velocity x (pose_latency + age + lead),
    * forgets everything after a gap in the data (a freeze), so stale numbers never feed the estimate,
    * keeps the rest-detection window in seconds, not in control ticks, and can compare window averages instead of single
      noisy samples (`rest_mean_test`).
"""

import math
from collections import deque

from lab3_controllers.common import wrap_angle

GAP_RESET = 1.0                      # s, timed mode: a gap in the sample stamps longer than this clears the estimates
REST_MEAN_SPAN = 2.0                 # s, timed mode: the mean pose of a settled robot uses samples from this window
DIAMOND_MARGIN = 0.97                # stay inside the feasible diamond with some margin
DEADBAND_MARGIN = 1.10               # leave the deadband by 10 % (lift to 1.1 * threshold)


class Compensator:
    """Base class of PathTracker and YawTracker. `p` is the parameter dict."""

    def __init__(self, p):
        self.p = p
        self.v_est = self.w_est = 0.0     # estimated velocities (from finite differences of the pose)
        self._hist = deque()              # recent poses, for the at-rest test
        self._vhist = deque()             # recent poses, for the velocity estimate
        self._timed = False               # True once the caller supplies sample time stamps
        self._tlast = None                # stamp of the newest sample
        self._tvhist = deque()            # timed mode: (stamp, pose) of recent samples for the velocity estimate
        self._thist = deque()             # timed mode: (stamp, pose) of the rest-detection window

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

    def _predict_timed(self, pose, stamp, age):
        """Timed variant of _predict. `stamp` is the time of the measurement (seconds, any clock: only differences matter),
        `age` the seconds since it ARRIVED. Returns the predicted present pose."""
        self._timed = True
        x, y, yaw = pose
        p = self.p
        if self._tlast is not None and stamp > self._tlast + GAP_RESET:          # data was missing: forget the estimates
            self._tvhist.clear()
            self._thist.clear()
            self.v_est = self.w_est = 0.0
        if self._tlast is None or stamp != self._tlast:                           # a NEW sample
            span = p['vel_span']
            if span > 0.0:
                # least-squares slope over the samples of the last `span` seconds: averages the noise of the poses
                self._tvhist.append((stamp, pose))
                while len(self._tvhist) > 2 and stamp - self._tvhist[0][0] > span:
                    self._tvhist.popleft()
                while len(self._tvhist) > 80:
                    self._tvhist.popleft()
                if len(self._tvhist) >= 3 and self._tvhist[-1][0] - self._tvhist[0][0] >= 0.4 * span:
                    vi, wi = self._slopes(yaw)
                    a = p['vel_filter']
                    self.v_est += a * (vi - self.v_est)
                    self.w_est += a * (wi - self.w_est)
            else:
                n = max(1, int(p['vel_window']))
                if self._tvhist and stamp > self._tvhist[-1][0]:
                    k = min(n, len(self._tvhist))
                    tk, (px, py, pyaw) = self._tvhist[-k]
                    dtk = stamp - tk
                    vi = ((x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)) / dtk
                    wi = wrap_angle(yaw - pyaw) / dtk
                    a = p['vel_filter']
                    self.v_est += a * (vi - self.v_est)
                    self.w_est += a * (wi - self.w_est)
                self._tvhist.append((stamp, pose))
                while len(self._tvhist) > n + 1:
                    self._tvhist.popleft()
            # an estimate is never larger than what the robot can do (noise must not turn into a fake prediction)
            wmax = 1.5 * p.get('max_angular_speed', 1.0)
            vmax = 1.5 * p.get('max_linear_speed', 0.3)
            self.w_est = max(-wmax, min(wmax, self.w_est))
            self.v_est = max(-vmax, min(vmax, self.v_est))
            if p['rest_window'] > 0.0:
                self._thist.append((stamp, pose))
                while self._thist and stamp - self._thist[0][0] > p['rest_window'] + 0.5:
                    self._thist.popleft()
            self._tlast = stamp
        horizon_v = p['lead_v'] + p['pose_latency'] + max(age, 0.0)
        horizon_w = p['lead_w'] + p['pose_latency'] + max(age, 0.0)
        if horizon_v == 0.0 and horizon_w == 0.0:
            return pose
        return (x + self.v_est * horizon_v * math.cos(yaw),
                y + self.v_est * horizon_v * math.sin(yaw),
                wrap_angle(yaw + self.w_est * horizon_w))

    def _slopes(self, yaw_now):
        """(forward speed, turn rate) from the least-squares slopes of x, y and the unwrapped heading over _tvhist."""
        h = list(self._tvhist)
        t0 = h[0][0]
        ts = [t - t0 for t, _ in h]
        tm = sum(ts) / len(ts)
        den = sum((t - tm) ** 2 for t in ts) or 1e-9

        def slope(vals):
            vm = sum(vals) / len(vals)
            return sum((t - tm) * (v - vm) for t, v in zip(ts, vals)) / den
        y0 = h[0][1][2]
        yaws = [wrap_angle(q[2] - y0) for _, q in h]                  # unwrapped relative to the first sample
        vx, vy = slope([q[0] for _, q in h]), slope([q[1] for _, q in h])
        return vx * math.cos(yaw_now) + vy * math.sin(yaw_now), slope(yaws)

    def _clear_rest_history(self):
        """The at-rest window must contain only data from after the last command."""
        self._hist.clear()
        self._thist.clear()

    # ---- 2. rest detection -------------------------------------------------------------------
    def _mean_pose(self, pose, n=10):
        """Average of the last n poses (noise reduction when the robot is at rest)."""
        if self._timed:
            h = [q for t, q in self._thist if self._tlast - t <= REST_MEAN_SPAN][-n:] or [pose]
            return (sum(q[0] for q in h) / len(h), sum(q[1] for q in h) / len(h),
                    math.atan2(sum(math.sin(q[2]) for q in h), sum(math.cos(q[2]) for q in h)))
        h = list(self._hist)[-n:] or [pose]
        return (sum(q[0] for q in h) / len(h), sum(q[1] for q in h) / len(h),
                math.atan2(sum(math.sin(q[2]) for q in h), sum(math.cos(q[2]) for q in h)))

    def _at_rest(self, dt):
        """True when the pose has hardly moved over the last rest_window seconds."""
        p = self.p
        if self._timed:
            return self._at_rest_timed()
        n = int(p['rest_window'] / dt) + 1
        if len(self._hist) < n:
            return False
        x1, y1, th1 = self._hist[-1]
        return all(math.hypot(x1 - x0, y1 - y0) <= p['rest_pos'] and
                   abs(wrap_angle(th1 - th0)) <= p['rest_yaw'] for x0, y0, th0 in self._hist)

    def _at_rest_timed(self):
        """Timed mode: the samples of the last rest_window seconds (at least 4, spanning most of the window) show no movement.
        With rest_mean_test the first and second half are compared by their AVERAGES, which noise cannot fool;
        otherwise every sample must lie within rest_pos / rest_yaw of the newest one."""
        p = self.p
        h = list(self._thist)
        if len(h) < 4 or h[-1][0] - h[0][0] < 0.8 * p['rest_window']:
            return False
        if p['rest_mean_test']:
            half = len(h) // 2
            a, b = [q for _, q in h[:half]], [q for _, q in h[half:]]

            def mean(qs):
                return (sum(q[0] for q in qs) / len(qs), sum(q[1] for q in qs) / len(qs),
                        math.atan2(sum(math.sin(q[2]) for q in qs), sum(math.cos(q[2]) for q in qs)))
            ma, mb = mean(a), mean(b)
            return (math.hypot(mb[0] - ma[0], mb[1] - ma[1]) <= p['rest_pos'] and
                    abs(wrap_angle(mb[2] - ma[2])) <= p['rest_yaw'])
        x1, y1, th1 = h[-1][1]
        return all(math.hypot(x1 - q[0], y1 - q[1]) <= p['rest_pos'] and abs(wrap_angle(th1 - q[2])) <= p['rest_yaw']
                   for _, q in h)

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
