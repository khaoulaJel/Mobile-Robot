"""Path tracker without ROS: the ONE implementation of the motion strategies.

`motion_controller.py` (ROS node) and the model tools (`tools/sim_compare.py`, `tools/shape_bench.py`)
all call `PathTracker.step()`, so what is tuned in the model is exactly what runs on the robot.

The tracker works in the world (odom) frame: the caller transforms a start-relative path first and calls
`start(pose)` once. `step(pose)` returns the raw (v, w) command; the caller applies acceleration limits
(`slew`) and publishes.
"""

import math
from collections import deque

from lab3_controllers.common import (
    go_to_point_control, lookahead_point, pose_control, project_on_path,
    pure_pursuit_control, wrap_angle, yaw_p_control)

GO, ALIGN, DONE = 'GO', 'ALIGN', 'DONE'
STRATEGIES = ('stop_turn', 'polar', 'pure_pursuit')

# Single source of defaults: the ROS node declares its parameters from this dict.
DEFAULTS = dict(
    strategy='stop_turn',            # stop_turn | polar | pure_pursuit
    # stop_turn gains
    kv=0.8,                          # 1/s   linear gain (higher coasts past the goal)
    kw=3.0,                          # 1/s   bearing gain
    k_yaw=1.0,                       # 1/s   final-heading gain
    turn_in_place_deg=35.0,
    # polar gains (stable for k_rho > 0, k_beta < 0, k_alpha > k_rho)
    k_rho=1.5, k_alpha=4.0, k_beta=-2.0,
    blend_radius=0.03,               # m (0.03 = reach every point; larger = faster, rounds corners)
    # pure pursuit
    lookahead=0.12,                  # m
    # common
    decel=0.4,                       # m/s^2, speed cap near the final point
    max_linear_speed=0.20,           # m/s
    max_angular_speed=1.0,           # rad/s
    min_linear_speed=0.0,            # m/s  (deadband compensation)
    min_angular_speed=0.0,           # rad/s
    pos_tolerance=0.01,              # m
    yaw_tolerance=0.01,              # rad
    # --- slow-plant compensation (all off by default; see tools/plant_models.py) ---
    lead_v=0.0,                      # s  predict the rest position: pos + v_meas * lead_v (plant lag)
    lead_w=0.0,                      # s  predict the rest heading: yaw + w_meas * lead_w
    vel_filter=0.3,                  # 0..1 low-pass gain of the velocity estimate (finite differences)
    vel_window=1,                    # samples: velocity = (pose now - pose N samples ago) / (N dt); >1 averages noise
    # --- verify at rest (0 = off): at corners and the final point, wait until the robot has really
    #     stopped, check the TRUE error, then correct the heading with exact minimum-size pulses and
    #     re-approach if the position is off ---
    rest_window=0.0,                 # s   the pose must move less than rest_pos / rest_yaw over this window
    pulse_w=0.0,                     # rad/s size of a heading pulse (0 = 1.1 * deadband_w, else 0.05)
    pulse_extra=1,                   # extra samples per pulse: the caller's acceleration ramp spends the
                                     # first sample below the deadband, where the robot drops it
    rest_pos=0.003,                  # m
    rest_yaw=0.01,                   # rad
    rest_timeout=4.0,                # s   stop waiting (noisy odometry never looks "at rest")
    settle_time=3.0,                 # s   minimum wait after a pulse: the plant answers slowly, so the robot
                                     #     still looks at rest for a while after the pulse has been sent
    rest_corner_deg=30.0,            # verify at points where the path turns by more than this
    smooth_yaw_tolerance=0.0,        # rad  looser heading tolerance at smooth (non-corner, non-final) points;
                                     #      0 = same as yaw_tolerance. Avoids pointless in-place turns on curves.
    rest_retries=4,                  # heading pulses per point before accepting the heading
    pos_retries=3,                   # position re-approaches per point before accepting the position
    # --- robot command filter (professor's input_filter): commands outside the diamond or inside the
    #     deadband are DROPPED by the robot, so the controller must never send them (0 = off) ---
    diamond_v=0.0,                   # m/s   feasible set |v|/diamond_v + |w|/diamond_w <= 1   (lab: 0.3)
    diamond_w=0.0,                   # rad/s                                                  (lab: 2.5)
    deadband_v=0.0,                  # m/s   command is dropped if |v| <= deadband_v and ...  (lab: 0.04)
    deadband_w=0.0,                  # rad/s ... |w| <= deadband_w                            (lab: 0.2)
)

DIAMOND_MARGIN = 0.97                # stay inside the diamond with some margin
DEADBAND_MARGIN = 1.10               # leave the deadband by 10 % (lift to 1.1 * threshold)


class _Compensator:
    """Shared by PathTracker and YawTracker: velocity estimate + lead prediction of the rest pose,
    at-rest detection, and shaping of commands for the robot's input filter."""

    def __init__(self, p):
        self.p = p
        self.v_est = self.w_est = 0.0
        self._hist = deque()         # recent poses, for the at-rest test
        self._vhist = deque()        # recent poses, for the velocity estimate

    # ---- plant-lag prediction and robot command filter -------------------------------------
    def _predict(self, pose, dt):
        """Pose the robot will come to rest at if we stopped commanding now (lead compensation):
        pose + measured velocity * coasting time. The velocity is a low-passed finite difference."""
        x, y, yaw = pose
        p = self.p
        n = max(1, int(p['vel_window']))
        if dt > 0.0 and self._vhist:
            k = min(n, len(self._vhist))
            px, py, pyaw = self._vhist[-k]
            vi = ((x - px) * math.cos(yaw) + (y - py) * math.sin(yaw)) / (k * dt)
            wi = wrap_angle(yaw - pyaw) / (k * dt)
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


class PathTracker(_Compensator):
    def __init__(self, path, params=None):
        p = dict(DEFAULTS)
        p.update(params or {})
        if p['strategy'] not in STRATEGIES:
            raise ValueError(f'strategy must be one of {STRATEGIES}')
        if not (p['k_rho'] > 0 and p['k_beta'] < 0 and p['k_alpha'] > p['k_rho']):
            raise ValueError('polar gains need k_rho > 0, k_beta < 0, k_alpha > k_rho')
        if len(path) == 0:
            raise ValueError('path must contain at least one point')
        super().__init__(p)
        self.strategy = p['strategy']
        self.turn_in_place = math.radians(float(p['turn_in_place_deg']))
        self.path = [tuple(map(float, pt)) for pt in path]     # world-frame (x, y, theta)
        self.polyline = []
        self.idx = 0
        self.seg = 0
        self.mode = GO
        self.events = []             # human-readable messages for the caller to log
        self._wait = 0               # steps spent waiting for the robot to settle
        self._retries = 0
        self._pos_retries = 0
        self._fine = 'turn'          # heading alignment: turn (P law) -> settle -> pulse -> settle ...
        self._pulse_left = 0
        self._pulse_w = 0.0
        self._min_wait = 0
        self.gx, self.gy, self.gth = self.path[0]

    def set_path(self, path):
        """Replace the (world-frame) path, e.g. after transforming a start-relative path."""
        self.path = [tuple(map(float, pt)) for pt in path]
        self.gx, self.gy, self.gth = self.path[0]

    def start(self, pose):
        """Fix the polyline used by pure pursuit: robot start position, then the path points."""
        self.polyline = [(pose[0], pose[1])] + [(a, b) for a, b, _ in self.path]

    def pop_events(self):
        ev, self.events = self.events, []
        return ev

    def _turn_at(self, i):
        """How much the path turns at point i (degrees); the last point counts as a full stop."""
        if i >= len(self.path) - 1 or i + 1 >= len(self.polyline):
            return 180.0
        ax, ay = self.polyline[i]
        bx, by = self.polyline[i + 1]
        cx, cy = self.polyline[i + 2]
        h1, h2 = math.atan2(by - ay, bx - ax), math.atan2(cy - by, cx - bx)
        return abs(math.degrees(wrap_angle(h2 - h1)))

    def _fine_align_applies(self):
        p = self.p
        return (p['rest_window'] > 0.0 and self.strategy != 'pure_pursuit' and
                self._turn_at(self.idx) > p['rest_corner_deg'])

    def _finish_point(self, x, y, yaw, err):
        self.events.append(
            f'point {self.idx + 1}/{len(self.path)} done '
            f'(pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f} deg, '
            f'yaw err {math.degrees(err):.2f} deg)')
        self.idx += 1
        self._wait = 0
        self._retries = 0
        self._pos_retries = 0
        self._fine = 'turn'
        if self.idx >= len(self.path) or self.strategy == 'pure_pursuit':
            self.mode = DONE
            self.events.append('path complete')
        else:
            self.mode = GO

    def _align_fine(self, pose, pred, gx, gy, gth, dt):
        """Heading alignment with verification at rest and exact minimum-size pulses.
        turn  : P law on the predicted heading until it is within tolerance
        settle: command 0 until the robot has really stopped (or rest_timeout), then check the TRUE error
        pulse : n samples of +-pulse_w, n = error / (pulse_w * dt); back to settle."""
        p = self.p
        ax, ay, ath = pose
        if self._fine == 'turn':
            w, err = yaw_p_control(gth, pred[2], p['k_yaw'], p['max_angular_speed'],
                                   p['min_angular_speed'], p['yaw_tolerance'])
            if abs(err) > p['yaw_tolerance']:
                return 0.0, w
            self._fine, self._wait, self._min_wait = 'settle', 0, 0
        if self._fine == 'settle':
            self._wait += 1
            if self._wait <= self._min_wait or (
                    not self._at_rest(dt) and self._wait <= int(p['rest_timeout'] / dt) + self._min_wait):
                return 0.0, 0.0
            ax, ay, ath = self._mean_pose(pose)            # noise-averaged pose of the settled robot
            e = wrap_angle(gth - ath)                      # true heading error, robot at rest
            off = math.hypot(gx - ax, gy - ay)
            heading_ok = abs(e) <= p['yaw_tolerance'] or self._retries >= p['rest_retries']
            if heading_ok:
                # heading is done (or its pulse budget is spent): the position must be right too. A separate
                # budget, so heading pulses can never use up the chance to correct a position error.
                if off > p['pos_tolerance'] and self._pos_retries < p['pos_retries']:
                    self._pos_retries += 1
                    self._retries = 0                  # the re-approach changes the heading: new pulse budget
                    self.events.append(f'point {self.idx + 1}/{len(self.path)} at rest but '
                                       f'{off * 100:.1f} cm off: re-approaching')
                    self.mode, self._fine, self._min_wait = GO, 'turn', 0
                    return 0.0, 0.0
                self._finish_point(ax, ay, ath, e)
                return 0.0, 0.0
            self._retries += 1
            wp = p['pulse_w'] or (p['deadband_w'] * DEADBAND_MARGIN if p['deadband_w'] > 0.0 else 0.05)
            self._pulse_left = max(1, round(abs(e) / (wp * dt))) + int(p['pulse_extra'])
            self._pulse_w = math.copysign(wp, e)
            self._fine = 'pulse'
        # pulse
        self._pulse_left -= 1
        if self._pulse_left <= 0:
            self._fine, self._wait = 'settle', 0
            self._min_wait = int(p['settle_time'] / dt)
            self._hist.clear()                    # the at-rest window must contain only post-pulse data
        return 0.0, self._pulse_w

    # ---- one control step ----------------------------------------------------------------
    def step(self, pose, dt=0.05):
        p = self.p
        if self.mode == DONE:
            return 0.0, 0.0
        x, y, yaw = self._predict(pose, dt)
        v_max, w_max = p['max_linear_speed'], p['max_angular_speed']
        v_min, w_min = p['min_linear_speed'], p['min_angular_speed']
        v = w = 0.0
        while True:
            gx, gy, gth = self.path[self.idx]
            last = self.idx == len(self.path) - 1
            v = w = 0.0
            if self.mode == GO:
                if self.strategy == 'stop_turn':
                    v, w, dist, _ = go_to_point_control(
                        x, y, yaw, gx, gy, p['kv'], p['kw'], v_max, w_max, self.turn_in_place)
                    arrived = dist <= p['pos_tolerance']
                elif self.strategy == 'polar':
                    v, w, dist, _ = pose_control(
                        x, y, yaw, gx, gy, gth, p['k_rho'], p['k_alpha'], p['k_beta'],
                        v_max, w_max, p['decel'] if last else 0.0)
                    if dist <= (p['pos_tolerance'] if last else p['blend_radius']):
                        if not last:     # blend: hand over to the next point without stopping
                            self.events.append(
                                f'point {self.idx + 1}/{len(self.path)} passed '
                                f'(pos=({x:.3f},{y:.3f}), {dist * 100:.1f} cm off)')
                            self.idx += 1
                            continue
                        arrived = True
                    else:
                        arrived = False
                else:  # pure_pursuit
                    self.seg, t = project_on_path(self.polyline, x, y, self.seg)
                    lx, ly, rem = lookahead_point(self.polyline, self.seg, t, p['lookahead'])
                    # path length left, but never less than the straight-line distance to the end:
                    # after an overshoot the projection is clamped to the end and rem would be 0
                    # (speed cap 0 = deadlock)
                    rem = max(rem, math.hypot(self.polyline[-1][0] - x, self.polyline[-1][1] - y))
                    v, w, _ = pure_pursuit_control(
                        x, y, yaw, (lx, ly), v_max, w_max, p['lookahead'], rem, p['decel'])
                    self.idx = min(self.seg, len(self.path) - 1)    # progress, for the log
                    gx, gy, gth = self.path[-1]
                    dist = math.hypot(gx - x, gy - y)
                    arrived = self.seg >= len(self.polyline) - 2 and dist <= p['pos_tolerance']
                if arrived:
                    v = w = 0.0
                    self.mode = ALIGN
                    self.events.append(
                        f'point {self.idx + 1}/{len(self.path)} position reached '
                        f'(err {dist * 100:.1f} cm), aligning')
                else:
                    if 0.0 < v < v_min:
                        v = v_min
                    if v == 0.0 and 0.0 < abs(w) < w_min:
                        w = math.copysign(w_min, w)
            break
        if self.mode == ALIGN:
            if self.strategy == 'pure_pursuit':
                gx, gy, gth = self.path[-1]
            if self._fine_align_applies():
                v, w = self._align_fine(pose, (x, y, yaw), gx, gy, gth, dt)
                self.gx, self.gy, self.gth = gx, gy, gth
                return self._shape(v, w)
            tol = p['yaw_tolerance']
            if (p['smooth_yaw_tolerance'] > tol and self.strategy != 'pure_pursuit' and
                    self._turn_at(self.idx) <= p['rest_corner_deg']):
                tol = p['smooth_yaw_tolerance']          # smooth curve point: no need to align exactly
            w, err = yaw_p_control(gth, yaw, p['k_yaw'], w_max, w_min, tol)
            v = 0.0
            if abs(err) <= tol:
                self._finish_point(x, y, yaw, err)
                w = 0.0
        self.gx, self.gy, self.gth = gx, gy, gth
        return self._shape(v, w)


# ---- yaw controller -----------------------------------------------------------------------------
# Parameters shared with the motion controller (same meaning, same defaults).
COMPENSATION_KEYS = ('lead_w', 'vel_filter', 'vel_window', 'rest_window', 'rest_pos', 'rest_yaw', 'rest_timeout',
                     'settle_time', 'rest_retries', 'pulse_w', 'pulse_extra',
                     'diamond_v', 'diamond_w', 'deadband_v', 'deadband_w')

YAW_DEFAULTS = dict(
    kp=1.0,                          # 1/s   P gain: w = kp * (yaw_ref - yaw)
    max_angular_speed=1.0,           # rad/s
    min_angular_speed=0.0,           # rad/s deadband compensation (legacy mode)
    tolerance=0.005,                 # rad
    hold_band=3.0,                   # x tolerance: a settled robot pushed further than this re-engages
    lead_v=0.0,                      # unused by the yaw controller (the shared predictor reads it)
    **{k: DEFAULTS[k] for k in COMPENSATION_KEYS})


class YawTracker(_Compensator):
    """Yaw reference tracking. With rest_window == 0 it is the plain P law (legacy behaviour). With
    rest_window > 0 it uses the lead prediction, waits for the robot to really stop, checks the true error
    and corrects with exact minimum-size pulses (needed when the robot has a command deadband and a slow
    velocity response)."""

    def __init__(self, params=None):
        p = dict(YAW_DEFAULTS)
        p.update(params or {})
        super().__init__(p)
        self.ref = None
        self.reached = False
        self.events = []
        self._reset()

    def _reset(self):
        self._fine = 'turn'          # turn -> settle -> pulse -> settle ... -> hold
        self._wait = self._min_wait = self._retries = 0
        self._pulse_left = 0
        self._pulse_w = 0.0
        self.reached = False

    def pop_events(self):
        ev, self.events = self.events, []
        return ev

    def step(self, yaw, ref, dt):
        """Return the angular speed command for the current heading `yaw` and reference `ref` (rad)."""
        p = self.p
        if self.ref is None or abs(wrap_angle(ref - self.ref)) > 1e-9:
            self.ref = ref
            self._reset()
        pose = (0.0, 0.0, yaw)
        pred = self._predict(pose, dt)[2]
        tol = p['tolerance']
        if p['rest_window'] <= 0.0:                                  # legacy P law (+ optional lead)
            w, err = yaw_p_control(ref, pred, p['kp'], p['max_angular_speed'], p['min_angular_speed'], tol)
            self.reached = w == 0.0
            return self._shape(0.0, w)[1]
        if self._fine == 'hold':
            if abs(wrap_angle(ref - yaw)) <= p['hold_band'] * tol:
                return 0.0
            self._reset()                                            # pushed away: re-engage
        if self._fine == 'turn':
            w, err = yaw_p_control(ref, pred, p['kp'], p['max_angular_speed'], p['min_angular_speed'], tol)
            if abs(err) > tol:
                return self._shape(0.0, w)[1]
            self._fine, self._wait, self._min_wait = 'settle', 0, 0
        if self._fine == 'settle':
            self._wait += 1
            if self._wait <= self._min_wait or (
                    not self._at_rest(dt) and self._wait <= int(p['rest_timeout'] / dt) + self._min_wait):
                return 0.0
            e = wrap_angle(ref - self._mean_pose(pose)[2])           # true error of the settled robot
            if abs(e) <= tol or self._retries >= p['rest_retries']:
                self._fine = 'hold'
                self.reached = True
                return 0.0
            self._retries += 1
            wp = p['pulse_w'] or (p['deadband_w'] * DEADBAND_MARGIN if p['deadband_w'] > 0.0 else 0.05)
            self._pulse_left = max(1, round(abs(e) / (wp * dt))) + int(p['pulse_extra'])
            self._pulse_w = math.copysign(wp, e)
            self._fine = 'pulse'
        self._pulse_left -= 1                                        # pulse
        if self._pulse_left <= 0:
            self._fine, self._wait = 'settle', 0
            self._min_wait = int(p['settle_time'] / dt)
            self._hist.clear()
        return self._shape(0.0, self._pulse_w)[1]
