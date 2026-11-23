"""Path tracker without ROS: the ONE implementation of the motion strategies.

`motion_controller.py` (ROS node) and the model tools (`tools/sim_compare.py`, `tools/shape_bench.py`)
all call `PathTracker.step()`, so what is tuned in the model is exactly what runs on the robot.

The tracker works in the world (odom) frame: the caller transforms a start-relative path first and calls
`start(pose)` once. `step(pose)` returns the raw (v, w) command; the caller applies acceleration limits
(`slew`) and publishes.

HOW IT WORKS (a small state machine, one point of the path at a time)

    GO     drive towards path point i with the chosen strategy
             stop_turn     turn towards the point, drive, slow down       (law: go_to_point_control)
             polar         one smooth motion that also reaches the heading (law: pose_control)
             pure_pursuit  chase a look-ahead point along the whole path   (law: pure_pursuit_control)
       |  position reached
       v
    ALIGN  rotate in place to the heading of point i (P law on the yaw error)
       |  heading reached -> next point (GO), or DONE after the last point
       v
    DONE   command 0

Slow-plant compensation (lead prediction, rest check, corrective pulses, command shaping) lives in
`compensation.py` and is off unless the parameters are set.
"""

import math

from lab3_controllers.common import (
    go_to_point_control, lookahead_point, pose_control, project_on_path,
    pure_pursuit_control, wrap_angle, yaw_p_control)
from lab3_controllers.compensation import Compensator
from lab3_controllers.observer import HeadingObserver

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
    # --- slow-plant compensation (all off by default; see compensation.py and tools/plant_models.py) ---
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
    pose_latency=0.0,                # s   timed mode: delay of the state message (the sample is older than its arrival)
    vel_span=0.0,                    # s   timed mode: speed = least-squares slope over this many seconds of samples (0 = use vel_window)
    rest_mean_test=False,            # timed mode: judge "at rest" by window averages (robust to noisy poses)
    rest_retries=4,                  # heading pulses per point before accepting the heading
    pos_retries=3,                   # position re-approaches per point before accepting the position
    # --- robot command filter (professor's input_filter): commands outside the diamond or inside the
    #     deadband are DROPPED by the robot, so the controller must never send them (0 = off) ---
    diamond_v=0.0,                   # m/s   feasible set |v|/diamond_v + |w|/diamond_w <= 1   (lab: 0.3)
    diamond_w=0.0,                   # rad/s                                                  (lab: 2.5)
    deadband_v=0.0,                  # m/s   command is dropped if |v| <= deadband_v and ...  (lab: 0.04)
    deadband_w=0.0,                  # rad/s ... |w| <= deadband_w                            (lab: 0.2)
)


class PathTracker(Compensator):
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
        self.polyline = []           # pure pursuit: start position, then the path points
        self.idx = 0                 # index of the current path point
        self.seg = 0                 # pure pursuit: current polyline segment
        self.mode = GO               # GO | ALIGN | DONE
        self.events = []             # human-readable messages for the caller to log
        # state of the "verify at rest" alignment (only used when rest_window > 0)
        self._fine = 'turn'          # turn (P law) -> settle (wait for rest) -> pulse -> settle ...
        self._wait = 0               # steps spent waiting for the robot to settle
        self._min_wait = 0           # steps to wait at least (after a pulse)
        self._retries = 0            # heading pulses used at this point
        self._pos_retries = 0        # position re-approaches used at this point
        self._pulse_left = 0         # samples left in the running pulse
        self._pulse_w = 0.0          # signed size of the running pulse
        self.gx, self.gy, self.gth = self.path[0]    # goal of the last step (for logging)

    # ---- small public helpers --------------------------------------------------------------
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

    # ======================================================================================
    # ONE CONTROL STEP
    # ======================================================================================
    def step(self, pose, dt=0.05, stamp=None, age=0.0):
        """Return the (v, w) command for the measured pose (x, y, yaw). `stamp` (time of the measurement) and `age` (seconds
        since it arrived) switch on the timed mode for a slow, late state (see compensation.py); None = the classic mode."""
        if self.mode == DONE:
            return 0.0, 0.0
        # With compensation on, control the pose the robot WILL rest at; otherwise this is the pose itself.
        x, y, yaw = self._predict(pose, dt) if stamp is None else self._predict_timed(pose, stamp, age)

        v = w = 0.0
        if self.mode == GO:
            v, w = self._step_go(x, y, yaw)

        if self.mode == ALIGN:             # also true in the same step the position was reached
            gx, gy, gth = self._goal()
            self.gx, self.gy, self.gth = gx, gy, gth
            if self._fine_align_applies():
                v, w = self._align_fine(pose, (x, y, yaw), gx, gy, gth, dt)
            else:
                v, w = self._align_simple(x, y, yaw, gth)
            return self._shape(v, w)

        self.gx, self.gy, self.gth = self._goal()
        return self._shape(v, w)

    def _goal(self):
        """Goal of the current step: pure pursuit always aims at the end of the path."""
        return self.path[-1] if self.strategy == 'pure_pursuit' else self.path[self.idx]

    # ======================================================================================
    # GO: drive towards the point (one method per strategy; each returns v, w, dist, arrived)
    # ======================================================================================
    def _step_go(self, x, y, yaw):
        p = self.p
        if self.strategy == 'stop_turn':
            v, w, dist, arrived = self._go_stop_turn(x, y, yaw)
        elif self.strategy == 'polar':
            v, w, dist, arrived = self._go_polar(x, y, yaw)
        else:
            v, w, dist, arrived = self._go_pure_pursuit(x, y, yaw)

        if arrived:                                        # stop and align the heading
            self.mode = ALIGN
            self.events.append(f'point {self.idx + 1}/{len(self.path)} position reached '
                               f'(err {dist * 100:.1f} cm), aligning')
            return 0.0, 0.0
        # deadband compensation: never command a non-zero speed below the minimum
        if 0.0 < v < p['min_linear_speed']:
            v = p['min_linear_speed']
        if v == 0.0 and 0.0 < abs(w) < p['min_angular_speed']:
            w = math.copysign(p['min_angular_speed'], w)
        return v, w

    def _go_stop_turn(self, x, y, yaw):
        """Turn towards the point (in place if the bearing error is large), drive, slow down."""
        p = self.p
        gx, gy, _ = self.path[self.idx]
        v, w, dist, _ = go_to_point_control(x, y, yaw, gx, gy, p['kv'], p['kw'],
                                            p['max_linear_speed'], p['max_angular_speed'], self.turn_in_place)
        return v, w, dist, dist <= p['pos_tolerance']

    def _go_polar(self, x, y, yaw):
        """Polar-coordinate law towards position AND heading. Intermediate points are not stopped at:
        within blend_radius the tracker hands over to the next point at once."""
        p = self.p
        while True:
            gx, gy, gth = self.path[self.idx]
            last = self.idx == len(self.path) - 1
            v, w, dist, _ = pose_control(x, y, yaw, gx, gy, gth, p['k_rho'], p['k_alpha'], p['k_beta'],
                                         p['max_linear_speed'], p['max_angular_speed'],
                                         p['decel'] if last else 0.0)
            if last:
                return v, w, dist, dist <= p['pos_tolerance']
            if dist > p['blend_radius']:
                return v, w, dist, False
            self.events.append(f'point {self.idx + 1}/{len(self.path)} passed '
                               f'(pos=({x:.3f},{y:.3f}), {dist * 100:.1f} cm off)')
            self.idx += 1                                  # hand over and recompute for the next point

    def _go_pure_pursuit(self, x, y, yaw):
        """Chase a look-ahead point on the path; slow down towards the end."""
        p = self.p
        self.seg, t = project_on_path(self.polyline, x, y, self.seg)
        lx, ly, rem = lookahead_point(self.polyline, self.seg, t, p['lookahead'])
        # path length left, but never less than the straight-line distance to the end:
        # after an overshoot the projection is clamped to the end and rem would be 0
        # (speed cap 0 = deadlock)
        rem = max(rem, math.hypot(self.polyline[-1][0] - x, self.polyline[-1][1] - y))
        v, w, _ = pure_pursuit_control(x, y, yaw, (lx, ly), p['max_linear_speed'], p['max_angular_speed'],
                                       p['lookahead'], rem, p['decel'])
        self.idx = min(self.seg, len(self.path) - 1)       # progress, for the log
        gx, gy, _ = self.path[-1]
        dist = math.hypot(gx - x, gy - y)
        return v, w, dist, self.seg >= len(self.polyline) - 2 and dist <= p['pos_tolerance']

    # ======================================================================================
    # ALIGN: rotate in place to the heading of the point
    # ======================================================================================
    def _align_simple(self, x, y, yaw, gth):
        """P law on the yaw error; the point is finished when the error is inside the tolerance."""
        p = self.p
        tol = p['yaw_tolerance']
        if (p['smooth_yaw_tolerance'] > tol and self.strategy != 'pure_pursuit' and
                self._turn_at(self.idx) <= p['rest_corner_deg']):
            tol = p['smooth_yaw_tolerance']              # smooth curve point: no need to align exactly
        w, err = yaw_p_control(gth, yaw, p['k_yaw'], p['max_angular_speed'], p['min_angular_speed'], tol)
        if abs(err) <= tol:
            self._finish_point(x, y, yaw, err)
            w = 0.0
        return 0.0, w

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
        """Verify-at-rest alignment is used at corners and at the last point, when it is enabled."""
        p = self.p
        return (p['rest_window'] > 0.0 and self.strategy != 'pure_pursuit' and
                self._turn_at(self.idx) > p['rest_corner_deg'])

    def _finish_point(self, x, y, yaw, err):
        """Point done: log it and move on (GO to the next point, or DONE)."""
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
                return 0.0, 0.0                                # still moving: keep waiting
            ax, ay, ath = self._mean_pose(pose)                # noise-averaged pose of the settled robot
            e = wrap_angle(gth - ath)                          # true heading error, robot at rest
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
            # heading still off: start a corrective pulse of exactly the right length
            self._retries += 1
            wp = self._pulse_size()
            self._pulse_left = max(1, round(abs(e) / (wp * dt))) + int(p['pulse_extra'])
            self._pulse_w = math.copysign(wp, e)
            self._fine = 'pulse'
        # pulse
        self._pulse_left -= 1
        if self._pulse_left <= 0:
            self._fine, self._wait = 'settle', 0
            self._min_wait = int(p['settle_time'] / dt)
            self._clear_rest_history()            # the at-rest window must contain only post-pulse data
        return 0.0, self._pulse_w


# ---- yaw controller (single-state P heading control) -------------------------------------------
# Parameters shared with the motion controller (same meaning, same defaults).
COMPENSATION_KEYS = ('lead_w', 'pose_latency', 'vel_span', 'rest_mean_test', 'vel_filter', 'vel_window', 'rest_window', 'rest_pos', 'rest_yaw', 'rest_timeout',
                     'settle_time', 'rest_retries', 'pulse_w', 'pulse_extra',
                     'diamond_v', 'diamond_w', 'deadband_v', 'deadband_w')

YAW_DEFAULTS = dict(
    kp=1.0,                          # 1/s   P gain: w = kp * (yaw_ref - yaw)
    max_angular_speed=1.0,           # rad/s
    min_angular_speed=0.0,           # rad/s deadband compensation (legacy mode)
    tolerance=0.005,                 # rad
    hold_band=3.0,                   # x tolerance: a settled robot pushed further than this re-engages
    # --- model-based observer (timed mode only): simulate the robot from the commands we sent, correct it with the late pose ---
    use_observer=False,              # predict the present heading with the model instead of extrapolating noisy poses
    obs_tau=0.8,                     # s   robot turn-rate lag (time constant)
    obs_gain=1.0,                    #     robot turn-rate gain (achieved / commanded)
    obs_k1=0.5,                      #     how much of the heading difference a new sample corrects (0..1)
    obs_k2=0.5,                      #     1/s how much of it corrects the turn rate
    ki=0.0,                          # 1/s^2 integral gain (PID form): pushes through the last degrees; 0 = off
    lead_v=0.0,                      # unused by the yaw controller (the shared predictor reads it)
    **{k: DEFAULTS[k] for k in COMPENSATION_KEYS})


class YawTracker(Compensator):
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
        self._obs = None             # HeadingObserver, built on first use
        self._clock = 0.0            # own clock when the caller passes no time
        self._obs_stamp = None
        self._u_sent = 0.0           # the command that was really sent (the model needs it)
        self._cmd_fed = False
        self._reset()

    def observe_command(self, w_sent):
        """Tell the controller the turn rate that was really published (after the caller's acceleration limit). Optional:
        without it the observer assumes the command this controller returned."""
        self._u_sent = w_sent
        self._cmd_fed = True

    def _reset(self):
        self._i = 0.0                # integral term
        self._fine = 'turn'          # turn -> settle -> pulse -> settle ... -> hold
        self._wait = self._min_wait = self._retries = 0
        self._pulse_left = 0
        self._pulse_w = 0.0
        self.reached = False

    def pop_events(self):
        ev, self.events = self.events, []
        return ev

    def step(self, yaw, ref, dt, stamp=None, age=0.0, now=None):
        """Return the angular speed command for the current heading `yaw` and reference `ref` (rad). `stamp` / `age`: see
        PathTracker.step (timed mode for a slow, late state); `now` is the caller's clock in seconds (observer only)."""
        w = self._step(yaw, ref, dt, stamp, age, now)
        if not self._cmd_fed:
            self._u_sent = w
        return w

    def _observed_prediction(self, yaw, stamp, age, now, dt):
        """Heading where the robot will rest, from the model-based observer (use_observer)."""
        p = self.p
        if self._obs is None:
            self._obs = HeadingObserver(tau=p['obs_tau'], gain=p['obs_gain'], latency=p['pose_latency'], k1=p['obs_k1'],
                                        k2=p['obs_k2'], deadband_w=p['deadband_w'])
        if now is None:
            self._clock += dt
            now = self._clock
        self._obs.advance(now, self._u_sent)
        if stamp != self._obs_stamp:                                   # a new sample
            self._obs_stamp = stamp
            self._obs.correct(now - max(age, 0.0), yaw)
            self._obs.advance(now, self._u_sent)
        return self._obs.predicted_rest(p['lead_w'])

    def _step(self, yaw, ref, dt, stamp, age, now):
        p = self.p
        if self.ref is None or abs(wrap_angle(ref - self.ref)) > 1e-9:
            self.ref = ref
            self._reset()
        pose = (0.0, 0.0, yaw)
        if stamp is None:
            pred = self._predict(pose, dt)[2]
        else:
            pred = self._predict_timed(pose, stamp, age)[2]           # also keeps the rest-test samples
            if p['use_observer']:
                pred = self._observed_prediction(yaw, stamp, age, now, dt)
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
                if p['ki'] > 0.0:                                     # PID form: integrate the error near the target
                    if abs(err) < 0.35:
                        self._i = max(-0.5 / p['ki'], min(0.5 / p['ki'], self._i + err * dt))
                    else:
                        self._i = 0.0
                    w = max(-p['max_angular_speed'], min(p['max_angular_speed'], w + p['ki'] * self._i))
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
            wp = self._pulse_size()
            self._pulse_left = max(1, round(abs(e) / (wp * dt))) + int(p['pulse_extra'])
            self._pulse_w = math.copysign(wp, e)
            self._fine = 'pulse'
        self._pulse_left -= 1                                        # pulse
        if self._pulse_left <= 0:
            self._fine, self._wait = 'settle', 0
            self._min_wait = int(p['settle_time'] / dt)
            self._clear_rest_history()
        return self._shape(0.0, self._pulse_w)[1]
