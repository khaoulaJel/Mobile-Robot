"""A small P-family servo that runs on MEASUREMENTS, not on a model of the robot (no ROS in this file).

    MapOdomFusion   state = fast wheel odometry (pose + MEASURED speed and turn rate), moved into the map frame by the
                    slower SLAM pose. The controller works in the professor's frame without waiting for SLAM.
    AxisServo       one axis (heading, or distance along a line). Two regimes:
                      FAR    e_pred = e - lead * rate          derivative action on the MEASURED rate
                             u = kp * e_pred (+ ki * integral), never between 0 and u_min (the robot ignores tiny commands)
                      NEAR   pulse, wait until standing still, look again. A pulse moves the robot by gain * command * length
                             whatever its lag and delay are, so the pulse LENGTH is made proportional to the error.
                    u_min (the robot's deadband) and the gain are LEARNED from what each command really did.
    PoseServo       go to (x, y, heading): turn to the point, drive to it, turn to the final heading, built from two AxisServo.
"""
import math
from collections import deque

from lab3_controllers.common import wrap_angle


def compose(frame, pose):
    """pose given in `frame` -> pose in the parent. Both are (x, y, theta)."""
    fx, fy, fth = frame
    x, y, th = pose
    c, s = math.cos(fth), math.sin(fth)
    return fx + c * x - s * y, fy + s * x + c * y, wrap_angle(fth + th)


def offset_between(map_pose, odom_pose):
    """The map<-odom frame that turns `odom_pose` into `map_pose` (both describe the robot at the same instant)."""
    dth = wrap_angle(map_pose[2] - odom_pose[2])
    c, s = math.cos(dth), math.sin(dth)
    return (map_pose[0] - (c * odom_pose[0] - s * odom_pose[1]), map_pose[1] - (s * odom_pose[0] + c * odom_pose[1]), dth)


class MapOdomFusion:
    """add_odom(stamp, pose, v, w) at the odometry rate; add_map(stamp, pose) whenever a map pose arrives; state() = latest
    odometry expressed in the map frame. The two stamps must come from the same clock (they do: SLAM stamps its pose with
    the odometry time). `blend` is the share of a new map correction applied at once (1 = jump immediately)."""

    def __init__(self, use_map=True, blend=0.5, history=5.0, max_pair_gap=0.3):
        self.use_map, self.blend, self.history, self.max_pair_gap = use_map, blend, history, max_pair_gap
        self._odom = deque()                 # (stamp, pose)
        self.v = self.w = 0.0
        self.offset = None if use_map else (0.0, 0.0, 0.0)
        self.rejected = 0                    # map samples that could not be paired with odometry

    def add_odom(self, stamp, pose, v, w):
        if self._odom and stamp <= self._odom[-1][0]:
            return                           # repeated or out-of-order sample
        self._odom.append((stamp, pose))
        while stamp - self._odom[0][0] > self.history:
            self._odom.popleft()
        self.v, self.w = v, w

    def odom_at(self, stamp):
        """Odometry pose at `stamp` (interpolated), or None if we have nothing near that time."""
        h = self._odom
        if not h:
            return None
        if stamp >= h[-1][0]:
            return h[-1][1] if stamp - h[-1][0] <= self.max_pair_gap else None
        if stamp <= h[0][0]:
            return h[0][1] if h[0][0] - stamp <= self.max_pair_gap else None
        for i in range(len(h) - 1, 0, -1):
            t0, a = h[i - 1]
            t1, b = h[i]
            if t0 <= stamp <= t1:
                if t1 - t0 > 2 * self.max_pair_gap:
                    return None
                f = (stamp - t0) / (t1 - t0)
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), wrap_angle(a[2] + f * wrap_angle(b[2] - a[2])))
        return None

    def add_map(self, stamp, pose):
        if not self.use_map:
            return False
        odom = self.odom_at(stamp)
        if odom is None:
            self.rejected += 1
            return False
        new = offset_between(pose, odom)
        if self.offset is None:
            self.offset = new
        else:
            # blend around the robot's present position so that blending the angle does not swing a distant offset
            now_old = compose(self.offset, odom)
            now_new = compose(new, odom)
            a = self.blend
            target = (now_old[0] + a * (now_new[0] - now_old[0]), now_old[1] + a * (now_new[1] - now_old[1]),
                      wrap_angle(now_old[2] + a * wrap_angle(now_new[2] - now_old[2])))
            self.offset = offset_between(target, odom)
        return True

    def ready(self):
        return bool(self._odom) and self.offset is not None

    def state(self):
        """((x, y, theta) in the map frame, forward speed, turn rate)."""
        return compose(self.offset, self._odom[-1][1]), self.v, self.w


# Heading values tuned by tools/servo_study.py (structure "PD + pulses + learning"), results/real_robot/servo_study.json
SERVO_DEFAULTS = dict(
    kp=2.042,                  # 1/s    proportional gain
    ki=0.0,                  # 1/s^2  integral gain (PI / PID); 0 = off
    lead=1.051,                # s      derivative: predicted error = error - lead * measured rate (start value of the learned one)
    lead_min=0.1, lead_max=2.5,  # s    the learned coasting time stays inside these
    u_max=0.914,               #        largest command (rad/s or m/s)
    accel=2.0,               #        command slew limit per second (0 = off)
    tol=0.012,               #        accepted error (rad or m)
    hold=2.0,                #        once arrived, re-engage only if the error exceeds hold * tol
    # the small commands the robot ignores
    u_min=0.252,               #        smallest command sent (start value of the learned one); 0 = no minimum
    u_min_floor=0.03,        #        the learned minimum stays inside [floor, cap]
    u_min_cap=0.6,
    adapt=True,              #        learn u_min and the robot gain while running
    stall_time=0.82,          # s      commanded but the POSITION has not changed for this long -> raise u_min
    stall_range=1.6,         #        ... judged only for small commands (up to this x u_min): a big one is just still starting
    grow=1.313,               #        factor when the robot ignored a command
    shrink=0.936,              #        factor when the shortest pulse moves far more than needed
    # final approach by pulses: displacement of a pulse = gain * command * length, whatever the lag and delay are
    pulses=True,
    pulse_below=0.252,         #        below this error (rad or m) a settled robot is corrected by pulses (start value, see near_max)
    near_max=0.8,            #        a continuous move that overshoots RAISES that threshold (never above this): too sluggish a robot
    pulse_max=1.0,           # s      longest single pulse
    # start-up loss: some robots lose a fixed time every time they start from rest, so a short pulse does nothing however strong.
    # Lost time = dead_fix + dead_imp / |command| (measure with tools/robot_identify.py). Both 0 = the robot has no such loss.
    dead_fix=0.0,            # s
    dead_imp=0.0,            # rad (m): command x time the robot swallows at every start
    dead_same=0.0,           # s      MORE is lost when the pulse goes the same way as the robot's last movement
    dead_step=0.04,          # s      a pulse that did nothing makes the next ones this much LONGER ...
    dead_extra_max=0.25,     # s      ... up to this much; only then is the command made stronger
    pulse_share=0.804,        #        aim for this share of the error (undershooting is safer than overshooting)
    rest_rate=0.02,          #        filtered |rate| below this = standing still
    rest_time=0.302,           # s      ... for this long
    settle_min=0.743,          # s      wait at least this long after the last command (command and sensor delays)
    settle_max=3.154,          # s      stop waiting (a noisy rate may never look still)
    rate_filter=0.748,         #        0..1 low-pass gain of the measured rate (1 = no filter)
)


class AxisServo:
    """States:  move (continuous PD)  ->  settle (command 0 until standing still)  ->  pulse  ->  settle ...  ->  arrived"""

    def __init__(self, params=None):
        p = dict(SERVO_DEFAULTS)
        p.update(params or {})
        if p['kp'] <= 0 or p['u_max'] <= 0 or p['tol'] <= 0 or p['u_min_floor'] > p['u_min_cap']:
            raise ValueError('servo needs kp > 0, u_max > 0, tol > 0 and u_min_floor <= u_min_cap')
        self.p = p
        # learned about the ROBOT: these survive reset()
        self.u_min = p['u_min']
        self.ignored = 0.0                   # the largest command the robot was seen to ignore
        self.gain = 1.0                      # achieved motion / commanded motion, from the pulses
        self.lead = p['lead']                # coasting time, measured after each move (includes every delay)
        self._coasts = 0
        # learned corrections of the start-up loss (s), kept apart: for a pulse that changes direction and for one that does not
        self.dead_extra = 0.0
        self.same_extra = 0.0
        self.moved_dir = 0                   # the way the robot last moved (+1, -1; 0 = not known): physical, survives reset()
        self._pulse_same = False
        self.near = p['pulse_below']         # use pulses below this error; grows if continuous moves overshoot
        self.reset()

    def reset(self):
        self.state = 'move'
        self.u = 0.0
        self.rate = 0.0
        self._i = 0.0
        self._stall = 0.0
        self._stall_ref = None               # error when the current stall check started
        self._quiet = self._wait = 0.0
        self._pulse_left = 0
        self._pulse_u = 0.0
        self._pulse = None                   # (error before, command, length) of the pulse being judged
        self._stop = None                    # (error, rate) at the moment a continuous move was stopped

    @property
    def arrived(self):
        return self.state == 'arrived'

    def pause(self):
        """The caller had to send zero (stale data): drop the running command, keep what was learned."""
        self.u = 0.0
        self._stall = 0.0
        self._stall_ref = None
        self._stop = None                    # data was lost: the coast cannot be measured
        if self.state == 'pulse':
            self._pulse = None               # an interrupted pulse teaches nothing
        if self.state != 'arrived':
            self._to_settle()

    def _to_settle(self):
        self.state, self.u, self._quiet, self._wait, self._i, self._stall = 'settle', 0.0, 0.0, 0.0, 0.0, 0.0
        self._stall_ref = None

    def _raise_minimum(self, ignored_u):
        if not self.p['adapt']:
            return
        self.ignored = max(self.ignored, min(abs(ignored_u), self.p['u_min_cap']))
        self.u_min = min(self.p['u_min_cap'], max(self.u_min, abs(ignored_u)) * self.p['grow'])

    def update(self, error, rate, dt):
        """Command for the error (target - measured, already wrapped for angles) and the MEASURED rate."""
        p = self.p
        self.rate += p['rate_filter'] * (rate - self.rate)
        if self.state == 'arrived':
            if abs(error) <= p['hold'] * p['tol']:
                return 0.0
            self._to_settle()
            self._pulse = None
        if self.state == 'settle':
            return self._settle(error, dt)
        if self.state == 'pulse':
            if self._pulse_left > 0:
                self._pulse_left -= 1
                self.u = self._pulse_u
                return self.u
            self._to_settle()
            return 0.0
        return self._move(error, dt)

    def lost_time(self, u, same=False):
        """Seconds a pulse of strength u loses at its start (without the learned correction); 0 = no start-up loss.
        `same`: the pulse goes the way the robot last moved."""
        if self.p['dead_fix'] <= 0.0 and self.p['dead_imp'] <= 0.0:
            return 0.0
        return self.p['dead_fix'] + self.p['dead_imp'] / max(abs(u), 1e-3) + (self.p['dead_same'] if same else 0.0)

    # ---- continuous PD (PID) ---------------------------------------------------------------
    def _move(self, error, dt):
        p = self.p
        pred = error - self.lead * self.rate
        if p['pulses'] and (abs(pred) <= p['tol'] or pred * error < 0):
            if abs(self.u) > 0.0:
                self._stop = (error, self.rate)
                self.moved_dir = 1 if self.u > 0 else -1
            self._to_settle()                # we will coast to (or past) the target: stop and look
            return 0.0
        if abs(error) <= p['tol'] and abs(self.rate) <= p['rest_rate']:
            self.state, self.u = 'arrived', 0.0
            return 0.0
        if abs(pred) <= p['tol']:
            self.u = 0.0
            return 0.0
        target = p['kp'] * pred
        if p['ki'] > 0.0:
            lim = 0.5 * p['u_max'] / p['ki']
            if abs(target) < p['u_max'] and abs(error) < 0.35:
                self._i = max(-lim, min(lim, self._i + error * dt))
            if self._i * error < 0:
                self._i = 0.0
            target += p['ki'] * self._i
        if abs(target) < self.u_min:
            target = math.copysign(self.u_min, pred)
        target = max(-p['u_max'], min(p['u_max'], target))
        u = target
        if p['accel'] > 0.0 and abs(target) > abs(self.u) and target * self.u >= 0.0:     # braking is never delayed
            u = math.copysign(min(abs(target), max(abs(self.u) + p['accel'] * dt, min(self.u_min, abs(target)))), target)
        self.u = u
        self._watch_stall(error, u, dt)
        return u

    def _watch_stall(self, error, u, dt):
        """A small command that does not change the POSITION means the robot's deadband is larger than it. (Judged on the position,
        not on the measured rate: encoder rate noise is larger than the rate threshold and would hide every stall.)"""
        p = self.p
        if u == 0.0 or abs(u) > p['stall_range'] * self.u_min:
            self._stall_ref, self._stall = None, 0.0
        elif self._stall_ref is None or abs(wrap_angle(error - self._stall_ref)) > p['tol']:
            self._stall_ref, self._stall = error, 0.0          # it moved: start a new check
        else:
            self._stall += dt
            if self._stall >= p['stall_time']:
                self._raise_minimum(u)
                self._stall_ref, self._stall = None, 0.0

    # ---- standing still, then decide -------------------------------------------------------
    def _settle(self, error, dt):
        p = self.p
        self.u = 0.0
        self._wait += dt
        self._quiet = self._quiet + dt if abs(self.rate) < p['rest_rate'] else 0.0
        if self._wait < p['settle_min'] or (self._quiet < p['rest_time'] and self._wait < p['settle_max']):
            return 0.0
        if self._pulse:                                            # judge the pulse that just ended
            self._learn(error)
        if self._stop:                                             # how far did the robot coast after the last move?
            self._learn_coast(error)
        if abs(error) <= p['tol']:
            self.state = 'arrived'
            return 0.0
        if not p['pulses'] or abs(error) > self.near:
            self.state, self._stall, self._stall_ref = 'move', 0.0, None
            return 0.0
        u = max(self.u_min, 1e-3)
        length = p['pulse_share'] * abs(error) / (self.gain * u)   # P law on the pulse LENGTH
        same = self.moved_dir != 0 and (error > 0) == (self.moved_dir > 0)
        lost = self.lost_time(u, same)
        if lost > 0.0:
            length += max(0.0, lost + (self.same_extra if same else self.dead_extra))   # ... plus what the start swallows
        self._pulse_same = same
        ticks = max(1, min(int(p['pulse_max'] / dt + 0.5), int(length / dt + 0.5)))
        self._pulse_u = math.copysign(u, error)
        self._pulse = (error, self._pulse_u, ticks * dt)
        self._pulse_left = ticks - 1
        self.state, self.u = 'pulse', self._pulse_u
        return self.u

    def _learn_coast(self, error):
        """coasting time = distance covered after the stop / speed at the stop. An overshoot also widens the pulse region."""
        e_stop, rate_stop = self._stop
        self._stop = None
        if self.p['adapt'] and error * e_stop < 0.0 and abs(error) > self.p['tol']:
            self.near = min(self.p['near_max'], max(self.near, 1.3 * abs(error)))      # that move was too coarse
        if not self.p['adapt'] or abs(rate_stop) < 5.0 * self.p['rest_rate']:
            return                                                  # too slow for a meaningful measurement
        coasted = (e_stop - error) * math.copysign(1.0, rate_stop)
        measured = min(self.p['lead_max'], max(self.p['lead_min'], coasted / abs(rate_stop)))
        weight = 0.8 if self._coasts == 0 else 0.5
        self.lead += weight * (measured - self.lead)
        self._coasts += 1

    def _learn_loss(self, moved, u, length, lost):
        """With a start-up loss, a pulse that fails was too SHORT before it was too weak: correct the lost time first."""
        p = self.p
        name = 'same_extra' if self._pulse_same else 'dead_extra'
        extra = getattr(self, name)
        useful = length - max(0.0, lost + extra)                   # the part that should have moved the robot
        # (at least a sliver: a pulse hardly longer than the loss that did nothing was too short too)
        if moved < 0.15 * self.gain * abs(u) * max(useful, 0.02):
            if extra >= p['dead_extra_max'] - 1e-9:
                self._raise_minimum(u)
            else:
                setattr(self, name, min(p['dead_extra_max'], extra + p['dead_step']))
            return
        self.moved_dir = 1 if u > 0 else -1
        seen = length - moved / (self.gain * abs(u)) - lost        # the correction this pulse points to
        seen = max(-lost, min(p['dead_extra_max'], seen))
        setattr(self, name, extra + 0.4 * (seen - extra))

    def _learn(self, error):
        """What did the last pulse really do?"""
        p = self.p
        before, u, length = self._pulse
        self._pulse = None
        if not p['adapt']:
            return
        moved = (before - error) * math.copysign(1.0, u)           # displacement in the pushed direction
        asked = abs(u) * length
        lost = self.lost_time(u, self._pulse_same)
        if lost > 0.0:
            self._learn_loss(moved, u, length, lost)
            return
        if moved < 0.15 * self.gain * asked:
            self._raise_minimum(u)                                 # ignored: the deadband is above u
            return
        self.gain = min(2.0, max(0.5, 0.6 * self.gain + 0.4 * moved / asked))
        if length <= 0.051 and moved > 1.8 * abs(before) and moved > 2 * p['tol']:
            # even the shortest pulse jumps too far: use a smaller command, but never one the robot is known to ignore
            self.u_min = max(p['u_min_floor'], self.ignored * 1.1, self.u_min * p['shrink'])


class FinalHeading:
    """After a path: turn to the final heading with the heading servo (the one measured on the robot). The turn rate is taken
    from the pose itself, so it works with any pose source."""

    def __init__(self, target, params=None, rate_window=0.15):
        self.target, self.window = target, rate_window
        self.servo = AxisServo(params)
        self._history = []
        self._t = 0.0

    @property
    def done(self):
        return self.servo.arrived

    def step(self, yaw, dt):
        """-> turn-rate command."""
        self._t += dt
        self._history.append((self._t, yaw))
        while len(self._history) > 2 and self._t - self._history[1][0] >= self.window:
            self._history.pop(0)
        t0, yaw0 = self._history[0]
        rate = wrap_angle(yaw - yaw0) / (self._t - t0) if self._t - t0 > 1e-6 else 0.0
        return self.servo.update(wrap_angle(self.target - yaw), rate, dt)


# the same servo for the distance along a line (metres, m/s); tuned by tools/servo_study.py --axis distance
LINEAR_SERVO_DEFAULTS = dict(SERVO_DEFAULTS, accel=0.4, u_min_floor=0.015, u_min_cap=0.1, near_max=0.25,
                             kp=0.7609, lead=1.0776, u_max=0.2, tol=0.0075, u_min=0.0574, stall_time=0.7839, grow=1.1272, shrink=0.8819, pulse_below=0.0359, pulse_share=0.9338, rest_rate=0.004, rest_time=0.5935, settle_min=0.8741, settle_max=4.742, rate_filter=0.849)


class PoseServo:
    """Go to (x, y, theta). Phases: TURN to face the point -> DRIVE to it (steering on the way) -> FINAL turn -> DONE.
    The point is approached forwards or backwards, whichever needs the smaller first turn if `allow_reverse`."""

    def __init__(self, heading_params=None, linear_params=None, steer_gain=1.5, face_tol=0.12, near=0.06, pos_tol=0.02,
                 allow_reverse=False):
        self.heading = AxisServo(heading_params)
        self.linear = AxisServo(dict(LINEAR_SERVO_DEFAULTS, **(linear_params or {})))
        self.steer_gain, self.face_tol, self.near, self.pos_tol = steer_gain, face_tol, near, pos_tol
        self.allow_reverse = allow_reverse
        self.set_target(None)

    def set_target(self, target):
        self.target = target
        self.phase = 'DONE' if target is None else 'TURN'
        self.direction = 1.0
        self.line = None                     # heading of the straight line being driven
        self.heading.reset()
        self.linear.reset()

    def set_distance(self, pose, distance):
        """Drive `distance` metres along the heading the robot has now (negative = backwards), holding that heading."""
        x, y, th = pose
        self.target = None
        self.line = (x, y, th, float(distance))
        self.phase = 'LINE'
        self.linear.reset()

    def remaining(self, pose):
        """Distance still to go in LINE mode (negative once passed)."""
        x0, y0, th0, dist = self.line
        return dist - ((pose[0] - x0) * math.cos(th0) + (pose[1] - y0) * math.sin(th0))

    def _bearing(self, x, y):
        gx, gy, _ = self.target
        return math.atan2(gy - y, gx - x)

    def update(self, pose, v, w, dt):
        """(v command, w command) from the pose (map frame) and the MEASURED speeds."""
        if self.phase == 'DONE':
            return 0.0, 0.0
        x, y, th = pose
        if self.phase == 'LINE':
            vc = self.linear.update(self.remaining(pose), v, dt)
            if self.linear.arrived:
                self.phase = 'DONE'
                return 0.0, 0.0
            wc = max(-0.4, min(0.4, self.steer_gain * wrap_angle(self.line[2] - th))) if vc != 0.0 else 0.0
            return vc, wc
        gx, gy, gth = self.target
        dist = math.hypot(gx - x, gy - y)
        if self.phase == 'TURN':
            if dist <= self.pos_tol:
                self.phase = 'FINAL'
                self.heading.reset()
            else:
                if self.line is None:
                    b = self._bearing(x, y)
                    if self.allow_reverse and abs(wrap_angle(b - th)) > math.pi / 2:
                        self.direction = -1.0
                    self.line = True
                face = self._bearing(x, y) if self.direction > 0 else wrap_angle(self._bearing(x, y) + math.pi)
                err = wrap_angle(face - th)
                u = self.heading.update(err, w, dt)
                if abs(err) <= self.face_tol and abs(w) <= 0.15:
                    self.phase = 'DRIVE'
                    self.heading.reset()
                    self.linear.reset()
                    return 0.0, 0.0
                return 0.0, u
        if self.phase == 'DRIVE':
            # distance still to go ALONG the robot's heading: becomes negative if the point is passed
            along = (gx - x) * math.cos(th) + (gy - y) * math.sin(th)
            vc = self.linear.update(along, v, dt)
            wc = 0.0
            if dist > self.near:
                face = self._bearing(x, y) if self.direction > 0 else wrap_angle(self._bearing(x, y) + math.pi)
                wc = max(-0.5, min(0.5, self.steer_gain * wrap_angle(face - th)))
            if self.linear.arrived:
                if dist <= self.pos_tol:
                    self.phase = 'FINAL'
                    self.heading.reset()
                else:                        # beside the point: face it again
                    self.phase = 'TURN'
                    self.line = None
                    self.heading.reset()
                return 0.0, 0.0
            return vc, wc
        # FINAL
        u = self.heading.update(wrap_angle(gth - th), w, dt)
        if self.heading.arrived:
            self.phase = 'DONE'
            return 0.0, 0.0
        return 0.0, u
