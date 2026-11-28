"""Paths for the robot: the shapes, the path controllers (ONE description used by the simulator and by the robot runner) and
how a run is judged. No ROS here.

Controllers
    stop_turn, polar, pure_pursuit   motion_controller (our P laws, tracker.py) with the robot settings of config/path_robot.json
    p_servo                          p_controller driven corner by corner (turn, drive, final heading), for any list of poses
    symbolic                         symbolic_controller: only the 1 m-diameter circle (its policy is built for that circle)
"""
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import robot_model as rm  # noqa: E402
import shape_bench as sb  # noqa: E402
from lab3_controllers.common import slew, wrap_angle  # noqa: E402
from lab3_controllers.servo import FinalHeading, PoseServo  # noqa: E402
from lab3_controllers.tracker import DEFAULTS as TRACKER_DEFAULTS, DONE, PathTracker  # noqa: E402

SETTINGS_FILE = os.path.join(HERE, '..', 'config', 'path_robot.json')
TRACKERS = ('stop_turn', 'polar', 'pure_pursuit')
CONTROLLERS = TRACKERS + ('p_servo', 'symbolic')
SYMBOLIC_SHAPE = 'circle_r50'
ABORT_OFF_PATH = 0.25                     # m

NAMES = {                                 # short command-line name -> name in shape_bench
    'line': None,
    'square': 'square 0.5', 'square_cw': 'square 0.5 CW', 'rectangle': 'rectangle 0.8x0.3', 'triangle': 'triangle',
    'hexagon': 'hexagon R0.3', 'star': 'star (36 deg tips)', 'circle_r40': 'circle R0.4', 'circle_r50': 'circle R0.5 (lab polar)',
    'circle_r20': 'circle R0.2 (tight)', 'ellipse': 'ellipse 0.6x0.3', 'eight': 'figure-eight', 'sine': 'sine wave',
    'spiral': 'spiral out', 'stadium': 'stadium',
}


def shape(name):
    """-> (kind 'poly'|'curve', ideal polyline starting at (0, 0) with the first piece along +x)."""
    if name not in NAMES:
        raise ValueError(f'unknown shape "{name}": one of {", ".join(NAMES)}')
    if name == 'line':
        return 'poly', [(0.0, 0.0), (0.5, 0.0)]
    kind, ideal = sb.shapes()[NAMES[name]]
    return kind, [(float(x), float(y)) for x, y in sb._normalize(ideal)] if name == 'sine' else [(float(x), float(y)) for x, y in ideal]


def dense(ideal, step=0.004):
    return sb._dense(ideal, step) if len(ideal) < 400 else ideal


def length_of(ideal):
    return sum(math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(ideal, ideal[1:]))


def room_needed(name, margin=0.25):
    """Free floor needed around the START pose: metres ahead, left, right and behind (robot half-size included)."""
    _, ideal = shape(name)
    xs, ys = [p[0] for p in ideal], [p[1] for p in ideal]
    return dict(ahead=max(xs) + margin, left=max(ys) + margin, right=-min(ys) + margin, behind=-min(xs) + margin)


def end_heading(name):
    """The heading the robot should have at the end: a closed polygon ends facing the way it started; a curve ends along its
    TANGENT there (not along the last chord between two waypoints, which can be many degrees off on a tight curve)."""
    kind, ideal = shape(name)
    if name == 'line' or (kind == 'poly'):
        return 0.0
    pts = dense(ideal, 0.002)
    (ax, ay), (bx, by) = pts[-4], pts[-1]
    last = math.atan2(by - ay, bx - ax)
    (cx, cy), (dx, dy) = pts[-7], pts[-4]
    before = math.atan2(dy - cy, dx - cx)
    return wrap_angle(last + 0.5 * wrap_angle(last - before))          # the chord's direction, carried on to the end point


def waypoints(name, ds):
    """Controller waypoints (x, y, theta) in the start frame."""
    kind, ideal = shape(name)
    if name == 'line':
        return [(0.5, 0.0, 0.0)]
    out = sb.to_waypoints(kind, ideal, ds)
    out[-1] = (out[-1][0], out[-1][1], end_heading(name))
    return out


def place(points, start):
    """Points given in the start frame -> the frame in which the start pose is `start`."""
    x0, y0, th0 = start
    c, s = math.cos(th0), math.sin(th0)
    out = []
    for p in points:
        q = (x0 + c * p[0] - s * p[1], y0 + s * p[0] + c * p[1])
        out.append(q + (wrap_angle(th0 + p[2]),) if len(p) == 3 else q)
    return out


class Judge:
    """Distance of every measured position from the IDEAL path (not from the waypoints: cutting corners counts)."""

    def __init__(self, name, start):
        self.ideal = place(dense(shape(name)[1]), start)
        self.length = length_of(self.ideal)
        self.errors = []
        self.covered = set()
        self.end = self.ideal[-1]
        self.end_heading = wrap_angle(start[2] + end_heading(name))
        self.index = 0

    def add(self, x, y):
        # the nearest point near the last one first (cheap), the whole path if that is far
        lo, hi = max(0, self.index - 60), min(len(self.ideal), self.index + 160)
        best = min(range(lo, hi), key=lambda i: (self.ideal[i][0] - x) ** 2 + (self.ideal[i][1] - y) ** 2)
        d = math.hypot(self.ideal[best][0] - x, self.ideal[best][1] - y)
        if d > 0.05:
            best = min(range(len(self.ideal)), key=lambda i: (self.ideal[i][0] - x) ** 2 + (self.ideal[i][1] - y) ** 2)
            d = math.hypot(self.ideal[best][0] - x, self.ideal[best][1] - y)
        self.index = best
        self.errors.append(d)
        self.covered.add(best * 50 // len(self.ideal))
        return d

    def result(self, pose):
        n = max(1, len(self.errors))
        return dict(max_cm=100 * max(self.errors, default=0.0), rms_cm=100 * math.sqrt(sum(e * e for e in self.errors) / n),
                    mean_cm=100 * sum(self.errors) / n,
                    end_cm=100 * math.hypot(self.end[0] - pose[0], self.end[1] - pose[1]),
                    end_deg=abs(math.degrees(wrap_angle(self.end_heading - pose[2]))),
                    covered=min(1.0, len(self.covered) / 50.0))


# ------------------------------------------------------------------ settings
FALLBACK = {                              # used only until path_tune.py has written config/path_robot.json
    'stop_turn': dict(ds=0.1, min_linear_speed=0.05, min_angular_speed=0.3, pos_tolerance=0.015, yaw_tolerance=0.02,
                      max_linear_speed=0.1, max_angular_speed=0.8, kv=1.0, kw=2.5, k_yaw=1.5, turn_in_place_deg=25.0),
    'polar': dict(ds=0.1, min_linear_speed=0.05, min_angular_speed=0.3, pos_tolerance=0.015, yaw_tolerance=0.02,
                  max_linear_speed=0.1, max_angular_speed=0.8, k_rho=1.5, k_alpha=4.0, k_beta=-1.0, blend_radius=0.08),
    'pure_pursuit': dict(ds=0.05, min_linear_speed=0.05, min_angular_speed=0.3, pos_tolerance=0.015, yaw_tolerance=0.02,
                         max_linear_speed=0.1, max_angular_speed=0.8, lookahead=0.08),
    'p_servo': dict(ds=0.1, steer_gain=1.5, face_tolerance=0.12, position_tolerance=0.02,
                    linear=dict(dead_fix=0.062, dead_imp=0.0082, dead_same=0.11, u_min=0.04, u_min_cap=0.1, tol=0.007,
                                pulse_share=0.7, pulse_below=0.03, lead=0.15, settle_min=0.5, kp=0.9, u_max=0.12)),
    'symbolic': {},
}


def settings(path=SETTINGS_FILE):
    out = json.loads(json.dumps(FALLBACK))
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            for k, v in json.load(f).items():
                if k in out:
                    out[k] = v
    return out


def heading_profile():
    """Our robot's measured turning settings (config/robot_measured.yaml): p_servo, and the final heading of the trackers."""
    global _PROFILE
    if _PROFILE is None:
        import robot_test
        _PROFILE = {k.split('.', 1)[1]: v for k, v in robot_test.read_profile(robot_test.DEFAULT_PROFILE).items()
                    if k.startswith('heading.')}
    return dict(_PROFILE)


_PROFILE = None


def tracker_params(controller, s):
    p = {k: v for k, v in s.items() if k in TRACKER_DEFAULTS}
    p['strategy'] = controller
    return p


# ------------------------------------------------------------------ simulation (the same code the nodes run)
class SymbolicLogic:
    """The control step of symbolic_controller.py (policy lookup every tau, goal arcs, neighbour fallback)."""

    def __init__(self, policy_file=None):
        from lab3_controllers.symbolic import Policy
        self.policy = Policy.load(policy_file or os.path.join(HERE, '..', 'config', 'symbolic_circle.npz'))
        self.k, self.u, self.next, self.mode, self.fallbacks = 0, (0.0, 0.0), 0.0, 'RUN', 0

    def step(self, t, pose):
        if self.mode != 'RUN':
            return 0.0, 0.0
        if t + 1e-9 >= self.next:
            self.next += self.policy.tau
            q = self.policy.state_index(*pose)
            if q >= 0:
                while self.k < len(self.policy.goals) and self.policy.goal_reached(self.k, q):
                    self.k += 1
            if self.k >= len(self.policy.goals):
                self.mode = 'DONE'
                return 0.0, 0.0
            u = self.policy.action(self.k, q) if q >= 0 else None
            if u is None:
                u = self.policy.robust_action(self.k, *pose)
                self.fallbacks += u is not None
            if u is None:
                self.mode = 'FAILED'
                return 0.0, 0.0
            self.u = u
        return self.u


def simulate(controller, name, s, model, seed=0, noise=0.0015, policy=None, tmax=None, late=True):
    """One run on the wheel model with the wheel odometry as state (late: by one period). -> result dict (+ xs, ys)."""
    rng = random.Random(seed)
    robot = rm.WheelRobot(model, rng=random.Random(seed + 1), jitter=0.025)
    dt = 0.05
    judge = Judge(name, (0.0, 0.0, 0.0))
    if tmax is None:
        tmax = 60.0 + 25.0 * judge.length + (4.0 if controller == 'p_servo' else 0.0) * len(waypoints(name, s.get('ds', 0.1)))
    state = (0.0, 0.0, 0.0, 0.0, 0.0)
    v_cmd = w_cmd = 0.0
    done = False
    tracker = servo = symbolic = None
    queue = []
    if controller in TRACKERS:
        path = waypoints(name, s.get('ds', 0.1))
        tracker = PathTracker(path, tracker_params(controller, s))
        tracker.start((0.0, 0.0, 0.0))
    elif controller == 'p_servo':
        queue = waypoints(name, s.get('ds', 0.1))
        servo = PoseServo(heading_profile(), s['linear'], steer_gain=s['steer_gain'], face_tol=s['face_tolerance'],
                          pos_tol=s['position_tolerance'])
        servo.set_target(queue.pop(0))
    else:
        if name != SYMBOLIC_SHAPE:
            raise ValueError('the symbolic policy exists only for ' + SYMBOLIC_SHAPE)
        symbolic = policy or SymbolicLogic()
    xs, ys, quiet, t = [], [], 0.0, 0.0
    final = None
    off_limit = False
    while t < tmax:
        x, y, th, v_meas, w_meas = state if late else (robot.x, robot.y, robot.th, robot.v, robot.w)
        measured = (x + rng.gauss(0, noise), y + rng.gauss(0, noise), wrap_angle(th + rng.gauss(0, 0.003)))
        if tracker is not None:
            v, w = tracker.step(measured, dt)
            v_cmd, w_cmd = slew(v_cmd, v, 0.4, dt), slew(w_cmd, w, 3.0, dt)
            if tracker.mode == DONE:                      # then the final heading, as motion_controller does it
                if final is None:
                    final = FinalHeading(waypoints(name, s.get('ds', 0.1))[-1][2], heading_profile())
                v_cmd, w_cmd = 0.0, final.step(measured[2], dt)
                done = final.done
        elif servo is not None:
            v_cmd, w_cmd = servo.update(measured, v_meas, w_meas, dt)
            if servo.phase == 'DONE':
                if queue:
                    servo.set_target(queue.pop(0))
                else:
                    done = True
        else:
            v_cmd, w_cmd = symbolic.step(t, measured)
            done = symbolic.mode != 'RUN'
        if done:
            v_cmd = w_cmd = 0.0
        state = (robot.x, robot.y, robot.th, robot.v, robot.w)            # what arrives next period (late)
        robot.step(v_cmd, w_cmd, dt=dt)
        d = judge.add(robot.x, robot.y)
        xs.append(robot.x)
        ys.append(robot.y)
        t += dt
        if d > ABORT_OFF_PATH:
            off_limit = True
            break
        if done:
            quiet += dt
            if quiet > 1.0:
                break
    out = judge.result((robot.x, robot.y, robot.th))
    finished = done and not off_limit and out['covered'] >= 0.9 and not (symbolic is not None and symbolic.mode == 'FAILED')
    out.update(finished=finished, time=t, off_limit=off_limit, xs=xs, ys=ys,
               fallbacks=symbolic.fallbacks if symbolic is not None else 0)
    return out
