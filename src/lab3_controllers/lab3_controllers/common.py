"""Pure-Python helpers shared by the controllers (no ROS imports, unit-testable)."""

import math


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    """Yaw (rotation about Z) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_p_control(ref, yaw, kp, w_max, w_min=0.0, tol=0.0):
    """P law for heading. Returns (w, error). w=0 inside the tolerance band.

    w_min is an optional minimum turn rate (deadband / static-friction
    compensation) applied outside the tolerance band.
    """
    err = wrap_angle(ref - yaw)
    if abs(err) <= tol:
        return 0.0, err
    w = kp * err
    if abs(w) < w_min:
        w = math.copysign(w_min, err)
    return clip(w, -w_max, w_max), err


def go_to_point_control(x, y, yaw, gx, gy, kv, kw, v_max, w_max,
                        turn_in_place_angle=math.radians(50.0)):
    """P law driving a unicycle to (gx, gy).

    v = kv * d * cos(alpha)   (never negative, zero when the goal is behind)
    w = kw * alpha
    where d is the distance and alpha the bearing error. If the bearing error is
    larger than turn_in_place_angle the robot rotates on the spot first, which
    avoids wide arcs. Returns (v, w, d, alpha).
    """
    dx, dy = gx - x, gy - y
    d = math.hypot(dx, dy)
    alpha = wrap_angle(math.atan2(dy, dx) - yaw)
    if abs(alpha) > turn_in_place_angle:
        v = 0.0
    else:
        v = clip(kv * d * math.cos(alpha), 0.0, v_max)
    w = clip(kw * alpha, -w_max, w_max)
    return v, w, d, alpha


def pose_control(x, y, yaw, gx, gy, gth, k_rho, k_alpha, k_beta, v_max, w_max,
                 decel=0.0, end_gain=1.5):
    """Polar-coordinate Lyapunov pose controller (Siegwart/Aicardi).

    rho   = distance to the goal
    alpha = bearing of the goal relative to the robot heading
    beta  = goal heading minus bearing of the goal (angle between the line of sight
            and the desired final heading)
    v = k_rho * rho * cos(alpha)          (>= 0: never reverses)
    w = k_alpha * alpha + k_beta * beta   stable if k_rho > 0, k_beta < 0, k_alpha > k_rho

    The robot reaches (gx, gy) already pointing along gth, in one smooth motion
    instead of stop - rotate - drive - rotate. decel > 0 caps v at sqrt(2*decel*rho)
    so the robot can actually stop at the goal. Returns (v, w, rho, alpha).
    """
    dx, dy = gx - x, gy - y
    rho = math.hypot(dx, dy)
    phi = math.atan2(dy, dx)
    alpha = wrap_angle(phi - yaw)
    beta = wrap_angle(gth - phi)
    v = k_rho * rho * math.cos(alpha)
    if v < 0.0:
        v = 0.0
    v = min(v, v_max)
    if decel > 0.0:
        v = min(v, math.sqrt(2.0 * decel * rho))
        if end_gain > 0.0:
            v = min(v, end_gain * rho)      # P-style creep so the robot does not coast past the goal
    w = clip(k_alpha * alpha + k_beta * beta, -w_max, w_max)
    return v, w, rho, alpha


def slew(prev, target, max_rate, dt):
    """Limit the change of a command to max_rate*dt per step (max_rate<=0 disables)."""
    if max_rate <= 0.0:
        return target
    step = max_rate * dt
    return prev + clip(target - prev, -step, step)


def circle_waypoints(radius, points, direction=1, laps=1):
    """Waypoints (x, y, theta) of a circle tangent to the start pose (start frame).

    The centre is at (0, direction*radius); theta is the tangent heading. direction=+1 is
    counter-clockwise. Returns a flat list [x, y, th, ...] ending back at the start.
    """
    out = []
    n = int(points) * int(laps)
    for k in range(1, n + 1):
        phi = 2.0 * math.pi * k / points
        out += [radius * math.sin(phi), direction * radius * (1.0 - math.cos(phi)),
                wrap_angle(direction * phi)]
    return out


def project_on_path(pts, x, y, seg, window=1):
    """Closest point on polyline `pts` searching segments seg..seg+window (forward only).

    Returns (seg, t) with t in [0,1] along that segment. Progress can advance by at most
    `window` segments per call, so on a closed path (end point == start point, e.g. a circle or
    a square) noisy odometry at the start cannot snap the projection to the END of the path.
    """
    best = None
    for i in range(seg, min(seg + window + 1, len(pts) - 1)):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0.0 else clip(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0)
        d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        if best is None or d < best[0] - 1e-9:
            best = (d, i, t)
    return best[1], best[2]


def lookahead_point(pts, seg, t, dist):
    """Point `dist` metres further along the polyline from (seg, t). Returns (x, y, remaining)
    where remaining is the path length left from the robot's projection to the end."""
    ax, ay = pts[seg]
    bx, by = pts[seg + 1]
    px, py = ax + t * (bx - ax), ay + t * (by - ay)
    # total remaining length
    rem = math.hypot(bx - px, by - py)
    for i in range(seg + 1, len(pts) - 1):
        rem += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
    cx, cy, i = px, py, seg
    left = dist
    while True:
        ex, ey = pts[i + 1]
        seglen = math.hypot(ex - cx, ey - cy)
        if seglen >= left or i + 1 >= len(pts) - 1:
            if seglen > 1e-9:
                f = min(1.0, left / seglen)
                return cx + f * (ex - cx), cy + f * (ey - cy), rem
            return ex, ey, rem
        left -= seglen
        cx, cy, i = ex, ey, i + 1


def pure_pursuit_control(x, y, yaw, target, v_des, w_max, lookahead, remaining, decel,
                         end_gain=1.5):
    """Pure pursuit toward the look-ahead `target`. Returns (v, w, alpha).

    curvature k = 2 sin(alpha)/Ld, w = v*k. Speed is reduced so |w| <= w_max, by
    sqrt(2*decel*remaining) near the end, and to zero if the target is behind
    (|alpha| > 90 deg -> rotate in place first).
    """
    dx, dy = target[0] - x, target[1] - y
    ld = max(math.hypot(dx, dy), 1e-6)
    alpha = wrap_angle(math.atan2(dy, dx) - yaw)
    if abs(alpha) > math.radians(60.0):
        return 0.0, clip(2.0 * alpha, -w_max, w_max), alpha
    kappa = 2.0 * math.sin(alpha) / ld
    v = v_des
    if decel > 0.0:
        v = min(v, math.sqrt(2.0 * decel * max(remaining, 0.0)))
        if end_gain > 0.0:
            v = min(v, end_gain * max(remaining, 0.0))   # avoid coasting past the end point
    if abs(kappa) > 1e-9:
        v = min(v, w_max / abs(kappa))
    w = clip(v * kappa, -w_max, w_max)
    if lookahead > 0 and abs(alpha) > 0.35:       # also steer in place a bit when misaligned
        w = clip(w + 1.0 * alpha, -w_max, w_max)
    return v, w, alpha
