import math

from lab3_controllers.common import (
    go_to_point_control, slew, wrap_angle, yaw_from_quaternion, yaw_p_control)


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi / 2) + math.pi / 2) < 1e-9
    assert abs(wrap_angle(-3 * math.pi / 2) - math.pi / 2) < 1e-9


def test_yaw_from_quaternion():
    th = 1.0
    assert abs(yaw_from_quaternion(0, 0, math.sin(th / 2), math.cos(th / 2)) - th) < 1e-9


def test_yaw_takes_short_way_round():
    w, err = yaw_p_control(math.radians(-170), math.radians(170), 1.0, 5.0)
    assert w > 0 and abs(math.degrees(err) - 20) < 1e-6


def test_yaw_saturation_and_tolerance():
    assert yaw_p_control(3.0, 0.0, 10.0, 1.0)[0] == 1.0
    assert yaw_p_control(0.01, 0.0, 10.0, 1.0, tol=0.02)[0] == 0.0


def test_go_to_point_goal_behind_rotates_in_place():
    v, w, d, a = go_to_point_control(0, 0, 0, -1, 0.1, 1, 2, 0.2, 1)
    assert v == 0.0 and w != 0.0


def test_go_to_point_straight():
    v, w, d, a = go_to_point_control(0, 0, 0, 1, 0, 1, 2, 0.2, 1)
    assert v == 0.2 and w == 0.0 and d == 1.0


def test_slew():
    assert abs(slew(0.0, 1.0, 1.0, 0.1) - 0.1) < 1e-12
    assert slew(0.0, 1.0, 0.0, 0.1) == 1.0


def test_pose_control_converges_to_pose():
    from lab3_controllers.common import pose_control
    x, y, th, dt = 0.0, 0.0, 1.0, 0.02
    for _ in range(5000):
        v, w, rho, _a = pose_control(x, y, th, 1.0, 0.5, 0.0, 1.5, 4.0, -0.3, 0.3, 2.0, 0.4)
        if rho < 0.01:
            break
        th = wrap_angle(th + w * dt)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
    assert rho < 0.02 and abs(wrap_angle(0.0 - th)) < 0.5  # ALIGN phase trims the rest


def test_circle_waypoints_closes_and_is_on_circle():
    from lab3_controllers.common import circle_waypoints
    wp = circle_waypoints(0.4, 24)
    pts = [(wp[i], wp[i + 1]) for i in range(0, len(wp), 3)]
    assert len(pts) == 24 and math.hypot(*pts[-1]) < 1e-9
    assert all(abs(math.hypot(x, y - 0.4) - 0.4) < 1e-9 for x, y in pts)


def test_path_projection_and_lookahead():
    from lab3_controllers.common import lookahead_point, project_on_path
    pts = [(0, 0), (1, 0), (1, 1)]
    seg, t = project_on_path(pts, 0.5, 0.1, 0)
    x, y, rem = lookahead_point(pts, seg, t, 0.8)
    assert seg == 0 and abs(t - 0.5) < 1e-9 and abs(x - 1.0) < 1e-9 and abs(y - 0.3) < 1e-9
    assert abs(rem - 1.5) < 1e-9


def test_pure_pursuit_goal_behind_rotates_and_speed_capped_at_end():
    from lab3_controllers.common import pure_pursuit_control
    v, w, _ = pure_pursuit_control(0, 0, 0, (-1, 0.2), 0.2, 1.0, 0.25, 1.0, 0.4)
    assert v == 0.0 and w != 0.0
    v, w, _ = pure_pursuit_control(0, 0, 0, (0.25, 0), 0.2, 1.0, 0.25, 0.01, 0.4)
    assert v < 0.1


def test_projection_on_closed_path_does_not_jump_to_the_end():
    """Regression: with noise at the start of a closed path the projection must stay on segment 0."""
    import random
    from lab3_controllers.common import project_on_path
    pts = [(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5), (0, 0)]
    rng = random.Random(0)
    for _ in range(200):
        seg, _t = project_on_path(pts, rng.gauss(0, 0.005), rng.gauss(0, 0.005), 0)
        assert seg == 0
