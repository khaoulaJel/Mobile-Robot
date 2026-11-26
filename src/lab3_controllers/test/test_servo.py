"""servo.py: fusion arithmetic, the AxisServo's guarantees, and behaviour on difficult simulated robots (no ROS)."""
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import servo_sim as ss  # noqa: E402
from lab3_controllers.common import wrap_angle  # noqa: E402
from lab3_controllers.servo import SERVO_DEFAULTS, AxisServo, MapOdomFusion, compose, offset_between  # noqa: E402

DT = 0.05


# ---- fusion -------------------------------------------------------------------------------------
def test_offset_between_is_exact():
    rng = random.Random(1)
    for _ in range(200):
        frame = (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3.1, 3.1))
        odom = (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3.1, 3.1))
        got = compose(offset_between(compose(frame, odom), odom), odom)
        want = compose(frame, odom)
        assert math.hypot(got[0] - want[0], got[1] - want[1]) < 1e-9 and abs(wrap_angle(got[2] - want[2])) < 1e-9


def test_fusion_pairs_by_time_stamp_not_by_arrival():
    """A map pose that arrives late must be matched with the odometry of ITS moment: while turning, using the latest odometry would be
    wrong by the distance turned since."""
    f = MapOdomFusion()
    for k in range(41):                                    # turning at 1 rad/s, odometry every 50 ms
        t = k * 0.05
        f.add_odom(t, (0.0, 0.0, t), 0.0, 1.0)
    assert f.add_map(1.525, (0.0, 0.0, wrap_angle(1.525 + 0.7)))       # map frame = odom rotated by 0.7 rad, sample 0.475 s old
    assert abs(wrap_angle(f.state()[0][2] - (2.0 + 0.7))) < 1e-6


def test_fusion_rejects_a_map_pose_with_no_odometry_near_its_stamp():
    f = MapOdomFusion()
    f.add_odom(10.0, (0.0, 0.0, 0.0), 0.0, 0.0)
    assert not f.add_map(3.0, (1.0, 1.0, 1.0)) and not f.ready() and f.rejected == 1


def test_fusion_keeps_going_on_odometry_after_the_last_map_pose():
    f = MapOdomFusion()
    f.add_odom(0.0, (0.0, 0.0, 0.0), 0.0, 0.0)
    f.add_map(0.0, (2.0, 1.0, math.pi / 2))
    f.add_odom(0.5, (0.1, 0.0, 0.0), 0.2, 0.0)             # 10 cm forward in the odometry frame = +y in the map
    (x, y, th), v, _ = f.state()
    assert math.isclose(x, 2.0, abs_tol=1e-9) and math.isclose(y, 1.1, abs_tol=1e-9) and math.isclose(th, math.pi / 2) and v == 0.2


# ---- servo guarantees ---------------------------------------------------------------------------
def test_inside_the_tolerance_it_sends_zero_and_confirms_standing_still_before_saying_arrived():
    s = AxisServo()
    assert s.update(0.005, 0.0, DT) == 0.0 and not s.arrived, 'one sample is not enough to declare arrival'
    for _ in range(int(1.0 / DT)):
        assert s.update(0.005, 0.0, DT) == 0.0
    assert s.arrived


def test_never_sends_a_command_the_robot_would_ignore_and_respects_the_limit():
    rng = random.Random(2)
    s = AxisServo(dict(u_min=0.22, u_max=0.7))
    for _ in range(4000):
        u = s.update(rng.uniform(-3, 3), rng.uniform(-1, 1), DT)
        assert u == 0.0 or abs(u) >= 0.22 * 0.999 or abs(u) >= s.u_min * 0.999, u
        assert abs(u) <= 0.7 + 1e-12


def test_pulse_length_is_proportional_to_the_error():
    def pulse_ticks(error):
        s = AxisServo(dict(u_min=0.25, pulse_share=1.0, pulse_below=1.0, settle_min=0.1, rest_time=0.05))
        s._to_settle()
        n, started = 0, False
        for _ in range(200):
            u = s.update(error, 0.0, DT)
            if u != 0.0:
                n, started = n + 1, True
            elif started:
                break
        return n
    short, long_ = pulse_ticks(0.05), pulse_ticks(0.15)
    assert short >= 1 and long_ in range(3 * short - 1, 3 * short + 2), (short, long_)


def test_pause_drops_the_command_and_keeps_what_was_learned():
    s = AxisServo()
    s.update(1.0, 0.0, DT)
    s.u_min, s.lead, s.gain = 0.31, 0.9, 1.2
    s.pause()
    assert s.u == 0.0 and (s.u_min, s.lead, s.gain) == (0.31, 0.9, 1.2)
    assert s.update(1.0, 0.0, DT) == 0.0, 'after stale data it first waits until the robot stands still'


@pytest.mark.parametrize('bad', [dict(kp=0.0), dict(u_max=-1.0), dict(tol=0.0), dict(u_min_floor=0.5, u_min_cap=0.1)])
def test_invalid_coefficients_are_refused(bad):
    with pytest.raises(ValueError):
        AxisServo(bad)


# ---- behaviour on robots it was not told about --------------------------------------------------
def turn_on(servo, robot, target, seconds=25.0):
    sent_since_stop = []
    for _ in range(int(seconds / DT)):
        u = servo.update(wrap_angle(target - robot.th), robot.w, DT)
        sent_since_stop.append(u)
        robot.step(0.0, u)
    return abs(math.degrees(wrap_angle(target - robot.th)))


def hard_case(**over):
    case = dict(plant='random', seed=1, db_w=0.2, db_v=0.04, gw=1.0, gv=1.0, tau_w=0.6, tau_v=0.5, cmd_delay=0.15)
    case.update(over)
    return case


@pytest.mark.parametrize('over', [dict(db_w=0.0), dict(db_w=0.29), dict(gw=0.7), dict(gw=1.3), dict(tau_w=1.0, cmd_delay=0.35),
                                  dict(tau_w=0.15, cmd_delay=0.05)])
def test_reaches_the_heading_on_very_different_robots(over):
    err = turn_on(AxisServo(), ss.Robot(hard_case(**over)), math.radians(100))
    assert err <= 2.0, f'{over}: final error {err:.2f} deg (limit 2)'


def test_it_learns_a_deadband_larger_than_it_was_told():
    s = AxisServo(dict(u_min=0.12))
    robot = ss.Robot(hard_case(db_w=0.28))
    err = turn_on(s, robot, math.radians(8))                # a small move: only commands above the real deadband can do it
    assert err <= 2.0 and s.u_min > 0.28 * 0.99, (err, s.u_min)


def test_long_run_stays_accurate_and_what_it_learns_stays_bounded():
    """25 moves in a row on a difficult robot: the learned values must never drift into a bad state."""
    rng = random.Random(4)
    s = AxisServo()
    robot = ss.Robot(hard_case(db_w=0.26, gw=1.25, tau_w=0.8, cmd_delay=0.25))
    errors = []
    for _ in range(25):
        s.reset()
        target = wrap_angle(robot.th + rng.choice([-1, 1]) * rng.uniform(0.1, 2.8))
        errors.append(turn_on(s, robot, target))
        assert SERVO_DEFAULTS['u_min_floor'] <= s.u_min <= SERVO_DEFAULTS['u_min_cap']
        assert SERVO_DEFAULTS['lead_min'] <= s.lead <= SERVO_DEFAULTS['lead_max'] and 0.5 <= s.gain <= 2.0
    assert max(errors) <= 2.5, f'worst of 25 moves {max(errors):.2f} deg: {[round(e, 1) for e in errors]}'
    assert sum(errors[-10:]) / 10 <= 1.2, f'last 10 moves average {sum(errors[-10:]) / 10:.2f} deg'
