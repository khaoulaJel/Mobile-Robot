"""The controller on the robot AS MEASURED (start-up loss on every start from rest). The first real run failed because the
default settings send pulses shorter than the loss; this locks that failure and its fix."""
import math
import os
import random
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, ROOT)
import servo_sim as ss  # noqa: E402
from lab3_controllers.servo import SERVO_DEFAULTS, AxisServo  # noqa: E402


def profile():
    out = {}
    with open(os.path.join(ROOT, 'config', 'robot_measured.yaml'), encoding='utf-8') as f:
        for line in f:
            text = line.split('#')[0].strip()
            if text.startswith('heading.'):
                name, value = text.split(':')
                out[name.split('.', 1)[1]] = float(value)
    return out


def final_errors(params, turns_deg, seed, spread):
    rng = random.Random(seed)
    out = []
    for turn in turns_deg:
        c = ss.measured_case(rng, spread)
        c['turn'] = math.radians(turn)
        out.append(abs(ss.run_heading(lambda: ss.ServoHeading(dict(params)), c, hold=25.0)['final']))
    return out


def test_profile_names_are_real_coefficients():
    assert profile() and set(profile()) <= set(SERVO_DEFAULTS)


def test_simulated_robot_reproduces_the_measured_pulses():
    measured = {(0.3, 0.5): 2.06, (0.3, 0.7): 5.22, (0.45, 0.5): 5.62, (0.45, 0.7): 11.07, (0.6, 0.3): 2.62,
                (0.6, 0.5): 9.93, (0.6, 0.7): 17.00}
    for (u, length), real in measured.items():
        c = ss.measured_case(random.Random(1))
        c['engage_jitter'] = 0.0
        robot = ss.Robot(c)
        for _ in range(int(round(length / ss.DT))):
            robot.step(0.0, u)
        for _ in range(20):
            robot.step(0.0, 0.0)
        assert math.degrees(robot.th) == pytest.approx(real, abs=0.8), (u, length)


def test_default_settings_fail_on_this_robot_as_the_first_real_run_did():
    assert min(final_errors({}, [20, -20], 1, 0.0)) > 2.0


def test_measured_settings_stay_under_one_degree_for_pi_over_2_and_other_turns():
    turns = [90, -90] * 15 + [10, -10, 20, -20, 45, -45, 135, -135, 160, -160]
    for spread, seed in ((0.0, 31), (0.2, 32), (0.4, 33)):
        errors = final_errors(profile(), turns, seed, spread)
        assert max(errors) < 1.0, (spread, max(errors))
        assert sum(errors) / len(errors) < 0.3, (spread, sum(errors) / len(errors))


def test_an_ignored_pulse_is_made_longer_before_it_is_made_stronger():
    servo = AxisServo(dict(dead_fix=0.08, dead_imp=0.093, u_min=0.3))
    lengths = []
    for _ in range(5):
        length = 0.6
        servo._pulse = (0.1, 0.3, length)
        servo._learn(0.1)                      # the robot did not move at all
        lengths.append((round(servo.dead_extra, 3), round(servo.u_min, 3)))
    assert lengths[0] == (0.04, 0.3) and lengths[3] == (0.16, 0.3)
    for _ in range(4):
        servo._pulse = (0.1, 0.3, 0.6)
        servo._learn(0.1)
    assert servo.dead_extra == pytest.approx(0.25) and servo.u_min > 0.3      # only after the length limit: stronger


def test_without_a_start_up_loss_nothing_changes():
    servo = AxisServo()
    assert servo.lost_time(0.3) == 0.0 and servo.dead_extra == 0.0


def test_the_two_directions_learn_separately():
    servo = AxisServo(dict(dead_fix=0.062, dead_imp=0.093, dead_same=0.11, u_min=0.3))
    assert servo.lost_time(0.3, same=True) == pytest.approx(servo.lost_time(0.3) + 0.11)
    servo._pulse, servo._pulse_same = (0.1, 0.3, 0.6), True
    servo._learn(0.1)                                    # a same-way pulse did nothing
    assert servo.same_extra == pytest.approx(0.04) and servo.dead_extra == 0.0
    servo.moved_dir = 1
    servo.reset()                                        # a new target: the robot's last movement is still known
    assert servo.moved_dir == 1


def times_to_arrive(params, n, seed):
    rng = random.Random(seed)
    out = []
    for i in range(n):
        c = ss.measured_case_v2(rng, 0.2)
        c['turn'] = (1 if i % 2 else -1) * math.radians((45, 90, 120)[i % 3])
        robot = ss.Robot(c, (0.0, 0.0, c['y0']))
        sens, ctrl = ss.Sensors(c, robot), ss.ServoHeading(dict(params))
        ref, arrived = ss.wrap_angle(c['y0'] + c['turn']), 25.0
        for k in range(500):
            sens.update(k * ss.DT)
            v, w = ctrl.tick(k * ss.DT, *sens.arrivals(k * ss.DT), ref)
            sens.move(robot.step(v, w))
            if ctrl.servo.arrived and arrived == 25.0:
                arrived = k * ss.DT
        out.append((abs(math.degrees(ss.wrap_angle(ref - sens.truth()[2]))), arrived))
    return out


def test_direction_aware_settings_are_accurate_and_faster_with_the_direction_effect():
    v1 = dict(dead_fix=0.08, dead_imp=0.093, u_min=0.3, pulse_share=0.62, tol=0.005, pulse_below=0.15, lead=0.1, settle_min=0.743)
    new, old = times_to_arrive(profile(), 90, 77), times_to_arrive(v1, 90, 77)
    assert max(e for e, _ in new) < 1.0 and sum(e for e, _ in new) / 90 < 0.25
    assert sum(t for _, t in new) < 0.85 * sum(t for _, t in old)
