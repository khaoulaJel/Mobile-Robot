"""Paths on the model of our robot: the shapes, how they are judged, the wheel model and its fit, the tuned controllers."""
import csv
import math
import os
import random
import sys

import pytest

ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, ROOT)
import paths  # noqa: E402
import robot_model as rm  # noqa: E402
import robot_path  # noqa: E402
from lab3_controllers.servo import FinalHeading  # noqa: E402

MEASURED_PULSES = {(0.3, 0.5): 2.06, (0.3, 0.7): 5.22, (0.45, 0.5): 5.62, (0.45, 0.7): 11.07, (0.6, 0.3): 2.62,
                   (0.6, 0.4): 5.48, (0.6, 0.5): 9.93, (0.6, 0.7): 17.00}


def test_every_shape_starts_at_the_origin_pointing_ahead_and_states_its_room():
    for name in paths.NAMES:
        _, ideal = paths.shape(name)
        assert math.hypot(*ideal[0]) < 1e-6, name
        j = next(i for i, p in enumerate(ideal) if math.hypot(*p) > 0.005)         # the tangent at the very start
        assert abs(math.atan2(ideal[j][1], ideal[j][0])) < math.radians(8), name
        room = paths.room_needed(name)
        assert all(v >= 0.25 - 1e-9 for v in room.values()), name
        assert room['ahead'] >= max(p[0] for p in ideal) + 0.249


def test_end_heading_is_the_tangent_not_the_last_chord():
    for name in ('circle_r20', 'circle_r40', 'circle_r50', 'square', 'triangle', 'line'):
        assert abs(math.degrees(paths.end_heading(name))) < 0.5, name
        assert paths.waypoints(name, 0.1)[-1][2] == pytest.approx(paths.end_heading(name))


def test_judge_gives_zero_on_the_ideal_path_in_any_start_frame_and_sees_a_cut_corner():
    start = (1.0, -2.0, 2.2)
    judge = paths.Judge('square', start)
    for p in judge.ideal[::3]:
        judge.add(*p[:2])
    r = judge.result((judge.end[0], judge.end[1], judge.end_heading))
    assert r['max_cm'] < 0.05 and r['end_cm'] < 0.05 and r['end_deg'] < 0.05 and r['covered'] == 1.0
    cut = paths.Judge('square', (0, 0, 0))
    cut.add(0.46, 0.04)                                    # 4 cm inside the first corner on both sides
    assert cut.result((0, 0, 0))['max_cm'] == pytest.approx(4.0, abs=0.3)


def test_relative_pose():
    assert robot_path.relative((1.0, 2.0, math.pi / 2), (1.0, 1.0, math.pi / 2)) == pytest.approx((1.0, 0.0, 0.0))


def test_wheel_model_reproduces_the_measured_turn_pulses():
    errors = [rm.pulse_move(rm.load(), 'turn', u, t) - m for (u, t), m in MEASURED_PULSES.items()]
    assert math.sqrt(sum(e * e for e in errors) / len(errors)) < 1.0 and max(map(abs, errors)) < 1.6


def test_fit_recovers_known_numbers_from_pulses(tmp_path):
    truth = rm.load()
    truth['turn'] = {'dead_fix': 0.12, 'dead_len': 0.006, 'gain': 0.96}
    truth['drive'] = {'dead_fix': 0.05, 'dead_len': 0.012, 'gain': 1.08}
    rows = []
    for axis, strengths in (('turn', (0.3, 0.45, 0.6)), ('drive', (0.06, 0.1, 0.15))):
        for length in (0.2, 0.3, 0.4, 0.5, 0.7):
            for u in strengths:
                for sign in (1, -1):
                    rows.append(dict(axis=axis, command=sign * u, real_length=length,
                                     moved=rm.pulse_move(truth, axis, u, length), length=length))
    with open(tmp_path / '1_pulses.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    model, report = rm.fit(str(tmp_path), str(tmp_path / 'model.json'))
    assert model['measured'] == {'turn': True, 'drive': True}, report
    for axis, unit, tol in (('turn', 'deg', 0.5), ('drive', 'cm', 0.3)):
        for u in ((0.3, 0.6) if axis == 'turn' else (0.06, 0.15)):
            for length in (0.4, 0.7):
                assert rm.pulse_move(model, axis, u, length) == pytest.approx(rm.pulse_move(truth, axis, u, length), abs=tol)


def test_final_heading_turns_a_modelled_robot_to_under_half_a_degree():
    for seed, turn in ((1, 90), (2, -35), (3, 4)):
        robot = rm.WheelRobot(rm.load(), rng=random.Random(seed), jitter=0.02)
        final = FinalHeading(math.radians(turn), paths.heading_profile())
        for _ in range(400):
            robot.step(0.0, 0.0 if final.done else final.step(robot.th, 0.05))
        assert final.done and abs(math.degrees(robot.th) - turn) < 0.5, (turn, math.degrees(robot.th))


@pytest.mark.parametrize('controller,limit_cm', [('stop_turn', 0.8), ('pure_pursuit', 1.5), ('polar', 2.5), ('p_servo', 1.0)])
def test_tuned_controllers_finish_square_and_circle_with_small_errors(controller, limit_cm):
    s = paths.settings()[controller]
    for shape in ('square', 'circle_r50'):
        r = paths.simulate(controller, shape, s, rm.load(), seed=123, tmax=600)
        assert r['finished'] and r['rms_cm'] < limit_cm and r['end_deg'] < 1.0 and r['end_cm'] < 2.5, (shape, r['rms_cm'], r['end_deg'])


def test_symbolic_controller_completes_the_circle_inside_the_tube_in_about_one_lap():
    times = []
    for seed in range(4):
        r = paths.simulate('symbolic', 'circle_r50', {}, rm.load(), seed=seed, policy=paths.SymbolicLogic(), tmax=400)
        assert r['finished'] and r['max_cm'] <= 8.0
        times.append(r['time'])
    assert sorted(times)[len(times) // 2] < 45.0          # one lap takes about 32 s; the old goal arcs needed 150-250 s
