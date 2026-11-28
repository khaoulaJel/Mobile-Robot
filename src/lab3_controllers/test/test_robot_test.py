"""The logbook tool's decisions that do not need ROS (tools/robot_test.py)."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import robot_test as rt  # noqa: E402

AXES = {'heading': {'kp': 2.0}, 'linear': {'kp': 0.7}}


def test_changes_accept_known_coefficients_only():
    assert rt.parse_changes(['heading.kp=2.5', 'linear.kp=1'], AXES) == {'heading.kp': 2.5, 'linear.kp': 1.0}
    for bad in ('heading.nope=1', 'kp=1', 'heading.kp', 'heading.kp=abc', 'heading.kp=nan'):
        with pytest.raises(ValueError):
            rt.parse_changes([bad], AXES)


def test_progress_of_a_drive_in_a_rotated_frame():
    start = (1.0, 2.0, math.pi / 2)                  # facing +y
    remaining, side = rt.progress('drive', start, (0.99, 2.18, math.pi / 2), 0.2)
    assert remaining == pytest.approx(2.0) and side == pytest.approx(1.0)


def test_turn_error_wraps():
    remaining, _ = rt.progress('turn', (0, 0, 3.0), (0, 0, -3.1), -3.1)
    assert remaining == pytest.approx(0.0)
    remaining, _ = rt.progress('turn', (0, 0, 3.0), (0, 0, 3.1), -3.1)
    assert remaining == pytest.approx(math.degrees(2 * math.pi - 6.2))


def test_limits():
    start = (0.0, 0.0, 0.0)
    assert rt.limit_broken('drive', start, (0.2, 0.0, 0.0), 0.2) is None
    assert rt.limit_broken('drive', start, (0.34, 0.0, 0.0), 0.2) is None
    assert 'too far past' in rt.limit_broken('drive', start, (0.36, 0.0, 0.0), 0.2)
    assert 'too far past' in rt.limit_broken('drive', start, (-0.36, 0.0, 0.0), -0.2)
    assert 'away' in rt.limit_broken('drive', start, (-0.16, 0.0, 0.0), 0.2)
    assert 'off the line' in rt.limit_broken('drive', start, (0.1, 0.21, 0.0), 0.2)
    goal = math.radians(30)
    assert rt.limit_broken('turn', start, (0.1, 0.0, 0.2), goal) is None
    assert 'moved away' in rt.limit_broken('turn', start, (0.16, 0.0, 0.2), goal)
    assert 'turning away' in rt.limit_broken('turn', start, (0.0, 0.0, math.radians(-61)), goal)


def test_overshoot_only_counts_travel_past_the_target():
    assert rt.overshoot(30.0, [30, 10, 1, 0.5]) == 0.0
    assert rt.overshoot(30.0, [30, 2, -4, -1, 0.2]) == 4.0
    assert rt.overshoot(-20.0, [-20, -3, 1.5, 0]) == 1.5
    assert rt.overshoot(179.0, [179.0, -179.5, 90, 0.1]) == 0.0      # the +-180 wrap is not an overshoot


def test_hand_measurement_and_book(tmp_path):
    row = {k: '' for k in rt.FIELDS}
    row.update(run=1, test='drive', asked='0.2', status='ARRIVED')
    rt.set_hand(row, 18.5)
    assert row['hand_error'] == '+1.50'
    rt.write_book(str(tmp_path), [row])
    back = rt.read_book(str(tmp_path / 'logbook.csv'))
    assert back[0]['hand_value'] == '18.5'
    assert '| 1 |' in (tmp_path / 'LOGBOOK.md').read_text(encoding='utf-8')
    row.update(test='heading', asked='90')
    rt.set_hand(row, 88)
    assert row['hand_error'] == ''                    # an absolute heading is not a turned angle


def test_series_is_balanced_and_never_reverses_the_plain_controller():
    for quick in (True, False):
        plan = rt.suite_plan(quick)
        assert sum(v for _, t, v in plan if t == 'turn' and _ == 'p') == 0
        assert sum(v for _, t, v in plan if t == 'turn' and _ == 'basic') == 0
        assert sum(v for _, t, v in plan if t == 'drive') == pytest.approx(0)
        assert all(v > 0 for c, t, v in plan if c == 'basic' and t == 'drive')
        assert all(abs(v) <= rt.LIMITS['turn_deg' if t == 'turn' else 'drive_m'] for _, t, v in plan)
        turns = {c: sorted(v for k, t, v in plan if k == c and t == 'turn') for c in ('p', 'basic')}
        assert turns['p'] == turns['basic']                       # same targets for both
    assert len(rt.suite_plan(True)) == 12


def test_at_rest_status():
    assert rt.at_rest_status(4.0, 4.0, (0, 0)) is None
    assert rt.at_rest_status(10.0, 3.0, (0, 0)) is None
    assert rt.at_rest_status(10.0, 4.5, (0.0, 0.0)) == 'SETTLED'
    assert rt.at_rest_status(10.0, 4.5, (0.0, 0.12)) == 'STUCK'


def test_summary_and_diagnosis():
    import robot_report as rr

    def row(run, status, error, asked='30', test='turn', over='0'):
        r = {k: '' for k in rt.FIELDS}
        r.update(run=run, session='s', controller='plain P', test=test, asked=asked, unit='deg', status=status, error=error,
                 overshoot=over, time_s='3')
        return r
    rows = [row(1, 'SETTLED', '+1.0'), row(2, 'STUCK', '+9.0'), row(3, 'ABORTED: robot x', '+50'),
            row(4, 'SETTLED', '+3.0', asked='-30')]
    s = rr.summarize(rows)[0]
    assert (s['runs'], s['aborted'], s['stuck'], s['judged'], s['within']) == (4, 1, 1, 3, 1)
    assert s['mean_error'] == pytest.approx(13 / 3) and s['worst_error'] == 9.0
    assert rr.diagnose(rows[0]) == ''
    assert 'still commanding' in rr.diagnose(rows[1])
    assert rr.diagnose(rows[2]).startswith('ABORTED')
    assert 'past the target' in rr.diagnose(rows[3])               # asked -30, +3 left: it went 3 deg too far
    assert 'short' in rr.diagnose(row(5, 'SETTLED', '+3.0'))
    assert 'overshot' in rr.diagnose(row(6, 'ARRIVED', '+0.5', over='9'))
