"""The response-measurement tool must recover known robot properties from synthetic data."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))

import measure_response as mr  # noqa: E402
import plant_models as pm  # noqa: E402


def measure(plant, noise=0.0, seed=0, **sched):
    schedule = mr.build_schedule(**sched)
    ts, v, w = mr.simulate(plant, schedule, noise=noise, seed=seed)
    return mr.analyse(ts, v, w, schedule)


def test_recovers_coasting_gain_and_deadband_of_the_professors_robot():
    res = measure(pm.IdentifiedPlant(use_filter=True))
    assert abs(res['v']['coast_lag'] - 0.61) < 0.08, res['v']
    assert abs(res['w']['coast_lag'] - 1.0) < 0.12, res['w']
    assert abs(res['v']['gain'] - 1.0) < 0.05 and abs(res['w']['gain'] - 1.0) < 0.06
    # the filter drops |v| <= 0.04 (with w = 0) and |w| <= 0.2 (with v = 0)
    assert abs(res['v_deadband'] - 0.04) < 1e-9 and abs(res['v_smallest_moving_command'] - 0.05) < 1e-9
    assert abs(res['w_deadband'] - 0.20) < 1e-9 and abs(res['w_smallest_moving_command'] - 0.25) < 1e-9


def test_reports_no_deadband_when_there_is_none():
    res = measure(pm.IdentifiedPlant(use_filter=False))
    assert res['v_deadband'] == 0.0 and res['w_deadband'] == 0.0
    lines, _warn = mr.suggestions(res)
    assert any('no v deadband' in l for l in lines) and any('no w deadband' in l for l in lines)


def test_fast_robot_gives_short_coasting_times():
    res = measure(pm.LagPlant(0.15))
    assert res['v']['coast_lag'] < 0.25 and res['w']['coast_lag'] < 0.25
    assert res['v']['latency'] is not None and res['v']['latency'] < 0.3


def test_noise_does_not_create_a_false_deadband_result():
    for seed in range(5):
        res = measure(pm.IdentifiedPlant(use_filter=True), noise=0.002, seed=seed)
        assert abs(res['v_deadband'] - 0.04) < 1e-9, (seed, res['v_levels'])
        assert abs(res['w_deadband'] - 0.20) < 1e-9, (seed, res['w_levels'])
        assert abs(res['v']['coast_lag'] - 0.61) < 0.15


def test_suggestions_contain_the_yaml_lines():
    res = measure(pm.IdentifiedPlant(use_filter=True))
    lines, warn = mr.suggestions(res)
    text = '\n'.join(lines)
    for key in ('lead_v:', 'lead_w:', 'deadband_v:', 'deadband_w:'):
        assert key in text
    assert not warn


def test_schedule_is_short_and_returns_the_robot_home():
    sched = mr.build_schedule()
    assert 70 < sum(s[3] for s in sched) < 95
    # alternating staircase signs: the forward and backward distances nearly cancel
    stair = [s[1] * s[3] for s in sched if s[0].startswith('v_level_')]
    assert abs(sum(stair)) < 0.5 * sum(abs(x) for x in stair)
