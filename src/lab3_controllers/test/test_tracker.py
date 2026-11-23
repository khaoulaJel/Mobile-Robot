"""Tests for the shared PathTracker, the professor's command filter and the identified plant."""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))

import plant_models as pm  # noqa: E402
import sim_compare as sc  # noqa: E402
from lab3_controllers.tracker import DEFAULTS, PathTracker  # noqa: E402

ROBOT = dict(diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2)


def tracker(**over):
    params = dict(over)
    return PathTracker([(1.0, 0.0, 0.0)], params)


# ---- the professor's filter, transcribed independently from the Simulink chart ------------------
def reference_filter(v, w):
    Vmax, Wmax, Vdead, Wdead = 0.3, 2.5, 0.04, 0.2
    feasible = (abs(v) / Vmax + abs(w) / Wmax) <= 1
    deadband = (abs(v) <= Vdead) and (abs(w) <= Wdead)
    return (v, w) if feasible and not deadband else (0.0, 0.0)


def test_input_filter_matches_simulink_chart():
    rng = random.Random(1)
    for _ in range(5000):
        v, w = rng.uniform(-0.4, 0.4), rng.uniform(-3, 3)
        assert pm.input_filter(v, w) == reference_filter(v, w)
    for v, w in [(0.04, 0.2), (0.0401, 0.0), (0.0, 0.2001), (0.3, 0.0), (0.3, 0.001), (0.0, 2.5), (0.0, 2.51)]:
        assert pm.input_filter(v, w) == reference_filter(v, w)


def test_shaping_off_by_default_is_identity():
    tr = tracker()
    for v, w in [(0.01, 0.01), (0.3, 3.0), (-0.1, -2.0), (0.0, 0.0)]:
        assert tr._shape(v, w) == (v, w)


def test_diamond_scaling_keeps_curvature_and_fits():
    tr = tracker(**ROBOT)
    v, w = tr._shape(0.3, 2.0)                 # far outside the diamond
    assert abs(v) / 0.3 + abs(w) / 2.5 <= 1.0
    assert abs(w / v - 2.0 / 0.3) < 1e-9       # same curvature
    assert tr._shape(0.1, 0.1) == (0.1, 0.1)   # already feasible: untouched


def test_deadband_is_left_in_the_right_direction():
    tr = tracker(**ROBOT)
    v, w = tr._shape(0.02, 0.05)               # creeping forward: lift v out of the deadband, same curvature
    assert v > 0.04 and abs(w / v - 0.05 / 0.02) < 1e-9
    v, w = tr._shape(0.0, -0.1)                # turning in place: lift |w|, keep the sign
    assert v == 0.0 and w < -0.2
    assert tr._shape(0.0, 0.0) == (0.0, 0.0)   # "stop" stays a stop


def test_shaped_command_always_survives_the_robot_filter():
    """The whole point: the robot must never silently drop what the controller sends."""
    tr = tracker(**ROBOT)
    rng = random.Random(2)
    for _ in range(5000):
        v, w = rng.uniform(-0.1, 0.35), rng.uniform(-2.5, 2.5)
        if rng.random() < 0.3:
            v = rng.uniform(0.0, 0.05)
        if rng.random() < 0.3:
            w = rng.uniform(-0.25, 0.25)
        if rng.random() < 0.2:
            v = 0.0
        if rng.random() < 0.05:
            v = w = 0.0
        out = tr._shape(v, w)
        assert pm.input_filter(*out) == out, (v, w, out)


def test_lead_predicts_the_rest_position():
    tr = tracker(lead_v=0.6, lead_w=1.0, vel_filter=1.0)
    x = 0.0
    pred = None
    for k in range(5):
        pred = tr._predict((x, 0.0, 0.0), 0.05)
        x += 0.1 * 0.05                        # moving at 0.1 m/s
    assert abs(pred[0] - (x - 0.005 + 0.1 * 0.6)) < 1e-9 and pred[1] == 0.0 and pred[2] == 0.0


def test_defaults_are_a_complete_valid_parameter_set():
    PathTracker([(0.5, 0.0, 0.0)], dict(DEFAULTS))
    with pytest.raises(ValueError):
        PathTracker([(0.5, 0.0, 0.0)], dict(strategy='bogus'))
    with pytest.raises(ValueError):
        PathTracker([(0.5, 0.0, 0.0)], dict(k_alpha=0.5))
    with pytest.raises(ValueError):
        PathTracker([], {})


def test_tracker_drives_the_square_on_the_lag_plant():
    r = sc.run('stop_turn', sc.SQUARE, tau=0.15)
    assert r['ok'] and r['pos_err'] < 0.015 and r['yaw_err'] < 1.0


# ---- the identified plant data ---------------------------------------------------------------------
def _coast(channel, cmd):
    plant = pm.IdentifiedPlant(use_filter=False)
    for _ in range(600):
        y = plant.step(*cmd)
    steady, area = y[channel], 0.0
    for _ in range(1200):
        y = plant.step(0.0, 0.0)
        area += y[channel] * 0.05
    return steady, area / steady


def test_identified_plant_gain_and_coasting_time():
    s, lag = _coast(0, (0.1, 0.0))
    assert abs(s / 0.1 - 1.0) < 0.03 and abs(lag - 0.61) < 0.03
    s, lag = _coast(1, (0.0, 0.5))
    assert abs(s / 0.5 - 1.0) < 0.05 and abs(lag - 1.0) < 0.05


# ---- yaw tracker ---------------------------------------------------------------------------------------
def test_yaw_tracker_legacy_mode_is_the_plain_p_law():
    from lab3_controllers.common import yaw_p_control
    from lab3_controllers.tracker import YawTracker
    tr = YawTracker(dict(kp=1.3, max_angular_speed=0.7, min_angular_speed=0.1, tolerance=0.02))
    for yaw, ref in [(0.0, 1.0), (0.0, 0.05), (3.0, -3.0), (0.5, 0.51), (0.0, 0.0)]:
        w = tr.step(yaw, ref, 0.05)
        assert w == yaw_p_control(ref, yaw, 1.3, 0.7, 0.1, 0.02)[0]


def test_yaw_tracker_lab_preset_settles_on_the_identified_plant():
    import yaw_bench
    for r in yaw_bench.run(yaw_bench.LAB, hold=40.0):
        assert r['settle'] < 15.0, r
        assert r['over'] < 4.0, r
        assert abs(r['final']) < 0.6, r


def test_yaw_tracker_legacy_p_law_fails_on_the_identified_plant():
    """Documents why the new logic exists: the old law limit-cycles (this test would catch a silent change
    of the plant data that made the comparison meaningless)."""
    import yaw_bench
    res = yaw_bench.run(dict(kp=1.0, **yaw_bench.ROBOT), hold=40.0)
    assert min(r['over'] for r in res) > 5.0


def test_pulse_length_matches_the_error():
    """A settled robot with a 3 deg error gets round(e / (pulse_w * dt)) + pulse_extra samples of +-pulse_w
    (pulse_w = 1.1 * deadband_w), then waits again."""
    from lab3_controllers.tracker import YawTracker
    tr = YawTracker(dict(kp=1.0, rest_window=0.2, rest_timeout=1.0, settle_time=0.1, tolerance=0.01, **ROBOT))
    e = math.radians(3.0)
    tr.ref = e                      # skip the turn phase: the robot is already at rest, 3 deg off
    tr._fine = 'settle'
    out = [tr.step(0.0, e, 0.05) for _ in range(40)]
    first = next(i for i, w in enumerate(out) if w != 0.0)
    run = 0
    while first + run < len(out) and out[first + run] != 0.0:
        run += 1
    assert abs(out[first] - 0.22) < 1e-9
    assert run == round(e / (0.22 * 0.05)) + 1
    # and the sign follows the error
    tr2 = YawTracker(dict(kp=1.0, rest_window=0.2, rest_timeout=1.0, settle_time=0.1, tolerance=0.01, **ROBOT))
    tr2.ref = -e
    tr2._fine = 'settle'
    assert abs(min(tr2.step(0.0, -e, 0.05) for _ in range(40)) + 0.22) < 1e-9


def test_lab_preset_completes_a_square_on_the_identified_plant():
    import shape_bench as sb
    import tune_lab_plant as tp
    over = dict(tp.FIXED)
    over.update(dict(kv=0.8, kw=2.2, k_yaw=0.8, turn_in_place_deg=15.0, lead_v=0.3, lead_w=0.8,
                     vel_filter=0.7, max_linear_speed=0.1))
    r = sb.run_all(0.1, 0.15, ['stop_turn'], over={'stop_turn': over}, plant='identified',
                   only=['square 0.5'], settle=12.0)[0]
    assert r['ok'] and r['max'] < 1.0 and r['fin'] < 1.0 and r['yaw'] < 0.8, r


def test_competition_defaults_are_unchanged():
    """The Gazebo-tuned defaults (controllers.yaml / node defaults) must not move silently."""
    expect = dict(strategy='stop_turn', kv=0.8, kw=3.0, k_yaw=1.0, turn_in_place_deg=35.0, k_rho=1.5,
                  k_alpha=4.0, k_beta=-2.0, blend_radius=0.03, lookahead=0.12, decel=0.4,
                  max_linear_speed=0.2, max_angular_speed=1.0, min_linear_speed=0.0, min_angular_speed=0.0,
                  pos_tolerance=0.01, yaw_tolerance=0.01, lead_v=0.0, lead_w=0.0, rest_window=0.0,
                  smooth_yaw_tolerance=0.0, diamond_v=0.0, diamond_w=0.0, deadband_v=0.0, deadband_w=0.0)
    for k, v in expect.items():
        assert DEFAULTS[k] == v, k
