"""The shipped p_controller values keep their accuracy on fixed robots the controller is not told about, and the config file agrees with
the code. The limits are the measured values with a margin: a change that makes the controller worse fails here."""
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import servo_sim as ss  # noqa: E402
from lab3_controllers.servo import LINEAR_SERVO_DEFAULTS, SERVO_DEFAULTS  # noqa: E402


@pytest.mark.parametrize('label,plant,seed', [('random robots', 'random', 501), ('different structure', 'identified', 502)])
def test_heading_accuracy(label, plant, seed):
    s = ss.evaluate_heading(lambda: ss.ServoHeading({}), ss.make_cases(60, seed=seed, plant=plant))
    assert s['mean'] <= 0.6 and s['within2'] >= 0.96, f'{label}: mean {s["mean"]:.2f} deg, within 2 deg {s["within2"] * 100:.0f} %'


@pytest.mark.parametrize('label,plant,seed', [('random robots', 'random', 503), ('different structure', 'identified', 504)])
def test_distance_accuracy(label, plant, seed):
    s = ss.evaluate_distance(lambda: ss.ServoDistance({}), ss.make_cases(60, seed=seed, plant=plant))
    assert s['mean'] <= 1.3 and s['within2'] >= 0.92, f'{label}: mean {s["mean"]:.2f} cm, within 3 cm {s["within2"] * 100:.0f} %'


def test_go_to_pose_accuracy():
    s = ss.evaluate_pose(lambda: ss.ServoPose({}), ss.make_cases(30, seed=505))
    assert s['ok'] >= 0.93 and s['mean'] <= 1.5 and s['head_mean'] <= 1.0, s


def test_far_better_than_a_plain_p_law_on_the_same_robots():
    cases = ss.make_cases(60, seed=506)
    plain = ss.evaluate_heading(lambda: ss.ServoHeading(dict(kp=1.2, lead=0.0, u_min=0.0, adapt=False, pulses=False)), cases)
    full = ss.evaluate_heading(lambda: ss.ServoHeading({}), cases)
    assert full['mean'] < plain['mean'] / 4.0, (full['mean'], plain['mean'])


def test_config_file_equals_the_code_defaults():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(HERE, '..', 'config', 'p_controller.yaml'), encoding='utf-8') as f:
        params = yaml.safe_load(f)['p_controller']['ros__parameters']
    for axis, defaults in (('heading', SERVO_DEFAULTS), ('linear', LINEAR_SERVO_DEFAULTS)):
        in_file = {k.split('.', 1)[1]: v for k, v in params.items() if k.startswith(axis + '.')}
        assert in_file == defaults, {k: (in_file.get(k), defaults.get(k)) for k in set(in_file) | set(defaults) if in_file.get(k) != defaults.get(k)}
