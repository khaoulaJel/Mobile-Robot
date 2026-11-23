"""The heading preset config/reference_lab_robot_tuned.yaml must keep the performance its header claims: the YAML is loaded, turned
into controller parameters exactly as the node does, and run through the real-conditions simulator (tools/yaw_real_sim.py) on fixed
cases. The limits are the measured values with a margin; a change that makes the preset worse fails here."""
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
yaml = pytest.importorskip('yaml')
import yaw_real_sim as ys  # noqa: E402
from lab3_controllers.tracker import DEFAULTS, YAW_DEFAULTS  # noqa: E402


def preset_params(name='reference_lab_robot_tuned.yaml'):
    with open(os.path.join(HERE, '..', 'config', name), encoding='utf-8') as f:
        p = yaml.safe_load(f)['reference_controller']['ros__parameters']
    keys = set(DEFAULTS) | set(YAW_DEFAULTS)
    out = {k: v for k, v in p.items() if k in keys}
    out['_timed'] = bool(p.get('use_state_stamps', False))
    return out


@pytest.mark.parametrize('label,kw,mean_limit,within1_min', [
    ('normal conditions', dict(seed=501), 0.6, 0.90),
    ('stress conditions', dict(seed=502, stress=True), 0.8, 0.85),
    ('wrong robot (deadband 0.12-0.30, extra lag)', dict(seed=503, mismatch=True), 4.0, 0.40),
])
def test_tuned_preset_keeps_its_claimed_accuracy(label, kw, mean_limit, within1_min):
    s = ys.evaluate(preset_params(), ys.make_cases(40, **kw))
    assert s['mean_abs'] <= mean_limit, f'{label}: mean error {s["mean_abs"]:.2f} deg (limit {mean_limit})'
    assert s['within1'] >= within1_min, f'{label}: only {s["within1"] * 100:.0f} % within 1 deg (need {within1_min * 100:.0f} %)'


def test_the_plain_p_law_is_far_worse_on_the_same_cases():
    """The reason for the whole design: bare P on this robot ends several degrees off."""
    cases = ys.make_cases(40, seed=502, stress=True)
    plain = ys.evaluate(dict(kp=1.0, tolerance=0.01, diamond_v=0.3, diamond_w=2.5, deadband_v=0.04, deadband_w=0.2), cases)
    tuned = ys.evaluate(preset_params(), cases)
    assert plain['mean_abs'] > 3.0 and tuned['mean_abs'] < plain['mean_abs'] / 5.0, (plain['mean_abs'], tuned['mean_abs'])
