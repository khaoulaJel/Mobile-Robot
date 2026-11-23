"""The reference-controller config files must only contain parameters the node really declares (ROS silently ignores a
misspelled parameter in a file, which would leave a coefficient at its default without any error)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
yaml = pytest.importorskip('yaml')
from lab3_controllers.tracker import DEFAULTS, YAW_DEFAULTS  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')
NODE_PARAMS = {'state_topic', 'state_type', 'state_wait_timeout', 'state_jump_warn', 'cmd_topic', 'reference_topic',
               'reference_yaw_topic', 'use_degrees', 'use_initial_yaw_reference', 'initial_yaw_reference',
               'relative_to_start', 'use_state_stamps', 'max_linear_accel', 'max_angular_accel', 'rate', 'state_timeout', 'start_delay'}
ALLOWED = NODE_PARAMS | set(DEFAULTS) | set(YAW_DEFAULTS)


@pytest.mark.parametrize('name', ['reference_controller.yaml', 'reference_lab_robot.yaml', 'reference_lab_robot_tuned.yaml'])
def test_every_key_is_a_declared_parameter(name):
    with open(os.path.join(CONFIG, name), encoding='utf-8') as f:
        params = yaml.safe_load(f)['reference_controller']['ros__parameters']
    unknown = sorted(set(params) - ALLOWED)
    assert not unknown, f'{name}: parameters the node does not declare (ignored by ROS): {unknown}'


def test_lab_robot_preset_matches_the_professors_robot_model():
    """The deadband / diamond / lead numbers must stay identical to lab_robot.yaml (one source of truth for the robot model)."""
    def load(name, node):
        with open(os.path.join(CONFIG, name), encoding='utf-8') as f:
            return yaml.safe_load(f)[node]['ros__parameters']
    ref = load('reference_lab_robot.yaml', 'reference_controller')
    lab = load('lab_robot.yaml', 'motion_controller')
    for key in ('diamond_v', 'diamond_w', 'deadband_v', 'deadband_w', 'lead_v', 'lead_w', 'vel_filter', 'vel_window',
                'rest_window', 'kv', 'kw', 'k_yaw', 'turn_in_place_deg', 'pos_tolerance', 'yaw_tolerance'):
        assert ref[key] == lab[key], f'{key}: reference_lab_robot.yaml {ref[key]} != lab_robot.yaml {lab[key]}'
