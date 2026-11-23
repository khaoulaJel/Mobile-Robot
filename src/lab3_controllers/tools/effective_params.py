#!/usr/bin/env python3
"""Print (or save) the COMPLETE effective parameter set of the reference controller for a config file: the file's values, the
command-line overrides, and the defaults for everything the file does not set. This is what a run really used.

    python3 tools/effective_params.py config/reference_lab_robot.yaml --set state_topic=/robot_pose \
        --set cmd_topic=/diff_cont/cmd_vel_unstamped --out results/real_robot/params_used_run1.yaml

`ros2 param dump /reference_controller` gives the same thing from a running node.
"""
import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from lab3_controllers.tracker import DEFAULTS, YAW_DEFAULTS  # noqa: E402

NODE_DEFAULTS = dict(
    state_topic='/diff_cont/odom', state_type='auto', state_wait_timeout=60.0, state_jump_warn=0.3,
    cmd_topic='/cmd_vel_tracker', reference_topic='/reference', reference_yaw_topic='/reference_yaw',
    use_degrees=False, use_initial_yaw_reference=False, initial_yaw_reference=0.0, relative_to_start=False,
    max_linear_accel=0.4, max_angular_accel=3.0, rate=20.0, state_timeout=0.5, start_delay=1.0)


def parse_value(text):
    return yaml.safe_load(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('config')
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE', help='command-line override')
    ap.add_argument('--out')
    a = ap.parse_args()
    with open(a.config, encoding='utf-8') as f:
        given = yaml.safe_load(f)['reference_controller']['ros__parameters']
    effective = {**NODE_DEFAULTS, **DEFAULTS, **{k: v for k, v in YAW_DEFAULTS.items() if k not in DEFAULTS}}
    source = {k: 'default' for k in effective}
    for k, v in given.items():
        effective[k], source[k] = v, 'file'
    for item in a.set:
        k, v = item.split('=', 1)
        effective[k], source[k] = parse_value(v), 'command line'
    lines = [f'# effective parameters of reference_controller: {os.path.basename(a.config)} + overrides + defaults',
             'reference_controller:', '  ros__parameters:']
    for k in sorted(effective):
        lines.append(f'    {k}: {yaml.safe_dump(effective[k], default_flow_style=True).strip()}   # {source[k]}')
    text = '\n'.join(lines) + '\n'
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        open(a.out, 'w', encoding='utf-8', newline='\n').write(text)
        print('written', a.out)
    else:
        print(text)


if __name__ == '__main__':
    main()
