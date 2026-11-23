#!/usr/bin/env python3
"""Re-test the tuned members of the P family (tools/p_family_study.py) against a robot that DIFFERS from the model they know:
wrong deadband, an extra robot lag the model is unaware of, and conditions beyond the range they were tuned for. No retuning.

    python3 tools/p_family_robustness.py results/real_robot/p_family_study.json [--cases 80]
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import p_family_study as ps  # noqa: E402
import yaw_real_sim as ys  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('study_json')
    ap.add_argument('--cases', type=int, default=80)
    a = ap.parse_args()
    tuned = json.load(open(a.study_json))['tuned']
    base = ys.make_cases(a.cases, seed=401, stress=True)
    names = list(tuned)
    print('mean final error (deg) [worst case in brackets] on stress cases, robot different from the controller\'s model')
    print(f'{"":<38}' + ''.join(f'{n:>16}' for n in names))
    for what in ps.ROBUSTNESS:
        cases = base if what == 'nominal (wrong robot)' else ps.perturbed_cases(base, what)
        row = f'{what:<38}'
        for n in names:
            s = ps.stats(tuned[n], cases)
            row += f'{s["mean"]:9.2f} [{s["worst"]:5.1f}]'
        print(row, flush=True)


if __name__ == '__main__':
    main()
