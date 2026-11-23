"""Regression tests on the pure-Python plant model (tools/sim_compare.py)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import sim_compare as s  # noqa: E402


def _ok(strategy, path, tau, seed):
    r = s.run(strategy, path, tau=tau, tmax=150.0, seed=seed, noise_p=0.002, noise_y=0.003)
    return r['ok'], r['pos_err']


def test_pure_pursuit_finishes_with_sluggish_plant_and_noise():
    """Regression: an overshoot at the end used to deadlock (remaining distance clamped to 0)."""
    for shape, path in (('square', s.SQUARE), ('circle', s.circle())):
        for seed in (1, 2, 3):
            ok, err = _ok('pure_pursuit', path, 0.5, seed)
            assert ok, f'{shape} seed {seed} did not finish'
            assert err < 0.05


def test_all_strategies_finish_on_the_square_with_noise():
    for strat in s.DEFAULTS:
        for tau in (0.10, 0.25, 0.50):
            ok, err = _ok(strat, s.SQUARE, tau, 1)
            assert ok, f'{strat} tau {tau} did not finish'
            assert err < 0.05
