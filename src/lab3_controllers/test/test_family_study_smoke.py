"""The study tools (heading and position) must keep working: every family member can be sampled, perturbed and run on a case without
errors, and gives a finite result. Fast (a few cases); the real studies take tens of minutes."""
import math
import os
import random
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import p_family_study as ps  # noqa: E402
import x_family_study as xf  # noqa: E402
import x_real_sim as xs  # noqa: E402
import yaw_real_sim as ys  # noqa: E402


def test_every_heading_family_runs_on_every_condition_set():
    rng = random.Random(1)
    for name in ps.FAMILY:
        prm = ps.sample(rng, name)
        prm = ps.perturb(rng, prm, 0.1, name)
        for kw in (dict(), dict(stress=True), dict(mismatch=True)):
            s = ys.evaluate(prm, ys.make_cases(2, seed=7, **kw))
            assert math.isfinite(s['mean_abs']), (name, kw, s)


def test_every_position_family_runs_on_every_condition_set():
    rng = random.Random(2)
    for name in xf.FAMILY:
        prm = xf.sample(rng, name)
        prm = xf.perturb(rng, prm, 0.1, name)
        for kw in (dict(), dict(stress=True), dict(mismatch=True)):
            s = xs.evaluate(prm, xs.make_cases(2, seed=7, **kw))
            assert math.isfinite(s['mean_abs']), (name, kw, s)


def test_robustness_perturbations_are_all_defined():
    base = ys.make_cases(2, seed=3, mismatch=True)
    for what in ps.ROBUSTNESS:
        assert len(ps.perturbed_cases(base, what)) == 2, what
    xbase = xs.make_cases(2, seed=3, mismatch=True)
    for what in xf.ROBUSTNESS:
        assert len(xf.perturbed(xbase, what)) == 2, what
