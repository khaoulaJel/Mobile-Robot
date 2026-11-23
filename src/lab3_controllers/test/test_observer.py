"""Timed mode and the model-based heading observer: strict numeric checks (no ROS)."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lab3_controllers.common import wrap_angle  # noqa: E402
from lab3_controllers.observer import HeadingObserver  # noqa: E402
from lab3_controllers.tracker import DEFAULTS, YawTracker, YAW_DEFAULTS  # noqa: E402


def simulate_first_order(commands, tau, gain, dt, deadband=0.0):
    """Reference robot: deadband, then a first-order lag on the turn rate. Returns heading and turn rate at every step."""
    yaw = w = 0.0
    out = []
    for u in commands:
        u_eff = 0.0 if abs(u) <= deadband else u
        w += (gain * u_eff - w) * (1 - math.exp(-dt / tau))
        yaw += w * dt
        out.append((yaw, w))
    return out


def test_observer_follows_the_robot_without_any_measurement():
    """With the right model and NO samples at all (a freeze), the prediction stays within 1 deg of the real heading."""
    dt, tau, gain = 0.05, 0.8, 1.0
    cmds = [0.6] * 60 + [0.0] * 60 + [-0.4] * 40 + [0.0] * 60
    truth = simulate_first_order(cmds, tau, gain, dt)
    obs = HeadingObserver(tau=tau, gain=gain, latency=0.0)
    obs.correct(0.0, 0.0)                                   # one initial sample, then nothing
    worst = 0.0
    for k, u in enumerate(cmds):
        obs.advance((k + 1) * dt, u)
        worst = max(worst, abs(math.degrees(wrap_angle(obs.yaw - truth[k][0]))))
    assert worst < 1.0, f'prediction error {worst:.2f} deg without measurements (limit 1)'


def test_observer_drops_commands_inside_the_deadband():
    obs = HeadingObserver(tau=0.8, gain=1.0, latency=0.0, deadband_w=0.2)
    obs.correct(0.0, 0.0)
    for k in range(100):
        obs.advance((k + 1) * 0.05, 0.15)                   # below the 0.2 rad/s deadband: the robot does not move
    assert abs(obs.yaw) < 1e-9 and abs(obs.w) < 1e-9, f'moved to {obs.yaw} although the command was inside the deadband'


def test_observer_corrects_a_wrong_model_with_delayed_noisy_samples():
    """The robot turns 20 % faster than the model says, samples arrive at 8 Hz, 0.3 s late, 0.5 deg noise.
    The corrected prediction must stay within 3 deg of the truth (without correction it would drift by far more)."""
    rng = random.Random(3)
    dt, tau, latency = 0.05, 0.8, 0.3
    cmds = [0.7] * 80 + [0.0] * 120
    truth = simulate_first_order(cmds, tau, 1.2, dt)        # real gain 1.2
    obs = HeadingObserver(tau=tau, gain=1.0, latency=latency, k1=0.4, k2=0.2)
    free = HeadingObserver(tau=tau, gain=1.0, latency=latency)
    obs.correct(0.0, 0.0)
    free.correct(0.0, 0.0)
    delayed = []
    err_corr = err_free = 0.0
    for k, u in enumerate(cmds):
        t = (k + 1) * dt
        obs.advance(t, u)
        free.advance(t, u)
        if k % 3 == 0:                                      # a sample every 0.15 s, arriving `latency` later
            delayed.append((t + latency, wrap_angle(truth[k][0] + rng.gauss(0, math.radians(0.5)))))
        while delayed and delayed[0][0] <= t:
            _, z = delayed.pop(0)
            obs.correct(t, z)
        if k > 120:
            err_corr = max(err_corr, abs(math.degrees(wrap_angle(obs.yaw - truth[k][0]))))
            err_free = max(err_free, abs(math.degrees(wrap_angle(free.yaw - truth[k][0]))))
    assert err_corr < 3.0, f'corrected prediction error {err_corr:.2f} deg (limit 3)'
    assert err_corr < err_free, 'the correction must improve on the uncorrected model'


def test_timed_mode_slope_estimate_averages_the_noise():
    """Constant turn of 0.5 rad/s sampled irregularly with 1.5 deg noise: the slope estimate is within 15 % of the truth."""
    rng = random.Random(5)
    tr = YawTracker(dict(vel_span=1.2, vel_filter=1.0, lead_w=0.5, rest_window=0.0))
    t, yaw, est = 0.0, 0.0, []
    for _ in range(80):
        t += rng.uniform(0.07, 0.14)
        yaw = 0.5 * t
        tr._predict_timed((0.0, 0.0, wrap_angle(yaw + rng.gauss(0, math.radians(1.5)))), t, 0.0)
        est.append(tr.w_est)
    w = sum(est[-30:]) / 30
    assert abs(w - 0.5) < 0.075, f'turn-rate estimate {w:.3f} rad/s, truth 0.5 (limit 15 %)'


def test_timed_mode_forgets_everything_after_a_gap():
    tr = YawTracker(dict(vel_span=1.0, vel_filter=1.0, rest_window=1.0))
    for k in range(20):
        tr._predict_timed((0.0, 0.0, 0.3 * 0.1 * k), 0.1 * k, 0.0)
    assert abs(tr.w_est) > 0.1
    tr._predict_timed((0.0, 0.0, 1.0), 1.9 + 2.0, 0.0)        # no samples for two seconds (a freeze)
    assert tr.w_est == 0.0 and tr.v_est == 0.0, 'estimates must be cleared after a gap in the data'


def test_rest_test_by_averages_ignores_noise_but_sees_motion():
    rng = random.Random(9)
    p = dict(rest_window=1.0, rest_mean_test=True, rest_yaw=0.02, rest_pos=0.003)
    still, moving = YawTracker(p), YawTracker(p)
    for k in range(40):
        t = 0.1 * k
        still._predict_timed((0.0, 0.0, rng.gauss(0, math.radians(1.0))), t, 0.0)       # 1 deg noise, no motion
        moving._predict_timed((0.0, 0.0, 0.2 * t + rng.gauss(0, math.radians(1.0))), t, 0.0)   # 0.2 rad/s
    assert still._at_rest(0.05), 'a robot standing still with noisy poses must count as at rest'
    assert not moving._at_rest(0.05), 'a turning robot must not count as at rest'


def test_new_parameters_are_declared_with_safe_defaults():
    """Everything new is off by default, so existing behaviour is unchanged."""
    for key, off in (('use_observer', False), ('ki', 0.0)):
        assert YAW_DEFAULTS[key] == off
    assert DEFAULTS['pose_latency'] == 0.0 and DEFAULTS['vel_span'] == 0.0 and DEFAULTS['rest_mean_test'] is False
