"""LinearTracker (forward position x and x-dot): strict numeric checks on a simple robot model (first-order speed response with a
command deadband); no ROS."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lab3_controllers.tracker import LINEAR_DEFAULTS, LinearTracker  # noqa: E402

DT = 0.05


def drive(params, ref, tau=0.5, deadband=0.0, seconds=40.0, timed=False, accel=0.4):
    """Closed loop: the tracker commands the speed, the robot answers with a first-order lag after an optional command deadband."""
    tr = LinearTracker(params)
    x = v = cmd = 0.0
    xs = []
    for k in range(int(seconds / DT)):
        if timed:
            u = tr.step(x, ref, DT, stamp=k * DT, age=0.0, now=k * DT)
        else:
            u = tr.step(x, ref, DT)
        cmd += max(-accel * DT, min(accel * DT, u - cmd))                      # acceleration limit
        tr.observe_command(cmd)
        eff = 0.0 if abs(cmd) <= deadband else cmd
        v += (eff - v) * (1 - math.exp(-DT / tau))
        x += v * DT
        xs.append(x)
    return xs


def test_p_law_reaches_the_target_on_an_ideal_robot():
    xs = drive(dict(kv=0.8, pos_tolerance=0.005, max_linear_speed=0.2), 0.5)
    assert abs(xs[-1] - 0.5) <= 0.01, f'final error {abs(xs[-1] - 0.5) * 100:.2f} cm (limit 1)'


def test_speed_limit_is_respected():
    tr = LinearTracker(dict(kv=5.0, max_linear_speed=0.15))
    assert abs(tr.step(0.0, 2.0, DT)) <= 0.15 + 1e-12


def test_commands_inside_the_deadband_are_lifted_out_of_it():
    """A small error would give a command below the robot's deadband: the controller must raise it above the deadband, never send it."""
    tr = LinearTracker(dict(kv=0.8, pos_tolerance=0.005, deadband_v=0.04, deadband_w=0.2, diamond_v=0.3, diamond_w=2.5))
    v = tr.step(0.0, 0.03, DT)                       # 0.8 * 0.03 = 0.024 m/s < 0.04
    assert abs(v) > 0.04, f'sent {v} m/s, inside the robot deadband'


def test_inside_the_tolerance_the_command_is_zero():
    tr = LinearTracker(dict(kv=0.8, pos_tolerance=0.01, deadband_v=0.04, deadband_w=0.2))
    assert tr.step(0.0, 0.004, DT) == 0.0


def test_observer_with_derivative_stops_close_with_a_deadband_robot():
    """PD with the model-based observer on a robot with a deadband and a lag: within 2 cm, with little overshoot."""
    p = dict(kv=0.8, pos_tolerance=0.006, max_linear_speed=0.2, deadband_v=0.04, deadband_w=0.2, diamond_v=0.3, diamond_w=2.5,
             x_use_observer=True, x_obs_tau=0.5, x_obs_gain=1.0, x_obs_k1=0.3, x_obs_k2=0.1, lead_v=0.5, pose_latency=0.0,
             vel_filter=0.0)
    xs = drive(p, 0.6, tau=0.5, deadband=0.04, timed=True)
    assert abs(xs[-1] - 0.6) <= 0.02, f'final error {abs(xs[-1] - 0.6) * 100:.2f} cm (limit 2)'
    assert max(xs) - 0.6 <= 0.05, f'overshoot {(max(xs) - 0.6) * 100:.2f} cm (limit 5)'


def test_negative_targets_work_too():
    xs = drive(dict(kv=0.8, pos_tolerance=0.005, max_linear_speed=0.2), -0.4)
    assert abs(xs[-1] + 0.4) <= 0.01


def test_integral_term_is_bounded():
    """Anti-windup: with a large constant error the integral term can never exceed half the speed limit."""
    tr = LinearTracker(dict(kv=0.0, ki_v=0.5, max_linear_speed=0.2, pos_tolerance=0.001))
    for _ in range(2000):
        v = tr.step(0.0, 0.1, DT)                    # error 0.1 m, kv = 0: only the integral acts
    assert abs(v) <= 0.2 * 0.5 + 1e-9, f'integral output {v} exceeds half the speed limit'


def test_defaults_are_safe_and_everything_new_is_off():
    assert LINEAR_DEFAULTS['x_use_observer'] is False and LINEAR_DEFAULTS['ki_v'] == 0.0 and LINEAR_DEFAULTS['rest_window'] == 0.0
