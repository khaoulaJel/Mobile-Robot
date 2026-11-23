"""tools/log_run.py: the verdict must tell the three causes of an unexpected position change apart (synthetic runs, no ROS)."""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import log_run  # noqa: E402


def write_run(path, odom_xy, map_xy, cmd_v=0.0, cmd_w=0.6, n=100, dyaw=math.radians(90)):
    """A 10 s turn sampled at 10 Hz. odom_xy / map_xy: functions of progress u in 0..1 giving (x, y) in metres."""
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'source', 'x', 'y', 'yaw', 'cmd_v', 'cmd_w'])
        for i in range(n):
            u = i / (n - 1)
            t = i * 0.1
            x, y = odom_xy(u)
            w.writerow([t, 'odom', x, y, dyaw * u, cmd_v, cmd_w])
            mx, my = map_xy(u)
            w.writerow([t + 0.01, 'map', mx, my, dyaw * u - 1.3, cmd_v, cmd_w])


def test_robot_that_does_not_turn_on_the_spot_is_blamed(tmp_path):
    p = tmp_path / 'r.csv'
    write_run(p, lambda u: (0.08 * u, 0.0), lambda u: (0.08 * u, 0.0))            # the wheels moved 8 cm during a pure turn
    verdict = log_run.analyze(str(p))
    assert any('ROBOT moved' in v for v in verdict), verdict


def test_slam_corrections_are_blamed_when_only_the_map_moves(tmp_path):
    p = tmp_path / 's.csv'
    write_run(p, lambda u: (0.003 * u, 0.0), lambda u: (0.0 if u < 0.5 else 0.12, 0.0))      # map jumps 12 cm, wheels 0.3 cm
    verdict = log_run.analyze(str(p))
    assert any('SLAM' in v for v in verdict), verdict
    assert not any('ROBOT moved' in v for v in verdict), verdict


def test_a_clean_turn_has_no_complaint(tmp_path):
    p = tmp_path / 'c.csv'
    write_run(p, lambda u: (0.004 * u, 0.0), lambda u: (0.004 * u, 0.0))
    assert log_run.analyze(str(p)) == []


def test_forward_commands_are_reported(tmp_path):
    p = tmp_path / 'f.csv'
    write_run(p, lambda u: (0.2 * u, 0.0), lambda u: (0.2 * u, 0.0), cmd_v=0.1)
    verdict = log_run.analyze(str(p))
    assert any('forward command' in v for v in verdict), verdict


def test_recording_without_commands_says_so(tmp_path):
    p = tmp_path / 'n.csv'
    write_run(p, lambda u: (0.0, 0.0), lambda u: (0.0, 0.0), cmd_w=0.0)
    assert log_run.analyze(str(p)) is None
