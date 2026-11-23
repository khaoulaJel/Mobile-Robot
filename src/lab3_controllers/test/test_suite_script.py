"""The acceptance script itself must be sound: every test it calls is defined, and it parses."""
import os
import re
import subprocess

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools', 'run_all.sh')


def read():
    return open(SCRIPT, encoding='utf-8').read()


def test_every_called_test_function_is_defined():
    """A deleted function would make the suite print 'command not found' for that test; this catches it first."""
    s = read()
    defined = set(re.findall(r'^(t_\w+)\(\)', s, re.M))
    called = set(re.findall(r'run_test\s+"[^"]*"\s+(t_\w+)', s))
    assert called, 'no run_test calls found: the pattern is out of date'
    assert not (called - defined), f'called but not defined: {sorted(called - defined)}'


def test_script_parses():
    out = subprocess.run(['bash', '-n', SCRIPT], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr


def test_the_new_real_robot_tests_are_in_the_suite():
    """name -> minimum occurrences: the launch-topic test is defined once and called once (before Gazebo, both paths
    share that part); the others are defined once and called on both the full and the shortcut path."""
    s = read()
    for name, minimum in (('t_launch_params', 2), ('t_measure', 3), ('t_symbolic', 3), ('t_circle_p', 3),
                          ('t_lab_square', 3), ('t_lab_yaw', 3)):
        assert f'{name}()' in s, f'{name} is not defined'
        assert s.count(name) >= minimum, f'{name} is not called where expected ({s.count(name)} occurrences)'
