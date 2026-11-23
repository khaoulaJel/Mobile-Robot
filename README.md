# Mobile-Robot

Controllers for a two-wheeled differential-drive robot, written for ROS 2 Humble:

- a **yaw controller** (heading control with a proportional law),
- a **motion controller** (path tracking with proportional laws; any path given as points `(x, y, theta)`),
- a **symbolic controller** (abstraction-based control with a transition table and a guarantee),

plus the tools to tune, test and commission them in simulation (a Python model and Gazebo) and on the real robot.

> **Status:** validated in simulation only (Python model and Gazebo Classic). This code has not been run on the real robot yet;
> `docs/real_robot.md` is the commissioning procedure.

## What is implemented

| Task | Implementation | Run |
| --- | --- | --- |
| Heading control (P law) | `yaw_controller`: `w = kp * wrap(yaw_ref - yaw)`, reference changeable at run time | `ros2 launch lab3_controllers yaw.launch.py` |
| Path tracking (P laws) | `motion_controller`: strategies `stop_turn`, `polar`, `pure_pursuit`; waypoint list or built-in circle | `ros2 launch lab3_controllers motion.launch.py` |
| Symbolic control | `symbolic_controller`: cell abstraction of the kinematic model, transition table with all successor cells, synthesized policy; 1 m-diameter circle | `ros2 launch lab3_controllers symbolic.launch.py` |
| Reference controller (generic) | `reference_controller`: the reference comes from a topic (`/reference` pose, `/reference_yaw` heading), the state from a topic, the coefficients are parameters (file, or live with `ros2 param set`) | `ros2 launch lab3_controllers reference.launch.py` |
| Real-robot behaviour (command deadband, slow response) | optional compensation in the shared control code: command shaping, stopping-point prediction, corrective pulses | `config:=lab_robot.yaml` / `hardware.yaml` |
| Commissioning | `tools/measure_response.py` (latency, coasting time, deadband), `tools/calibrate_odometry.py`, `tools/check_state_source.py` | see `docs/real_robot.md` |

All controllers subscribe to the robot state (`nav_msgs/Odometry`, `PoseStamped`, `Pose` or `PoseWithCovarianceStamped`, auto-detected) and publish
`geometry_msgs/Twist` (`linear.x` = v in m/s, `angular.z` = w in rad/s). They limit speed and acceleration, stop when the state goes stale (0.5 s)
and send zero commands on shutdown.

## Repository layout

```
src/cc_robot_urdf/        (simulation workspace) robot model for Gazebo (course material)
src/lab3_gazebo/          Gazebo world and launch file (course material)
src/lab3_controllers/
  lab3_controllers/   common.py (control laws), tracker.py (path/yaw state machine), compensation.py (slow-robot compensation),
                      symbolic.py (abstraction, table, synthesis),
                      yaw_controller.py, motion_controller.py, symbolic_controller.py, state_input.py
  config/             controllers.yaml (Gazebo), robust.yaml, lab_robot.yaml, hardware.yaml, symbolic_circle.npz
  launch/             yaw / motion / symbolic launch files
  test/               unit and regression tests (pytest)
  tools/              model and simulation tools, tuning, measurement, acceptance suite (run_all.sh)
pi_ws/src/             Raspberry Pi workspace (course material): articubot_one (hardware.launch.py), diffdrive_arduino, ldlidar_stl_ros2
laptop_ws/             laptop workspace (course material): run_desktop.sh, EKF, SLAM, twist_mux, joystick, RViz
docs/                 controllers.md, symbolic_control.md, real_robot.md
results/              generated result tables (lab robot model, symbolic model)
```

## Requirements

- ROS 2 Humble, Gazebo Classic 11 (for the simulation tests), Python 3.10.
- Python packages for the tools and tests: `numpy`, `matplotlib`, `pytest`, `PyYAML`.
- Everything needed is in this repository. Course material is included unchanged: `src/cc_robot_urdf`, `src/lab3_gazebo` (simulation), `pi_ws/` (robot side) and `laptop_ws/` (laptop side). Licenses: Apache-2.0 / MIT, see each package.

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src       # only src/ (pi_ws and laptop_ws are separate workspaces)
source install/setup.bash

ros2 launch lab3_gazebo gazebo.launch.py                 # simulation; run ONE controller at a time
ros2 launch lab3_controllers yaw.launch.py
ros2 launch lab3_controllers motion.launch.py            # 0.5 m square, stop_turn
ros2 run lab3_controllers motion_controller --ros-args -p use_sim_time:=true \
    -p strategy:=pure_pursuit -p path_shape:=circle -p circle_radius:=0.5 -p relative_to_start:=true   # circle of 1 m diameter
ros2 launch lab3_controllers symbolic.launch.py
```

Presets (`config:=...` on the yaw and motion launch files):

| File | Use |
| --- | --- |
| `controllers.yaml` | Gazebo, lowest error |
| `robust.yaml` | slower or noisier motors, relaxed tolerances |
| `lab_robot.yaml` | a robot that drops small commands (|v| <= 0.04 and |w| <= 0.2) and responds slowly (coasting about 0.6 s / 1.0 s) |
| `hardware.yaml` | real robot, first run: state `/diff_cont/odom`, command `/cmd_vel_tracker`, conservative limits |

## Real robot (three parts)

```
Raspberry Pi   pi_ws/       robot driver: Arduino wheels, encoders, lidar        ros2 launch articubot_one hardware.launch.py
Laptop         laptop_ws/   robot_description, EKF, SLAM, twist_mux, joystick   ./run_desktop.sh      (install_dependencies.sh, build.sh first)
Laptop         this repo    OUR controllers (publish /cmd_vel_tracker)           ros2 launch lab3_controllers motion.launch.py config:=hardware.yaml use_sim_time:=false
```

Both machines: same LAN, `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, synchronised clocks (see `laptop_ws/START_HERE.md`).
Pi build: copy `pi_ws/` to the Pi and run `colcon build --symlink-install` there. The joystick keeps priority over our controllers in `twist_mux`.
Network, firewall, CycloneDDS peers and start order that worked on the real setup: `docs/network_setup.md`. Commissioning order (measure, first motion, tests): `docs/real_robot.md`.

## Tests

```bash
cd src/lab3_controllers && PYTHONPATH=. python3 -m pytest -q test      # 55 unit and regression tests, no ROS graph needed
bash src/lab3_controllers/tools/run_all.sh                             # acceptance suite in Gazebo (strict numeric limits)
```

The acceptance suite builds the workspace in a temporary folder, runs safety tests, Gazebo bring-up, both launch files, yaw references, six
path/strategy runs, repeatability, the circle of 1 m diameter, the symbolic controller, an emulation of a deadband/slow robot, and the
measurement tool. It exits non-zero if any check fails.

## Results (simulation)

| Check | Result |
| --- | --- |
| Yaw controller, 5 references (Gazebo) | final errors within 0.3 deg |
| Square 0.5 m, `stop_turn` (Gazebo, 10 runs) | 0.78 +/- 0.03 cm, 0.55 +/- 0.02 deg |
| Circle of 1 m diameter, tangent start (Gazebo) | max distance from the circle 0.39 cm (`stop_turn`), 0.26 cm (`pure_pursuit`) |
| 14 path shapes on the model of a deadband/slow robot | path error 0.2-1.6 cm, final heading error at most 0.45 deg (earlier settings: up to 4.4 cm and 33 deg) |
| Reference controller (simulated robot, 0.15 s lag, strict limits) | pose reference 0.83 cm / 0.45 deg, yaw reference 0.23 deg, built-in target heading pi/2 0.22 deg, coefficient changes while running accepted, invalid coefficients rejected (`tools/check_reference.py`) |
| Heading on the professor's robot model under real pose-source conditions (slow, late, freezing map pose) | tuned observer + PID: 0.31 deg mean, 100 % within 1 deg (normal); 0.51 deg, 90 % within 1 deg (stress); plain P law 4.7 / 8.1 deg (`results/real_robot/tuning_report.md`) |
| Symbolic controller | 345,888 cells; lap inside the +-8 cm tube on the exact model; in Gazebo 7.6-7.8 cm worst distance (little margin) |

Details: `results/lab_robot_results.md`, `results/symbolic/summary.md`.

## Limits

- Simulation only. Whether the real robot has the command deadband and the slow response is unknown until measured (`tools/measure_response.py`).
- The symbolic guarantee holds for the kinematic model it was built on; it fails if the robot's response is slower than about 0.3 s.
- Gazebo itself is not an ideal plant (measured: latency 0.18 s, coasting 0.39 s forward / 0.28 s turning, a drift of about 1.3 mm/s with a zero command).

## Documentation

- `docs/controllers.md`: control laws, strategies, parameters, tools.
- `docs/symbolic_control.md`: the symbolic model, the transition table, synthesis, results.
- `docs/network_setup.md`: Pi + laptop (WSL) network, firewall, DDS config, start order.
- `docs/real_robot.md`: real-robot interface and commissioning procedure.
