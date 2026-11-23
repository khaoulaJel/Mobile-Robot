# Controllers: laws, strategies, parameters, tools

## Interface

- **State** (parameter `state_topic`, type auto-detected, `state_type` to force): `nav_msgs/Odometry`, `geometry_msgs/PoseStamped`, `Pose` or
  `PoseWithCovarianceStamped`. The node logs the first `frame_id`, warns on a frame change and on a position jump above `state_jump_warn` (0.3 m),
  and exits with code 2 if the topic is missing, unsupported or ambiguous.
- **Command** (parameter `cmd_topic`): `geometry_msgs/Twist`, `linear.x` in m/s and `angular.z` in rad/s.
- **Safety:** speed and acceleration limits, stop within 0.5 s when the state stops, three zero commands on shutdown. These do not replace a
  hardware stop.
- Paths are given in the start frame (`relative_to_start: true`), so the odometry origin does not matter.

## Yaw controller

`w = kp * wrap(yaw_ref - yaw)`, saturated at `max_angular_speed`, acceleration-limited, zero inside `tolerance`. The reference is a parameter
(degrees if `use_degrees`) and can be changed while running: `ros2 param set /yaw_controller yaw_ref 180.0`. The error is wrapped, so the short way
round is always taken (170 deg to -170 deg is a 20 deg turn).

## Motion controller

The path is a flat list `[x, y, theta, ...]` or a generated circle (`path_shape: circle`, `circle_radius`, `circle_points`, `circle_direction`,
`circle_laps`). `strategy` selects:

| Strategy | Law | Behaviour |
| --- | --- | --- |
| `stop_turn` (default) | `v = kv * d * cos(alpha)`, `w = kw * alpha` (rotate in place first when the bearing error exceeds `turn_in_place_deg`); at the point: stop, then `w = k_yaw * (theta - yaw)` | reaches and aligns every point; slowest, lowest error |
| `polar` | `v = k_rho * rho * cos(alpha)`, `w = k_alpha * alpha + k_beta * beta` (stable for `k_rho > 0`, `k_beta < 0`, `k_alpha > k_rho`) | one smooth motion, hands over within `blend_radius`; rounds corners |
| `pure_pursuit` | look-ahead point on the path, curvature `2 sin(alpha) / lookahead`, speed capped by `sqrt(2 * decel * remaining)` | fast; best on smooth curves |

The control logic lives in `lab3_controllers/tracker.py` (`PathTracker`, `YawTracker`) and is shared by the ROS nodes and the model tools, so what is
tuned in the model is what runs on the robot.

## Compensation for a robot that drops small commands and responds slowly

All off by default (Gazebo behaviour is unchanged); enabled by `config/lab_robot.yaml` and `config/hardware.yaml`.

| Parameters | Effect |
| --- | --- |
| `diamond_v/w`, `deadband_v/w` | shape every command so the robot's own filter never drops it (scale into the feasible set `|v|/diamond_v + |w|/diamond_w <= 1`, lift out of the deadband by the smallest factor, curvature kept) |
| `lead_v`, `lead_w`, `vel_window`, `vel_filter` | predict the rest pose: pose + measured velocity x coasting time (velocity from finite differences of the state) |
| `rest_window` (> 0 enables), `settle_time`, `rest_timeout`, `rest_retries`, `pos_retries`, `pulse_w`, `pulse_extra`, `rest_corner_deg` | at corners and the last point: wait until the robot has stopped, check the true error, correct the heading with exactly the number of minimum-size pulses needed, re-approach if the position is off |
| `smooth_yaw_tolerance` | looser heading tolerance at smooth (non-corner) points, avoiding pointless in-place turns on curves |

Typical coasting times and the deadband of a given robot are measured with `tools/measure_response.py`, which prints the YAML lines.

## Tools (`src/lab3_controllers/tools/`)

| Tool | Purpose |
| --- | --- |
| `run_all.sh` | acceptance suite (Gazebo), strict numeric limits |
| `sim_compare.py`, `plant_models.py` | model of the robot and the strategies, no ROS; plant models: first-order lag, and an identified order-4 velocity model behind a command filter (`data/identified_plant.json`) |
| `shape_bench.py` | 14 path shapes, error measured against the ideal curve |
| `tune_lab_plant.py`, `robustness.py`, `lab_robot_report.py`, `yaw_bench.py` | random-search tuning with held-out shapes, robustness (noise, gain error, latency, waypoint spacing), report with pass/fail limits |
| `sensitivity.py`, `targeted_study.py` | sensitivity studies of the Gazebo settings |
| `measure_response.py` | real-robot latency, gain, coasting time and deadband (85 s, small commands, aborts on lost state) |
| `calibrate_odometry.py`, `check_state_source.py`, `trial_stats.py` | odometry calibration, state-topic inspection, repeatability statistics |
| `lab_robot_relay.py`, `lab_robot_gazebo_check.py` | put the command filter and identified plant in front of Gazebo to test the nodes against a deadband/slow robot |
| `simulink_replay/` | replay a command stream through the Simulink filter and state-space blocks and compare with the Python model (MATLAB) |
| `generate_symbolic.py`, `symbolic_sim.py` | build the symbolic model and controller; closed-loop simulation of it |
| `preview_lqr.py`, `tune_preview.py`, `lqr_compare.py` | experimental preview LQR (linear MPC) with the identified plant: better on smooth curves, sensitive to unmodelled delay; not used by the nodes |
| `evaluate.py`, `plot_results.py`, `check_results.py`, `check_circle.py`, `validate_yaw.py`, `fake_robot.py`, `test_safety.py` | log evaluation, plots, acceptance checks, safety tests |

## The deadband/slow robot model

`tools/plant_models.py` implements the command filter and the plant: commands with `|v| <= 0.04` and `|w| <= 0.2`, or outside
`|v|/0.3 + |w|/2.5 <= 1`, are replaced by zero; the velocity response is an identified order-4 discrete model (coasting about 0.61 s forward, 1.0 s turning).
`tools/simulink_replay/` checks this model against the original Simulink blocks (agreement to 1e-16). The identified data's origin is unverified,
so treat the model as a stress test until the real robot is measured.
