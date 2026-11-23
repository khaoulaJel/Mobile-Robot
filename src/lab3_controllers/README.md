# lab3_controllers

Two ROS 2 (Humble) nodes. State: `/odom` (`nav_msgs/Odometry`). Command:
`/diff_cont/cmd_vel_unstamped` (`geometry_msgs/Twist`). Both clip speeds, limit
acceleration, and stop if `/odom` goes stale or on Ctrl+C.

## Nodes

**`yaw_controller`**: `w = kp * wrap(yaw_ref - yaw)`, saturated, zero inside a tolerance band.

**`motion_controller`** follows a list of `(x, y, theta)` points in order. `strategy` selects:

| `strategy` | Idea | Use when |
| --- | --- | --- |
| `stop_turn` (**default**) | At every point: P law `v = kv·d·cosα`, `w = kw·α` (rotate in place first if the bearing error is > 35 deg), stop, rotate to the point's heading `w = k_yaw·(θ - yaw)`, next point | the lab wants each point reached and aligned exactly (safest reading of the lab) |
| `polar` | Lyapunov polar-coordinate pose law `v = k_rho·ρ·cosα`, `w = k_alpha·α + k_beta·β`; hands over to the next point within `blend_radius` (3 cm default; larger = faster, rounder corners) | speed matters, rounded corners are fine |
| `pure_pursuit` | Look-ahead point on the polyline through the points, curvature `2 sinα / Ld`, speed limited by `w_max / curvature` and by `sqrt(2·decel·remaining)` near the end | curves (circle), smooth continuous paths |

Path: `path_shape: waypoints` (flat list `[x, y, theta, ...]`, see `config/controllers.yaml`) or
`path_shape: circle` (`circle_radius`, `circle_points`, `circle_direction`, `circle_laps`),
always starting from the robot's pose when `relative_to_start: true`.

## Any path shape

The motion controller takes an arbitrary list of `(x, y, theta)` points (or the built-in circle), so the path
shape is not hard-coded. `tools/shape_bench.py` checks 14 shapes (square, rectangle, triangle, hexagon, 36-degree
star, three circles, ellipse, figure-eight, sine wave, spiral, stadium) and measures the distance of the TRUE
trajectory from the IDEAL curve, not only from the waypoints. Give the path as dense waypoints (<= 0.1 m apart on
curves): with sparse points the straight chords between them are the error (0.2 m spacing: 3.7 cm; 0.05 m: 0.9 cm).

## Professor's robot: command filter and slow response (`config/lab_robot.yaml`)

The Simulink files define a command filter (feasible diamond `|v|/0.3 + |w|/2.5 <= 1`, deadband
`|v| <= 0.04 and |w| <= 0.2`, both replaced by zero) and a slow identified velocity model (coasting time 0.61 s
for v, 1.0 s for w). The competition settings stall or limit-cycle on it. Optional features (all off by default,
so Gazebo behaviour is unchanged) handle this; the control logic is in `lab3_controllers/tracker.py` and is shared
by the ROS nodes and the model tools:

| Parameter | Effect |
| --- | --- |
| `diamond_v/w`, `deadband_v/w` | commands are shaped so the robot's filter never drops them (scaled into the diamond, lifted out of the deadband, curvature kept) |
| `lead_v`, `lead_w`, `vel_window`, `vel_filter` | predict the rest pose = pose + measured velocity x coasting time (lead compensation) |
| `rest_window` (> 0 enables) | at corners and the last point: wait until the robot has really stopped, check the true error, correct the heading with exact minimum-size pulses, re-approach if the position is off |
| `smooth_yaw_tolerance` | no pointless in-place turns at smooth curve points |

    ros2 launch lab3_controllers motion.launch.py config:=lab_robot.yaml use_sim_time:=false
    ros2 launch lab3_controllers yaw.launch.py    config:=lab_robot.yaml use_sim_time:=false

Results on the model of that robot (not the real robot): `results/lab_robot_results.md` (repository root). Tools:
`tools/plant_models.py` (identified plant + filter), `shape_bench.py`, `tune_lab_plant.py` (random search with a
held-out validation set), `robustness.py` (noise, gain error, latency, waypoint spacing), `yaw_bench.py`,
`lab_robot_report.py` (strict pass/fail + report), `simulink_replay/` (replays a command stream through the
professor's own Simulink blocks: differences 1e-16).

## Symbolic controller (lab note, task 3)

`symbolic_controller` drives a circle of 1 m diameter (tangent start, counter-clockwise) with a controller synthesized from
a finite abstraction of the kinematic robot: cells over (x, y, heading), 13 inputs held for 1 s, a transition table listing
ALL successor cells (sound growth bound, tested), reach-and-stay-safe synthesis. Guarantee: the robot stays within 8 cm of
the circle and reaches each of 12 goal arcs within 34 steps, **for the model it was built on**. It fails when the robot's
response is slower than about 0.3 s. A neighbour-cell fallback (`robust_lookup`, default on) helps with noise and is
logged as outside the guarantee.

    ros2 launch lab3_controllers symbolic.launch.py                  # Gazebo running, one controller at a time
    python3 tools/generate_symbolic.py [--full-table table.npz]      # rebuild model + controller (~90 s)
    python3 tools/symbolic_sim.py --plant lag:0.15 --noise 0.002 --seeds 10 --robust

See `docs/symbolic_control.md` for the design and results.

## Run (simulation)

    source /opt/ros/humble/setup.bash
    cd <repository root> && colcon build --symlink-install && source install/setup.bash
    ros2 launch lab3_gazebo gazebo.launch.py                 # terminal 1 (course package, see the root README)

    # terminal 2: ONE controller at a time (all publish to the same command topic)
    ros2 launch lab3_controllers yaw.launch.py               # then: ros2 param set /yaw_controller yaw_ref 180.0   (degrees)
    ros2 launch lab3_controllers motion.launch.py            # 0.5 m square, stop_turn
    ros2 run lab3_controllers motion_controller --ros-args -p use_sim_time:=true \
        -p strategy:=pure_pursuit -p path_shape:=circle -p circle_radius:=0.5 -p relative_to_start:=true   # circle of 1 m diameter
    ros2 launch lab3_controllers symbolic.launch.py

Add `-p log_csv:=/tmp/run.csv` to record a log; `tools/evaluate.py` turns logs into metrics and plots.

## Real robot

    ros2 launch lab3_controllers motion.launch.py config:=hardware.yaml use_sim_time:=false

`config/hardware.yaml` is a conservative first-run preset (state `/diff_cont/odom`, command `/cmd_vel_tracker`, 0.1 m/s). Measure the robot's
response first with `tools/measure_response.py` and put the printed `lead_*` / `deadband_*` values into the config. The full procedure is in
`docs/real_robot.md`. The launch arguments `state_topic:=` and `cmd_topic:=` override the config file only when given.

## How the gains were chosen

Gains were searched in a pure-Python model of the robot (`tools/sim_compare.py`: acceleration limits plus a velocity lag, checked for lags of
0.1 to 0.5 s and with odometry noise, `tools/sensitivity.py`) and then confirmed in Gazebo. The `stop_turn` defaults are `kv 0.8`, `kw 3`,
`k_yaw 1.0`, `turn_in_place_deg 35`, tolerances 1 cm / 0.01 rad; `kv` was lowered from 1.8 because a higher gain coasts past the goal when the motors are
sluggish. The `lab_robot.yaml` settings come from a random search on six path shapes, checked on eight other shapes (`tools/tune_lab_plant.py`,
`tools/robustness.py`). Details: `docs/controllers.md`.

## Tests

    cd src/lab3_controllers && PYTHONPATH=. python3 -m pytest -q test      # unit and regression tests (no ROS graph needed)
    bash tools/run_all.sh                                                  # acceptance suite in Gazebo, strict numeric limits

## References

- Polar-coordinate posture regulation (Siegwart / Aicardi): Stanford AA274 notes, arXiv 2005.06181.
- Pure pursuit with look-ahead; MathWorks "Path Following for a Differential Drive Robot".
- Abstraction-based (symbolic) control, e.g. the SCOTS tool; the approach used here is described in `docs/symbolic_control.md`.
