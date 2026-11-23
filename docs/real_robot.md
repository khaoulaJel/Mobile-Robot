# Real-robot commissioning and validation guide

Procedure to take the controllers from Gazebo to the physical robot in a controlled way.
Everything in this repository is **validated in simulation only**; this file is the commissioning procedure for the real robot.

**Principle:** validate hardware and ROS communication first, calibrate odometry second, tune controllers last.
Otherwise a wiring, sign or calibration problem gets mistaken for a controller problem. Odometry is *not*
ground truth: every number we report must also be checked against a tape measure / floor marks.

The controllers' safety features: speed and acceleration limits, immediate stop when `/odom` is stale
(> 0.5 s), repeated zero command on shutdown. They do **not** replace a hardware watchdog or an
emergency stop: keep one reachable and have someone ready to cut motor power. Nothing below has been
run on hardware yet.

## Robot day: the short version (everything else in this guide is the detail)

Already done in simulation: all code, tests and configs. What needs the robot: the real numbers and the first motion.

```bash
# 0. hardware: motor battery connected and switched on (the driver needs it; the Arduino alone is powered by USB),
#    USB cable Pi <-> Arduino secured, robot on the floor with 1.5 m x 1.5 m free, joystick in hand (it overrides everything)
# 1. start the robot side (docs/network_setup.md):  Pi      ros2 launch articubot_one hardware.launch.py lidar:=false
#                                                   laptop  ./run_desktop.sh
# 2. from the laptop, in this order:
bash src/lab3_controllers/tools/robot_session.sh check        # odometry rate, no duplicate nodes, mux connected
bash src/lab3_controllers/tools/robot_session.sh measure      # real deadband / coasting; copy the printed lines into your coefficients file
COEFFS=/path/to/the_numbers.yaml bash src/lab3_controllers/tools/robot_session.sh reference     # leave running
bash src/lab3_controllers/tools/robot_session.sh send-yaw 0.35          # first test: a 20 degree turn
bash src/lab3_controllers/tools/robot_session.sh send-pose 0.3 0 0      # then a short straight move
bash src/lab3_controllers/tools/robot_session.sh set kv 0.6             # tune a coefficient while it runs
bash src/lab3_controllers/tools/robot_session.sh stop
# square / circle: ros2 launch lab3_controllers motion.launch.py config:=hardware.yaml use_sim_time:=false
```

If nothing moves: motor battery first (without it the encoders read 0 and the wheels stay still), then `ros2 topic hz /diff_cont/odom`, then the Pi's IP
(a DHCP change breaks SSH and the DDS peer file; reserve the Pi's address in the router).

## The robot's software (professor's Pi workspace `robot_lab_ws_wheel_odometry_only.zip`, received 2026-11-23)

Read from the files (`articubot_one`, `diffdrive_arduino`, and `CC_ros2_ws.zip` inside it). **Confirmed by the files, not yet by the robot.**

```
 our controller --> /cmd_vel_tracker --+
 joystick (laptop) -> /cmd_vel_joy ----+--> twist_mux (laptop) --> /diff_cont/cmd_vel_unstamped --> Pi
 Simulink ---------> /cmd_vel ---------+     priorities 100 / 50 / 10, each input times out after 0.5 s
 Pi: ros2_control + diff_drive_controller (cmd timeout 0.5 s) --serial 57600--> Arduino (wheel-speed PID, encoders)
 Pi --> /diff_cont/odom (wheel odometry, 30 Hz) --> laptop wheel-only EKF --> /odometry/filtered, TF odom->base_link
```

| Fact | Source | What it means for us |
| --- | --- | --- |
| Pi publishes `/diff_cont/odom` (`nav_msgs/Odometry`, 30 Hz, wheel odometry only, no TF), `/joint_states`, `/scan`; receives `/diff_cont/cmd_vel_unstamped` (`Twist`, unstamped) | `hardware.launch.py`, README | **state topic = `/diff_cont/odom`**: local on the Pi, continuous, no extra hop |
| the laptop EKF only smooths the same wheel velocities into `/odometry/filtered`; `/robot_pose` is the SLAM map pose | `ekf.yaml`, `tf_to_pose.py` | do not use `/odometry/filtered` first (extra lag and network hop) nor `/robot_pose` (can jump) |
| `twist_mux` inputs: `/cmd_vel_joy` priority 100, `/cmd_vel` 50 (Simulink), **`/cmd_vel_tracker` 10**; output -> `/diff_cont/cmd_vel_unstamped` | `mux.yaml`, `desktop.launch.py` | **command topic = `/cmd_vel_tracker`**: the joystick overrides our controller at once; if our node dies the mux and the Pi each stop within 0.5 s |
| wheel radius 0.034 m, separation 0.177046 m, 30 Hz control, odometry velocity window of 3 samples, no speed or acceleration limits in the controller | `hardware_controllers.yaml` | same numbers as the simulation; **our own limits are the only ones**: start at 0.1 m/s |
| 1980 encoder counts per revolution, 30 Hz loop, 57600 baud; a wheel command is rounded to whole counts per loop | `robot.hardware.urdf.xacro`, `diffbot_system.cpp` | one count per loop = 3.2 mm/s per wheel: the command resolution is fine, far below any deadband |
| the wheel-speed PID and the motors live in the Arduino firmware, which is NOT in the files | `arduino_comms.hpp` (only the serial protocol `m l r`, `e`, `u p:d:i:o`) | the real deadband and response are **unknown until measured** |
| the validation notes say the earlier tests were on the **IMU-enabled** workspace; this one removed the IMU | `CC_ros2_ws/VALIDATION.md` (in the course Pi workspace) | the supplied identification data may come from a different configuration: re-measure |
| teleop scales: 0.18 m/s (turbo 0.25), 0.75 rad/s (turbo 1.0) | `joy.yaml` | the robot is used up to about 0.25 m/s and 1 rad/s |

**Why the identified plant may not describe the raw odometry.** The Lab 2 identification feedback was `/odometry/filtered`, a smoothed copy
of `/diff_cont/odom` velocities. If the dataset came from there, the 0.6 s / 1.0 s coasting times include the EKF's smoothing
and the real robot seen through `/diff_cont/odom` may be faster. That is a hypothesis; `tools/measure_response.py` settles it
in 85 s per state topic (run it on `/diff_cont/odom` and on `/odometry/filtered` and compare).

**Where to run our nodes:** on the Pi (the state topic is local; python only, numpy for the symbolic controller). The command still
crosses the laptop's mux (two LAN hops): our tests with +0.1 s and +0.2 s extra latency kept `stop_turn` within spec.
Use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, the same `ROS_DOMAIN_ID` on both machines, and run **one** controller at a time.

**Updated first 30 minutes on the robot day** (everything on the Pi, as the professor said):

```bash
# 0. Pi:    cd ~/robot_lab_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
#           (our package: copy lab3/src/lab3_controllers into ~/robot_lab_ws/src and build it the same way)
#    Laptop: ./run_desktop.sh        Pi: ros2 launch articubot_one hardware.launch.py
# 1. topics alive?        ros2 topic list -t;  ros2 topic hz /diff_cont/odom      # expect about 30 Hz
# 2. state topic sane?    python3 src/lab3_controllers/tools/check_state_source.py --topic /diff_cont/odom --rotate --forward
# 3. real response:       python3 src/lab3_controllers/tools/measure_response.py --state-topic /diff_cont/odom --cmd-topic /cmd_vel_tracker --out resp_raw.json
#                         python3 src/lab3_controllers/tools/measure_response.py --state-topic /odometry/filtered --cmd-topic /cmd_vel_tracker --out resp_ekf.json
# 4. put the printed lead_v / lead_w / deadband_v / deadband_w into config/hardware.yaml, then:
ros2 launch lab3_controllers yaw.launch.py    config:=hardware.yaml use_sim_time:=false
ros2 launch lab3_controllers motion.launch.py config:=hardware.yaml use_sim_time:=false
```
`hardware.yaml` already sets `/diff_cont/odom` and `/cmd_vel_tracker`, and conservative limits. Keep the physical stop in reach;
the joystick enable button (6) takes over at once if the controller misbehaves.

## What the professor's Simulink files say about the robot (read this first)

His template `robot_mimo_identification.slx` contains a command filter, `input_filter`, between the joystick and
the robot, and the supplied dataset identifies a slow velocity response. Both change how a controller must behave
and **neither exists in Gazebo**:

| Fact (from the files) | Meaning for our controllers |
| --- | --- |
| Commands outside `\|v\|/0.3 + \|w\|/2.5 <= 1` are replaced by 0 | never ask for e.g. 0.2 m/s and 1 rad/s together |
| Commands with `\|v\| <= 0.04` **and** `\|w\| <= 0.2` are replaced by 0 | a P law that slows down near the goal is silently switched off: the robot stops 3-5 cm short |
| Identified response: coasting after a command stops is equivalent to 0.61 s (v) and 1.0 s (w), 90 % rise 1 s / 2 s | the robot keeps moving long after the command ends: the old P gains overshoot and limit-cycle |
| 0.1 m/s forward produces about -0.019 rad/s of heading drift (cross-coupling) | feedback on heading is needed even on straight legs |

`config/lab_robot.yaml` is built for exactly this (details and numbers: `results/lab_robot_results.md`). Use it
instead of `controllers.yaml` when the robot has this filter:
`ros2 launch lab3_controllers motion.launch.py config:=lab_robot.yaml use_sim_time:=false`.

**Not verified:** whether the *real* robot runs this filter and has this response. The dataset's provenance is
unconfirmed. On the day, check it directly: publish `v = 0.03` (inside the deadband) and see whether the robot
moves; publish `v = 0.3, w = 2.0` (outside the diamond) and see whether it stops. If the real robot has no such
filter and is fast, `controllers.yaml` or `robust.yaml` is the better preset. Re-measure the coasting times
(command a velocity, drop it to 0, integrate the odometry) and put them into `lead_v` / `lead_w`.

**Symbolic controller on the robot:** its guarantee needs a robot that follows speed commands within about 0.3 s (it was
built on the kinematic model). Measure the response first (above). If it is slower, use `lab_robot.yaml` with the P controllers.
Run it with `ros2 launch lab3_controllers symbolic.launch.py use_sim_time:=false state_topic:=<topic>` on an empty floor of at
least 1.5 m x 1.5 m, with the emergency stop reachable: it drives a full lap of 1 m diameter.

## Order of work

1. Mechanical inspection
2. ROS interface discovery
3. Motor direction and encoder check (on blocks)
4. Odometry calibration
5. Yaw controller commissioning
6. Motion controller commissioning
7. Repeated trials and analysis
8. Final demo and report package

**Default tolerances are Gazebo-tight** (position 1 cm, heading 0.01 rad, yaw controller 0.005 rad). A real
robot with motor deadband and encoder noise may hover around such a band and never finish. If so, relax them
first: `-p pos_tolerance:=0.02 -p yaw_tolerance:=0.03` (yaw controller `-p tolerance:=0.02`) and set
`min_linear_speed` / `min_angular_speed`; raise `lookahead` to 0.2 m if pure pursuit oscillates. Re-run
`tools/run_all.sh` after any code change; its safety tests (stale odometry, shutdown zeros) are the ones that matter on hardware.

## 1. Mechanical inspection

- Both wheels spin freely, aligned, touching the floor; caster not binding.
- Measure wheel diameter (expect 0.068 m) and axle-to-axle distance (expect 0.177046 m). These are the
  URDF's nominal values, not verified for the real robot.
- Cables clear of wheels, battery secure, emergency stop / power cut within reach.

## 2. ROS interface discovery

**What the professor's Pi workspace says (see the section at the top):** command `/diff_cont/cmd_vel_unstamped` (`Twist`),
reached through the laptop mux, so our controllers publish on **`/cmd_vel_tracker`**; the Pi publishes **`/diff_cont/odom`**
(`nav_msgs/Odometry`, wheel odometry, 30 Hz), which is the state topic to use first. `/odometry/filtered` (laptop EKF of the same
wheel velocities, extra lag) and `/robot_pose` (SLAM map pose, can jump) are the alternatives, not the first choice. Gazebo's
`/odom` does not exist on that robot. Our nodes auto-detect the state message type (`state_type: auto`): Odometry, PoseStamped,
Pose or PoseWithCovarianceStamped. With launch files (the arguments default to the config file's values):
`ros2 launch lab3_controllers motion.launch.py config:=hardware.yaml use_sim_time:=false`.
**Do not choose the state topic only because our nodes accept its message type.** Inspect it first with
`tools/check_state_source.py` (robot stationary, wheels on blocks or a clear floor):

```bash
python3 src/lab3_controllers/tools/check_state_source.py --topic /diff_cont/odom --seconds 10
python3 src/lab3_controllers/tools/check_state_source.py --topic /diff_cont/odom --rotate --forward   # moves the robot slowly (3 s turn, 2 s drive)
python3 src/lab3_controllers/tools/check_state_source.py --topic /odometry/filtered --seconds 10      # then compare (EKF lag, rate)
```

It reports the message type, `header.frame_id`, rate, largest position step between messages (jumps = localization
corrections), noise at rest, header latency, and, with `--rotate` / `--forward`, whether +`angular.z` increases yaw
(counter-clockwise positive, as the controllers assume) and whether +`x` moves the robot along its heading. Any FAIL must be
understood before launching a controller. Prefer a topic whose frame is continuous (odometry); a map-frame pose that jumps will
make the path controller misbehave. The nodes themselves log the first frame id, warn on a frame change and warn on a position
jump above `state_jump_warn` (0.3 m), but only the inspector tells you whether the frame and yaw convention are the expected ones.

If the state topic is missing, has an unsupported type, or has several supported types (ambiguous), the controllers do not
start silently: they print the reason and exit with code 2 (`state_wait_timeout`, default 60 s, applies to a topic that
has not appeared yet; force the type with `-p state_type:=...` when ambiguous).

Ask the instructor which feedback source the lab expects. Check that the pose is in a fixed frame and that yaw increases
counter-clockwise; our paths are relative to the start pose (`relative_to_start: true`), so the frame origin does not matter.



```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t
ros2 node list
ros2 topic info /odom --verbose            # type, publisher count, QoS
ros2 topic info /diff_cont/cmd_vel_unstamped --verbose
ros2 topic hz /odom                        # expect a steady rate (10-50 Hz)
ros2 topic echo /odom --once               # frame ids, plausible values
```

Check: exactly one odometry publisher; correct message types (`nav_msgs/Odometry`, `geometry_msgs/Twist`);
no other node publishing velocity commands. If names differ from simulation, override at launch:
`--ros-args -p state_topic:=<odom topic> -p cmd_topic:=<cmd topic>` (message type is auto-detected, or force it with
`-p state_type:=odometry|pose_stamped|pose|pose_cov`).

## 3. Motor and encoder check (robot on blocks, wheels off the ground)

Do **not** run our controllers yet. Publish tiny commands by hand:

```bash
ros2 topic pub --rate 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
ros2 topic pub --rate 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{angular: {z: 0.3}}'
ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{}'     # stop
```

Verify: `+x` drives both wheels forward; `+z` turns counter-clockwise (left wheel back, right forward);
the odometry's x increases when driving forward and yaw increases when turning CCW. Direction or sign
errors are fixed in the driver / configuration, never by changing controller gains.

## 4. Odometry calibration (use `tools/calibrate_odometry.py`)

Do this on the floor, in a clear area, with a tape measure and floor marks. Wheel radius first,
separation second (do not estimate both from one test).

```bash
cd ~/lab3/src/lab3_controllers/tools
# A. straight, 1 m at 0.1 m/s. Mark the start; measure the real distance after it stops. 3-5 times, forward and --reverse
python3 calibrate_odometry.py straight --distance 1.0 --speed 0.1
python3 calibrate_odometry.py calc-radius --odom <odom value> --real <tape value>
# B. rotation, 360 deg both ways. Mark the robot and floor; judge the real angle (e.g. photos / protractor lines)
python3 calibrate_odometry.py rotate --angle 360
python3 calibrate_odometry.py rotate --angle 360 --cw
python3 calibrate_odometry.py calc-separation --odom <odom deg> --real <real deg>
```

Formulas (first order): `r_new = r * D_real / D_odom`, `b_new = b * A_odom / A_real`.
Apply them in the robot's driver / odometry configuration. Then repeat A and B to confirm the
correction worked. Large differences between forward and reverse, or CW and CCW, indicate wheel
asymmetry, friction or encoder problems rather than a pure scale error.

C. Bidirectional square (UMBmark-style, simplified): run the same square CCW and CW (3 times each,
`stop_turn`, slow), measure the real closing error each time. If the errors flip sign between CW and
CCW, the cause is systematic (unequal wheels / separation) and is a calibration problem.

## 5. Yaw controller commissioning

Start with a safe, low-gain configuration and the robot on the floor, clear area:

```bash
ros2 run lab3_controllers yaw_controller --ros-args -p yaw_ref:=0.0 -p kp:=0.8 \
    -p max_angular_speed:=0.4 -p max_angular_accel:=1.5
ros2 param set /yaw_controller yaw_ref 45.0       # degrees if use_degrees:=true (default in the yaml)
```

Note: with the shipped yaml (`yaw_ref: 90`) the robot starts turning to 90 deg the moment the node
starts. Set the reference you intend before launching. `use_sim_time` must be `false` on hardware
(`yaw.launch.py use_sim_time:=false`).

Steps: +90, -90, +170, -170, 0 deg, in both directions, 3 times each. Measure settling time, overshoot,
steady-state error and CW/CCW asymmetry. Raise `kp` and the speed limits gradually. If it stalls short
of the target, check motor deadband first (`min_angular_speed`), not just gain. Check the final real
heading against a protractor/marks.

## 6. Motion controller commissioning

Use `stop_turn` (default; reaches and aligns every point), a small square first:

```bash
ros2 launch lab3_controllers motion.launch.py use_sim_time:=false
ros2 run lab3_controllers motion_controller --ros-args -p max_linear_speed:=0.1 \
    -p max_angular_speed:=0.5 -p relative_to_start:=true -p log_csv:=/tmp/hw_square_01.csv
```

Increase speed only after a clean run. `polar` / `pure_pursuit` and the circle are secondary
experiments (a circle needs enough floor; R = 0.4 m needs about 1 m x 1 m). Keep the same waypoints as in
Gazebo so the results are comparable. Record while testing:

```bash
ros2 bag record -o results/hardware_square_01 /odom /tf /tf_static /diff_cont/cmd_vel_unstamped
```

(only topics that exist).

## 7. Repeatability and analysis

```bash
python3 src/lab3_controllers/tools/trial_stats.py /tmp/hw_square_01.csv /tmp/hw_square_02.csv /tmp/hw_square_03.csv
python3 src/lab3_controllers/tools/evaluate.py <dir with square_<strategy>.csv>   # plots in the start frame
```

`trial_stats.py` reports duration, final position error and final heading error per run and the mean and
standard deviation across runs (relative to odometry). Add the **measured** final pose (tape and
marks) alongside; that is the real accuracy.

Data sheet to fill for each trial:

| trial | path/strategy | speed limit | odom final (x, y, yaw) | measured final (x, y, yaw) | real error [cm / deg] | notes |
| --- | --- | --- | --- | --- | --- | --- |

## 8. What to change when something is wrong

| Observed | Investigate first |
| --- | --- |
| Travels too far / too little | wheel radius calibration |
| Systematically under/over-rotates | wheel separation, motor response |
| CW and CCW results differ | wheel asymmetry, friction, encoder signs |
| Overshoot after turning | angular gain, acceleration limit |
| Oscillates around the reference | gain too high, odometry noise or latency |
| Does not move at small commands | motor deadband (`min_linear_speed`, `min_angular_speed`) |
| Inconsistent behaviour | odometry rate, latency, state validity |

## 9. Before the session (no robot needed)

- Copy this repository to the Raspberry Pi and work on a copy (git makes it easy to restore the original).
- Prepare a table to record, per trial: odometry final pose, measured final pose, real error.
- Equipment: tape measure, masking tape for start marks, a protractor or floor marks for rotation, a way to stop motors immediately (emergency stop / power switch), spare battery.
- Know the stop commands: `Ctrl+C` in the controller terminal, then `ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{}'`.
- Day sequence: assemble and inspect, run `check_state_source.py` on the candidate state topics, `ros2 topic list -t`, identify the topics, wheel/encoder directions, calibrate radius then separation, re-check, robust preset at 0.1 m/s, then the competition preset.

## 10. Checklist

- [ ] Mechanical inspection and emergency stop
- [ ] Real topics, types, rates, QoS, one publisher each
- [ ] Wheel directions and encoder signs
- [ ] Odometry calibration (forward, reverse, CW, CCW), then confirmed
- [ ] Yaw controller: 5 references, both directions, 3 repeats
- [ ] Motion controller: square, `stop_turn`, 3+ repeats
- [ ] Circle (if space allows) with `pure_pursuit`
- [ ] Stale-state and interruption behaviour tried safely
- [ ] Logs, rosbags, plots and measured data saved in `results/`
- [ ] Final clean demonstration

## 11. Out of scope (by design)

PID/MPC (the lab asks for P controllers), obstacle avoidance, and further Gazebo tuning: the
remaining uncertainty is hardware, not simulation. Simulink is only needed if the instructor requires it.
