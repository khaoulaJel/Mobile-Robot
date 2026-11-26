# The P controller for the real robot: design, fair comparison, results, limits

Code: `lab3_controllers/servo.py` (control, no ROS), `p_controller.py` (ROS node), `config/p_controller.yaml`.
**All accuracy figures are from simulation and from the ROS node driving a simulated robot. The controller has not yet been run on the real robot.**

## 1. Why a new design

The earlier designs depended on a model of the robot (the professor's Simulink model) that was never measured on our robot, and used only the slow map pose.
The new one is built to work on a robot whose deadband, gain, lag and delay are **unknown**.

| Principle | Implementation |
| --- | --- |
| Feed back measurements, not a model | pose and **measured turn rate and speed** from the wheel encoders (`/diff_cont/odom`) |
| Work in the professor's frame | `/robot_pose` (SLAM) only places the odometry in the map frame, matched **by time stamp**; a late map pose is compared with the odometry of its own moment |
| Far from the target | PD: `u = kp * (e - lead * rate)` (derivative on the measured rate) |
| Near the target | **pulse, wait until still, check**. A pulse moves the robot by gain x command x length whatever the lag and delay, so the pulse *length* is proportional to the error (a P law in time) |
| Learn the robot while running | **deadband** (smallest command that moves it), **coasting time** (distance covered after a stop / speed at the stop), **gain** (from each pulse), **pulse region** (widened when a continuous move overshoots) |
| Safety | zero command if the odometry is older than 0.5 s, before the first map pose is matched, before a reference, and at shutdown; joystick override through `/cmd_vel_tracker` |

## 2. The fair comparison

`tools/servo_study.py`: seven members of the P family, **each tuned with the same optimiser, budget and 80 training robots** (only its own coefficients free).
Each robot is random and unknown to the controller: deadband 0-0.30 rad/s (0-0.06 m/s), gain +-30 %, lag 0.15-1.0 s, command delay 0.03-0.35 s, encoder noise, odometry
drift, map noise and delay, SLAM jumps, network freezes. Test sets: new random robots; a **structurally different** robot (the professor's 4th-order identified model,
never used for tuning); and **outside the tuning range** (deadband 0.30-0.40, delay 0.35-0.6 s, lag 1.0-1.4 s, gain far off, freezes up to 6 s).

### Heading: mean final error in degrees (share within 2 deg)

| Structure | New robots | Different structure | Outside range |
| --- | --- | --- | --- |
| P | 2.20 (58 %) | 3.88 (34 %) | 8.52 (8 %) |
| PI | 1.16 (87 %) | 4.70 (40 %) | 8.14 (12 %) |
| PD | 1.69 (70 %) | 3.49 (55 %) | 7.06 (18 %) |
| PD + fixed minimum command | 1.21 (78 %) | 1.40 (73 %) | 3.47 (22 %) |
| PID + fixed minimum command | 0.66 (99 %) | 0.72 (98 %) | 2.39 (55 %) |
| PD + pulses | 0.32 (100 %) | 0.36 (100 %) | 6.45 (32 %) |
| **PD + pulses + learning** | **0.30 (100 %)** | **0.29 (100 %)** | 5.40 (65 %) -> fixed, see section 3 |

### Driven distance: mean final error in cm (share within 3 cm)

| Structure | New robots | Different structure | Outside range |
| --- | --- | --- | --- |
| P | 1.02 (98 %) | 4.85 (39 %) | 3.95 (64 %) |
| PI | 1.00 (98 %) | 1.02 (98 %) | 5.92 (49 %) |
| PD | 0.87 (98 %) | 3.26 (51 %) | 2.27 (77 %) |
| PD + fixed minimum command | 1.02 (96 %) | 2.45 (66 %) | 2.36 (62 %) |
| PID + fixed minimum command | 1.02 (98 %) | 4.09 (38 %) | 2.86 (64 %) |
| PD + pulses | 0.92 (98 %) | 0.78 (100 %) | 4.23 (38 %) |
| **PD + pulses + learning** | **0.77 (100 %)** | **0.70 (99 %)** | **1.22 (91 %)** |

**Choice: PD + pulses + learning on both axes.** It has the lowest mean error on every distance set and on the two heading sets inside the tuning range.

## 3. Three flaws found by tracing bad runs, and their fixes

1. **Limit cycle.** After overshoots the learned minimum command fell far below the real deadband. Fix: never below a command the robot was seen to ignore, and the pulse approach.
2. **Wrong coasting time.** A sluggish robot kept turning about 50 degrees after the stop. Fix: measure the coasting time after every move.
3. **Repeated overshoot outside the range.** With long delays every continuous move overshot by about 20 degrees, then started another one. Fix: a move that overshoots widens the pulse region.

Two earlier explanations (false "ignored" verdicts; rate noise hiding stalls) were tested and **did not** explain the bad runs. The stall check was still changed to use the
position instead of the noisy rate.

## 4. Final results (shipped code, fresh robots from seeds used for nothing before)

| Mean error (95th percentile, worst; share within the limit) | New robots | Different structure | Outside range |
| --- | --- | --- | --- |
| **Heading, this controller** (deg) | **0.28** (0.74, 1.7; 100 % <= 2) | **0.31** (0.75, 1.1; 100 %) | **1.33** (4.7, 17.1; 85 %) |
| Heading, previous preset | 3.51 (16.3, 20.2; 63 %) | 2.42 (11.6, 18.2; 71 %) | 11.08 (20.5, 27.6; 6 %) |
| Heading, plain P | 2.92 (8.4, 11.9; 53 %) | 3.70 (10.5, 12.0; 41 %) | 11.93 (40.2, 75.0; 15 %) |
| **Distance, this controller** (cm) | **0.70** (1.85, 3.2; 99 % <= 3) | **0.70** (1.94, 2.7; 100 %) | **1.28** (5.9, 7.7; 90 %) |

Go to a pose (100 robots per set): new 0.66 cm and 0.33 deg; different structure 0.79 cm and 0.42 deg (both 100 % within 4 cm and 4 deg); outside range 0.85 cm and
2.22 deg (93 % within 4 cm and 4 deg, 83 % finished within 60 s).

The previous preset did not use the wheel odometry, so part of its gap comes from having less information. The plain P row uses the same measurements as the new controller.

**ROS node check** (`tools/check_p_controller.py`, real topics, simulated robot with deadband 0.26, gain 1.2, 0.2 s delay, odometry in a rotated frame): 8 of 8.
Headings 0.41 and 0.27 deg; drives 0.2 and 0.0 cm; pose 0.1 cm and 0.72 deg; live coefficient accepted and an invalid one refused; zero command 0.43 s after the odometry stops.

## 5. Limits

* **Not yet run on the real robot.** The simulator is ours; a real robot can behave in ways it does not model (wheel slip, battery voltage, asymmetric motors, caster drag).
* Learned values start again at each node start (the first move teaches the most).
* The pulse approach makes the last degrees slower than a continuous law: a turn typically settles in about 7-10 s in simulation.
* Far outside the tuning range a few runs still miss (heading worst 17 deg).
* The odometry and the map pose must be stamped with the same clock (true for the professor's `ros2_lab_ws`, where SLAM stamps with the odometry time).
* The map pose needs the LiDAR. Without it set `map_topic: ''` (odometry frame only).

## 6. Robot day

```bash
ros2 launch lab3_controllers p_controller.launch.py                     # idle until a reference arrives
ros2 topic pub --once /reference_yaw std_msgs/msg/Float64 "{data: 1.5708}"
ros2 topic pub --once /reference_x   std_msgs/msg/Float64 "{data: 0.3}"
```
Capture for each run: `-p log_csv:=<file>` (every control step: measured state, command, reference, learned deadband / coasting time / gain), the `ARRIVED:` log line,
`ros2 param dump /p_controller > coefficients_used.yaml`, and an independent measurement of the final pose (tape, protractor). `tools/log_run.py` records odometry
and map pose together to explain unexpected position changes.

## 7. Reproduce

```bash
python3 tools/servo_study.py                      # heading study
python3 tools/servo_study.py --axis distance      # distance study
PYTHONPATH=. python3 -m pytest -q test            # includes the accuracy limits (test_servo_performance.py)
python3 tools/check_p_controller.py               # ROS node check (after colcon build; use an unused ROS_DOMAIN_ID)
```
Results: `servo_study.json`, `servo_study_distance.json`. The `params` in those files are the tuned values from the study; the shipped defaults are the same values rounded,
plus the learning rules of section 3.
