# Test day on the real robot

One command in WSL walks through everything:

```bash
bash src/lab3_controllers/tools/robot.sh guide
```

(On a computer where `robot.sh` is wrapped as `~/robot.sh`, use `bash ~/robot.sh guide`.) For each step: Enter = do it,
`s` = skip, `b` = back, a number = jump, `q` = quit (progress is remembered). Nothing moves without a second confirmation.

First time on a new computer or robot: build the workspace, create a login key for the Pi (`ssh-keygen`, `ssh-copy-id`), then
run `tools/laptop_setup.sh <PI_IP> 12` in WSL and `tools/pi_setup.sh <LAPTOP_IP> 12` on the Pi (our own ROS domain, 12).
The laptop programs (SLAM, command mux, RViz) are the course workspace in `~/ros2_lab_ws`.

| Step | What it does | Moves? |
| --- | --- | --- |
| 1 Connect | Pi address; rewrites both network settings if addresses changed | no |
| 2 Hardware | Arduino, LiDAR, Pi power warning; restarts the Pi driver | no |
| 3 Clocks | laptop and Pi within 0.15 s (the LiDAR map needs it) | no |
| 4 Status | driver running, exactly one robot on the domain | no |
| 5 LiDAR check | `/scan` and `/robot_pose`: rate, delay, noise at rest | no |
| 6 Heading pi/2 | six turns to 90 deg from different angles | turns |
| 7, 8 | settings A/B; plain P comparison (optional) | turns |
| 9 Identify | how far short pulses really move the robot, turning and driving | little |
| 10 Fit and re-tune | robot model from the measurements, then new path settings (about 6 min) | no |
| 11 Straight drives | forward and back | yes |
| 12, 13 Paths on odometry | square and 1 m circle with every controller; the LiDAR map, if running, is an independent check | yes |
| 14 Paths on the map | the controllers steer by wheel odometry placed in the LiDAR map | yes |
| 15 More shapes | triangle, rectangle, hexagon, star, circles, ellipse, figure-eight, sine, spiral, stadium | yes |
| 16 Results | `report.html` with tables and pictures, plus a zip backup | no |
| 17 Stop | stops all programs | no |

**Safety.** Every run: exactly one odometry source, no other controller running; abort if odometry stops, if a turn moves
15 cm, or if a path run is 25 cm from the path; the controller cannot outlive the tool; the robot's own 0.5 s command
timeout stops it if the link drops. The robot does **not** see obstacles: keep the floor space each test prints, and keep
the motor battery switch within reach. Ctrl+C stops the present run.

**What the numbers mean.** Errors "used" are measured by the position the controller steers by. With wheel odometry only,
they cannot show wheel slip or a wrong wheel-separation value. With the LiDAR on and steering by odometry, the map columns
are an independent estimate (its own noise is about a centimetre).

## Results so far

Real robot, wheel odometry, heading pi/2 (2026-11-28): `p_controller` **12 of 12 under 1 deg** (mean 0.125, worst 0.22 deg);
plain P without deadband compensation 0 of 6 (mean 2.6 deg). The newer direction-aware settings and all path results are
**simulation only** until run on the robot (`results/real_robot/paths_simulated.png`).
