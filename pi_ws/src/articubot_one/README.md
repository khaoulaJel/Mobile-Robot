# Raspberry Pi hardware workspace

Source ~/CC_robot_ws/install/setup.bash and run:

    ros2 launch articubot_one hardware.launch.py

This single launch starts Arduino ros2_control, wheel odometry and joint states,
and LD19 scans. It uses CycloneDDS. It receives
/diff_cont/cmd_vel_unstamped. Driver paths default to /dev/ttyACM0 (Arduino,
set in laptop description), /dev/ttyUSB0 (LiDAR).

Run the companion CC_ros2_ws desktop.launch.py on the laptop. The Pi controller
manager waits for the laptop's transient-local /robot_description. 
The three application data streams are /diff_cont/odom, /joint_states,
and /scan. ROS also has infrastructure topics and controller
services. All URDF, TF, wheel-only EKF, SLAM, joystick, mux and RViz belong to the laptop.
Do not launch the old launch_robot_localization or joystick launch alongside
this setup. Use the same ROS_DOMAIN_ID on both machines, ROS_LOCALHOST_ONLY=0,
and synchronized clocks. A lost command stream times out after 0.5 seconds.

CC_ros2_ws.zip in the source root contains portable laptop sources and setup
scripts. Build it on the laptop; do not copy Raspberry Pi binaries.
