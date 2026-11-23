# CC desktop workspace (ROS 2 Humble)
Run install_dependencies.sh, build.sh, then run_desktop.sh on the laptop.
Stop all old joystick, SLAM, EKF and robot launches first. Use matching ROS_DOMAIN_ID (default 0), ROS_LOCALHOST_ONLY=0 and CycloneDDS on both machines. Connect the same LAN and synchronize clocks (NTP). Allow DDS multicast through the firewall.

Pi: source ~/CC_robot_ws/install/setup.bash; ros2 launch articubot_one hardware.launch.py
Laptop: ./run_desktop.sh
Either launch may start first. The Pi waits for the laptop robot_description before activating the Arduino controllers. device:=/dev/ttyACM0 refers to the device ON THE PI.

Pi publishes /diff_cont/odom, /joint_states, /scan and receives /diff_cont/cmd_vel_unstamped. ROS infrastructure also publishes discovery, parameter events, rosout and controller services; exactly three total ROS topics is not realistic.
Laptop owns robot_description, all TF, wheel-only EKF, SLAM, mux, joystick and RViz. EKF uses only encoder-derived wheel forward and angular velocities from /diff_cont/odom; it does not subscribe to any IMU. No wheel pose is fused separately. EKF owns odom→base_link; SLAM owns map→odom. URDF supplies base_link→sensor/wheel frames. /robot_pose is global map pose; /odometry/filtered is local odometry.
Wheel encoder integration can accumulate position and yaw drift; SLAM can correct global map pose. Wheel dimensions and covariance require real measurements for best accuracy.
Pi has a 0.5-second command timeout. Network latency/jitter can affect remote EKF and commands. Monitor it before driving.

MATLAB requires a licensed laptop installation with ROS Toolbox and Simulink. ./run_desktop.sh matlab:=true opens MATLAB and the supplied model builder. It does not start simulation or command robot movement. Review ROS Network settings (domain and CycloneDDS), then Run. MATLAB cannot be verified on the Pi.

If libdiagnostic_updater.so is missing, run install_dependencies.sh again. It explicitly reinstalls ros-humble-diagnostic-updater. The runner checks binary dependencies before launching to avoid a shutdown cascade. It explicitly sources Humble and then this workspace local_setup.bash.

Three original STL files contain zero triangles: base_footprint, caster_wheel and chassis_Link. Their links and joints are retained, but empty visual/collision references were removed. Supply real CAD meshes to display those parts. For RViz OpenGL shader errors try CC_SOFTWARE_RENDERING=1 ./run_desktop.sh; this increases laptop CPU usage. Scan driver acquisition-start timestamp fix must also be rebuilt on Pi. Initial TF drops can occur before EKF/SLAM initializes. Synchronize clocks on both computers; do not hide ongoing skew by shifting TF timestamps.
