# CC robot simulation model

`urdf/CC_robot_URDF.urdf`, all meshes, `CMakeLists.txt`, and `package.xml`
were copied unchanged from `the course workspace`.
The existing CMake directory installation includes both URDF files and all
meshes. The original workspace is not modified.

`urdf/CC_robot_gazebo.urdf` is the simulation adaptation used by the Lab 3
launch file. It retains the original link and joint names, visual meshes,
and fixed frame transforms. These changes make the CAD export driveable in
Gazebo Classic:

- Both wheel joints are continuous. The original zero position, effort,
  and velocity limits would lock the wheels. Simulation limits are 1 N m
  and 30 rad/s, with joint damping 0.001; these are lab settings, not motor
  specifications.
- The mirrored wheel-frame rotations are set to exact positive/negative
  pi/2. The left joint axis is `0 0 1`; the right axis remains `0 0 -1`.
  Both then rotate around positive Y in the base frame, so positive wheel
  speed drives the robot along positive X.
- Wheel separation is 0.177046 m and radius is 0.034 m. Wheel collisions
  use cylinders, radius 0.034 m and length 0.031750001 m, centered at local
  Z = -0.00134786777 m to match the original STL bounds. All original mesh
  visuals remain in use.
- The duplicate body inertias on the empty `base_footprint`, `chassis_Link`,
  and `caster_wheel` links are removed. The original 1.76637995132488 kg
  `base_inertia` and both original wheel inertias remain. The invalid
  zero-mass LiDAR inertia is removed; its geometry is fixed to the body.
  Gazebo combines the body and LiDAR fixed links into `base_link`. Keeping the body
  inertia on the original fixed child also avoids a root-inertia KDL
  warning in `robot_state_publisher`.
- The source caster has no usable mesh. A visual and collision sphere of
  radius 0.023 m is added at the existing caster frame, whose base-relative
  position is `(0.11932, 0, -0.011)` m. Its bottom meets the drive-wheel
  ground plane. The sphere is assigned 0.02 kg and uniform-sphere inertia
  (0.000004232 kg m² on each diagonal). This is a passive sliding support
  approximation, not a calibrated caster model.
- Drive-wheel friction coefficients are 1.0. Caster coefficients are 0.001,
  allowing the support sphere to slide when turning. The caster's fixed
  joint is preserved in Gazebo so this friction setting survives URDF-to-SDF
  conversion. Other fixed body links are combined normally.
- Empty material names are replaced with unique names without changing
  the original colors.

The model origin remains at wheel-axle height. The launch spawns it above
the ground and lets gravity settle it at approximately Z = 0.034 m.
The unchanged `base_footprint` frame is coincident with `base_link`, as in
the source model; it is not a newly redefined ground-projection frame.

## Gazebo plugins and topics

The `libgazebo_ros_diff_drive.so` plugin runs at 50 Hz and uses:

| Interface | Topic or frame |
| --- | --- |
| Command (`geometry_msgs/msg/Twist`) | `/diff_cont/cmd_vel_unstamped` |
| Odometry (`nav_msgs/msg/Odometry`) | `/odom` |
| Odometry reference frame | `odom` |
| Robot body frame | `base_link` |
| TF | `odom` to `base_link` |

Odometry source is Gazebo world pose (`odometry_source = 1`), so odometry
reflects simulated motion, including contact effects. Maximum wheel torque
is 1 N m and maximum wheel acceleration is 2 m/s² at the wheel rim.
The joint-state plugin publishes `/joint_states` at 50 Hz. Wheel TF is
disabled in the drive plugin because `robot_state_publisher` publishes the
link transforms from these joint states.

The companion `lab3_gazebo` package publishes `/robot_pose` and provides the
launch file. This simulation uses the Gazebo drive plugin directly and
does not require a `ros2_control` controller manager.
