#!/bin/bash
# Robot-day helper, run on the LAPTOP (WSL). Everything for the real robot in one place, step by step.
#
#   bash robot_session.sh check            1. is the robot reachable? (network, odometry, no duplicate nodes)
#   bash robot_session.sh measure          2. measure the robot's response (about 85 s, small movements, aborts at 0.4 m)
#   bash robot_session.sh reference        3. start the reference controller with the coefficients file
#   bash robot_session.sh send-pose X Y YAW_RAD     send a pose reference           (after step 3)
#   bash robot_session.sh send-yaw  YAW_RAD         send a heading reference        (after step 3)
#   bash robot_session.sh set NAME VALUE            change a coefficient while it runs, e.g. set kv 0.6
#   bash robot_session.sh stop             stop the controller (zero commands are sent)
#
# Environment (same on the Pi and the laptop, see docs/network_setup.md):
#   COEFFS=/path/file.yaml   the coefficients you were given (default: config/reference_controller.yaml)
#   CDDS=~/cdds.xml          CycloneDDS peer file
# The robot stack (Pi: hardware.launch.py, laptop: run_desktop.sh) must already be running.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$HERE/../../../install/setup.bash" ] && source "$HERE/../../../install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
[ -f "${CDDS:-$HOME/cdds.xml}" ] && export CYCLONEDDS_URI="file://${CDDS:-$HOME/cdds.xml}"
COEFFS="${COEFFS:-reference_controller.yaml}"

case "${1:-help}" in
  check)
    echo "== odometry from the robot (expect 25-40 Hz, longest gap < 0.3 s)"
    timeout 12 ros2 topic hz /diff_cont/odom 2>&1 | tail -2
    echo "== duplicate node names (expect nothing)"; ros2 node list 2>/dev/null | sort | uniq -d
    echo "== who receives our commands (expect twist_mux)"; ros2 topic info /cmd_vel_tracker | sed -n 1,3p
    echo "== arduino link seen by the driver"; ros2 control list_controllers 2>/dev/null | head -3
    ;;
  measure)
    python3 "$HERE/measure_response.py" --state-topic /diff_cont/odom --cmd-topic /cmd_vel_tracker \
        --max-distance 0.4 --out "$HOME/measured_response.json"
    echo "Put the printed lead_* / deadband_* lines into your coefficients file."
    ;;
  reference)
    exec ros2 launch lab3_controllers reference.launch.py config:="$COEFFS" use_sim_time:=false
    ;;
  send-pose)
    ros2 topic pub --once /reference geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: odom}, pose: {position: {x: $2, y: $3}, orientation: {z: $(python3 -c "import math;print(math.sin($4/2))"), w: $(python3 -c "import math;print(math.cos($4/2))")}}}"
    ;;
  send-yaw)
    ros2 topic pub --once /reference_yaw std_msgs/msg/Float64 "{data: $2}"
    ;;
  set)
    ros2 param set /reference_controller "$2" "$3"
    ;;
  stop)
    pkill -TERM -f "[r]eference_controller" && echo "stopped (zero commands sent)"
    ;;
  *)
    sed -n 2,16p "$0"
    ;;
esac
