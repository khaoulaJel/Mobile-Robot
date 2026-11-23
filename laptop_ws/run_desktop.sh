#!/bin/bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0
# Report missing binary dependencies before starting the supervised launch.
for binary in /opt/ros/humble/lib/robot_localization/ekf_node /opt/ros/humble/lib/twist_mux/twist_mux /opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node; do
    if [[ ! -x "$binary" ]]; then
        echo "Missing ROS executable: $binary. Run ./install_dependencies.sh" >&2
        exit 1
    fi
    dependencies=$(ldd "$binary" 2>&1) || { echo "$dependencies" >&2; exit 1; }
    if [[ "$dependencies" == *"not found"* ]]; then
        echo "Missing shared libraries for $binary:" >&2
        echo "$dependencies" >&2
        echo "Run ./install_dependencies.sh and retry." >&2
        exit 1
    fi
done
# Optional fallback for laptop OpenGL shader failures.
if [[ "${CC_SOFTWARE_RENDERING:-0}" == "1" ]]; then
    export LIBGL_ALWAYS_SOFTWARE=1
fi
exec ros2 launch articubot_one desktop.launch.py "$@"
