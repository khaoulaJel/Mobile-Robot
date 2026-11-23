#!/bin/bash
set -e
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-matplotlib ros-humble-rmw-cyclonedds-cpp ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-joy ros-humble-teleop-twist-joy ros-humble-twist-mux ros-humble-rviz2 ros-humble-xacro ros-humble-diagnostic-updater

# Repair a missing shared library even if apt records this package as installed.
sudo apt-get install -y --reinstall ros-humble-diagnostic-updater
