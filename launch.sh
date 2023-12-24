#!/bin/bash

# Location of script's directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

unset GTK_PATH  # Needed for Rviz, else crashes

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

ros2 launch lidar_camera_fusion launch.py
