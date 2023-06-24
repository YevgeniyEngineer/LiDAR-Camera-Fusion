#!/bin/bash

# Make directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BUILD_DIR="${SCRIPT_DIR}/build"
LAUCH_DIR="${BUILD_DIR}"

# Launch
unset GTK_PATH

cd "${LAUCH_DIR}"
./kitti_data_reader_nodes/camera_frame_reader_publisher_node &
./kitti_data_reader_nodes/point_cloud_reader_publisher_node &

cd "${SCRIPT_DIR}"
rviz2 -d "visualisation/rviz2_config.rviz" &

wait