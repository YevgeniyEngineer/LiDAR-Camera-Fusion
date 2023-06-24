#!/bin/bash

# Make directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BUILD_DIR="${SCRIPT_DIR}/build"
LAUCH_DIR="${BUILD_DIR}"

# Launch
unset GTK_PATH

cd "${LAUCH_DIR}"

DATA_FOLDER="/home/yevgeniy/Documents/GitHub/LiDAR-Camera-Fusion/a_kitti_dataset/2011_09_26_drive_0013_sync"

./kitti_data_reader_nodes/camera_frame_reader_publisher_node \
    "${DATA_FOLDER}" \
    "image_02" \
    "camera_1" &

./kitti_data_reader_nodes/camera_frame_reader_publisher_node \
    "${DATA_FOLDER}" \
    "image_03" \
    "camera_2" &

./kitti_data_reader_nodes/point_cloud_reader_publisher_node \
    "${DATA_FOLDER}" \
    "pointcloud" &

cd "${SCRIPT_DIR}"
rviz2 -d "visualisation/rviz2_config.rviz" &

wait