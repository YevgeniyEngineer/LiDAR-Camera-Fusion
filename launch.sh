#!/bin/bash

# Make directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BUILD_DIR="${SCRIPT_DIR}/build"
LAUCH_DIR="${BUILD_DIR}"

# Launch
unset GTK_PATH

cd "${LAUCH_DIR}"

DATA_FOLDER="/home/yevgeniy/Documents/GitHub/LiDAR-Camera-Fusion/a_kitti_dataset/2011_09_26_drive_0013_sync"

LIDAR_DATA_PATH="${DATA_FOLDER}/velodyne_points"
CAMERA_1_DATA_PATH="${DATA_FOLDER}/image_00"
CAMERA_2_DATA_PATH="${DATA_FOLDER}/image_01"
CAMERA_3_DATA_PATH="${DATA_FOLDER}/image_02"
CAMERA_4_DATA_PATH="${DATA_FOLDER}/image_03"

LIDAR_TOPIC="lidar"
CAMERA_1_TOPIC="camera_1"
CAMERA_2_TOPIC="camera_2"
CAMERA_3_TOPIC="camera_3"
CAMERA_4_TOPIC="camera_4"

./kitti_data_reader_nodes/sensor_data_publisher_node \
    "${LIDAR_DATA_PATH}" \
    "${CAMERA_1_DATA_PATH}" \
    "${CAMERA_2_DATA_PATH}" \
    "${CAMERA_3_DATA_PATH}" \
    "${CAMERA_4_DATA_PATH}" \
    "${LIDAR_TOPIC}" \
    "${CAMERA_1_TOPIC}" \
    "${CAMERA_2_TOPIC}" \
    "${CAMERA_3_TOPIC}" \
    "${CAMERA_4_TOPIC}" &

cd "${SCRIPT_DIR}"
rviz2 -d "visualisation/rviz2_config.rviz" &

wait
