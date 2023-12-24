#!/bin/bash

unset GTK_PATH  # Needed for Rviz, else crashes

ros2 launch lidar_camera_fusion launch.py
