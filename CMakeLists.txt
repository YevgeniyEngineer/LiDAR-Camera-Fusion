cmake_minimum_required(VERSION 3.18)

project(lidar_camera_fusion)

set(BUILD_TESTS TRUE)

add_subdirectory(./data_types)
add_subdirectory(./utilities)
add_subdirectory(./sensor_data_publisher_node)
