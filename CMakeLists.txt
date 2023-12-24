cmake_minimum_required(VERSION 3.18)

project(lidar_camera_fusion)

set(BUILD_TESTS TRUE)

# Libraries
add_subdirectory(./libraries/data_types_lib)
add_subdirectory(./libraries/utilities_lib)
add_subdirectory(./libraries/lidar_processing_lib)

# # Nodes
# add_subdirectory(./sensor_data_publisher_node)
# add_subdirectory(./data_processor_node)

# Include the ROS2 workspace
# add_subdirectory(./ros2_ws)
