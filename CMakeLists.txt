cmake_minimum_required(VERSION 3.18)

project(lidar_camera_fusion)

set(BUILD_TESTS TRUE)

# ROS
find_package(ament_cmake REQUIRED)

# Libraries
add_subdirectory(./libraries/data_types_lib)
add_subdirectory(./libraries/utilities_lib)
add_subdirectory(./libraries/lidar_processing_lib)

# # Nodes
add_subdirectory(./nodes/sensor_data_publisher_node)
add_subdirectory(./nodes/lidar_data_processor_node)

# Install the launch
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install the visualization
install(DIRECTORY visualization
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
