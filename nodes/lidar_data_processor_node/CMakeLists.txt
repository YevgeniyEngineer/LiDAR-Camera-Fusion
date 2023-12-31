# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

# CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Project name
project(lidar_data_processor_node)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find need packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# target build rules
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_data_processor_node.cpp
)

# Libraries
target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    lidar_processing_lib
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
)

# Install the executable (ROS2 convention)
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Install the config
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# Install the launch
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
