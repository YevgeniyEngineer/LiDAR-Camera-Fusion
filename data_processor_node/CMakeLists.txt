# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

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
add_executable(data_processor_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data_processor_node.cpp
)
ament_target_dependencies(data_processor_node
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
)
target_link_libraries(data_processor_node
    lidar_segmentation
)
