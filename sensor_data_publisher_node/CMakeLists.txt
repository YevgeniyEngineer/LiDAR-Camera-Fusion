# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# To Read Sensor Data
add_executable(sensor_data_publisher_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_data_publisher_node.cpp
)
target_include_directories(sensor_data_publisher_node
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(sensor_data_publisher_node
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBS}
    utilities
)
ament_target_dependencies(sensor_data_publisher_node
    rclcpp
    sensor_msgs
)
