# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)

# To Read Point Cloud Data
add_executable(point_cloud_reader_publisher_node
    ./point_cloud_reader_publisher_node.cpp
)
target_include_directories(point_cloud_reader_publisher_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(point_cloud_reader_publisher_node
    communication
    ${rclcpp_LIBRARIES}
)
ament_target_dependencies(point_cloud_reader_publisher_node
    rclcpp
    sensor_msgs
)

# To Read Video Frames
add_executable(camera_frame_reader_publisher_node
    ./camera_frame_reader_publisher_node.cpp
)
target_include_directories(camera_frame_reader_publisher_node
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(camera_frame_reader_publisher_node
    communication
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBS}
)
ament_target_dependencies(camera_frame_reader_publisher_node
    rclcpp
    sensor_msgs
)

# To Read Sensor Data
add_executable(sensor_data_publisher_node
    ./sensor_data_publisher_node.cpp
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
