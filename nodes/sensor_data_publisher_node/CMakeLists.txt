# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

# CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Project name
project(sensor_data_publisher_node)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Global packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Executable
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_data_publisher_node.cpp
)

# Includes
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

# Libraries
target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBS}
    utilities_lib
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
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
