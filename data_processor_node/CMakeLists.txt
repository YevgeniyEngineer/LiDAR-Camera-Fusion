# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(lidar_camera_fusion)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find need packages
find_package(TBB REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_subdirectory(./Common-Library)

# target build rules
add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
    data_reader_node.cpp
    data_processor_node.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME}
    PRIVATE
    ${PCL_LIBRARY_DIRS}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    ${PCL_DEFINITIONS}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    builtin_interfaces
)

target_link_libraries(${PROJECT_NAME}
    TBB::tbb
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
)