# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(data_reader_node
    data_reader_node.cpp
)

target_link_libraries(data_reader_node
    communication
)

target_link_libraries(data_reader_node
    ${rclcpp_LIBRARIES}
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(data_reader_node
    rclcpp
    sensor_msgs
)

install(TARGETS common
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
