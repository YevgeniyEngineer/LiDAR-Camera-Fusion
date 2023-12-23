# Set CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Set project name
project(lidar_segmentation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(PCL 1.7.2 REQUIRED)

# Set library
add_library(${PROJECT_NAME}
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ransac_segmenter.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

# Link directories
target_link_directories(${PROJECT_NAME}
    PRIVATE
    ${PCL_LIBRARY_DIRS}
)

# Compile definitions
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    ${PCL_DEFINITIONS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    utilities
)
