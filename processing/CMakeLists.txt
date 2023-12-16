# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18)

project(processing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(TBB REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_subdirectory(./Common-Library)

add_library(${PROJECT_NAME}
    pipeline.hpp
    pipeline.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    common_library
    TBB::tbb
    Eigen3::Eigen
    utilities
)
