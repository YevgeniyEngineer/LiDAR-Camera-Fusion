cmake_minimum_required(VERSION 3.18)

project(data_types)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} INTERFACE

    # Sources
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>

    # PRIVATE
    # ${CMAKE_CURRENT_SOURCE_DIR}/include
)
