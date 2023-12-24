# CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Project name
project(utilities_lib VERSION 1.0.0 LANGUAGES CXX)

# C++ Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities_lib/tlsf/tlsf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/file_operations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/thread_pool.cpp
)

# Header files
set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities_lib/tlsf/tlsf.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities_lib/bounded_vector.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities_lib/file_operations.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities_lib/thread_pool.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities_lib/tlsf_allocator.hpp
)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

# Specify include directories for the library
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    data_types_lib
)

# Installation rules for the library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the headers
install(DIRECTORY include/ DESTINATION include)

# Export the targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Install the Config and ConfigVersion files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Generate and install the Config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Unit testing
if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()

    file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
    add_executable(test_${PROJECT_NAME} ${TEST_SOURCES})
    target_link_libraries(test_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(test_${PROJECT_NAME})
endif()

# if(BUILD_TESTS)
# # Find GTest
# find_package(GTest REQUIRED)
# enable_testing()

# # Glob all test source files
# file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

# # Create an executable for each test source file
# foreach(TEST_SOURCE ${TEST_SOURCES})
# # Get a human-readable name for the test executable
# get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

# # Add test executable
# add_executable(${TEST_NAME} ${TEST_SOURCE})

# # Link libraries
# target_link_libraries(${TEST_NAME}
# PRIVATE
# ${PROJECT_NAME}
# GTest::gtest_main
# )

# # Discover test cases
# include(GoogleTest)
# gtest_discover_tests(${TEST_NAME})
# endforeach()
# endif()
