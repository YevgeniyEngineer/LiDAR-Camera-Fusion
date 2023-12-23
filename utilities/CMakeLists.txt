cmake_minimum_required(VERSION 3.18)

project(utilities)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} SHARED

    # Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/file_operations.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities/tlsf/tlsf.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    data_types
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

if(BUILD_TESTS)
    # Find GTest
    find_package(GTest REQUIRED)
    enable_testing()

    # Glob all test source files
    file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

    # Create an executable for each test source file
    foreach(TEST_SOURCE ${TEST_SOURCES})
        # Get a human-readable name for the test executable
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

        # Add test executable
        add_executable(${TEST_NAME} ${TEST_SOURCE})

        # Link libraries
        target_link_libraries(${TEST_NAME}
            PRIVATE
            ${PROJECT_NAME}
            GTest::gtest_main
        )

        # Discover test cases
        include(GoogleTest)
        gtest_discover_tests(${TEST_NAME})
    endforeach()
endif()
