cmake_minimum_required(VERSION 3.18)

project(utilities)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} SHARED

    # Sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/file_operations.cpp
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

    # Add test executable
    add_executable(test_bounded_vector ${CMAKE_CURRENT_SOURCE_DIR}/test/test_bounded_vector.cpp)

    # Link libraries
    target_link_libraries(test_bounded_vector
        PRIVATE
        ${PROJECT_NAME}
        GTest::gtest_main
    )

    # Discover test cases
    include(GoogleTest)
    gtest_discover_tests(test_bounded_vector)
endif()
