# CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Project name
project(data_types_lib VERSION 1.0.0 LANGUAGES CXX)

# C++ Version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCE_FILES
)

# Header files
set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/data_types_lib/cartesian_return.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/data_types_lib/classification_labels.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/data_types_lib/segmentation_labels.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/data_types_lib/spherical_return.hpp
)

# Create library
add_library(${PROJECT_NAME} INTERFACE ${SOURCE_FILES} ${HEADER_FILES})

# Specify include directories for the library
target_include_directories(${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>

    # PRIVATE
    # ${CMAKE_CURRENT_SOURCE_DIR}/include
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
