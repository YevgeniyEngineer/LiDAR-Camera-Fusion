cmake_minimum_required(VERSION 3.18)

project(lidar_camera_fusion)

add_subdirectory(./common)
add_subdirectory(./communication)
add_subdirectory(./kitti_data_reader_nodes)

# add_subdirectory(./processing)
# add_subdirectory(./data_processor_node)