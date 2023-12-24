import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    package_directory = get_package_share_directory("lidar_camera_fusion")

    # Node 1: Sensor Data Publisher Node
    sensor_data_publisher_node = Node(
        package="lidar_camera_fusion",
        executable="sensor_data_publisher_node",
        name="sensor_data_publisher_node",
        output="screen",
        parameters=[
            os.path.join(
                package_directory, "config", "sensor_data_publisher_node.param.yaml"
            )
        ],
    )

    # Create and return the launch description
    return LaunchDescription(
        [
            sensor_data_publisher_node,
        ]
    )
