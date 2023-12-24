from launch.launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sensor_data_publisher_node"),
        "config",
        "sensor_data_publisher_node.config.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="sensor_data_publisher_node",
                executable="sensor_data_publisher_node",
                name="sensor_data_publisher_node",
                parameters=[config],
                output="screen",
            )
        ]
    )
