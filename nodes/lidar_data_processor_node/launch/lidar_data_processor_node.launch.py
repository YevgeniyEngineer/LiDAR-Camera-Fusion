from launch.launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("lidar_data_processor_node"),
        "config",
        "lidar_data_processor_node.config.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="lidar_data_processor_node",
                executable="lidar_data_processor_node",
                name="lidar_data_processor_node",
                parameters=[config],
                output="screen",
            )
        ]
    )
