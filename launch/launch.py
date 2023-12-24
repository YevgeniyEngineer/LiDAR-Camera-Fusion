import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Sensor Data Publisher Node
    package_directory = get_package_share_directory("sensor_data_publisher_node")

    config_file = os.path.join(
        package_directory, "config", "sensor_data_publisher_node.param.yaml"
    )

    sensor_data_publisher_node = Node(
        package="sensor_data_publisher_node",
        executable="sensor_data_publisher_node",
        name="sensor_data_publisher_node",
        output="screen",
        parameters=[config_file],
    )

    # RViz
    package_directory = os.path.join(
        get_package_share_directory("lidar_camera_fusion"), "visualisation"
    )

    config_file = os.path.join(package_directory, "rviz2_config.rviz")

    rviz = ExecuteProcess(cmd=["rviz2", "-d", config_file], output="screen")

    # Create and return the launch description
    return LaunchDescription([rviz, sensor_data_publisher_node])
