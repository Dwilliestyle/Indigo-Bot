import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lidar_config = os.path.join(
        get_package_share_directory("indigo_bringup"),
        "config",
        "ydlidar.yaml",
    )

    ydlidar_node = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_node",  # must match the top-level key in ydlidar.yaml
        output="screen",
        emulate_tty=True,
        parameters=[lidar_config],
    )

    return LaunchDescription([
        ydlidar_node,
    ])
