import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('indigo_bringup')
    mapping_dir = get_package_share_directory('indigo_mapping')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(mapping_dir, 'config', 'slam_toolbox.yaml')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')))

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'lidar.launch.py')))

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        robot,
        lidar,
        # Give the controllers, TF and lidar a few seconds to come up first
        TimerAction(period=5.0, actions=[slam]),
    ])