import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="keyboard",
        description="Teleop input mode: 'keyboard' or 'joystick'",
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("indigo_description"),
                "launch",
                "description.launch.py"
            )
        ),
    )

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("indigo_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        ),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("indigo_control"),
                "launch",
                "teleop.launch.py"
            )
        ),
        launch_arguments={
            "mode": LaunchConfiguration("mode")
        }.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("indigo_control"),
                "launch",
                "control.launch.py"
            )
        ),
    )

    return LaunchDescription([
        mode_arg,
        description,
        hardware_interface,
        teleop,
        controller,
    ])