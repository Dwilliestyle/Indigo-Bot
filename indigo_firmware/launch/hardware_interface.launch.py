import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("indigo_description"),
                    "urdf",
                    "indigo.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("indigo_control"),
                "config",
                "indigobot_controller.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    indigobot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["indigobot_controller"],
    )

    mpu_driver = Node(
        package="indigo_firmware",
        executable="mpu_driver.py"
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            indigobot_controller_spawner,
            mpu_driver
        ]
    )