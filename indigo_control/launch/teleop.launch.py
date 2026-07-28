from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    teleop_mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="keyboard",
        description="Teleop input mode: 'keyboard' or 'joystick'",
    )

    mode = LaunchConfiguration("mode")

    # --- Keyboard teleop ---
    keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",  # needs an interactive terminal for raw keystrokes
        parameters=[{"stamped": True}],
        remappings=[("/cmd_vel", "/indigobot_controller/cmd_vel")],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'keyboard'"])),
    )

    # --- Joystick teleop ---
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", mode, "' == 'joystick'"])),
    )

    teleop_joy_config = PathJoinSubstitution(
        [get_package_share_directory("indigo_control"), "config", "teleop_joy.yaml"]
    )

    teleop_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[teleop_joy_config],
        remappings=[("/cmd_vel", "/indigobot_controller/cmd_vel")],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'joystick'"])),
    )

    return LaunchDescription([
        teleop_mode_arg,
        keyboard_node,
        joy_node,
        teleop_joy_node,
    ])