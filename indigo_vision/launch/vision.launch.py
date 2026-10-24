from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_id_arg = DeclareLaunchArgument('device_id', default_value='0')
    width_arg = DeclareLaunchArgument('width', default_value='640')
    height_arg = DeclareLaunchArgument('height', default_value='480')
    fps_arg = DeclareLaunchArgument('fps', default_value='30')
    web_video_arg = DeclareLaunchArgument('web_video', default_value='true')

    camera_publisher = Node(
        package='indigo_vision',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
        }],
    )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080, 'address': '0.0.0.0'}],
        condition=IfCondition(LaunchConfiguration('web_video')),
    )

    return LaunchDescription([
        device_id_arg,
        width_arg,
        height_arg,
        fps_arg,
        web_video_arg,
        camera_publisher,
        web_video_server,
    ])