from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('secaro_ros2_converter')

    # Declare the namespace argument (it can be provided when launching)

    return LaunchDescription([
        Node(
            package='secaro_ros2_converter',
            executable='wifi',
            parameters= [
                {'enable_log': True},
                {'cmd_vel_topic_name': '/cmd_vel'},
            ]
        )
    ])