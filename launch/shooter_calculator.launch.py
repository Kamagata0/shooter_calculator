import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_frame = LaunchConfiguration('target_frame')
    target_frame_fallback = LaunchConfiguration('target_frame_fallback')
    robot_frame = LaunchConfiguration('robot_frame')

    # Shooter Calculator Node
    shooter_calculator_node = Node(
        package='shooter_calculator',
        executable='shooter_calculator_node',
        name='shooter_calculator',
        output='screen',
        parameters=[
            {'target_frame': target_frame},
            {'target_frame_fallback': target_frame_fallback},
            {'robot_frame': robot_frame},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_frame', default_value='moving_bucket'),
        DeclareLaunchArgument('target_frame_fallback', default_value='movingbacket'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        shooter_calculator_node,
    ])
