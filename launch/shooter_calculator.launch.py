import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    shooter_calculator_node = Node(
        package='shooter_calculator',
        executable='shooter_calculator_node',
        name='shooter_calculator',
        output='screen',
        parameters=[{
            'target_frame': 'moving_bucket',
            'target_frame_fallback': 'moving_bucket',
            'robot_frame': 'base_link',
            'launch_angle_deg': 45.0,
            'cloth_mass_kg': 0.12,
            'cloth_area_m2': 0.01,
            'drag_coefficient': 0.1,
        }]
    )

    return LaunchDescription([
        shooter_calculator_node,
    ])