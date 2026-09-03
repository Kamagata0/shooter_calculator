import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='moving_bucket',
        description='Target TF frame'
    )

    shooter_calculator_node = Node(
        package='shooter_calculator',
        executable='shooter_calculator_node',
        name='shooter_calculator',
        output='screen',
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'target_frame_fallback': LaunchConfiguration('target_frame'),
            'robot_frame': 'base_link',
            'launch_angle_deg': 50.0,
            'cloth_mass_kg': 0.12,
            'cloth_area_m2': 0.01,
            'drag_coefficient': 0.1,
        }]
    )

    return LaunchDescription([
        target_frame_arg,
        shooter_calculator_node,
    ])