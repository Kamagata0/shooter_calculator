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
    enable_real_output_arg = DeclareLaunchArgument(
        'enable_real_output',
        default_value='false',
        description='Enable publishing belt speed commands to robot-port'
    )
    auto_initialize_belt_arg = DeclareLaunchArgument(
        'auto_initialize_belt',
        default_value='false',
        description='Publish belt initialization while real output is enabled'
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
            'enable_real_output': LaunchConfiguration('enable_real_output'),
            'auto_initialize_belt': LaunchConfiguration('auto_initialize_belt'),
            'belt_speed_command_max_mps': 1.0,
        }]
    )

    return LaunchDescription([
        target_frame_arg,
        enable_real_output_arg,
        auto_initialize_belt_arg,
        shooter_calculator_node,
    ])