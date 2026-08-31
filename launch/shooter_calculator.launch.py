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
    launch_angle_deg = LaunchConfiguration('launch_angle_deg')
    cloth_mass_kg = LaunchConfiguration('cloth_mass_kg')
    cloth_area_m2 = LaunchConfiguration('cloth_area_m2')
    drag_coefficient = LaunchConfiguration('drag_coefficient')

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
            {'launch_angle_deg': launch_angle_deg},
            {'cloth_mass_kg': cloth_mass_kg},
            {'cloth_area_m2': cloth_area_m2},
            {'drag_coefficient': drag_coefficient},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_frame', default_value='moving_bucket'),
        DeclareLaunchArgument('target_frame_fallback', default_value='moving_bucket'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('launch_angle_deg', default_value='45.0'),
        DeclareLaunchArgument('cloth_mass_kg', default_value='0.12'),
        DeclareLaunchArgument('cloth_area_m2', default_value='0.36'),
        DeclareLaunchArgument('drag_coefficient', default_value='0.8'),
        shooter_calculator_node,
    ])
