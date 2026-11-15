"""Serve reachability queries from a map built with rm4d_tool.

  ros2 launch reachability reachability.launch.py map_file:=/path/ur10e.rm4d

`base_profile:=/path/profile.yaml` (a robot profile with a `base:` section) lets `~/plan_stands` use the
base's footprint and drive type.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file'),
        DeclareLaunchArgument('robot_fingerprint', default_value=''),
        DeclareLaunchArgument('base_profile', default_value=''),
        Node(package='reachability', executable='reachability_node', output='screen',
             parameters=[{'map_file': LaunchConfiguration('map_file'),
                          'robot_fingerprint': LaunchConfiguration('robot_fingerprint'),
                          'base_profile': LaunchConfiguration('base_profile')}]),
    ])
