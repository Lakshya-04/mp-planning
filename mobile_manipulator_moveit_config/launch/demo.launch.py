"""MoveIt demo on mock hardware.

  ros2 launch mobile_manipulator_moveit_config demo.launch.py arm:=ur10e
  ros2 launch mobile_manipulator_moveit_config demo.launch.py arm:=generic_6dof rviz:=false

Starts robot_state_publisher, ros2_control (mock hardware) with the joint state broadcaster and
one 7-joint trajectory controller (lift + arm), move_group, and optionally RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_manipulator_moveit_config import build_moveit_config, controllers_file


def _nodes(context):
    arm = LaunchConfiguration('arm').perform(context)
    config_pkg = get_package_share_directory('mobile_manipulator_moveit_config')
    moveit_config = build_moveit_config(arm, hardware='mock')
    controllers = controllers_file(arm)

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[moveit_config.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[moveit_config.robot_description, controllers]),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['lift_arm_controller', '--controller-manager', '/controller_manager']),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[moveit_config.to_dict()]),
        Node(package='rviz2', executable='rviz2', output='log', condition=IfCondition(
                LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(config_pkg, 'rviz', 'moveit.rviz')],
             parameters=[moveit_config.robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics,
                         moveit_config.planning_pipelines,
                         moveit_config.joint_limits]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='ur10e',
                              description='ur10e, ur5e, ... or generic_6dof'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
