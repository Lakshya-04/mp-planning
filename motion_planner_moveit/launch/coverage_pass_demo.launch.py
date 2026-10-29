"""Launches coverage_pass_demo_node with this robot's MoveIt parameters.

MoveItPlanningBackend constructs its own MoveGroupInterface node ("motion_planner_backend") at
runtime, and MoveGroupInterface's constructor reads robot_description, the semantic description
and kinematics off its own node's parameters rather than move_group's, so this node needs the
same parameters handed to it directly, exactly as move_group itself gets them. They come from
mobile_manipulator_moveit_config's shared builder.

The demo drives a JTAC (compliant trajectory controller) instance: start the robot with a
controller of that type under `controller_name` (default lift_arm_controller) first, then

    ros2 launch motion_planner_moveit coverage_pass_demo.launch.py arm:=ur10e
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_manipulator_moveit_config import build_moveit_config


def _node(context):
    arm = LaunchConfiguration('arm').perform(context)
    moveit_config = build_moveit_config(arm, hardware='mock')
    return [Node(
        package='motion_planner_moveit',
        executable='coverage_pass_demo_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
            {'planning_pipelines.pipeline_names': ['ompl']},
            {'planning_group': LaunchConfiguration('planning_group').perform(context)},
            {'controller_name': LaunchConfiguration('controller_name').perform(context)},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='ur10e'),
        DeclareLaunchArgument('planning_group', default_value='lift_arm'),
        DeclareLaunchArgument('controller_name', default_value='lift_arm_controller'),
        OpaqueFunction(function=_node),
    ])
