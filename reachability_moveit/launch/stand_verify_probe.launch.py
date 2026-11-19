"""Runs stand_verify_probe with this robot's MoveIt parameters (see motion_planner_moveit's
coverage_pass_demo.launch.py for why the node needs them): start the demo first (mock hardware),

    ros2 launch mobile_manipulator_moveit_config demo.launch.py profile:=ur10e_telescopic rviz:=false
    ros2 launch reachability_moveit stand_verify_probe.launch.py map_file:=/path/ur10e.rm4d wall_x:=1.4

The map is built for the same arm (docs/reachability.md, `rm4d_tool build`). Extra probe parameters
(wall_y_min, wall_z_max, max_triples, ...) are in apps/stand_verify_probe.cpp; pass them with `extra:=`
as `name=value,name=value`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_manipulator_moveit_config import build_moveit_config, resolve_profile
import yaml


def _node(context):
    arm = LaunchConfiguration('arm').perform(context) or None
    profile = LaunchConfiguration('profile').perform(context) or None
    moveit_config = build_moveit_config(arm, hardware='mock', profile=profile)
    probe = {'map_file': LaunchConfiguration('map_file').perform(context),
             'wall_x': float(LaunchConfiguration('wall_x').perform(context)),
             'ik_extra_seeds': int(LaunchConfiguration('ik_seeds').perform(context))}
    for item in filter(None, LaunchConfiguration('extra').perform(context).split(',')):
        key, value = item.split('=', 1)
        probe[key] = yaml.safe_load(value)
    return [Node(
        package='reachability_moveit',
        executable='stand_verify_probe',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': False},
                    {'planning_pipelines.pipeline_names': ['ompl']},
                    {'profile': resolve_profile(arm, profile).name}, probe],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value=''),
        DeclareLaunchArgument('profile', default_value=''),
        DeclareLaunchArgument('map_file', description='RM4D map of the arm (rm4d_tool build)'),
        DeclareLaunchArgument('wall_x', default_value='1.4'),
        DeclareLaunchArgument('ik_seeds', default_value='0'),
        DeclareLaunchArgument('extra', default_value='', description='name=value,name=value probe parameters'),
        OpaqueFunction(function=_node),
    ])
