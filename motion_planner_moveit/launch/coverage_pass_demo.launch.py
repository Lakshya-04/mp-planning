"""Launches coverage_pass_demo_node with this robot's MoveIt parameters.

MoveItPlanningBackend constructs its own MoveGroupInterface node ("motion_planner_backend") at
runtime, and MoveGroupInterface's constructor reads robot_description, the semantic description
and kinematics off its own node's parameters rather than move_group's, so this node needs the
same parameters handed to it directly, exactly as move_group itself gets them. They come from
mobile_manipulator_moveit_config's shared builder.

The demo drives a JTAC (compliant trajectory controller) instance: start the robot with a
controller of that type under `controller_name` (default lift_arm_controller) first, then

    ros2 launch motion_planner_moveit coverage_pass_demo.launch.py profile:=ur10e_telescopic

`ik_seeds:=N` turns on seed-preferring IK in the backend (N extra solves from perturbed seeds, the
solution closest to the caller's seed wins, see motion_planner::SeedPreferringIk); 0 = one plain solve.
`ik_seed_perturbation` is the noise half-width in radians. The parameters reach the backend's own
node because this node is launched without a name, so they are written with a wildcard node name.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_manipulator_moveit_config import build_moveit_config, resolve_profile


def _node(context):
    arm = LaunchConfiguration('arm').perform(context) or None
    profile = LaunchConfiguration('profile').perform(context) or None
    moveit_config = build_moveit_config(arm, hardware='mock', profile=profile)
    # Empty planning_group: the node takes the whole-body group name from the profile.
    group = LaunchConfiguration('planning_group').perform(context)
    return [Node(
        package='motion_planner_moveit',
        executable='coverage_pass_demo_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
            {'planning_pipelines.pipeline_names': ['ompl']},
            {'profile': resolve_profile(arm, profile).name},
            {'planning_group': group},
            {'controller_name': LaunchConfiguration('controller_name').perform(context)},
            {'ik_extra_seeds': int(LaunchConfiguration('ik_seeds').perform(context))},
            {'ik_seed_perturbation': float(LaunchConfiguration('ik_seed_perturbation').perform(context))},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='',
                              description='arm name; picks the matching profile when `profile` is empty'),
        DeclareLaunchArgument('profile', default_value='',
                              description='robot profile (default ur10e_telescopic)'),
        DeclareLaunchArgument('planning_group', default_value='',
                              description='empty = the profile\'s whole-body group'),
        DeclareLaunchArgument('controller_name', default_value='lift_arm_controller'),
        DeclareLaunchArgument('ik_seeds', default_value='0',
                              description='extra perturbed-seed IK solves (0 = off)'),
        DeclareLaunchArgument('ik_seed_perturbation', default_value='0.3',
                              description='seed noise half-width [rad]'),
        OpaqueFunction(function=_node),
    ])
