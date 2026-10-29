"""Launches coverage_pass_demo_node with the same MoveIt config params move_group.launch.py
supplies -- MoveItPlanningBackend constructs its own internal MoveGroupInterface node
("motion_planner_backend") at runtime, and MoveGroupInterface's constructor reads
robot_description/robot_description_semantic/kinematics off its own node's parameters, not
move_group's -- so this node needs the same params handed to it directly, same as move_group
itself gets them (see moveit_config/launch/move_group.launch.py's own header).

    ros2 launch motion_planner coverage_pass_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("description")
    urdf_xacro_path = os.path.join(description_share, "urdf", "robot.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="moveit_config")
        .robot_description(
            file_path=urdf_xacro_path,
            # Must match what's actually running in Gazebo this session (ranger_mini) -- serum
            # and ranger_mini mount the lift+arm at completely different heights (0 vs +0.3965m,
            # see robot.urdf.xacro's own lift_origin_xyz per base_type), so planning against the
            # wrong one collision-checks a robot shaped nothing like the real one.
            mappings={"base_type": "ranger_mini", "manipulation": "true"},
        )
        .robot_description_semantic(file_path="srdf/robot.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    coverage_pass_demo_node = Node(
        package="motion_planner",
        executable="coverage_pass_demo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"planning_pipelines.pipeline_names": ["ompl"]},
        ],
    )

    return LaunchDescription([coverage_pass_demo_node])
