"""One place that assembles the MoveIt configuration for a chosen arm.

Used by this package's own demo and by any launch file that needs the same parameters (for
example a node that creates its own MoveGroupInterface and therefore needs the MoveIt
parameters handed to it directly).
"""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE = 'mobile_manipulator_moveit_config'
DESCRIPTION = 'mobile_manipulator_description'


def arm_family(arm):
    """Controller / joint-name family of an arm: 'ur' for the UR range, else 'generic_6dof'."""
    return 'ur' if arm.startswith('ur') else 'generic_6dof'


def build_moveit_config(arm='ur10e', hardware='mock'):
    """MoveItConfigs for `arm` with the lift + arm planning groups and the given hardware."""
    description = get_package_share_directory(DESCRIPTION)
    return (
        MoveItConfigsBuilder('mobile_manipulator', package_name=PACKAGE)
        .robot_description(
            file_path=os.path.join(description, 'urdf', 'mobile_manipulator.urdf.xacro'),
            mappings={'arm': arm, 'hardware': hardware})
        .robot_description_semantic(
            file_path=os.path.join(description, 'srdf', 'mobile_manipulator.srdf.xacro'),
            mappings={'arm': arm})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path=f'config/{arm_family(arm)}/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )


def controllers_file(arm):
    """Path of the ros2_controllers.yaml for `arm`."""
    return os.path.join(get_package_share_directory(PACKAGE), 'config', arm_family(arm),
                        'ros2_controllers.yaml')
