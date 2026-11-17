"""One place that assembles the MoveIt configuration for a chosen robot profile.

Used by this package's own demo and by any launch file that needs the same parameters (for
example a node that creates its own MoveGroupInterface and therefore needs the MoveIt
parameters handed to it directly).
"""
import os

from ament_index_python.packages import get_package_share_directory
from mobile_manipulator_description import DEFAULT_PROFILE, load_profile, profile_for_arm
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE = 'mobile_manipulator_moveit_config'
DESCRIPTION = 'mobile_manipulator_description'

KINEMATICS = {'kdl': 'config/kinematics.yaml', 'pick_ik': 'config/kinematics_pick_ik.yaml',
              'trac_ik': 'config/kinematics_trac_ik.yaml'}

# Group names the kinematics files are written with; renamed to the profile's names.
_KINEMATICS_GROUPS = {'arm': 'arm_group', 'lift_arm': 'whole_body_group'}


def resolve_profile(arm=None, profile=None):
    """Profile for the request: `profile` (name or path) wins, else the one that goes with `arm`,
    else the default (ur10e_telescopic)."""
    if profile:
        return load_profile(profile)
    return load_profile(profile_for_arm(arm) if arm else DEFAULT_PROFILE)


def arm_family(arm=None, profile=None):
    """Controller / joint-name family of the robot: the profile's `arm.controller_family`."""
    return resolve_profile(arm, profile).controller_family


def _config_path(*parts):
    return os.path.join(get_package_share_directory(PACKAGE), 'config', *parts)


def build_moveit_config(arm=None, hardware='mock', ik='trac_ik', profile=None, live_octomap=False):
    """MoveItConfigs for a profile with the lift + arm planning groups and the given hardware.

    `profile` (name or path, default ur10e_telescopic) says which arm, how many joints and what
    the groups are called; `arm` alone picks the matching profile, and an explicit `arm` also
    overrides the profile's arm only where the joint names stay the same (ur5e on a UR profile);
    otherwise a ValueError says they disagree. `ik` picks the kinematics solver: 'trac_ik' (default: stays on
    the seed's branch on large moves), 'kdl' (ships with MoveIt) or 'pick_ik' (opt-in, untuned);
    see docs/moveit_config.md. `live_octomap=True` adds config/sensors_3d.yaml (MoveIt's
    PointCloudOctomapUpdater on /depth_sensor/points; needs moveit_ros_perception, docs/perception_live.md).
    Use resolve_profile(arm, profile) for the same profile object.
    """
    prof = resolve_profile(arm, profile)
    if arm and arm != prof.arm and profile and \
            load_profile(profile_for_arm(arm)).joints != prof.joints:
        raise ValueError(
            f"arm '{arm}' does not match profile '{prof.name}' (arm '{prof.arm}', joints "
            f'{prof.joints}); drop `arm` or pick the matching profile')
    arm = arm or prof.arm
    description = get_package_share_directory(DESCRIPTION)
    family = prof.controller_family
    # A family may ship its own limits (e.g. a 7-DOF arm); otherwise the shared file applies.
    limits = (f'config/{family}/joint_limits.yaml'
              if os.path.isfile(_config_path(family, 'joint_limits.yaml'))
              else 'config/joint_limits.yaml')
    builder = (
        MoveItConfigsBuilder(prof.robot_name, package_name=PACKAGE)
        .robot_description(
            file_path=os.path.join(description, 'urdf', 'mobile_manipulator.urdf.xacro'),
            mappings=dict(prof.xacro_mappings(), arm=arm, profile=prof.path or prof.name,
                          hardware=hardware))
        .robot_description_semantic(
            file_path=os.path.join(description, 'srdf', 'mobile_manipulator.srdf.xacro'),
            mappings={'arm': arm, 'profile': prof.path or prof.name})
        .robot_description_kinematics(file_path=KINEMATICS[ik])
        .joint_limits(file_path=limits)
        .trajectory_execution(file_path=f'config/{family}/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
    )
    if live_octomap:
        builder = builder.sensors_3d(file_path='config/sensors_3d.yaml')
    cfg = builder.to_moveit_configs()
    # {'robot_description_kinematics': {<group>: {...}}}: rename the groups to the profile's.
    kin = cfg.robot_description_kinematics['robot_description_kinematics']
    cfg.robot_description_kinematics['robot_description_kinematics'] = {
        **{k: v for k, v in kin.items() if k not in _KINEMATICS_GROUPS},
        **{getattr(prof, attr): kin[k] for k, attr in _KINEMATICS_GROUPS.items() if k in kin}}
    return cfg


CONTROLLERS = {'trajectory': 'ros2_controllers.yaml', 'jtac': 'jtac_controllers.yaml'}


def hardware_for(controller):
    """ros2_control hardware mode a controller needs ('jtac' needs velocity commands + F/T)."""
    return {'trajectory': 'mock', 'jtac': 'jtac_mock'}[controller]


def controllers_file(arm=None, controller='trajectory', profile=None):
    """Path of the controller parameter file for the profile/arm: 'trajectory'
    (joint_trajectory_controller) or 'jtac' (the compliant JtacController)."""
    return _config_path(arm_family(arm, profile), CONTROLLERS[controller])
