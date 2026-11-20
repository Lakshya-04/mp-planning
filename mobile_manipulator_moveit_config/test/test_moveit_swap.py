"""Arm swap test through MoveIt: for each profile build the MoveIt config, check the planning
groups, joint names and DOF against the profile (read with the profile loader), and run an FK/IK
round trip on the whole-body group with the KDL plugin (test/fk_ik_check.cpp)."""
import os
import subprocess
import tempfile

import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from mobile_manipulator_description import load_profile
from mobile_manipulator_moveit_config import build_moveit_config, controllers_file, resolve_profile

PROFILES = ['generic_6dof_telescopic', 'panda_telescopic', 'fr3_telescopic', 'iiwa7_telescopic',
            'kinova_gen3_telescopic', 'xarm7_telescopic']
try:
    get_package_share_directory('ur_description')
    PROFILES += ['ur10e_telescopic', 'ur3e_telescopic', 'ur5e_telescopic', 'ur16e_telescopic',
                 'ur20_telescopic']
except Exception:  # noqa: BLE001
    pass


def _tool():
    path = os.path.join(get_package_prefix('mobile_manipulator_moveit_config'), 'lib',
                        'mobile_manipulator_moveit_config', 'fk_ik_check')
    if not os.path.isfile(path):
        pytest.skip('fk_ik_check not built')
    return path


@pytest.mark.parametrize('name', PROFILES)
def test_moveit_config_follows_the_profile(name):
    p = load_profile(name)
    cfg = build_moveit_config(profile=name, ik='kdl')
    assert resolve_profile(profile=name).name == name
    import xml.etree.ElementTree as ET
    srdf = ET.fromstring(cfg.robot_description_semantic['robot_description_semantic'])
    assert {g.get('name') for g in srdf.findall('group')} == {
        p.arm_group, p.whole_body_group, p.tool_group}
    assert set(cfg.robot_description_kinematics['robot_description_kinematics']) >= {p.arm_group, p.whole_body_group}
    urdf = ET.fromstring(cfg.robot_description['robot_description'])
    system = urdf.find('ros2_control')
    assert [j.get('name') for j in system.findall('joint')] == p.joints


@pytest.mark.parametrize('name', PROFILES)
def test_controller_files_list_the_profile_joints(name):
    import yaml
    p = load_profile(name)
    for kind in ('trajectory', 'jtac'):
        data = yaml.safe_load(open(controllers_file(profile=name, controller=kind)))
        assert data['lift_arm_controller']['ros__parameters']['joints'] == p.joints
    moveit = yaml.safe_load(open(os.path.join(
        get_package_share_directory('mobile_manipulator_moveit_config'), 'config',
        p.controller_family, 'moveit_controllers.yaml')))
    assert moveit['moveit_simple_controller_manager']['lift_arm_controller']['joints'] == p.joints


@pytest.mark.parametrize('name', PROFILES)
def test_fk_ik_through_moveit(name):
    p = load_profile(name)
    cfg = build_moveit_config(profile=name, ik='kdl')
    with tempfile.TemporaryDirectory() as tmp:
        urdf = os.path.join(tmp, 'robot.urdf')
        srdf = os.path.join(tmp, 'robot.srdf')
        open(urdf, 'w').write(cfg.robot_description['robot_description'])
        open(srdf, 'w').write(cfg.robot_description_semantic['robot_description_semantic'])
        for group, dof, joints in ((p.whole_body_group, p.dof, p.joints),
                                   (p.arm_group, p.arm_dof, p.arm_joints)):
            out = subprocess.run([_tool(), urdf, srdf, group, '20'], capture_output=True,
                                 text=True, timeout=300)
            assert out.returncode == 0, out.stderr
            got = dict(line.split('=', 1) for line in out.stdout.split() if '=' in line)
            assert int(got['dof']) == dof
            assert got['joints'].split(',') == joints
            assert got['tip'] == p.tcp_link
            # IK from a nearby seed must recover the pose for most samples.
            assert int(got['ik_ok']) >= 12, out.stdout
            assert float(got['max_pos_error_m']) < 0.01, out.stdout
            assert float(got['max_rot_error_rad']) < 0.01, out.stdout


def test_profile_given_as_a_path_is_used_for_the_urdf_and_srdf(tmp_path):
    """A profile file whose basename matches an installed one must still win over the installed one."""
    import shutil
    import xml.etree.ElementTree as ET
    src = os.path.join(get_package_share_directory('mobile_manipulator_description'), 'config',
                       'profiles', 'panda_telescopic.yaml')
    custom = tmp_path / 'ur10e_telescopic.yaml'       # same name as an installed UR profile
    shutil.copy(src, custom)
    p = load_profile(str(custom))
    cfg = build_moveit_config(profile=str(custom), ik='kdl')
    urdf = ET.fromstring(cfg.robot_description['robot_description'])
    assert [j.get('name') for j in urdf.find('ros2_control').findall('joint')] == p.joints
    srdf = ET.fromstring(cfg.robot_description_semantic['robot_description_semantic'])
    home = [s for s in srdf.findall('group_state') if s.get('name') == 'home'][0]
    assert {j.get('name') for j in home.findall('joint')} >= set(p.arm_joints)


def test_arm_that_disagrees_with_the_profile_is_rejected():
    with pytest.raises(ValueError, match='does not match'):
        build_moveit_config(arm='ur10e', profile='panda_telescopic', ik='kdl')
