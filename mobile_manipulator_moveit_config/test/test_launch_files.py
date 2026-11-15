"""Every launch file in the workspace imports, builds its description, and only names packages
that exist here (src/**/package.xml) or are known ROS / MoveIt packages."""
import ast
import importlib.util
import pathlib
import re
import sys

import ament_index_python.packages as ament_packages
import pytest

SRC = pathlib.Path(__file__).resolve().parents[3]
LAUNCH_FILES = sorted(SRC.rglob('launch/*.launch.py'))

# Packages launch files may look up that are not built from this workspace.
KNOWN_EXTERNAL = {
    'ros_gz_sim', 'ros_gz_bridge', 'gz_ros2_control', 'moveit_configs_utils', 'moveit_ros_move_group',
    'moveit_ros_visualization', 'controller_manager', 'robot_state_publisher', 'rviz2', 'xacro',
    'ur_description', 'tf2_ros', 'joint_state_publisher', 'joint_state_publisher_gui',
    'joint_state_broadcaster', 'joint_trajectory_controller',
}


def _add_workspace_python_packages():
    """Launch files import their package's python module (e.g. sim_gazebo); make those
    importable from the source tree when the package is not in the sourced overlay."""
    for init in SRC.glob('*/*/*/__init__.py'):
        if init.parent.name == init.parent.parent.name and str(init.parent.parent) not in sys.path:
            sys.path.append(str(init.parent.parent))


_add_workspace_python_packages()
LOOKUPS = {'get_package_share_directory', 'get_package_prefix', 'FindPackageShare'}


def workspace_packages():
    names = set()
    for xml in SRC.rglob('package.xml'):
        m = re.search(r'<name>\s*([\w-]+)\s*</name>', xml.read_text())
        if m:
            names.add(m.group(1))
    return names


def literal_lookups(path):
    """Package names passed as a string literal to the lookup functions in a launch file."""
    found = []
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Call):
            fn = node.func
            name = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, 'id', '')
            if name in LOOKUPS and node.args and isinstance(node.args[0], ast.Constant) \
                    and isinstance(node.args[0].value, str):
                found.append(node.args[0].value)
    return found


def test_launch_files_found():
    assert len(LAUNCH_FILES) >= 10


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda p: p.parent.parent.name + '/' + p.name)
def test_launch_file(path, monkeypatch):
    allowed = workspace_packages() | KNOWN_EXTERNAL
    for name in literal_lookups(path):
        assert name in allowed, f'{path.name} looks up unknown package {name!r}'

    # Record the lookups made while the file is imported and its description is generated.
    # Lookups inside OpaqueFunctions (run at launch time) are covered by the static scan above.
    recorded = []
    real = ament_packages.get_package_share_directory

    def recording(name, *args, **kwargs):
        recorded.append(name)
        return real(name, *args, **kwargs)

    monkeypatch.setattr(ament_packages, 'get_package_share_directory', recording)
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, 'get_package_share_directory'):
        monkeypatch.setattr(module, 'get_package_share_directory', recording)
    description = module.generate_launch_description()
    assert description is not None
    for name in recorded:
        assert name in allowed, f'{path.name} looks up unknown package {name!r}'
