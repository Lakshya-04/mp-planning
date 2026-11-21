# mobile_manipulator_moveit_config

MoveIt 2 configuration for `mobile_manipulator_description`: planning groups `arm` (N DOF) and `lift_arm` (1 + N DOF) named by the robot profile, KDL / trac_ik / pick_ik kinematics, OMPL, joint limits, and a mock-hardware demo with ros2_control.

## Tier

ROS node or plugin: configuration files, a Python builder and launch files (no compiled code).

## Inputs and outputs

- Python API `mobile_manipulator_moveit_config` (`builder.py`): `build_moveit_config(arm, hardware, ik, profile, live_octomap)`, `resolve_profile`, `arm_family`, `hardware_for(controller)`, `controllers_file(arm, controller, profile)`. Other packages' launch files use it to get the same MoveIt parameters as `move_group`.
- Launch `launch/demo.launch.py`; arguments `profile`, `arm`, `controller` (`trajectory` default, or `jtac`), `ik` (`trac_ik` default, `kdl`, `pick_ik`), `live_octomap`, `rviz`. It starts `robot_state_publisher`, ros2_control (mock hardware), the joint state broadcaster, one trajectory controller named `lift_arm_controller`, `move_group` and optionally RViz.
- Config: `config/` (`ompl_planning.yaml`, `kinematics*.yaml`, `joint_limits.yaml`, `sensors_3d.yaml`) and one folder per controller family (`ur`, `generic_6dof`, `arm7`, `panda`) with `ros2_controllers.yaml`, `moveit_controllers.yaml`, `jtac_controllers.yaml`.
- Scripts: `scripts/smoke_plan.py`, `scripts/sample_collisions.py`.

## Dependencies

- In this repo: `mobile_manipulator_description`, `compliant_trajectory_controller`, `ft_bridge_hardware` (the last two for `controller:=jtac`); `robot_self_filter` is a test dependency.
- External: MoveIt 2 (`moveit_ros_move_group`, `moveit_configs_utils`, `moveit_planners_ompl`, `moveit_kinematics`), `trac_ik_kinematics_plugin`, `pick_ik`, `controller_manager`, `joint_trajectory_controller`, `rviz2`, `xacro`.
- Needs ROS 2 and MoveIt. No GPU. No simulator (mock hardware).

## Use it on its own

```bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py profile:=ur10e_telescopic rviz:=false
ros2 launch mobile_manipulator_moveit_config demo.launch.py arm:=generic_6dof controller:=jtac ik:=kdl
```

For your arm: add a profile in `mobile_manipulator_description`, then pass `profile:=<name>`.

## Tests

7 test cases in `test/` (counted from the sources; the file `fk_ik_check.cpp` is a check program, not a gtest).

```bash
colcon test --packages-select mobile_manipulator_moveit_config
```

## Status and limits

- Verified on mock hardware and in the simulations through the demo, a plan smoke test and a collision-matrix sampling script. Nothing here has run on hardware.
- Collision matrices are generated per arm; a new arm needs its own generated matrix.
- Joint limit scaling values are conservative starting points, not tuned.

## Related docs

- [moveit_config.md](../../../docs/moveit_config.md)
- [robot_description.md](../../../docs/robot_description.md)
- [backend_selection.md](../../../docs/backend_selection.md)
