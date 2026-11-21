# reachability_moveit

Stand verification for the reachability planner: confirms RM4D base stands with inverse kinematics and collision checking through the `motion_planner` ports, so a stand the map promises but IK or the scene refuses is dropped.

## Tier

ROS node or plugin: a ROS-free verifier library (`port_stand_verifier`) plus two MoveIt-backed executables.

## Inputs and outputs

- C++: `PortStandVerifier` (`include/reachability_moveit/port_stand_verifier.hpp`) implements `reachability::StandVerifier` over `motion_planner`'s `IkPort`, `CollisionPort` and `ScenePort`; any backend that implements the ports works.
- Node `reachability_verified_node`: the `reachability_node` with a verifier attached; adds `~/plan_verified_stands`, and `~/plan_stands` verifies when the request sets `verified` or the parameter `verify_by_default` is true. Needs a running `move_group`.
- Node `stand_verify_probe` (launch `launch/stand_verify_probe.launch.py`, arguments `map_file`, `wall_x`, `ik_seeds`, `extra`): measures RM4D predictions against IK and collision checks.
- Parameters of the node are those of `reachability_node` (`map_file`, `base_profile`, ...) plus `arm_mount_offset_z` and `lift_max`; see the header comment of `apps/reachability_verified_node.cpp`.

## Dependencies

- In this repo: `reachability`, `motion_planner`, `motion_planner_moveit`, `mobile_manipulator_moveit_config`; also `core_types`, `ros_adapters`, `reachability_msgs` in CMake.
- External: `Eigen3`; the two executables need `rclcpp`, `tf2_ros`, MoveIt and `octomap`.
- The verifier library and its tests do not need MoveIt. No GPU. No simulator (the probe uses the mock-hardware demo).

## Use it on its own

```bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py profile:=ur10e_telescopic rviz:=false
ros2 launch reachability_moveit stand_verify_probe.launch.py map_file:=/path/arm.rm4d wall_x:=1.4
```

## Tests

6 test cases in `test/port_stand_verifier_test.cpp` (counted from the source), over fake ports.

```bash
colcon test --packages-select reachability_moveit
```

## Status and limits

- A verified request takes minutes on a real wall (the header comment gives about 5 minutes measured on one wall).
- The scene is whatever `move_group` holds (one fixed stand's view); stand-relative obstacle posing is not wired here.
- Verified in simulation only; nothing here has run on hardware.

## Related docs

- [reachability.md](../../../docs/reachability.md)
- [moveit_config.md](../../../docs/moveit_config.md)
- [backend_selection.md](../../../docs/backend_selection.md)
