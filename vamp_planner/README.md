# vamp_planner

VAMP (a SIMD CPU sampling-based planner) behind the `motion_planner` ports: free-space planning and collision checks. Upstream VAMP is used unmodified.

## Tier

ROS-free core library with pluginlib plugins (`vamp_planner::VampPlanner` plus three plugins). No `rclcpp` dependency.

## Inputs and outputs

- C++: `vamp_planner::VampPlanner` (`include/vamp_planner/vamp_planner.hpp`) is a free-space planner and collision checker over one compiled-in VAMP robot. `Options` has `robot` (default `ur5`; `VampPlanner::availableRobots()` lists what is compiled in), `base_frame` and planner settings. Plans joint-space goals only.
- Plugins (`plugins.xml`, library `vamp_planner_plugins`): `vamp` (planner), `vamp_collision`, `vamp_scene` (world points and boxes). Plugin parameters: `robot`, `max_iterations`, `range`, `point_radius`, `nominal_joint_speed`, `simplify`, `require_world`, `base_frame`.
- Geometry must already be in the robot base frame.
- Robot models are VAMP headers compiled in; extra robots come from `docker/tools/generate_vamp_robot.sh` and are added with `-DVAMP_PLANNER_ROBOT_DIR=...`.

## Dependencies

- In this repo: `motion_planner`, `core_types`.
- External: `Eigen3`, `pluginlib`, and VAMP itself installed at `VAMP_ROOT` (default `/opt/vamp`, built by the repo's Docker file). The SIMD flags must match the VAMP build (`VAMP_PLANNER_ARCH`, default AVX2).
- Without VAMP installed the package builds nothing and the rest of the workspace is unaffected. Runs on the CPU, no GPU. No simulator, no ROS node of its own.

## Use it on its own

Select it in a planning config:

```yaml
planning:
  free_space: vamp
  segments: {contact: cartesian, transition: free_space}
  plugin_params: {vamp: {robot: ur5}}
```

```bash
ros2 run manipulation_manager manipulation_manager_node --ros-args -p planning_config:=/path/planning.yaml
```

## Tests

12 test cases in 2 files (`vamp_planner_test.cpp`, `vamp_plugins_test.cpp`), counted from the sources.

```bash
colcon test --packages-select vamp_planner
```

## Status and limits

- VAMP plans geometry only: it is not an IK solver, a Cartesian planner or a time parameteriser; pair it with an IK backend and a time parameteriser.
- One robot per planner instance, compiled in; the build is tied to the SIMD baseline.
- Used in planner benchmarks in simulation; nothing here has run on hardware.

## Related docs

- [vamp_notes.md](../../../docs/vamp_notes.md)
- [backend_selection.md](../../../docs/backend_selection.md)
- [planner_benchmark.md](../../../docs/planner_benchmark.md)
- [spherization_benchmark.md](../../../docs/spherization_benchmark.md)
