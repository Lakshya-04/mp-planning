# curobo_planner

C++ client that implements the `motion_planner` planner, IK, collision and scene ports by calling the cuRobo node's services (`curobo_bridge`), so the planning layer can select cuRobo like any other backend.

## Tier

ROS node or plugin (pluginlib plugins over a C++ client library).

## Inputs and outputs

- C++: `curobo_planner::CuroboClientPlanner` (a `motion_planner::Planner` and `ScenePort`), `Options` (`node_name` default `/curobo_planner`, `wait_for_service_sec`, `timeout_sec` default 45, `base_frame`, `joint_limits`), joint limit guard (`joint_limit_guard.hpp`).
- Plugins (`plugins.xml`, library `curobo_planner_plugins`): `curobo` (planner), `curobo_ik`, `curobo_collision`, `curobo_scene`. They are selected by name in the planning config of the manager.
- Calls the services of `curobo_bridge_msgs` on the cuRobo node.
- Executables: `curobo_planner_benchmark`, `curobo_world_loader`.

## Dependencies

- In this repo: `motion_planner`, `core_types`, `scene_model`, `curobo_bridge_msgs`.
- External: `rclcpp`, `pluginlib`, `tf2_ros`, `Eigen3`, `yaml-cpp`.
- Needs ROS 2. Builds and tests without a GPU (it is a client); planning needs the cuRobo node running on one. No simulator needed.

## Use it on its own

Run the cuRobo node (see `curobo_bridge`), then select the plugin in a planning config:

```yaml
planning:
  free_space: curobo
  plugin_params:
    curobo: {node_name: /curobo_planner, timeout_sec: 45}
```

```bash
ros2 run manipulation_manager manipulation_manager_node --ros-args -p planning_config:=/path/planning.yaml
```

`manipulation_manager/config/planning_curobo.yaml` is a shipped example.

## Tests

36 test cases in 3 files (counted from the sources): the client over a fake service, the joint limit guard and the plugin registration.

```bash
colcon test --packages-select curobo_planner
```

## Status and limits

- `curobo_collision` takes joint-space inputs only; the backend doc lists what each plugin supports.
- A result with status `success_graph_seed` is a validated but not optimised path.
- Falling back to another planner when cuRobo fails is unit-tested with fakes only. Nothing here has run on hardware.

## Related docs

- [backend_selection.md](../../../docs/backend_selection.md)
- [curobo_v2_notes.md](../../../docs/curobo_v2_notes.md)
- [planner_benchmark.md](../../../docs/planner_benchmark.md)
