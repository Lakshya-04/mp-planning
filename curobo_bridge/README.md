# curobo_bridge

cuRobo v2 (GPU trajectory optimisation) behind a ROS-free Python planner that returns the same structured results as the `motion_planner` ports, plus a ROS 2 node exposing it as services. Upstream cuRobo is used unmodified.

## Tier

ROS node (Python) over a ROS-free planner class (`curobo_bridge.planner.CuroboPlanner`).

## Inputs and outputs

- Python: `CuroboPlanner`, `PlanResult`, `IkResult`, `Box`, `EsdfSpec`, `voxelize_points`, `points_to_esdf`, `resolve_ik_weights`, `closest_solution` in `curobo_bridge/planner.py`.
- Node `curobo_planner_node` (`curobo_bridge.ros_node`), services under the node name, types from `curobo_bridge_msgs`: `~/plan_joint`, `~/plan_pose`, `~/set_world`, `~/set_esdf`, `~/solve_ik`, `~/check_motion`. Until the planner has warmed up (CUDA kernels compile on first start) the services answer `unavailable`.
- Parameters: `robot` (a cuRobo robot config: a bundled name or the path of one made by `docker/tools/generate_curobo_robot.py`), `max_boxes`, `esdf_dims`, `esdf_voxel_size`, `esdf_center`, `prismatic_joints`, `ik_prismatic_factor`, `ik_weights`, `seed_fallback`, `fallback_attempts`, `max_attempts`.
- Everything is in the robot base frame.

## Dependencies

- In this repo: `curobo_bridge_msgs`.
- External: `rclpy`, `sensor_msgs_py`, `trajectory_msgs`, `python3-numpy`; cuRobo v2 and PyTorch with CUDA (provided by the repo's `gpu` Docker image).
- Needs a GPU to plan (the unit tests that do not touch cuRobo run without one). Needs ROS only for the node. No simulator.

## Use it on its own

Run the node with the Python that has torch and cuRobo, not the system Python that `ros2 run` picks:

```bash
python -m curobo_bridge.ros_node --ros-args -p robot:=/path/robot.yml
```

A C++ client that plugs this into the planning layer is `curobo_planner`.

## Tests

56 test cases in 5 files (counted from the sources). Tests that need cuRobo or a CUDA device skip themselves when it is absent; the node tests skip without `rclpy`.

```bash
colcon test --packages-select curobo_bridge
```

## Status and limits

- `route` in a result tells how a path was made: `trajopt` (optimised) or `graph_seed` (a validated graph path that was not optimised).
- The node is single-threaded; a client that times out leaves the abandoned plan running and later calls queue behind it.
- Joint limits come from the cuRobo robot config, not from MoveIt's `joint_limits.yaml`.
- Timings and success rates were measured in simulation setups only; nothing here has run on hardware.

## Related docs

- [curobo_v2_notes.md](../../../docs/curobo_v2_notes.md)
- [backend_selection.md](../../../docs/backend_selection.md)
- [planner_benchmark.md](../../../docs/planner_benchmark.md)
- [docker.md](../../../docs/docker.md)
