# curobo_bridge_msgs

Services of the cuRobo planner node: joint and pose planning, inverse kinematics, motion checks, and world updates.

## Tier

Message package (`rosidl` interfaces only, no code).

## Inputs and outputs

- Services: `PlanJoint`, `PlanPose` (a collision-free move from a joint state to a tool pose; the reply carries `success`, `failure_reason_code`, `route`, a `JointTrajectory` and `planning_time_s`), `SolveIk`, `CheckMotion` (validity of joint states, optional sampled motion check), `SetWorld` (boxes and a point cloud), `SetEsdf`.
- Message `Box`: an oriented box obstacle (`id`, `pose`, `size`) in the robot base frame.

## Dependencies

- In this repo: none.
- External: `rosidl_default_generators`, `rosidl_default_runtime`, `geometry_msgs`, `sensor_msgs`, `trajectory_msgs`.
- Needs ROS 2 to generate the interfaces. No GPU. No simulator.

## Use it on its own

```xml
<depend>curobo_bridge_msgs</depend>
```

```bash
colcon build --packages-select curobo_bridge_msgs
ros2 interface show curobo_bridge_msgs/srv/PlanPose
```

## Tests

No tests: the package only holds interface definitions. Servers and clients that use it carry the tests (`curobo_bridge` (server) and `curobo_planner` (client)).

## Status and limits

- Everything is in the robot base frame; the server decides what it can do (see `curobo_bridge`).
- Nothing here has run on hardware.

## Related docs

- [curobo_v2_notes.md](../../../docs/curobo_v2_notes.md)
- [backend_selection.md](../../../docs/backend_selection.md)
