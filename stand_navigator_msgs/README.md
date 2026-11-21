# stand_navigator_msgs

Two base-motion actions: `NavigateToStand` (drive the arm base to a stand, shaped like Nav2's `NavigateToPose`) and `WallDock` (servo to a standoff and yaw against the fitted wall plane).

## Tier

Message package (`rosidl` interfaces only, no code).

## Inputs and outputs

- Action `NavigateToStand`: goal is a `geometry_msgs/PoseStamped` in the odometry frame (the arm base position and the base yaw); the result carries an error code (`NONE`, `UNKNOWN`, `INVALID_GOAL`, `TIMEOUT`); feedback gives the distance remaining.
- Action `WallDock`: goal `standoff_m` and `timeout_s` (0 uses the node default); result with an error code.

## Dependencies

- In this repo: none.
- External: `rosidl_default_generators`, `rosidl_default_runtime`, `geometry_msgs`.
- Needs ROS 2 to generate the interfaces. No GPU. No simulator.

## Use it on its own

```xml
<depend>stand_navigator_msgs</depend>
```

```bash
colcon build --packages-select stand_navigator_msgs
ros2 interface show stand_navigator_msgs/action/NavigateToStand
```

## Tests

No tests: the package only holds interface definitions. Servers and clients that use it carry the tests (`stand_navigator` (server) and `manipulation_manager` (client)).

## Status and limits

- `NavigateToStand` is shaped so a Nav2 bridge could replace the server; no such bridge exists in this repo.
- Nothing here has run on hardware.

## Related docs

- [stand_navigation.md](../../../docs/stand_navigation.md)
- [reachability.md](../../../docs/reachability.md)
