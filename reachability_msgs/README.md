# reachability_msgs

Reachability services: forward query, base candidates, lift-band planning and multi-stand planning.

## Tier

Message package (`rosidl` interfaces only, no code).

## Inputs and outputs

- Services: `IsReachable` (tool pose in the arm base frame), `GetBaseCandidates` (where can the base stand for a world tool pose), `PlanLiftBands` (fewest lift heights that reach all targets from a base station), `PlanStands` (base stands that cover a wall's targets, in visiting order, with an optional `verified` flag).
- Messages: `BaseCandidate`, `LiftBand`, `Stand` (arm base position, yaw, standoff, footprint clearance, travel time, reached target indices, lift bands).

## Dependencies

- In this repo: none.
- External: `rosidl_default_generators`, `rosidl_default_runtime`, `geometry_msgs`.
- Needs ROS 2 to generate the interfaces. No GPU. No simulator.

## Use it on its own

```xml
<depend>reachability_msgs</depend>
```

```bash
colcon build --packages-select reachability_msgs
ros2 interface show reachability_msgs/srv/PlanStands
```

## Tests

No tests: the package only holds interface definitions. Servers and clients that use it carry the tests (`reachability` (server), `manipulation_manager` (client)).

## Status and limits

- Field semantics are defined by the comments in the `.srv` files; zero values often select a default.
- Nothing here has run on hardware.

## Related docs

- [reachability.md](../../../docs/reachability.md)
- [stand_navigation.md](../../../docs/stand_navigation.md)
