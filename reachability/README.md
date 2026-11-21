# reachability

RM4D reachability and inverse-reachability map for a serial arm: build the map from an arm chain, answer forward and inverse queries, plan lift bands and base stands for a wall, read and write map files, with a command-line tool and an optional ROS service node.

## Tier

ROS-free core library (`reachability_core`, `rm4d_tool`, `column_reach_tool`) plus an optional ROS node, built only when `rclcpp` and `reachability_msgs` are found.

## Inputs and outputs

- Core headers under `include/reachability/core/`: `arm_model`, `builder`, `map4d`, `io`, `lift_bands`, `station`, `stands`, `global_stands` (`selectStands`), `horizontal_stands`, `column_stands`, `stand_verifier` (the `StandVerifier` port you can implement), `base_model`, `redundancy`, `pose_quality`.
- Tool: `rm4d_tool build chain.yaml out.rm4d`, `info`, `bands`, `station` (see the header comment of `apps/rm4d_tool.cpp` for the options); `column_reach_tool`.
- Chain input: a chain YAML written from a URDF by `scripts/urdf_to_chain.py`.
- ROS node `reachability_node` (launch `launch/reachability.launch.py`): parameters `map_file`, `robot_fingerprint`, `base_profile`, `verify_by_default`, `stand_strategy`; services `~/is_reachable`, `~/get_base_candidates`, `~/plan_lift_bands`, `~/plan_stands` (`reachability_msgs`).

## Dependencies

- In this repo: `reachability_msgs` (ROS node only).
- External: `Eigen3`, `yaml-cpp`, `Threads`; node: `rclcpp`; chain script: `python3-numpy`, `urdfdom_py`, `python3-yaml`.
- The core and the tools build and run without ROS sourced in the `reachability_core` target; the package is an `ament_cmake` package. No GPU. No simulator.

## Use it on its own

```bash
xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e -o /tmp/robot.urdf     # in mobile_manipulator_description
urdf_to_chain.py /tmp/robot.urdf --base arm_base_link --tip tcp_link -o chain.yaml
rm4d_tool build chain.yaml arm.rm4d --samples 3000000 --resolution 0.05 --extent 2.0 --zmin -1.8 --zmax 2.0
rm4d_tool bands arm.rm4d --wall-x 0.8 --z-min 0.1 --z-max 3.0 --lift-max 1.5 --mount-z 0.2
ros2 launch reachability reachability.launch.py map_file:=arm.rm4d
```

## Tests

167 test cases in 16 files (counted from the sources): map, builder, I/O, lift bands, stationing, stands, column sequences, verifier and the node.

```bash
colcon test --packages-select reachability
```

## Status and limits

- The built-in chain does not check self-collision; confirm stands with a `StandVerifier` (see `reachability_moveit`).
- The reduction assumes the first joint can turn a full turn and the last joint rotates fully (checked for the UR family; check a new arm against brute force).
- Sampling density matters: too few samples leave empty cells.
- Not done: RViz slices, orientation-constrained wall queries. Verified in simulation and unit tests only; nothing here has run on hardware.

## Related docs

- [reachability.md](../../../docs/reachability.md)
- [stand_navigation.md](../../../docs/stand_navigation.md)
- [robot_assets.md](../../../docs/robot_assets.md)
