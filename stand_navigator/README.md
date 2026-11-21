# stand_navigator

Drives the mobile base to a stand chosen by the reachability planner, and servos it to a standoff and yaw against the fitted wall plane. A small stand-in until Nav2 is used.

## Tier

ROS node (Python, `rclpy`) with a ROS-free core (`core.py`, `wall_docking.py`).

## Inputs and outputs

- Node `stand_navigator_node`: action server `navigate_to_stand` (`stand_navigator_msgs/NavigateToStand`); subscribes `odom_topic` (default `odom`); publishes `geometry_msgs/Twist` on `cmd_vel_topic` (default `cmd_vel`). Parameters: `base_profile` (a profile YAML with a `base:` section, required), `tolerance_xy_m`, `tolerance_yaw_rad`, `timeout_s`, `rate_hz`, `odom_timeout_s`.
- Node `wall_docking_node`: action server `wall_dock` (`WallDock`); also subscribes the wall scene (`wall_scene_topic`, default `/wall_detector/wall_scene`, `std_msgs/String`, transient local). Parameters include `base_profile`, `ref_xy`, `facing_rad`, `tolerance_standoff_m`, `tolerance_yaw_rad`, `max_linear_mps`, `max_angular_radps`, `plane_max_age_s`.
- ROS-free API: `core.py` (`BaseProfile`, `load_profile`, `arm_to_footprint`, `footprint_to_arm`, `StandController`), `wall_docking.py` (`measure_wall`, `WallDockController`, optional lateral term `LateralConfig` with `PlannedStandLateral` / `WallEdgeLateral`, typed `DockOutcome`, `DockResult`, `plan_lateral_manoeuvre`), `dock_montecarlo.py` (noise Monte Carlo of the controller logic).
- Controller: speed and acceleration limited, for omni and differential bases (profile `base.drive_type`).

## Dependencies

- In this repo: `stand_navigator_msgs`.
- External: `rclpy`, `geometry_msgs`, `nav_msgs`, `python3-yaml`; `pytest` for tests.
- The core runs without ROS (plain Python). No GPU. No simulator needed (the simulations supply `/odom` and take `/cmd_vel`).

## Use it on its own

```bash
ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=/path/profile.yaml
ros2 run stand_navigator wall_docking_node --ros-args -p base_profile:=/path/profile.yaml
```

`/path/profile.yaml` is any robot profile that has a `base:` section (see `mobile_manipulator_description/config/profiles`).

## Tests

`test/test_core.py`, `test/test_wall_docking.py`, `test/test_wall_docking_lateral.py` and `test/test_dock_montecarlo.py` (95 tests in the last host run).

```bash
colcon test --packages-select stand_navigator
```

## Status and limits

- Open-loop on `/odom`: no obstacle avoidance, no map; the goal frame must be the odometry frame (no frame transform).
- Exercised in the MuJoCo simulation (`run_stand_navigation.sh`, `run_wall_docking.sh`); nothing here has run on hardware.
- Wheels are not modelled in the simulations; the base is a planar velocity-commanded body there.

## Dock Monte Carlo

```bash
python3 -m stand_navigator.dock_montecarlo --runs 200 --seed 1 [--base diff] [--lateral off|planned|edge|all] [--sweep]
```

Runs the approach and the dock cores on a kinematic base with injected odometry error and plane noise, and prints the docked rate, failure reasons and the true final error distribution (mean, p50, p95, max). It checks the controller logic against synthetic noise, not a simulator or hardware; how to read it and a sample run are in [stand_navigation.md](../../../docs/stand_navigation.md). The lateral term and the typed outcomes are in the core only (the `WallDock` action does not carry them yet).

## Related docs

- [stand_navigation.md](../../../docs/stand_navigation.md)
- [reachability.md](../../../docs/reachability.md)
- [sim_mujoco.md](../../../docs/sim_mujoco.md)
