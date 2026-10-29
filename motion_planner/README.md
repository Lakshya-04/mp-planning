# motion_planner (core)

ROS-free motion planning core. No rclcpp, MoveIt, tf2 or message headers.

- **Ports:** `Planner` (plan + ik + collision), `IkPort`, `CollisionPort`, `ScenePort`
  (world geometry: points, named boxes) in `include/motion_planner/core/`.
- **Types:** `PlanningRequest/Result`, `IkRequest/Result`, `CollisionRequest/Result`
  (`planning_types.hpp`), built on `core_types::Pose` / `Trajectory`.
- **`MotionPlanner`:** segment planning with Cartesian-first routing, free-space fallback and
  bridge-resume after a partial failure, over any `Planner`.
- **Geometry helpers:** wall penetration check, reachability tilt.
- **`motion_planner_pcd`** (optional, needs PCL): `.pcd` / `.ply` loading into a `ScenePort` update.

Backends implement the ports in their own adapter packages. Today that is
[`motion_planner_moveit`](../motion_planner_moveit); cuRobo v2 and VAMP adapters are planned
(see `docs/architecture.md`).

Layout and rules: `docs/package_layout.md`.
