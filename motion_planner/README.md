# motion_planner (core)

ROS-free motion planning core. No rclcpp, MoveIt, tf2 or message headers.

- **Ports:** `Planner` (plan + ik + collision), `IkPort`, `CollisionPort`, `ScenePort`
  (world geometry: points, named boxes) in `include/motion_planner/core/`.
- **Types:** `PlanningRequest/Result`, `IkRequest/Result`, `CollisionRequest/Result`
  (`planning_types.hpp`), built on `core_types::Pose` / `Trajectory`.
- **`MotionPlanner`:** segment planning with Cartesian-first routing, free-space fallback and
  bridge-resume after a partial failure, over any `Planner`.
- **Time parameterisation** (`time_parameterization.hpp`): `TimeParameterizer` port plus
  `parameterizeSegment`. A contact seam runs at the tool's stroke speed, jerk limited, and a limit
  violation fails it; a transition is time-optimal and violations are only reported. Near-duplicate
  points are dropped and the path is split where the joint-space direction changes by more than 150
  degrees (each piece rests at the cut). Bad input (zero acceleration limit, NaN, joint mismatch, too
  few points) gives a typed `TimingError`. The MoveIt adapter (`MoveItTimeParameterizer`) uses TOTG
  and Ruckig; the manager enables it with `retime:=true` (off by default).
- **Geometry helpers:** wall penetration check, reachability tilt.
- **`motion_planner_pcd`** (optional, needs PCL): `.pcd` / `.ply` loading into a `ScenePort` update.

Backends implement the ports in their own adapter packages. Today that is
[`motion_planner_moveit`](../motion_planner_moveit); cuRobo v2 and VAMP adapters are planned
(see `docs/architecture.md`).

Layout and rules: `docs/package_layout.md`.
