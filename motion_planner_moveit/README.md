# motion_planner_moveit

MoveIt / OMPL backend for the `motion_planner` ports. The only package that depends on MoveIt.

- `MoveItPlanningBackend`: owns the `MoveGroupInterface` / planning scene access.
- `CartesianPlanner`: Cartesian segment planning with jump / collision diagnosis and recovery.
- `OmplPlanner`: free-space planning.
- `MoveItSceneAdapter`: implements `ScenePort` (points become one octomap, boxes become named
  collision objects).
- Nodes: `world_model_loader_node` (load a point cloud into the planning scene),
  `coverage_pass_demo_node`.

The demo launch expects a robot description package for your robot to be available in your own
workspace; none is bundled here.
