# mp-planning

Planning for the Motion-Planner stack: motion planners (MoveIt/OMPL, cuRobo, VAMP), reachability maps and base placement, stand navigation, the MoveIt config.

This repository is a submodule of [Lakshya-04/Motion-Planner](https://github.com/Lakshya-04/Motion-Planner), which holds the core packages, the Docker setup, the documentation and the contributing guide. It is checked out at `src/mp-planning` and built together with the rest of the workspace (colcon); the packages depend on packages in the main repository and in the other `mp-*` repositories, so it does not build on its own.

Packages: motion_planner, motion_planner_moveit, curobo_planner, curobo_bridge, curobo_bridge_msgs, vamp_planner, reachability, reachability_moveit, reachability_msgs, stand_navigator, stand_navigator_msgs, mobile_manipulator_moveit_config.

Issues, pull requests and the contributing rules (Conventional Commits, one theme per PR, squash merge) are in the main repository: open them there and say which package is concerned.

Licence: Apache-2.0 (see `LICENSE`).
