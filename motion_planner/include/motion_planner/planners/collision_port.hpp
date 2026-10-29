/**
 * @file collision_port.hpp
 * @brief Port for collision-checking, decoupled from any specific backend.
 * @details Decomposition of Planner::collision() (planner.hpp) into its own
 *          interface so CartesianPlanner and OmplPlanner can share one
 *          injected implementation instead of each independently owning a
 *          MoveIt PlanningSceneMonitor -- see MoveItPlanningBackend, the
 *          current (and, at time of writing, only) implementation. A future
 *          non-MoveIt backend (e.g. cuRobo) implements this same interface;
 *          nothing about this port is MoveIt-specific.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include "motion_planner/planners/planning_types.hpp"

namespace motion_planner
{

class CollisionPort
{
public:
  virtual ~CollisionPort() = default;
  virtual CollisionResult collision(const CollisionRequest & request) = 0;
};

}  // namespace motion_planner
