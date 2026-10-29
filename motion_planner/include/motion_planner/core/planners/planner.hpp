#pragma once

#include <string>

#include "motion_planner/core/planners/planning_types.hpp"

namespace motion_planner
{

class Planner
{
public:
  virtual ~Planner() = default;
  virtual std::string name() const = 0;

  /**
   * @brief Plan a trajectory for the requested segment. Plans only -- never
   *        executes on the robot. The caller (ManipulationManager) owns
   *        execution, handing the returned PlanningResult::trajectory to an
   *        ExecutionPort (see execution/execution_port.hpp). This is the
   *        plan/execute decoupling described in
   *        implementation_plan_manipulation_architecture.md §6.1/§6.5.
   */
  virtual PlanningResult plan(const PlanningRequest & request) = 0;
  virtual IkResult ik(const IkRequest & request) = 0;
  virtual CollisionResult collision(const CollisionRequest & request) = 0;
};

}  // namespace motion_planner
