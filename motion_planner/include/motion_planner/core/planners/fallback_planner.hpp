#pragma once

#include <memory>
#include <string>
#include <utility>

#include "motion_planner/core/planners/planner.hpp"

namespace motion_planner
{

/**
 * @brief Two planners behind one Planner: `primary` first, and when it does not succeed `secondary` plans the same
 *        request. IK and collision queries go to `primary` only. name() is the backend that produced the last plan
 *        (the secondary's name after a fallback), so the manager's `fsp_backend=` log field tells which one worked.
 *        Used for a free-space role such as cuRobo with MoveIt/OMPL behind it: a primary that is unavailable or finds
 *        no path no longer stops a mission.
 */
class FallbackPlanner : public Planner
{
public:
  FallbackPlanner(std::unique_ptr<Planner> primary, std::unique_ptr<Planner> secondary)
  : primary_(std::move(primary)), secondary_(std::move(secondary)) {}

  std::string name() const override {return last_used_;}

  PlanningResult plan(const PlanningRequest & request) override
  {
    last_used_ = primary_->name();
    PlanningResult result = primary_->plan(request);
    if (result.success) {
      return result;
    }
    PlanningResult second = secondary_->plan(request);
    if (!second.success) {
      // keep the primary's reason: it is the planner the operator configured
      result.message += " (fallback " + secondary_->name() + ": " + second.message + ")";
      return result;
    }
    last_used_ = secondary_->name();
    second.planning_time_ms += result.planning_time_ms;
    second.message = "fallback " + secondary_->name() + " after " + primary_->name() + " [" +
      result.failure_reason_code + "] " + result.message + (second.message.empty() ? "" : "; " + second.message);
    return second;
  }

  IkResult ik(const IkRequest & request) override {return primary_->ik(request);}
  CollisionResult collision(const CollisionRequest & request) override {return primary_->collision(request);}

private:
  std::unique_ptr<Planner> primary_;
  std::unique_ptr<Planner> secondary_;
  std::string last_used_;
};

}  // namespace motion_planner
