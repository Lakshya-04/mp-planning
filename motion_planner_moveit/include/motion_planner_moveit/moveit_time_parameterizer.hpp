/**
 * @file moveit_time_parameterizer.hpp
 * @brief motion_planner::TimeParameterizer on MoveIt: TOTG for transitions, constant tool speed
 *        plus Ruckig smoothing for contact seams.
 * @details Reuses MoveIt's time_optimal_trajectory_generation and ruckig_traj_smoothing; nothing
 *          is hand-rolled except the constant-speed stamps (core, by tool path length through FK).
 *          Joint limits come from the caller (JointLimit), not from the URDF, so a config can be
 *          tighter than the model; modelLimits() reads the model's bounds as a default.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.hpp>

#include "motion_planner/core/time_parameterization.hpp"

namespace motion_planner
{

class MoveItTimeParameterizer : public TimeParameterizer
{
public:
  /// tcp_link empty: the last link of the group (the tool tip) is used for the seam speed.
  MoveItTimeParameterizer(
    std::shared_ptr<const moveit::core::RobotModel> model, std::string group,
    std::string tcp_link = "");

  bool retime(
    core_types::Trajectory & piece, SegmentRole role, const std::vector<JointLimit> & limits,
    double stroke_speed, std::string & message) override;

  /// Active joint names of the group, the order modelLimits() uses.
  std::vector<std::string> jointNames() const;

  /// Velocity from the model bounds; acceleration from the bounds when set, else default_acc;
  /// jerk as given (0 = unbounded).
  std::vector<JointLimit> modelLimits(double default_acc, double jerk) const;

private:
  std::shared_ptr<const moveit::core::RobotModel> model_;
  std::string group_;
  std::string tcp_link_;
};

}  // namespace motion_planner
