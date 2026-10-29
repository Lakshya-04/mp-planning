/**
 * @file cartesian_planner.hpp
 * @brief computeCartesianPath()-based planner for surface-following motion.
 * @details Ports the proven eef_step/jump_threshold/relaxation-retry and
 *          TOTG->IPTP time-parameterization logic from
 *          wall_cleaning_action_server.cpp, plus a scoped Allowed Collision
 *          Matrix exclusion (pushed to the real move_group process via
 *          PlanningSceneInterface::applyPlanningScene(), not just a local
 *          monitor mutation -- see cartesian_planner.cpp) and an independent
 *          wall-penetration-depth bound for tool-vs-wall contact. Plans only
 *          -- never executes; the caller hands PlanningResult::trajectory to
 *          an ExecutionPort (see execution/execution_port.hpp and
 *          Motion_Validator_Architecture.md Part A). See
 *          implementation_plan_manipulation_architecture.md §6.1/§6.3 and
 *          Cartesian_Planner_Architecture.md, Part B.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "motion_planner/core/planners/planner.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace moveit::planning_interface
{
class MoveGroupInterface;
class PlanningSceneInterface;
}  // namespace moveit::planning_interface

namespace moveit::core
{
class RobotState;
}  // namespace moveit::core

namespace planning_scene_monitor
{
class PlanningSceneMonitor;
}  // namespace planning_scene_monitor

namespace motion_planner
{

class MoveItPlanningBackend;

class CartesianPlanner : public Planner
{
public:
  /// backend must be non-null and, in production, the SAME instance injected
  /// into OmplPlanner -- that's what makes "no shared ground truth between
  /// the two planners" (the bug this constructor-injection exists to fix)
  /// structurally impossible rather than merely discouraged. See
  /// MoveItPlanningBackend's own file comment.
  explicit CartesianPlanner(std::shared_ptr<MoveItPlanningBackend> backend);

  std::string name() const override;
  PlanningResult plan(const PlanningRequest & request) override;
  IkResult ik(const IkRequest & request) override;
  CollisionResult collision(const CollisionRequest & request) override;

  /// Public so the RAII restore guard in cartesian_planner.cpp (an
  /// anonymous-namespace helper, not a member) can call it from its
  /// destructor -- see setWallContactAcmEntry's doc comment below for the
  /// full rationale on why this restore step exists at all.
  bool restoreWallContactAcmEntry(
    const std::string & tool_link,
    const std::string & target_object_id,
    bool had_previous_entry,
    bool previous_allowed,
    std::string & error_message);

private:
  /// Delegates to backend_->ensureMoveGroup(), then refreshes node_/
  /// move_group_/move_group_name_ from the backend's own accessors -- kept
  /// as cached members (rather than reaching through backend_ everywhere)
  /// so the rest of plan() doesn't need rewriting.
  bool ensureMoveGroup(const std::string & planning_group, std::string & error_message);
  /// Same pattern: delegates to backend_, refreshes planning_scene_monitor_.
  bool ensurePlanningSceneMonitor(const std::string & planning_group, std::string & error_message);
  /// Same pattern: delegates to backend_, refreshes planning_scene_interface_.
  bool ensurePlanningSceneInterface(std::string & error_message);

  /// Adds/removes a single scoped ACM entry (tool_link vs. target_object_id)
  /// and pushes the change to the real move_group process synchronously via
  /// PlanningSceneInterface::applyPlanningScene() -- a locally-mutated
  /// PlanningSceneMonitor mirror has no effect on move_group's own collision
  /// checking (computeCartesianPath() runs in that separate process), see
  /// cartesian_planner.cpp for the full rationale. previous_allowed only
  /// distinguishes ALWAYS (true) from NEVER (false); a prior CONDITIONAL
  /// entry (a predicate-based exception, not used anywhere in this repo for
  /// tool/wall pairs) would be restored as a plain NEVER/ALWAYS -- an
  /// accepted simplification, not a silent gap for the pairs this is
  /// actually used for.
  bool setWallContactAcmEntry(
    const std::string & tool_link,
    const std::string & target_object_id,
    bool allowed,
    bool & had_previous_entry,
    bool & previous_allowed,
    std::string & error_message);

  /// Synthetic model-default state (seeded with the trajectory's own first
  /// waypoint positions) -- deliberately never queries
  /// move_group_->getCurrentState(), which was found live to stall ~10s per
  /// call (and never actually succeed beyond the very first call of a run)
  /// in an environment where CurrentStateMonitor never sees a full joint set
  /// because some other part of the robot's controller stack never
  /// publishes. Same fix as ik()/collision() below. This reference state is
  /// only used as an FK/other-joint-value container when building a
  /// robot_trajectory::RobotTrajectory from an already-computed trajectory
  /// message -- not the actual planning start state, which is governed
  /// server-side by move_group's own scene and unaffected by this.
  std::shared_ptr<moveit::core::RobotState> resolveReferenceState(
    const moveit_msgs::msg::RobotTrajectory & trajectory_msg) const;

  /// Cumulative-arc-length/target-speed time parameterization (the
  /// previously dead/unimplemented "cleaning_velocity" concept, now real).
  bool applyConstantSpeedTimeParameterization(
    moveit_msgs::msg::RobotTrajectory & trajectory_msg,
    const std::string & tool_tcp_link,
    double target_eef_speed) const;
  /// TOTG -> IPTP -> IPTP-at-half-scale fallback chain, ported from
  /// wall_cleaning_action_server.cpp's applyTimeParameterization().
  bool applyFallbackTimeParameterization(moveit_msgs::msg::RobotTrajectory & trajectory_msg) const;

  std::shared_ptr<MoveItPlanningBackend> backend_;

  /// Cached views, refreshed from backend_ by ensureMoveGroup()/
  /// ensurePlanningSceneMonitor()/ensurePlanningSceneInterface() -- kept so
  /// plan()'s own ~900 lines (ACM diffs, time-parameterization, FK) don't
  /// need rewriting to reach through backend_ at every use.
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::string move_group_name_;
};

}  // namespace motion_planner
