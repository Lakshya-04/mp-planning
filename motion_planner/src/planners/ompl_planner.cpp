#include "motion_planner/planners/ompl_planner.hpp"

#include <chrono>
#include <exception>
#include <limits>

#include <Eigen/Geometry>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "rclcpp/rclcpp.hpp"

#include "motion_planner/moveit_planning_backend.hpp"
#include "motion_planner/planners/wall_penetration_check.hpp"
#include "ros_adapters/ros_conversions.hpp"

namespace motion_planner
{

OmplPlanner::OmplPlanner(std::shared_ptr<MoveItPlanningBackend> backend)
: backend_(std::move(backend))
{
}

std::string OmplPlanner::name() const
{
  return "ompl";
}

bool OmplPlanner::ensureMoveGroup(const std::string & planning_group, std::string & error_message)
{
  if (!backend_->ensureMoveGroup(planning_group, error_message)) {
    return false;
  }
  node_ = backend_->node();
  move_group_ = backend_->moveGroup();
  move_group_name_ = backend_->activeMoveGroupName();
  return true;
}

PlanningResult OmplPlanner::plan(const PlanningRequest & request)
{
  const auto planning_start = std::chrono::steady_clock::now();

  if (request.target_poses.empty() && request.target_joint_state.empty()) {
    PlanningResult invalid_request;
    invalid_request.success = false;
    invalid_request.status = "failed";
    invalid_request.message =
      "OMPL planning requires target_poses or target_joint_state; received neither.";
    invalid_request.waypoints_completed = 0;
    invalid_request.failure_reason_code = "missing_target_poses";
    invalid_request.failure_stage = FailureStage::kPrecheck;
    invalid_request.planning_time_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() -
        planning_start)
      .count());
    return invalid_request;
  }

  std::string initialization_error;
  if (!ensureMoveGroup(request.planning_group, initialization_error)) {
    PlanningResult init_failure;
    init_failure.success = false;
    init_failure.status = "failed";
    init_failure.message = "Failed to initialize OMPL MoveGroup: " + initialization_error;
    init_failure.waypoints_completed = 0;
    init_failure.failure_reason_code = "planner_init";
    init_failure.failure_stage = FailureStage::kPrecheck;
    init_failure.planning_time_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() -
        planning_start)
      .count());
    return init_failure;
  }

  PlanningResult result;

  // Joint-space goal takes priority when given -- it isn't a Cartesian-path
  // concept, so there's no pose to fall back to reading here anyway.
  bool used_joint_target = false;
  if (!request.target_joint_state.empty()) {
    const auto robot_model = move_group_->getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
    if (joint_model_group &&
      request.target_joint_state.size() == joint_model_group->getVariableCount())
    {
      move_group_->setJointValueTarget(request.target_joint_state);
      used_joint_target = true;
    } else {
      PlanningResult invalid_target;
      invalid_target.success = false;
      invalid_target.status = "failed";
      invalid_target.message =
        "OmplPlanner: target_joint_state size mismatch for planning group '" +
        move_group_name_ + "'.";
      invalid_target.failure_reason_code = "invalid_target_joint_state";
      invalid_target.failure_stage = FailureStage::kPrecheck;
      invalid_target.planning_time_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() -
          planning_start)
        .count());
      return invalid_target;
    }
  } else {
    auto goal_pose = ros_adapters::toRos(request.target_poses.back());
    // goal_pose is expressed in request.frame_id -- resolve into the planning frame before
    // handing it to MoveGroupInterface, same fix as CartesianPlanner::plan() (see its own
    // comment for the confirmed-live gap this closes).
    if (!request.frame_id.empty()) {
      geometry_msgs::msg::Pose resolved_pose;
      std::string resolve_error;
      if (!backend_->resolvePoseToPlanningFrame(
          request.frame_id, goal_pose, resolved_pose, resolve_error))
      {
        PlanningResult frame_failure;
        frame_failure.success = false;
        frame_failure.status = "failed";
        frame_failure.message = "Failed to resolve target pose from frame '" + request.frame_id +
          "' into the planning frame: " + resolve_error;
        frame_failure.failure_reason_code = "frame_resolution_failed";
        frame_failure.failure_stage = FailureStage::kPrecheck;
        frame_failure.planning_time_ms = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() -
            planning_start)
          .count());
        return frame_failure;
      }
      goal_pose = resolved_pose;
    }
    move_group_->setPoseTarget(goal_pose);
  }

  bool used_explicit_start_state = false;
  if (!request.start_joint_state.empty()) {
    const auto robot_model = move_group_->getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
    if (joint_model_group &&
      request.start_joint_state.size() == joint_model_group->getVariableCount())
    {
      moveit::core::RobotState start_state(robot_model);
      start_state.setToDefaultValues();
      start_state.setJointGroupPositions(joint_model_group, request.start_joint_state);
      start_state.update();
      move_group_->setStartState(start_state);
      used_explicit_start_state = true;
    } else if (node_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "OmplPlanner: request.start_joint_state size mismatch (got %zu); "
        "falling back to current state.",
        request.start_joint_state.size());
    }
  }
  if (!used_explicit_start_state) {
    move_group_->setStartStateToCurrentState();
  }

  moveit::planning_interface::MoveGroupInterface::Plan moveit_plan;
  const auto plan_result = move_group_->plan(moveit_plan);

  result.success = static_cast<bool>(plan_result);
  result.status = result.success ? "planned" : "failed";
  result.message = result.success ? "OMPL plan succeeded." : "OMPL plan failed.";
  // A joint-target segment has no waypoint_indices to count (there's no pose
  // array it indexes into) -- treat reaching the target configuration as one
  // unit of work, same as any single-pose segment.
  result.waypoints_completed = result.success ?
    (used_joint_target ? 1 : request.waypoint_indices.size()) : 0;
  result.path_fraction = result.success ? 1.0 : 0.0;
  result.failure_reason_code = result.success ? "none" : "planning_failed";
  result.failure_stage = result.success ? FailureStage::kNone : FailureStage::kPlanning;
  if (result.success) {
    result.trajectory = ros_adapters::toCore(moveit_plan.trajectory.joint_trajectory);
  }

  // Independent, ACM-blind wall-penetration bound -- FSP is a transit mode
  // with no legitimate reason to be near the wall, so its tolerance should
  // be near-zero. move_group_->plan() is a service call handled entirely
  // inside the separate move_group process; there is no per-state validity
  // callback available from this client, so this is a post-hoc check over
  // the planned trajectory (the same mechanism CartesianPlanner uses,
  // applied here for consistency and because it's the only one actually
  // reachable from this API) rather than something OMPL's own search can
  // route around mid-plan. See Cartesian_Planner_Architecture.md, Part B.5.
  // wall_reference_pose is expressed in request.frame_id too -- resolve it into the planning
  // frame here (once), same fix as CartesianPlanner::plan().
  core_types::Pose wall_reference_pose = request.wall_reference_pose;
  if (result.success && !request.frame_id.empty() && hasWallReference(wall_reference_pose)) {
    geometry_msgs::msg::Pose resolved_wall_pose;
    std::string resolve_error;
    if (backend_->resolvePoseToPlanningFrame(
        request.frame_id, ros_adapters::toRos(wall_reference_pose), resolved_wall_pose,
        resolve_error))
    {
      wall_reference_pose = ros_adapters::toCore(resolved_wall_pose);
    } else {
      result.success = false;
      result.status = "failed";
      result.message = "Failed to resolve wall_reference_pose from frame '" + request.frame_id +
        "' into the planning frame: " + resolve_error;
      result.failure_reason_code = "frame_resolution_failed";
      result.failure_stage = FailureStage::kPrecheck;
    }
  }

  if (result.success && hasWallReference(wall_reference_pose)) {
    // Synthetic model-default state (seeded with the trajectory's own first
    // waypoint) -- deliberately never queries move_group_->getCurrentState(),
    // which was found live to stall ~10s per call (and never actually
    // succeed beyond the very first call of a run) in an environment where
    // CurrentStateMonitor never sees a full joint set because some other
    // part of the robot's controller stack never publishes. Same fix as
    // CartesianPlanner::resolveReferenceState() -- this state is only an
    // FK/other-joint-value container for reading back the already-planned
    // trajectory below, not the actual planning start (governed
    // server-side), so it always fell through to this exact synthetic state
    // anyway; skipping the query outright changes nothing about the result,
    // only removes the wasted wait.
    std::shared_ptr<moveit::core::RobotState> reference_state;
    const auto robot_model = move_group_->getRobotModel();
    if (robot_model) {
      reference_state = std::make_shared<moveit::core::RobotState>(robot_model);
      reference_state->setToDefaultValues();
      const auto * joint_model_group = robot_model->getJointModelGroup(move_group_name_);
      if (joint_model_group && !moveit_plan.trajectory.joint_trajectory.points.empty()) {
        reference_state->setJointGroupPositions(
          joint_model_group, moveit_plan.trajectory.joint_trajectory.points.front().positions);
      }
      reference_state->update();
    }
    if (reference_state) {
      const std::string tool_tcp_link = move_group_->getEndEffectorLink();
      robot_trajectory::RobotTrajectory rt_check(move_group_->getRobotModel(), move_group_name_);
      rt_check.setRobotTrajectoryMsg(*reference_state, moveit_plan.trajectory);
      for (size_t index = 0; index < rt_check.getWayPointCount(); ++index) {
        const auto & waypoint_state = rt_check.getWayPoint(index);
        const auto & tcp_translation =
          waypoint_state.getGlobalLinkTransform(tool_tcp_link).translation();
        if (!isWithinWallPenetrationBound(
            wall_reference_pose,
            tcp_translation.x(), tcp_translation.y(), tcp_translation.z(),
            request.max_wall_penetration))
        {
          result.success = false;
          result.status = "failed";
          result.message = "Waypoint " + std::to_string(index) +
            " exceeds max_wall_penetration for fsp mode.";
          result.waypoints_completed = 0;
          result.path_fraction = 0.0;
          result.trajectory = core_types::Trajectory{};
          result.failure_reason_code = "wall_penetration_exceeded";
          result.failure_stage = FailureStage::kPlanning;
          break;
        }
      }
    }
  }

  result.planning_time_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() -
      planning_start)
    .count());

  move_group_->clearPoseTargets();
  return result;
}

IkResult OmplPlanner::ik(const IkRequest & request)
{
  return backend_->ik(request);
}

CollisionResult OmplPlanner::collision(const CollisionRequest & request)
{
  return backend_->collision(request);
}

}  // namespace motion_planner
