#include "motion_planner_moveit/cartesian_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <vector>

#include <Eigen/Geometry>

#include "moveit/collision_detection/collision_matrix.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_state/cartesian_interpolator.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "rclcpp/rclcpp.hpp"

#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "motion_planner/core/planners/reachability_tilt.hpp"
#include "motion_planner/core/planners/wall_penetration_check.hpp"
#include "ros_adapters/ros_conversions.hpp"

namespace motion_planner
{

namespace
{

constexpr double kDefaultEefStep = 0.01;
constexpr double kRelaxationFactor = 2.0;
constexpr double kMinTimeStepSeconds = 1e-3;
// Initial, deliberately-conservative defaults -- jump detection is now always on (see
// plan()'s Step 4), whereas before this session it was silently disabled. Needs live
// tuning against real scenarios, not a value to treat as final.
constexpr double kDefaultRevoluteJumpThresholdRad = 0.3;
constexpr double kDefaultPrismaticJumpThresholdMeters = 0.05;
// Bounds how many distinct collision points along one pass get a local recovery attempt
// (see plan()'s collision-recovery loop) -- mirrors MotionPlanner's kMaxBridgeAttempts.
constexpr int kMaxCollisionRecoveryAttempts = 3;
// Bounds how many distinct kIkFailure points along one pass get a reachability-tilt retry
// attempt (see plan()'s reachability-tilt loop) -- same shape as kMaxCollisionRecoveryAttempts.
constexpr int kMaxReachabilityTiltAttempts = 3;

// CartesianInterpolator::computeCartesianPath() takes EigenSTL::vector_Isometry3d, not
// geometry_msgs poses -- this file's waypoints are already converted to the latter (via
// ros_adapters::toRos()) for other reasons, so this converts once more at the Step 4
// boundary rather than threading Eigen types earlier through code that doesn't need them.
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  transform.linear() = quaternion.normalized().toRotationMatrix();
  return transform;
}

// Reuses MoveIt's own jump-distance definition (rather than reimplementing per-joint-type
// delta math, which would risk silently diverging from it -- e.g. continuous/revolute
// wraparound handling) so a candidate rejected here is judged by the exact same rule
// computeCartesianPath()'s own post-hoc filter would have applied, just evaluated proactively,
// per-candidate, inside plan()'s own collision-checking validity callback -- see that
// callback's own comment for why. checkAbsoluteJointSpaceJump() mutates its traj argument
// in place (truncating it on a detected jump), so build a throwaway 2-element vector rather
// than touching real trajectory state.
bool jointSpaceJumpDetected(
  const moveit::core::JointModelGroup * group,
  const moveit::core::RobotState & from,
  const moveit::core::RobotState & to,
  double revolute_jump_threshold,
  double prismatic_jump_threshold)
{
  if (revolute_jump_threshold <= 0.0 && prismatic_jump_threshold <= 0.0) {
    return false;  // 0/0 means jump detection disabled -- mirrors JumpThreshold's own semantics.
  }
  std::vector<std::shared_ptr<moveit::core::RobotState>> pair{
    std::make_shared<moveit::core::RobotState>(from),
    std::make_shared<moveit::core::RobotState>(to)};
  moveit::core::CartesianInterpolator::checkJointSpaceJump(
    group, pair,
    moveit::core::JumpThreshold::absolute(revolute_jump_threshold, prismatic_jump_threshold));
  return pair.size() < 2;
}

// Restores the scoped ACM entry (see setWallContactAcmEntry) no matter which
// return path plan() takes -- the plan promises this is never a permanent
// change, so every early return after the entry is applied must restore it.
struct AcmRestoreGuard
{
  CartesianPlanner * planner{nullptr};
  bool active{false};
  std::string tool_link;
  std::string object_id;
  bool had_previous_entry{false};
  bool previous_allowed{false};

  ~AcmRestoreGuard()
  {
    if (active && planner) {
      std::string restore_error;
      planner->restoreWallContactAcmEntry(
        tool_link, object_id, had_previous_entry, previous_allowed, restore_error);
    }
  }
};

}  // namespace

CartesianPlanner::CartesianPlanner(std::shared_ptr<MoveItPlanningBackend> backend)
: backend_(std::move(backend))
{
}

std::string CartesianPlanner::name() const
{
  return "cartesian";
}

bool CartesianPlanner::ensureMoveGroup(
  const std::string & planning_group,
  std::string & error_message)
{
  if (!backend_->ensureMoveGroup(planning_group, error_message)) {
    return false;
  }
  node_ = backend_->node();
  move_group_ = backend_->moveGroup();
  move_group_name_ = backend_->activeMoveGroupName();
  return true;
}

bool CartesianPlanner::ensurePlanningSceneMonitor(
  const std::string & planning_group,
  std::string & error_message)
{
  if (!backend_->ensurePlanningSceneMonitor(planning_group, error_message)) {
    return false;
  }
  planning_scene_monitor_ = backend_->planningSceneMonitor();
  return true;
}

bool CartesianPlanner::ensurePlanningSceneInterface(std::string & error_message)
{
  if (!backend_->ensurePlanningSceneInterface(error_message)) {
    return false;
  }
  planning_scene_interface_ = backend_->planningSceneInterface();
  return true;
}

bool CartesianPlanner::setWallContactAcmEntry(
  const std::string & tool_link,
  const std::string & target_object_id,
  bool allowed,
  bool & had_previous_entry,
  bool & previous_allowed,
  std::string & error_message)
{
  if (!ensurePlanningSceneMonitor(move_group_name_, error_message) || !planning_scene_monitor_) {
    return false;
  }
  if (!ensurePlanningSceneInterface(error_message)) {
    return false;
  }

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  {
    // Read the CURRENT ACM from our own monitor (kept in sync with
    // move_group's scene via its scene-monitor subscription), mutate a local
    // copy, then push the FULL resulting ACM back below. Sending the whole
    // ACM (not just the one changed entry) is what makes this correct
    // regardless of how MoveIt merges vs. replaces ACM diffs.
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    if (!scene) {
      error_message = "Failed to lock planning scene to read ACM.";
      return false;
    }
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
    collision_detection::AllowedCollision::Type previous_type =
      collision_detection::AllowedCollision::NEVER;
    had_previous_entry = acm.getEntry(tool_link, target_object_id, previous_type);
    previous_allowed = had_previous_entry &&
      (previous_type == collision_detection::AllowedCollision::ALWAYS);
    acm.setEntry(tool_link, target_object_id, allowed);
    acm.getMessage(diff.allowed_collision_matrix);
  }

  // Mutating our local PlanningSceneMonitor's mirror has no effect on the
  // move_group process's own collision checking -- computeCartesianPath()
  // is a service call handled entirely inside that separate process. The
  // change must be pushed there explicitly via applyPlanningScene(), which
  // synchronously updates move_group's real planning scene.
  if (!planning_scene_interface_->applyPlanningScene(diff)) {
    error_message = "applyPlanningScene() rejected the ACM diff.";
    return false;
  }
  return true;
}

bool CartesianPlanner::restoreWallContactAcmEntry(
  const std::string & tool_link,
  const std::string & target_object_id,
  bool had_previous_entry,
  bool previous_allowed,
  std::string & error_message)
{
  if (!planning_scene_monitor_ || !planning_scene_interface_) {
    error_message = "Planning scene monitor/interface unavailable for ACM restore.";
    return false;
  }

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    if (!scene) {
      error_message = "Failed to lock planning scene to restore ACM.";
      return false;
    }
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();
    if (had_previous_entry) {
      acm.setEntry(tool_link, target_object_id, previous_allowed);
    } else {
      acm.removeEntry(tool_link, target_object_id);
    }
    acm.getMessage(diff.allowed_collision_matrix);
  }

  if (!planning_scene_interface_->applyPlanningScene(diff)) {
    error_message = "applyPlanningScene() rejected the ACM restore diff.";
    return false;
  }
  return true;
}

std::shared_ptr<moveit::core::RobotState> CartesianPlanner::resolveReferenceState(
  const moveit_msgs::msg::RobotTrajectory & trajectory_msg) const
{
  if (!move_group_) {
    return nullptr;
  }

  // Same fix as ik()/collision() (see their own comments): every caller of
  // this reference state (applyConstantSpeedTimeParameterization()) fully
  // overwrites the planning group's own joint positions per-waypoint before
  // using it, so a live query never bought real accuracy for the group this
  // trajectory belongs to -- only for OTHER joints (e.g. the lift) outside
  // it, which affects FK-based speed timing but not planning/collision
  // safety. Found live: in an environment where CurrentStateMonitor never
  // completes after its first call, this was a real ~10s stall on EVERY
  // collision-recovery attempt (multiple per segment) -- and since it always
  // fell through to this exact synthetic state anyway, skipping the query
  // outright changes nothing about the result, only removes the wasted wait.
  const auto robot_model = move_group_->getRobotModel();
  if (!robot_model) {
    return nullptr;
  }
  auto synthetic_state = std::make_shared<moveit::core::RobotState>(robot_model);
  synthetic_state->setToDefaultValues();
  const auto * joint_model_group = robot_model->getJointModelGroup(move_group_name_);
  if (joint_model_group && !trajectory_msg.joint_trajectory.points.empty()) {
    synthetic_state->setJointGroupPositions(
      joint_model_group, trajectory_msg.joint_trajectory.points.front().positions);
  }
  synthetic_state->update();
  return synthetic_state;
}

namespace
{

// Index bounds for a first-order (forward/backward/central) finite
// difference at trajectory index i, given point_count total points.
void neighborIndices(size_t i, size_t point_count, size_t & lo, size_t & hi)
{
  lo = (i == 0) ? 0 : i - 1;
  hi = (i + 1 >= point_count) ? point_count - 1 : i + 1;
}

}  // namespace

bool CartesianPlanner::applyConstantSpeedTimeParameterization(
  moveit_msgs::msg::RobotTrajectory & trajectory_msg,
  const std::string & tool_tcp_link,
  double target_eef_speed) const
{
  auto & points = trajectory_msg.joint_trajectory.points;
  if (points.size() < 2 || target_eef_speed <= 0.0 || !move_group_) {
    return false;
  }

  auto reference_state = resolveReferenceState(trajectory_msg);
  if (!reference_state) {
    return false;
  }

  const auto * joint_model_group = reference_state->getJointModelGroup(move_group_name_);
  if (!joint_model_group) {
    return false;
  }
  const size_t joint_count = joint_model_group->getVariableCount();

  moveit::core::RobotState waypoint_state(*reference_state);
  std::vector<double> cumulative_distance(points.size(), 0.0);

  waypoint_state.setJointGroupPositions(joint_model_group, points.front().positions);
  waypoint_state.update();
  Eigen::Vector3d previous_tcp = waypoint_state.getGlobalLinkTransform(tool_tcp_link).translation();

  for (size_t i = 1; i < points.size(); ++i) {
    if (points[i].positions.size() != joint_count) {
      return false;
    }
    waypoint_state.setJointGroupPositions(joint_model_group, points[i].positions);
    waypoint_state.update();
    const Eigen::Vector3d current_tcp =
      waypoint_state.getGlobalLinkTransform(tool_tcp_link).translation();
    cumulative_distance[i] = cumulative_distance[i - 1] + (current_tcp - previous_tcp).norm();
    previous_tcp = current_tcp;
  }

  std::vector<double> time_from_start_seconds(points.size(), 0.0);
  for (size_t i = 0; i < points.size(); ++i) {
    time_from_start_seconds[i] = cumulative_distance[i] / target_eef_speed;
  }
  for (size_t i = 1; i < time_from_start_seconds.size(); ++i) {
    if (time_from_start_seconds[i] <= time_from_start_seconds[i - 1]) {
      time_from_start_seconds[i] = time_from_start_seconds[i - 1] + kMinTimeStepSeconds;
    }
  }
  for (auto & point : points) {
    point.velocities.assign(joint_count, 0.0);
    point.accelerations.assign(joint_count, 0.0);
  }

  for (size_t i = 0; i < points.size(); ++i) {
    points[i].time_from_start = rclcpp::Duration::from_seconds(time_from_start_seconds[i]);

    size_t lo = 0;
    size_t hi = 0;
    neighborIndices(i, points.size(), lo, hi);
    const double dt = time_from_start_seconds[hi] - time_from_start_seconds[lo];
    if (dt > 1e-9) {
      for (size_t j = 0; j < joint_count; ++j) {
        points[i].velocities[j] = (points[hi].positions[j] - points[lo].positions[j]) / dt;
      }
    }
  }

  for (size_t i = 0; i < points.size(); ++i) {
    size_t lo = 0;
    size_t hi = 0;
    neighborIndices(i, points.size(), lo, hi);
    const double dt = time_from_start_seconds[hi] - time_from_start_seconds[lo];
    if (dt > 1e-9) {
      for (size_t j = 0; j < joint_count; ++j) {
        points[i].accelerations[j] = (points[hi].velocities[j] - points[lo].velocities[j]) / dt;
      }
    }
  }

  return true;
}

bool CartesianPlanner::applyFallbackTimeParameterization(
  moveit_msgs::msg::RobotTrajectory & trajectory_msg) const
{
  if (!move_group_) {
    return false;
  }

  auto reference_state = resolveReferenceState(trajectory_msg);
  if (!reference_state) {
    return false;
  }

  robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_name_);
  rt.setRobotTrajectoryMsg(*reference_state, trajectory_msg);

  constexpr double kFullVelScaling = 1.0;
  constexpr double kFullAccScaling = 1.0;

  bool success = false;
  try {
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    success = totg.computeTimeStamps(rt, kFullVelScaling, kFullAccScaling);
  } catch (const std::exception &) {
    success = false;
  }

  if (!success) {
    try {
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      success = totg.computeTimeStamps(rt, kFullVelScaling * 0.5, kFullAccScaling * 0.5);
    } catch (const std::exception &) {
      success = false;
    }
  }

  if (success) {
    rt.getRobotTrajectoryMsg(trajectory_msg);
  }
  return success;
}

PlanningResult CartesianPlanner::plan(const PlanningRequest & request)
{
  const auto planning_start = std::chrono::steady_clock::now();
  auto elapsed_ms = [&planning_start]() {
      return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - planning_start)
        .count());
    };

  if (request.target_poses.empty()) {
    PlanningResult result;
    result.success = false;
    result.status = "failed";
    result.message = "Cartesian planning requires target poses; received an index-only request.";
    result.failure_reason_code = "missing_target_poses";
    result.failure_stage = FailureStage::kPrecheck;
    result.planning_time_ms = elapsed_ms();
    return result;
  }

  std::string init_error;
  if (!ensureMoveGroup(request.planning_group, init_error)) {
    PlanningResult result;
    result.success = false;
    result.status = "failed";
    result.message = "Failed to initialize Cartesian MoveGroup: " + init_error;
    result.failure_reason_code = "planner_init";
    result.failure_stage = FailureStage::kPrecheck;
    result.planning_time_ms = elapsed_ms();
    return result;
  }

  // Step 1/2: convert waypoints, guard against degenerate quaternions.
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(request.target_poses.size());
  for (const auto & pose : request.target_poses) {
    auto ros_pose = ros_adapters::toRos(pose);
    const double quat_norm_sq = ros_pose.orientation.x * ros_pose.orientation.x +
      ros_pose.orientation.y * ros_pose.orientation.y +
      ros_pose.orientation.z * ros_pose.orientation.z +
      ros_pose.orientation.w * ros_pose.orientation.w;
    if (quat_norm_sq < 1e-9) {
      ros_pose.orientation.x = 0.0;
      ros_pose.orientation.y = 0.0;
      ros_pose.orientation.z = 0.0;
      ros_pose.orientation.w = 1.0;
    }
    // Every target pose is expressed in request.frame_id (documented, but silently ignored
    // before this -- confirmed live: a frame_id naming a TF-anchored frame numerically far from
    // the planning frame, e.g. a wall mounted near base_link but expressed in its own captured
    // coordinates, got its raw numbers treated as literal planning-frame offsets). Resolve into
    // the planning frame here, once per pose, before anything downstream touches it.
    if (!request.frame_id.empty()) {
      geometry_msgs::msg::Pose resolved_pose;
      std::string resolve_error;
      if (!backend_->resolvePoseToPlanningFrame(
          request.frame_id, ros_pose, resolved_pose, resolve_error))
      {
        PlanningResult result;
        result.success = false;
        result.status = "failed";
        result.message = "Failed to resolve target pose from frame '" + request.frame_id +
          "' into the planning frame: " + resolve_error;
        result.failure_reason_code = "frame_resolution_failed";
        result.failure_stage = FailureStage::kPrecheck;
        result.planning_time_ms = elapsed_ms();
        return result;
      }
      ros_pose = resolved_pose;
    }
    waypoints.push_back(ros_pose);
  }

  // wall_reference_pose (step 5's penetration bound) is expressed in the same request.frame_id --
  // resolve it once, alongside the waypoints above, rather than at its point of use.
  core_types::Pose wall_reference_pose = request.wall_reference_pose;
  if (!request.frame_id.empty() && hasWallReference(request.wall_reference_pose)) {
    geometry_msgs::msg::Pose resolved_wall_pose;
    std::string resolve_error;
    if (!backend_->resolvePoseToPlanningFrame(
        request.frame_id, ros_adapters::toRos(request.wall_reference_pose), resolved_wall_pose,
        resolve_error))
    {
      PlanningResult result;
      result.success = false;
      result.status = "failed";
      result.message = "Failed to resolve wall_reference_pose from frame '" + request.frame_id +
        "' into the planning frame: " + resolve_error;
      result.failure_reason_code = "frame_resolution_failed";
      result.failure_stage = FailureStage::kPrecheck;
      result.planning_time_ms = elapsed_ms();
      return result;
    }
    wall_reference_pose = ros_adapters::toCore(resolved_wall_pose);
  }

  const auto robot_model = move_group_->getRobotModel();
  const auto * joint_model_group =
    robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
  const std::string tool_tcp_link = move_group_->getEndEffectorLink();
  const auto * tip_link_model = robot_model ? robot_model->getLinkModel(tool_tcp_link) : nullptr;
  if (!robot_model || !joint_model_group || !tip_link_model) {
    PlanningResult result;
    result.success = false;
    result.status = "failed";
    result.message = "Unable to resolve robot model/joint group/tip link for Cartesian planning.";
    result.failure_reason_code = "missing_robot_model";
    result.failure_stage = FailureStage::kPrecheck;
    result.planning_time_ms = elapsed_ms();
    return result;
  }

  // Step 4 (below) computes the path locally against our own RobotState instead of
  // through move_group_ -- setStartState()/setStartStateToCurrentState() no longer govern
  // that computation, so this only validates the request and seeds our own local state.
  bool used_explicit_start_state = false;
  moveit::core::RobotState start_state(robot_model);
  if (!request.start_joint_state.empty()) {
    if (request.start_joint_state.size() == joint_model_group->getVariableCount()) {
      start_state.setToDefaultValues();
      start_state.setJointGroupPositions(joint_model_group, request.start_joint_state);
      used_explicit_start_state = true;
    } else if (node_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "CartesianPlanner: request.start_joint_state size mismatch (got %zu); "
        "falling back to current state.",
        request.start_joint_state.size());
    }
  }
  if (!used_explicit_start_state) {
    std::string scene_error;
    if (ensurePlanningSceneMonitor(
        request.planning_group,
        scene_error) && planning_scene_monitor_)
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
      start_state = scene ? scene->getCurrentState() : start_state;
      if (!scene) {
        start_state.setToDefaultValues();
      }
    } else {
      start_state.setToDefaultValues();
    }
  }
  start_state.update();
  if (node_) {
    const Eigen::Isometry3d & start_pose = start_state.getGlobalLinkTransform(tip_link_model);
    RCLCPP_INFO(
      node_->get_logger(),
      "CartesianPlanner: resolved start pose (x=%.3f, y=%.3f, z=%.3f) [%s]",
      start_pose.translation().x(), start_pose.translation().y(), start_pose.translation().z(),
      used_explicit_start_state ? "explicit/continuity" : "live-scene/default");
  }

  // Step 3: scoped ACM entry -- a MoveIt-API workaround so
  // computeCartesianPath() doesn't reject the intended contact outright.
  // Not the safety boundary; step 5 (the post-hoc wall penetration bound) is.
  AcmRestoreGuard acm_guard;
  acm_guard.planner = this;
  if (request.allow_target_contact && !request.target_collision_object_id.empty()) {
    // The ACM exclusion must target whichever link actually carries the tool's real collision
    // geometry, NOT necessarily tool_tcp_link (the end-effector link, used above/below for the
    // wall-penetration-bound FK check -- that's about the coverage path's controlled point, a
    // different concern). Falls back to tool_tcp_link only when the caller hasn't set
    // contact_link, preserving prior behavior for any not-yet-updated caller.
    const std::string & contact_collision_link =
      request.contact_link.empty() ? tool_tcp_link : request.contact_link;
    std::string acm_error;
    if (setWallContactAcmEntry(
        contact_collision_link, request.target_collision_object_id, true,
        acm_guard.had_previous_entry, acm_guard.previous_allowed, acm_error))
    {
      acm_guard.active = true;
      acm_guard.tool_link = contact_collision_link;
      acm_guard.object_id = request.target_collision_object_id;
    } else if (node_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to apply ACM exclusion for tool='%s' object='%s': %s. "
        "Proceeding without it -- a genuine contact attempt will correctly "
        "fail as a collision.",
        contact_collision_link.c_str(), request.target_collision_object_id.c_str(),
        acm_error.c_str());
    }
  }

  // Step 4: compute the Cartesian path locally (moveit::core::CartesianInterpolator)
  // instead of via MoveGroupInterface's remote computeCartesianPath() service call, so a
  // partial/truncated result can be genuinely diagnosed -- jump discontinuity vs collision
  // vs IK failure -- rather than the opaque "some fraction, unknown why" the high-level API
  // gives (confirmed: it doesn't expose this distinction at all, client-side -- the
  // disambiguating detail only exists in log lines inside the separate move_group
  // process). Jump detection is enabled by default now (previously silently disabled --
  // this logic was ported from wall_cleaning_action_server.cpp, which *did* enable it).
  EigenSTL::vector_Isometry3d waypoints_eigen;
  waypoints_eigen.reserve(waypoints.size());
  for (const auto & pose : waypoints) {
    waypoints_eigen.push_back(poseToIsometry(pose));
  }

  // Runs one CartesianInterpolator::computeCartesianPath() call for a given constraint
  // combination. Truncation *cause* (collision vs jump vs IK failure) is captured directly,
  // at the real rejection point inside the validity callback below, via the optional
  // saw_collision_out/saw_jump_out out-params -- NOT by re-running a second, differently
  // configured search afterward and inferring cause from how far it gets (that approach used
  // to live here; see the validity callback's own comment for why it was unreliable).
  auto run_local_path = [&](
    double step, double revolute_jt, double prismatic_jt, bool check_collisions,
    std::vector<std::shared_ptr<moveit::core::RobotState>> & traj_out,
    const moveit::core::RobotState & seed,
    const EigenSTL::vector_Isometry3d & waypoints_to_walk,
    bool * saw_collision_out = nullptr,
    bool * saw_jump_out = nullptr) -> double {
      moveit::core::GroupStateValidityCallbackFn validity_callback;
      if (check_collisions && planning_scene_monitor_) {
        auto last_accepted = std::make_shared<moveit::core::RobotState>(seed);
        validity_callback = [this, last_accepted, revolute_jt, prismatic_jt, saw_collision_out,
          saw_jump_out](
          moveit::core::RobotState * robot_state, const moveit::core::JointModelGroup * jmg,
          const double * joint_positions) -> bool {
            robot_state->setJointGroupPositions(jmg, joint_positions);
            robot_state->update();
            bool collision_free = true;
            std::string error_message;
            if (!backend_->isStateCollisionFree(
                *robot_state, move_group_name_, collision_free,
                error_message))
            {
              return false;
            }
            if (!collision_free) {
              if (saw_collision_out) {*saw_collision_out = true;}
              // Identify exactly which link pair collided, at the exact rejected state --
              // not reconstructed afterward via a second, independently-solved search. That
              // approach (formerly here) was unreliable: this arm is kinematically redundant
              // for a 6D pose target (6 revolute joints + the lift's prismatic axis), so a
              // fresh IK solve using a different validity callback can converge to an
              // entirely different joint configuration for the very same Cartesian waypoint.
              // Confirmed live 2026-08-12: a full, unscoped checkCollision() re-derived from
              // that second search repeatedly found ZERO contacts for what the old
              // disambiguation logic was calling "the rejected state".
              if (planning_scene_monitor_ && node_) {
                planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
                if (scene) {
                  collision_detection::CollisionRequest contact_request;
                  contact_request.contacts = true;
                  contact_request.max_contacts = 10;
                  collision_detection::CollisionResult contact_result;
                  scene->checkCollision(contact_request, contact_result, *robot_state);
                  for (const auto & contact_entry : contact_result.contacts) {
                    RCLCPP_WARN(
                      node_->get_logger(),
                      "CartesianPlanner: collision truncation at link pair '%s' <-> '%s'",
                      contact_entry.first.first.c_str(), contact_entry.first.second.c_str());
                  }
                }
              }
              return false;
            }
            // Jump-checking folded in here instead of left to computeCartesianPath's own
            // post-hoc filter (see this call's own JumpThreshold(0, 0) below): rejecting a
            // jump-violating candidate through the SAME validity callback IK already consults
            // gives its own consistency/retry search an immediate chance to land on a
            // different, jump-safe branch -- this arm's redundant DOF usually means more than
            // one IK solution exists per waypoint, so the first-found branch failing a jump
            // check doesn't mean no jump-safe branch exists. Compared against last_accepted
            // (this callback's own record of the last candidate IT accepted for this walk),
            // not the walk's original seed -- jump distance is inherently step-to-step.
            if (jointSpaceJumpDetected(
                jmg, *last_accepted, *robot_state, revolute_jt, prismatic_jt))
            {
              if (saw_jump_out) {*saw_jump_out = true;}
              return false;
            }
            *last_accepted = *robot_state;
            return true;
          };
      }
      moveit::core::RobotState seed_state(seed);
      traj_out.clear();
      // Jump thresholds are enforced inside validity_callback above when check_collisions is
      // true (so a rejection can trigger IK's own retry-with-a-different-branch behavior,
      // instead of only being discovered after the fact with no way to ask for another try);
      // passing them to computeCartesianPath() too would just re-check the same candidates it
      // already rejected. check_collisions=false calls (the min_fraction retry tier's own
      // relaxed attempt below) install no callback at all, so they still rely on
      // computeCartesianPath's own jump_threshold param.
      const auto jump_threshold = check_collisions ?
        moveit::core::JumpThreshold(0.0, 0.0) :
        moveit::core::JumpThreshold(revolute_jt, prismatic_jt);
      return moveit::core::CartesianInterpolator::computeCartesianPath(
        &seed_state, joint_model_group, traj_out, tip_link_model, waypoints_to_walk, true,
        moveit::core::MaxEEFStep(step), jump_threshold, validity_callback);
    };

  const double eef_step = request.eef_step > 0.0 ? request.eef_step : kDefaultEefStep;
  const double revolute_jump_threshold = request.revolute_jump_threshold > 0.0 ?
    request.revolute_jump_threshold : kDefaultRevoluteJumpThresholdRad;
  const double prismatic_jump_threshold = request.prismatic_jump_threshold > 0.0 ?
    request.prismatic_jump_threshold : kDefaultPrismaticJumpThresholdMeters;
  const double min_fraction = request.min_path_fraction > 0.0 ? request.min_path_fraction : 0.5;

  std::vector<std::shared_ptr<moveit::core::RobotState>> traj;
  bool saw_collision = false;
  bool saw_jump = false;
  double fraction = run_local_path(
    eef_step, revolute_jump_threshold, prismatic_jump_threshold, true, traj, start_state,
    waypoints_eigen, &saw_collision, &saw_jump);

  if (fraction < min_fraction) {
    std::vector<std::shared_ptr<moveit::core::RobotState>> retry_traj;
    bool retry_saw_collision = false;
    bool retry_saw_jump = false;
    const double retry_fraction = run_local_path(
      eef_step * kRelaxationFactor, revolute_jump_threshold * kRelaxationFactor,
      prismatic_jump_threshold * kRelaxationFactor, true, retry_traj, start_state,
      waypoints_eigen, &retry_saw_collision, &retry_saw_jump);
    if (retry_fraction > fraction) {
      traj = retry_traj;
      fraction = retry_fraction;
      saw_collision = retry_saw_collision;
      saw_jump = retry_saw_jump;
    }
  }

  if (traj.empty()) {
    PlanningResult result;
    result.success = false;
    result.status = "failed";
    result.message = "Local Cartesian path computation produced no usable trajectory.";
    result.failure_reason_code = "cartesian_path_failed";
    result.failure_stage = FailureStage::kPlanning;
    result.planning_time_ms = elapsed_ms();
    return result;
  }

  // Only the *not-yet-reached* waypoints -- not the full waypoints_eigen list -- for any call
  // seeded from somewhere other than the true start_state (collision-recovery resume). Those
  // calls seed from traj.back(), already partway *past* however many leading waypoints the
  // real attempt already reached; re-passing the full list would make computeCartesianPath
  // walk backward to revisit an already-passed waypoint before continuing forward again -- a
  // genuine back-and-forth artifact. How many waypoints are already behind us isn't always
  // exactly one: continuity normally places start_state at (or very near) waypoints_eigen[0],
  // but whenever a *previous* chained segment didn't fully complete (including one this same
  // collision-recovery mechanism only partially recovered), the next segment's start can land
  // nowhere near its own declared first waypoint -- so "already passed 0 waypoints" and
  // "already passed all but the last" both genuinely happen. Determined geometrically
  // (translation distance only, a reasonable proxy for MoveIt's own internal Percentage)
  // rather than assumed, since guessing wrong here reproduces exactly the bug this comment is
  // fixing.
  moveit::core::RobotState continuity_seed(start_state);
  continuity_seed.update();
  // Mutable, NOT const: the collision-recovery loop below re-derives this after every
  // successful recovered-and-resumed leg (mirrors the reachability-tilt retry tier's own
  // `reachability_remaining` further down) -- see that loop's own comment for why reusing a
  // stale list here reproduces the exact "genuine back-and-forth artifact" this comment block
  // warns about, just one level removed (across recovery attempts instead of at first entry).
  EigenSTL::vector_Isometry3d remaining_waypoints_eigen = unconsumedWaypoints(
    continuity_seed.getGlobalLinkTransform(tip_link_model), waypoints_eigen, fraction);

  // Truncation cause is read directly off what the real, safety-respecting search above
  // actually rejected (saw_collision/saw_jump) -- not inferred by re-running a second,
  // differently-configured search and seeing how far it gets. Confirmed live 2026-08-12 that
  // inference is unsound on this redundant-DOF arm: removing the collision check doesn't just
  // change acceptance, it changes which IK branch the solver converges to in the first place,
  // so a second search finding a way through proves nothing about why the FIRST search's own
  // actual candidate was rejected.
  constexpr double kFractionEpsilon = 1e-6;
  CartesianTruncationReason truncation_reason = CartesianTruncationReason::kNone;
  if (fraction < 1.0 - kFractionEpsilon) {
    truncation_reason = saw_collision ? CartesianTruncationReason::kCollision :
      saw_jump ? CartesianTruncationReason::kJumpDiscontinuity :
      CartesianTruncationReason::kIkFailure;
  }

  // Collision recovery: this chain has a genuine redundant DOF (lift_prismatic_joint plus
  // 6 revolute arm joints, for a 6D pose target) that today's single-seed-per-step IK never
  // exploits. When the real search was genuinely blocked by a collision (not a jump, not IK
  // failure), retry IK directly at the exact pose it was trying to reach --
  // remaining_waypoints_eigen's own front(), the next original target waypoint still ahead per
  // unconsumedWaypoints()'s own arc-length derivation, not re-solved via a second IK search --
  // with consistency_limits bounded to the SAME jump thresholds already used for jump
  // detection. Any solution KDL returns under that bound is, by construction, never a jump --
  // so recovering here can never be confused with a genuine jump discontinuity (that's Stage
  // 2's MotionPlanner-level bridge, a different mechanism for a different problem: escaping a
  // jump requires leaving this neighborhood by definition, which this search deliberately
  // never does). Not attempted for kJumpDiscontinuity/kIkFailure -- a jump-safe local search
  // can't help either of those.
  int collision_recovery_attempts = 0;
  while (truncation_reason == CartesianTruncationReason::kCollision &&
    collision_recovery_attempts < kMaxCollisionRecoveryAttempts && fraction < 1.0)
  {
    ++collision_recovery_attempts;

    if (remaining_waypoints_eigen.empty()) {
      break;
    }
    const Eigen::Isometry3d recovery_target = remaining_waypoints_eigen.front();

    moveit::core::RobotState recovery_seed(*traj.back());
    std::vector<double> consistency_limits;
    consistency_limits.reserve(joint_model_group->getActiveJointModels().size());
    for (const auto * joint : joint_model_group->getActiveJointModels()) {
      consistency_limits.push_back(
        joint->getType() == moveit::core::JointModel::PRISMATIC ?
        prismatic_jump_threshold : revolute_jump_threshold);
    }

    moveit::core::GroupStateValidityCallbackFn recovery_validity_callback =
      [this](
      moveit::core::RobotState * robot_state, const moveit::core::JointModelGroup * jmg,
      const double * joint_positions) -> bool {
        robot_state->setJointGroupPositions(jmg, joint_positions);
        robot_state->update();
        bool collision_free = true;
        std::string error_message;
        if (!backend_->isStateCollisionFree(
            *robot_state, move_group_name_, collision_free,
            error_message))
        {
          return false;
        }
        return collision_free;
      };

    const bool recovered = recovery_seed.setFromIK(
      joint_model_group, recovery_target, tip_link_model->getName(), consistency_limits,
      0.0, recovery_validity_callback);

    if (!recovered) {
      if (node_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "CartesianPlanner: collision recovery attempt %d found no jump-safe, "
          "collision-free configuration -- treating truncation as genuine.",
          collision_recovery_attempts);
      }
      break;
    }
    if (node_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "CartesianPlanner: collision recovery attempt %d found an alternate, jump-safe "
        "configuration -- resuming the Cartesian path from there.",
        collision_recovery_attempts);
    }

    traj.push_back(std::make_shared<moveit::core::RobotState>(recovery_seed));

    recovery_seed.update();
    const Eigen::Isometry3d resume_anchor = recovery_seed.getGlobalLinkTransform(tip_link_model);

    std::vector<std::shared_ptr<moveit::core::RobotState>> resumed_traj;
    bool resumed_saw_collision = false;
    bool resumed_saw_jump = false;
    const double resumed_fraction = run_local_path(
      eef_step, revolute_jump_threshold, prismatic_jump_threshold, true, resumed_traj,
      recovery_seed, remaining_waypoints_eigen, &resumed_saw_collision, &resumed_saw_jump);
    traj.insert(traj.end(), resumed_traj.begin(), resumed_traj.end());
    // computeCartesianPath()'s own returned fraction is relative to ITS OWN start_state --
    // since the recovered point is already partway along the original path, express the
    // combined fraction as what was already covered plus this resumed leg's own share of
    // what remained.
    fraction = fraction + (1.0 - fraction) * resumed_fraction;

    if (fraction >= 1.0 - kFractionEpsilon) {
      truncation_reason = CartesianTruncationReason::kNone;
      break;
    }

    // Re-derive which of remaining_waypoints_eigen's waypoints are still ahead (mirrors the
    // reachability-tilt retry tier's identical re-derivation below) before retrying further --
    // resumed_fraction is relative to remaining_waypoints_eigen's own length AS WALKED FROM
    // resume_anchor, which may have made progress past more than the recovery target alone.
    // Without this, the NEXT recovery attempt re-walks from the front of a list that this leg
    // already partially consumed -- a genuine back-and-forth/repeated-segment artifact,
    // confirmed live (boundary_pass on 2026-08-12: 3 recovery attempts ballooned one segment's
    // execution to 133s by re-walking already-covered ground on attempts 2 and 3).
    remaining_waypoints_eigen = unconsumedWaypoints(
      resume_anchor, remaining_waypoints_eigen, resumed_fraction);
    truncation_reason = resumed_saw_collision ? CartesianTruncationReason::kCollision :
      resumed_saw_jump ? CartesianTruncationReason::kJumpDiscontinuity :
      CartesianTruncationReason::kIkFailure;
  }

  // Reachability-tilt retry tier: the natural third fallback for kIkFailure -- today's dead end
  // (no nearby joint configuration reaches the ideal pose AT ALL: the real search above never
  // rejected a candidate for collision or a jump, so nothing converged there at all). Unlike
  // collision recovery above (same target pose, different joint
  // configuration), the ideal target itself is what's unreachable here, so this perturbs the
  // TARGET ORIENTATION via a small candidate-angle search (generateReachabilityTiltCandidates)
  // instead of re-seeding IK at an unchanged pose. Hard safety exclusion, checked explicitly
  // (never inferred from reachability_tilt_deg_max alone): contact tools NEVER get this --
  // tilting a tool meant to be in contact with the wall changes the force/contact-normal
  // semantics, not just reach.
  //
  // `reachability_remaining` is this tier's OWN evolving waypoint list, seeded from
  // remaining_waypoints_eigen but never shared with collision recovery above: each waypoint this
  // loop successfully resolves via tilt is PERMANENTLY substituted in place (the ideal/unreachable
  // pose is replaced by the accepted tilted one) so a resumed walk -- and any later re-diagnosis
  // against this same list -- targets the pose that was actually proven reachable, not the one
  // that just failed.
  EigenSTL::vector_Isometry3d reachability_remaining = remaining_waypoints_eigen;
  int reachability_tilt_attempts = 0;
  while (truncation_reason == CartesianTruncationReason::kIkFailure &&
    !request.tool_contact && request.reachability_tilt_deg_max > 0.0 &&
    reachability_tilt_attempts < kMaxReachabilityTiltAttempts && fraction < 1.0)
  {
    ++reachability_tilt_attempts;

    if (reachability_remaining.empty()) {
      break;
    }
    const Eigen::Isometry3d ideal_target = reachability_remaining.front();
    const double max_tilt_rad = request.reachability_tilt_deg_max * M_PI / 180.0;
    const auto candidates = generateReachabilityTiltCandidates(ideal_target, max_tilt_rad);

    moveit::core::GroupStateValidityCallbackFn tilt_validity_callback =
      [this](
      moveit::core::RobotState * robot_state, const moveit::core::JointModelGroup * jmg,
      const double * joint_positions) -> bool {
        robot_state->setJointGroupPositions(jmg, joint_positions);
        robot_state->update();
        bool collision_free = true;
        std::string error_message;
        if (!backend_->isStateCollisionFree(
            *robot_state, move_group_name_, collision_free,
            error_message))
        {
          return false;
        }
        return collision_free;
      };

    bool recovered = false;
    moveit::core::RobotState recovery_seed(*traj.back());
    Eigen::Isometry3d accepted_target = ideal_target;
    for (const auto & candidate : candidates) {
      moveit::core::RobotState candidate_seed(*traj.back());
      if (candidate_seed.setFromIK(
          joint_model_group, candidate, tip_link_model->getName(), 0.0,
          tilt_validity_callback))
      {
        recovery_seed = candidate_seed;
        accepted_target = candidate;
        recovered = true;
        break;
      }
    }

    if (!recovered) {
      if (node_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "CartesianPlanner: reachability-tilt attempt %d found no reachable orientation "
          "within %.1f deg of the ideal pose -- treating truncation as genuine.",
          reachability_tilt_attempts, request.reachability_tilt_deg_max);
      }
      break;
    }
    if (node_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "CartesianPlanner: reachability-tilt attempt %d found a reachable tilted orientation -- "
        "resuming the Cartesian path from there.",
        reachability_tilt_attempts);
    }

    traj.push_back(std::make_shared<moveit::core::RobotState>(recovery_seed));
    reachability_remaining.front() = accepted_target;

    recovery_seed.update();
    const Eigen::Isometry3d resume_anchor = recovery_seed.getGlobalLinkTransform(tip_link_model);

    std::vector<std::shared_ptr<moveit::core::RobotState>> resumed_traj;
    bool resumed_saw_collision = false;
    bool resumed_saw_jump = false;
    const double resumed_fraction = run_local_path(
      eef_step, revolute_jump_threshold, prismatic_jump_threshold, true, resumed_traj,
      recovery_seed, reachability_remaining, &resumed_saw_collision, &resumed_saw_jump);
    traj.insert(traj.end(), resumed_traj.begin(), resumed_traj.end());
    fraction = fraction + (1.0 - fraction) * resumed_fraction;

    if (fraction >= 1.0 - kFractionEpsilon) {
      truncation_reason = CartesianTruncationReason::kNone;
      break;
    }

    // Re-derive which of reachability_remaining's waypoints are still ahead (mirrors Step 4's
    // own initial derivation) before retrying further -- resumed_fraction is relative to
    // reachability_remaining's own length AS WALKED FROM resume_anchor, which may have made
    // progress past more than just the one waypoint this attempt substituted.
    reachability_remaining = unconsumedWaypoints(
      resume_anchor, reachability_remaining, resumed_fraction);
    // Read directly off this resumed leg's own rejection, same as the collision-recovery loop
    // above -- if it's no longer kIkFailure (a jump or collision showed up instead), this
    // tier's own while condition naturally stops re-entering, handing off to Stage 2's
    // MotionPlanner-level bridge via the returned truncation_reason.
    truncation_reason = resumed_saw_collision ? CartesianTruncationReason::kCollision :
      resumed_saw_jump ? CartesianTruncationReason::kJumpDiscontinuity :
      CartesianTruncationReason::kIkFailure;
  }

  // Convert the accumulated RobotState sequence into a RobotTrajectory message -- no
  // timing yet (dt=0.0 placeholder per waypoint); Step 6 below re-times the whole thing
  // via TOTG/IPTP/constant-speed, same as before.
  robot_trajectory::RobotTrajectory rt(robot_model, joint_model_group);
  for (const auto & state : traj) {
    rt.addSuffixWayPoint(state, 0.0);
  }
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  rt.getRobotTrajectoryMsg(trajectory_msg);

  // Step 5: independent, ACM-blind wall-penetration bound -- the actual
  // enforced safety boundary, not the ACM entry from step 3/4.
  if (hasWallReference(wall_reference_pose)) {
    auto reference_state = resolveReferenceState(trajectory_msg);
    if (reference_state) {
      robot_trajectory::RobotTrajectory rt_check(move_group_->getRobotModel(), move_group_name_);
      rt_check.setRobotTrajectoryMsg(*reference_state, trajectory_msg);
      for (size_t i = 0; i < rt_check.getWayPointCount(); ++i) {
        const auto & waypoint_state = rt_check.getWayPoint(i);
        const Eigen::Isometry3d & tcp_transform = waypoint_state.getGlobalLinkTransform(
          tool_tcp_link);
        const auto & tcp_translation = tcp_transform.translation();
        if (!isWithinWallPenetrationBound(
            wall_reference_pose,
            tcp_translation.x(), tcp_translation.y(), tcp_translation.z(),
            request.max_wall_penetration))
        {
          PlanningResult result;
          result.success = false;
          result.status = "failed";
          result.message = "Waypoint " + std::to_string(i) +
            " exceeds max_wall_penetration -- rejecting rather than trusting the ACM exclusion.";
          result.failure_reason_code = "wall_penetration_exceeded";
          result.failure_stage = FailureStage::kPlanning;
          result.planning_time_ms = elapsed_ms();
          return result;
        }
      }
    }
  }

  // Step 6: time-parameterization -- constant-speed if requested, else the
  // proven TOTG -> IPTP -> IPTP-at-half-scale fallback chain.
  bool time_param_ok = false;
  if (request.target_eef_speed > 0.0) {
    time_param_ok = applyConstantSpeedTimeParameterization(
      trajectory_msg, tool_tcp_link, request.target_eef_speed);
    if (!time_param_ok && node_) {
      // Silent before this: a caller requesting constant EE speed would
      // otherwise get ordinary joint-space timing back with no indication
      // that its request was dropped -- exactly the kind of gap that let
      // this feature go unexercised despite being wired end-to-end.
      RCLCPP_WARN(
        node_->get_logger(),
        "CartesianPlanner: applyConstantSpeedTimeParameterization failed for "
        "requested target_eef_speed=%.4f m/s; falling back to default "
        "(non-constant-speed) time parameterization.",
        request.target_eef_speed);
    }
  }
  if (!time_param_ok) {
    time_param_ok = applyFallbackTimeParameterization(trajectory_msg);
  }

  if (!time_param_ok) {
    PlanningResult result;
    result.success = false;
    result.status = "failed";
    result.message = "All time parameterization methods failed.";
    result.failure_reason_code = "time_parameterization_failed";
    result.failure_stage = FailureStage::kPlanning;
    result.planning_time_ms = elapsed_ms();
    return result;
  }

  // Step 7: populate PlanningResult with the planned (not yet executed)
  // trajectory -- the caller (ManipulationManager) owns execution via an
  // ExecutionPort. See Planner::plan()'s doc comment.
  PlanningResult result;
  result.success = true;
  result.status = "planned";
  result.message = "Cartesian plan computed successfully.";
  // waypoint_indices is legitimately empty for inline-pose segments (force_segment_expander's
  // synthesized approach/retract poses -- see ManipulationManager::buildSegmentPlanningInputs()),
  // which populate target_poses directly instead. Falling back to target_poses.size() there
  // mirrors the denominator ManipulationManager's own metrics.attempted_waypoints already uses --
  // without it, every inline-pose segment reports waypoints_completed=0 even on full success.
  const size_t waypoint_count = !request.waypoint_indices.empty() ?
    request.waypoint_indices.size() : request.target_poses.size();
  result.waypoints_completed = static_cast<size_t>(fraction * static_cast<double>(waypoint_count));
  result.path_fraction = fraction;
  result.trajectory = ros_adapters::toCore(trajectory_msg.joint_trajectory);
  result.failure_reason_code = "none";
  result.failure_stage = FailureStage::kNone;
  result.truncation_reason = truncation_reason;
  result.planning_time_ms = elapsed_ms();
  return result;
}

IkResult CartesianPlanner::ik(const IkRequest & request)
{
  return backend_->ik(request);
}

CollisionResult CartesianPlanner::collision(const CollisionRequest & request)
{
  return backend_->collision(request);
}

}  // namespace motion_planner
