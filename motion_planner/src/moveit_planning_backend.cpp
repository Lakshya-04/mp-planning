#include "motion_planner/moveit_planning_backend.hpp"

#include <exception>

#include <Eigen/Geometry>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ros_adapters/ros_conversions.hpp"

namespace motion_planner
{

namespace
{

// CLAUDE.md flags this exact planning-group string as a recurring hardcoded-fallback footgun --
// named here (rather than an inline literal) for discoverability alongside any future defaults.
constexpr char kDefaultPlanningGroup[] = "arm_7dof";

}  // namespace

MoveItPlanningBackend::MoveItPlanningBackend() = default;

bool MoveItPlanningBackend::parseTargetPose(
  const std::vector<double> & pose_values,
  geometry_msgs::msg::Pose & pose,
  std::string & error_message)
{
  if (pose_values.size() == 7) {
    pose.position.x = pose_values[0];
    pose.position.y = pose_values[1];
    pose.position.z = pose_values[2];
    pose.orientation.x = pose_values[3];
    pose.orientation.y = pose_values[4];
    pose.orientation.z = pose_values[5];
    pose.orientation.w = pose_values[6];
    return true;
  }

  if (pose_values.size() == 6) {
    const double roll = pose_values[3];
    const double pitch = pose_values[4];
    const double yaw = pose_values[5];
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    pose.position.x = pose_values[0];
    pose.position.y = pose_values[1];
    pose.position.z = pose_values[2];
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return true;
  }

  error_message = "target_pose must contain 6 values (xyz+rpy) or 7 values (xyz+quaternion)";
  return false;
}

bool MoveItPlanningBackend::ensureMoveGroup(
  const std::string & planning_group,
  std::string & error_message)
{
  const std::string effective_group = planning_group.empty() ?
    kDefaultPlanningGroup : planning_group;

  if (!rclcpp::ok()) {
    error_message = "rclcpp_not_initialized";
    return false;
  }

  if (move_group_ && move_group_name_ == effective_group) {
    return true;
  }

  try {
    if (!node_) {
      node_ = std::make_shared<rclcpp::Node>("motion_planner_backend");
    }

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_,
      effective_group);
    // Harmless to a caller that never invokes MoveGroupInterface::plan()
    // directly (CartesianPlanner plans locally via CartesianInterpolator) --
    // OmplPlanner's own former default, kept so it doesn't need to set this
    // itself post-injection.
    move_group_->setPlanningPipelineId("ompl");
    move_group_name_ = effective_group;
    return true;
  } catch (const std::exception & e) {
    move_group_.reset();
    move_group_name_.clear();
    error_message = e.what();
    return false;
  }
}

bool MoveItPlanningBackend::ensurePlanningSceneMonitor(
  const std::string & planning_group,
  std::string & error_message)
{
  if (planning_scene_monitor_) {
    return true;
  }

  if (!rclcpp::ok()) {
    error_message = "rclcpp_not_initialized";
    return false;
  }

  if (!node_) {
    error_message = "planner_node_not_initialized";
    return false;
  }

  (void)planning_group;

  try {
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_,
      "robot_description");
  } catch (const std::exception & e) {
    error_message = e.what();
    return false;
  }

  if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
    planning_scene_monitor_.reset();
    error_message = "failed_to_create_planning_scene";
    return false;
  }

  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  planning_scene_monitor_->startStateMonitor();

  // The start*Monitor() calls above only subscribe to *future* scene diffs --
  // they never fetch the *current* scene, so anything already in move_group's
  // scene before this monitor was constructed (e.g. an obstacle injected via
  // MoveItSceneAdapter ahead of this node starting) would otherwise be
  // permanently invisible to every collision()/isStateCollisionFree() call for
  // the rest of this instance's life (planning_scene_monitor_ is a lazy
  // singleton, cached forever -- see the early-return above). One synchronous
  // /get_planning_scene pull here resolves that for the common case (obstacle
  // injected before this backend's first use), for BOTH planners that now
  // share this one instance -- previously only CartesianPlanner had this
  // pull, so OmplPlanner alone could silently miss such an obstacle. Known,
  // accepted gap: an obstacle injected *after* this point still won't be
  // picked up without some other trigger -- not reachable today (fresh
  // process per mission, injection sequenced strictly before the consumer
  // starts), but worth knowing if that usage pattern ever changes.
  if (!planning_scene_monitor_->requestPlanningSceneState()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "requestPlanningSceneState() failed -- world geometry already present in move_group's "
      "scene before now may be invisible to this backend's collision checks.");
  }

  return true;
}

bool MoveItPlanningBackend::ensurePlanningSceneInterface(std::string & error_message)
{
  if (planning_scene_interface_) {
    return true;
  }

  try {
    planning_scene_interface_ =
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    return true;
  } catch (const std::exception & e) {
    error_message = e.what();
    return false;
  }
}

bool MoveItPlanningBackend::isStateCollisionFree(
  moveit::core::RobotState & state,
  const std::string & planning_group,
  bool & collision_free,
  std::string & error_message)
{
  std::string scene_error;
  if (ensurePlanningSceneMonitor(planning_group, scene_error) && planning_scene_monitor_) {
    const auto robot_model = state.getRobotModel();
    const auto * joint_model_group =
      robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
    if (!robot_model || !joint_model_group) {
      error_message = "Unable to resolve planning group for collision evaluation: " +
        move_group_name_;
      return false;
    }

    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    if (!scene) {
      error_message =
        "PlanningSceneMonitor is available but could not be locked for collision checking.";
      return false;
    }

    collision_free = !scene->isStateColliding(state, joint_model_group->getName());
    return true;
  }

  const auto robot_model = state.getRobotModel();
  if (!robot_model) {
    error_message = "Robot model unavailable for collision fallback.";
    return false;
  }

  planning_scene::PlanningScene planning_scene(robot_model);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkCollision(collision_request, collision_result, state);
  collision_free = !collision_result.collision;
  return true;
}

bool MoveItPlanningBackend::resolvePoseToPlanningFrame(
  const std::string & source_frame,
  const geometry_msgs::msg::Pose & pose_in_source,
  geometry_msgs::msg::Pose & pose_in_planning_frame,
  std::string & error_message)
{
  if (!move_group_) {
    error_message =
      "resolvePoseToPlanningFrame: MoveGroup not initialized -- call ensureMoveGroup() first.";
    return false;
  }

  const std::string planning_frame = move_group_->getPlanningFrame();
  if (source_frame.empty() || source_frame == planning_frame) {
    pose_in_planning_frame = pose_in_source;
    return true;
  }

  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  geometry_msgs::msg::PoseStamped stamped_in;
  stamped_in.header.frame_id = source_frame;
  stamped_in.pose = pose_in_source;

  geometry_msgs::msg::PoseStamped stamped_out;
  try {
    const auto transform = tf_buffer_->lookupTransform(
      planning_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
    tf2::doTransform(stamped_in, stamped_out, transform);
  } catch (const tf2::TransformException & e) {
    error_message = "resolvePoseToPlanningFrame: TF lookup planning_frame='" + planning_frame +
      "' <- source_frame='" + source_frame + "' failed: " + e.what();
    return false;
  }

  pose_in_planning_frame = stamped_out.pose;
  return true;
}

std::shared_ptr<rclcpp::Node> MoveItPlanningBackend::node() const
{
  return node_;
}

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveItPlanningBackend::moveGroup()
const
{
  return move_group_;
}

std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>
MoveItPlanningBackend::planningSceneMonitor() const
{
  return planning_scene_monitor_;
}

std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
MoveItPlanningBackend::planningSceneInterface() const
{
  return planning_scene_interface_;
}

const std::string & MoveItPlanningBackend::activeMoveGroupName() const
{
  return move_group_name_;
}

IkResult MoveItPlanningBackend::ik(const IkRequest & request)
{
  IkResult result;

  if (request.target_pose.empty()) {
    result.success = false;
    result.message = "IK request is missing a target pose.";
    result.failure_reason_code = "missing_target_pose";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  std::string initialization_error;
  if (!ensureMoveGroup(request.planning_group, initialization_error)) {
    result.success = false;
    result.message = "Failed to initialize MoveGroup for IK: " + initialization_error;
    result.failure_reason_code = "planner_init";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  const auto robot_model = move_group_->getRobotModel();
  const auto * joint_model_group =
    robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
  if (!robot_model || !joint_model_group) {
    result.success = false;
    result.message = "Unable to resolve planning group for IK: " + move_group_name_;
    result.failure_reason_code = "missing_joint_model_group";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  // Build the reference state locally from the robot model rather than
  // querying getCurrentState() -- in environments where the
  // CurrentStateMonitor never completes, that live query is a real ~10s
  // stall per call, tolerable once per plan but not for a caller that calls
  // ik() many times in a loop (e.g. a per-point feasibility sweep). No
  // established caller of ik() depends on searching from the robot's true
  // live pose -- an explicit joint_seed already covers "start the search
  // from a specific configuration," and the no-seed case just means "search
  // from the model's own default."
  std::shared_ptr<moveit::core::RobotState> reference_state =
    std::make_shared<moveit::core::RobotState>(robot_model);
  reference_state->setToDefaultValues();
  if (!request.joint_seed.empty()) {
    if (request.joint_seed.size() != joint_model_group->getVariableCount()) {
      result.success = false;
      result.message = "Joint seed size does not match planning group variable count.";
      result.failure_reason_code = "invalid_joint_seed";
      result.failure_stage = FailureStage::kPrecheck;
      return result;
    }
    reference_state->setJointGroupPositions(joint_model_group, request.joint_seed);
  }
  reference_state->update();

  geometry_msgs::msg::Pose target_pose;
  std::string pose_error;
  if (!parseTargetPose(request.target_pose, target_pose, pose_error)) {
    result.success = false;
    result.message = pose_error;
    result.failure_reason_code = "invalid_target_pose";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  if (!request.frame_id.empty()) {
    geometry_msgs::msg::Pose resolved_pose;
    std::string resolve_error;
    if (!resolvePoseToPlanningFrame(request.frame_id, target_pose, resolved_pose, resolve_error)) {
      result.success = false;
      result.message = resolve_error;
      result.failure_reason_code = "frame_resolution_failed";
      result.failure_stage = FailureStage::kPrecheck;
      return result;
    }
    target_pose = resolved_pose;
  }

  Eigen::Isometry3d target_transform = Eigen::Isometry3d::Identity();
  target_transform.translation() = Eigen::Vector3d(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);
  const Eigen::Quaterniond quaternion(
    target_pose.orientation.w,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z);
  target_transform.linear() = quaternion.normalized().toRotationMatrix();

  const std::string end_effector_link = request.end_effector_link.empty() ?
    move_group_->getEndEffectorLink() :
    request.end_effector_link;

  moveit::core::RobotState ik_state(*reference_state);
  const double timeout_seconds = 0.1;
  const bool ik_found = ik_state.setFromIK(
    joint_model_group,
    target_transform,
    end_effector_link,
    timeout_seconds);

  result.success = ik_found;
  result.message = ik_found ? "IK solution found." : "IK solver failed to find a solution.";
  result.failure_reason_code = ik_found ? "none" : "ik_failed";
  result.failure_stage = ik_found ? FailureStage::kNone : FailureStage::kPlanning;
  if (ik_found) {
    ik_state.copyJointGroupPositions(joint_model_group, result.joint_solution);
  }
  return result;
}

CollisionResult MoveItPlanningBackend::collision(const CollisionRequest & request)
{
  CollisionResult result;

  const bool has_single_state_input = !request.joint_values.empty() || !request.target_pose.empty();
  const bool has_trajectory_input = !request.trajectory_joint_values.empty() ||
    !request.trajectory_poses.empty();

  if (!has_single_state_input && !has_trajectory_input) {
    result.success = false;
    result.collision_free = false;
    result.message = "Collision request requires either joint values or a target pose.";
    result.failure_reason_code = "missing_collision_input";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  if (!request.trajectory_joint_values.empty()) {
    if (!request.trajectory_poses.empty()) {
      result.success = false;
      result.collision_free = false;
      result.message =
        "Collision request should specify either trajectory joint values or trajectory poses, not both.";
      result.failure_reason_code = "ambiguous_collision_input";
      result.failure_stage = FailureStage::kPrecheck;
      return result;
    }

    for (size_t index = 0; index < request.trajectory_joint_values.size(); ++index) {
      if (request.trajectory_joint_values[index].empty()) {
        result.success = false;
        result.collision_free = false;
        result.message = "Trajectory joint waypoint is empty at index " + std::to_string(index) +
          ".";
        result.failure_reason_code = "invalid_joint_values";
        result.failure_stage = FailureStage::kPrecheck;
        return result;
      }
    }
  }

  std::string initialization_error;
  if (!ensureMoveGroup(request.planning_group, initialization_error)) {
    result.success = false;
    result.collision_free = false;
    result.message = "Failed to initialize MoveGroup for collision check: " + initialization_error;
    result.failure_reason_code = "planner_init";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  const auto robot_model = move_group_->getRobotModel();
  const auto * joint_model_group =
    robot_model ? robot_model->getJointModelGroup(move_group_name_) : nullptr;
  if (!robot_model || !joint_model_group) {
    result.success = false;
    result.collision_free = false;
    result.message = "Unable to resolve planning group for collision check: " + move_group_name_;
    result.failure_reason_code = "missing_joint_model_group";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  // Same fix as ik(): every caller below (evaluate_pose/evaluate_joint_values)
  // fully overwrites the group's joint positions before checking, so this
  // never needs to start from the robot's true live state -- only skips the
  // ~10s getCurrentState() stall in environments where the
  // CurrentStateMonitor never completes.
  auto state = std::make_shared<moveit::core::RobotState>(robot_model);
  state->setToDefaultValues();
  state->update();

  const std::string end_effector_link = request.end_effector_link.empty() ?
    move_group_->getEndEffectorLink() :
    request.end_effector_link;

  auto evaluate_pose =
    [&](const geometry_msgs::msg::Pose & pose, const std::string & source_label) -> bool {
      geometry_msgs::msg::Pose resolved_pose = pose;
      if (!request.frame_id.empty()) {
        std::string resolve_error;
        if (!resolvePoseToPlanningFrame(request.frame_id, pose, resolved_pose, resolve_error)) {
          result.success = false;
          result.collision_free = false;
          result.message = resolve_error;
          result.failure_reason_code = "frame_resolution_failed";
          result.failure_stage = FailureStage::kPrecheck;
          return false;
        }
      }

      Eigen::Isometry3d target_transform = Eigen::Isometry3d::Identity();
      target_transform.translation() = Eigen::Vector3d(
        resolved_pose.position.x,
        resolved_pose.position.y,
        resolved_pose.position.z);
      const Eigen::Quaterniond quaternion(
        resolved_pose.orientation.w,
        resolved_pose.orientation.x,
        resolved_pose.orientation.y,
        resolved_pose.orientation.z);
      target_transform.linear() = quaternion.normalized().toRotationMatrix();

      const bool ik_found = state->setFromIK(
        joint_model_group,
        target_transform,
        end_effector_link,
        0.1);
      if (!ik_found) {
        result.success = false;
        result.collision_free = false;
        result.message = "Collision check aborted because IK failed for " + source_label + ".";
        result.failure_reason_code = "ik_failed";
        result.failure_stage = FailureStage::kPlanning;
        return false;
      }

      bool collision_free = true;
      std::string collision_error;
      if (!isStateCollisionFree(*state, request.planning_group, collision_free, collision_error)) {
        result.success = false;
        result.collision_free = false;
        result.message = collision_error;
        result.failure_reason_code = "collision_check_failed";
        result.failure_stage = FailureStage::kPrecheck;
        return false;
      }

      if (!collision_free) {
        result.success = true;
        result.collision_free = false;
        result.message = "Trajectory is in collision at " + source_label + ".";
        result.failure_reason_code = "collision_detected";
        result.failure_stage = FailureStage::kPlanning;
        return false;
      }

      return true;
    };

  auto evaluate_joint_values = [&](const std::vector<double> & joint_values,
      const std::string & source_label) -> bool {
      if (joint_values.size() != joint_model_group->getVariableCount()) {
        result.success = false;
        result.collision_free = false;
        result.message = "Joint value vector does not match planning group variable count for " +
          source_label + ".";
        result.failure_reason_code = "invalid_joint_values";
        result.failure_stage = FailureStage::kPrecheck;
        return false;
      }

      state->setJointGroupPositions(joint_model_group, joint_values);
      state->update();

      bool collision_free = true;
      std::string collision_error;
      if (!isStateCollisionFree(*state, request.planning_group, collision_free, collision_error)) {
        result.success = false;
        result.collision_free = false;
        result.message = collision_error;
        result.failure_reason_code = "collision_check_failed";
        result.failure_stage = FailureStage::kPrecheck;
        return false;
      }

      if (!collision_free) {
        result.success = true;
        result.collision_free = false;
        result.message = "Trajectory is in collision at " + source_label + ".";
        result.failure_reason_code = "collision_detected";
        result.failure_stage = FailureStage::kPlanning;
        return false;
      }

      return true;
    };

  if (!request.trajectory_joint_values.empty() || !request.trajectory_poses.empty()) {
    if (!request.trajectory_joint_values.empty()) {
      for (size_t index = 0; index < request.trajectory_joint_values.size(); ++index) {
        if (!evaluate_joint_values(
            request.trajectory_joint_values[index],
            "trajectory waypoint " + std::to_string(index)))
        {
          return result;
        }
      }
      result.success = true;
      result.collision_free = true;
      result.message = "Joint trajectory is collision-free.";
      result.failure_reason_code = "none";
      result.failure_stage = FailureStage::kNone;
      return result;
    }

    for (size_t index = 0; index < request.trajectory_poses.size(); ++index) {
      const auto pose = ros_adapters::toRos(request.trajectory_poses[index]);
      if (!evaluate_pose(pose, "trajectory waypoint " + std::to_string(index))) {
        return result;
      }
    }
    result.success = true;
    result.collision_free = true;
    result.message = "Pose trajectory is collision-free.";
    result.failure_reason_code = "none";
    result.failure_stage = FailureStage::kNone;
    return result;
  }

  if (!request.joint_values.empty()) {
    if (!evaluate_joint_values(request.joint_values, "joint state")) {
      return result;
    }
  } else {
    geometry_msgs::msg::Pose target_pose;
    std::string pose_error;
    if (!parseTargetPose(request.target_pose, target_pose, pose_error)) {
      result.success = false;
      result.collision_free = false;
      result.message = pose_error;
      result.failure_reason_code = "invalid_target_pose";
      result.failure_stage = FailureStage::kPrecheck;
      return result;
    }
    if (!evaluate_pose(target_pose, "target pose")) {
      return result;
    }
  }

  bool collision_free = true;
  std::string collision_error;
  if (!isStateCollisionFree(*state, request.planning_group, collision_free, collision_error)) {
    result.success = false;
    result.collision_free = false;
    result.message = collision_error;
    result.failure_reason_code = "collision_check_failed";
    result.failure_stage = FailureStage::kPrecheck;
    return result;
  }

  result.success = true;
  result.collision_free = collision_free;
  result.message = collision_free ? "State is collision-free." : "State is in collision.";
  result.failure_reason_code = collision_free ? "none" : "collision_detected";
  result.failure_stage = collision_free ? FailureStage::kNone : FailureStage::kPlanning;
  return result;
}

}  // namespace motion_planner
