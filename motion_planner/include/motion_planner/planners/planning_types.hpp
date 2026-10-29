/**
 * @file planning_types.hpp
 * @brief Request/result POD types exchanged with the Planner port.
 * @details Poses use core_types::Pose, not a ROS message type, so this
 *          header (and the Planner interface built on it) stays ROS-optional.
 *          Adapters (e.g. OmplPlanner) convert to/from geometry_msgs at their
 *          own boundary via motion_planner::toRos()/toCore().
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "core_types/pose.hpp"
#include "core_types/trajectory.hpp"

namespace motion_planner
{

/**
 * @enum FailureStage
 * @brief Which stage of planning/execution a failure occurred in.
 */
enum class FailureStage : uint8_t
{
  kNone = 0,
  kPrecheck,
  kPlanning,
  kExecution,
};

/**
 * @enum CartesianTruncationReason
 * @brief Why a CartesianPlanner result's path_fraction is below 1.0.
 * @details MoveIt's own client-side computeCartesianPath() API doesn't expose this --
 *          jump-threshold truncation, a genuine collision, and a failed IK solve all just
 *          shorten the returned path, indistinguishably, from the caller's point of view.
 *          CartesianPlanner now computes the path locally specifically so it can tell these
 *          apart for real (see cartesian_planner.cpp's plan()) -- callers (MotionPlanner) use
 *          this to decide whether a recovery strategy applies (e.g. bridging a jump via FSP)
 *          rather than treating every partial result the same way.
 */
enum class CartesianTruncationReason : uint8_t
{
  kNone = 0,          ///< path_fraction is 1.0 (or planning failed before reaching Step 4 at all).
  kJumpDiscontinuity, ///< A joint-space jump between redundant-DOF IK solutions was detected.
  kCollision,         ///< A valid IK solution existed but was in collision.
  kIkFailure,         ///< No IK solution existed at all, with or without constraints relaxed.
};

/**
 * @struct PlanningRequest
 * @brief Input to Planner::plan() for a single motion segment.
 */
struct PlanningRequest
{
  std::string mode;               ///< Resolved motion mode ("cartesian"|"fsp").
  std::string planner;            ///< Resolved planner name for this segment.
  std::string frame_id;           ///< Reference frame for target_poses.
  std::string planning_group;     ///< MoveIt planning group to plan for.
  std::vector<size_t> waypoint_indices;    ///< Indices into the caller's waypoint list.
  std::vector<core_types::Pose> target_poses;  ///< Poses for each waypoint index.

  /// Direct joint-space goal, as an alternative to target_poses. Empty =
  /// today's pose-target behavior. When non-empty, must match the planning
  /// group's joint count. Only meaningful for FSP/OmplPlanner -- a joint
  /// configuration isn't a Cartesian-path concept, so CartesianPlanner
  /// ignores this field entirely (still requires target_poses).
  std::vector<double> target_joint_state;

  /// Explicit seed for the planner's start state. Empty = today's behavior
  /// (query current/default state). Non-empty must match the planning
  /// group's joint count; used to thread continuity segment-to-segment
  /// instead of an implicit live-state query.
  std::vector<double> start_joint_state;

  /// CartesianPlanner-specific planner parameters (ignored by OmplPlanner
  /// unless noted otherwise). See Cartesian_Planner_Architecture.md Part A.
  double eef_step{0.0};            ///< computeCartesianPath step (m); 0 = planner default.
  /// Absolute jump-detection thresholds (0 = use CartesianPlanner's own
  /// default, not "disabled" -- jump detection is always on now; see
  /// cartesian_planner.cpp). Separate revolute (radians) / prismatic (meters)
  /// thresholds match this robot's mixed joint chain (6 revolute arm joints +
  /// 1 prismatic lift joint) -- a single relative "factor" scalar (the only
  /// option MoveGroupInterface::computeCartesianPath()'s high-level API
  /// exposes) doesn't handle mixed units well.
  double revolute_jump_threshold{0.0};   ///< Max per-step revolute joint delta (rad); 0 = planner default.
  double prismatic_jump_threshold{0.0};  ///< Max per-step prismatic joint delta (m); 0 = planner default.
  double min_path_fraction{0.5};   ///< Below this achieved fraction, retry with relaxed params.
  double target_eef_speed{0.0};    ///< Constant EE speed (m/s) for time-parameterization; 0 = off.

  /// Contact-aware collision handling (CartesianPlanner: ACM workaround +
  /// post-hoc bound; OmplPlanner: max_wall_penetration/wall_reference_pose
  /// only, no ACM -- see Context/Part B.5 in Cartesian_Planner_Architecture.md).
  bool allow_target_contact{false};             ///< Final waypoint's target surface is intentional contact.
  std::string target_collision_object_id;       ///< Named object the ACM exclusion applies to (never wall/tool-wide).
  core_types::Pose wall_reference_pose;     ///< Point on wall plane (position) + wall normal (local +Z).
  double max_wall_penetration{0.0};             ///< Max allowed signed depth past wall_reference_pose's plane.
  /// Link the ACM exclusion is scoped to (core_types::ToolGeometry::contact_allowed_links'
  /// first entry, populated by the caller from the active ToolProfile) -- deliberately NOT always
  /// the same link as the wall-penetration-bound FK check above, which stays keyed on the
  /// planning group's own end-effector link (the coverage path's actual controlled point). A
  /// tool's real collision geometry commonly lives on a different, separately-attached link (e.g.
  /// this robot's `other_geometry_link`, fixed-jointed off `link_eef`) -- excluding the
  /// end-effector link alone protects nothing if that link carries no collision geometry itself.
  /// Empty (default) falls back to the end-effector link, matching this field's pre-existing
  /// behavior for every caller that hasn't been updated to set it yet.
  std::string contact_link;

  /// Reachability-tilt retry tier (CartesianPlanner only; see cartesian_planner.cpp's plan()):
  /// when a waypoint's ideal orientation is kinematically unreachable
  /// (CartesianTruncationReason::kIkFailure) and this is > 0.0, retry with a small set of tilted
  /// candidate orientations (up to this many degrees off the ideal) before giving up on that
  /// waypoint. 0.0 (default) disables the tier entirely -- today's kIkFailure behavior
  /// (unchanged) is a dead end. Populate from the active tool's
  /// core_types::ToolTask::reachability_tilt_deg_max.
  double reachability_tilt_deg_max{0.0};
  /// Hard safety exclusion: reachability tilt is NEVER attempted when true, regardless of
  /// reachability_tilt_deg_max -- tilting a tool meant to be in contact with the wall changes the
  /// force/contact-normal semantics, not just reach (core_types::ToolTask::contact's own
  /// invariant). Checked explicitly here rather than trusting reachability_tilt_deg_max to default
  /// to 0 for contact profiles alone -- defense in depth against a profile author setting both.
  bool tool_contact{false};
};

/**
 * @struct PlanningResult
 * @brief Outcome of Planner::plan(). Planners plan only -- they never execute
 *        (see Planner::plan()'s doc comment) -- so this carries the planned
 *        trajectory back to the caller instead of an execution_time_ms field;
 *        execution timing lives on ExecutionResult (execution_port.hpp).
 */
struct PlanningResult
{
  bool success{false};                            ///< True if planning succeeded.
  std::string status;                              ///< Human-readable status string.
  std::string message;                             ///< Additional detail/error message.
  size_t waypoints_completed{0};                   ///< Waypoints actually reached.
  double path_fraction{0.0};                       ///< Achieved fraction of the requested path (0-1).
  core_types::Trajectory trajectory;           ///< Planned trajectory, empty on failure.
  std::string failure_reason_code{"none"};         ///< Machine-readable failure reason.
  FailureStage failure_stage{FailureStage::kNone}; ///< Stage the failure occurred in.
  uint64_t planning_time_ms{0};                    ///< Time spent planning.
  /// CartesianPlanner only: why path_fraction < 1.0 (kNone otherwise, including
  /// for OmplPlanner results, which don't have a partial-path concept).
  CartesianTruncationReason truncation_reason{CartesianTruncationReason::kNone};
};

/**
 * @struct IkRequest
 * @brief Input to Planner::ik() for a single inverse-kinematics query.
 */
struct IkRequest
{
  std::string frame_id;             ///< Reference frame for target_pose.
  std::string planning_group;       ///< MoveIt planning group to solve IK for.
  std::string end_effector_link;    ///< Link to solve the target pose for.
  std::vector<double> target_pose;  ///< Flat xyz+rpy(6) or xyz+quat(7) target.
  std::vector<double> joint_seed;   ///< Optional seed joint values for the solver.
};

/**
 * @struct IkResult
 * @brief Outcome of Planner::ik().
 */
struct IkResult
{
  bool success{false};                             ///< True if a valid IK solution was found.
  std::string message;                             ///< Additional detail/error message.
  std::string failure_reason_code{"none"};         ///< Machine-readable failure reason.
  FailureStage failure_stage{FailureStage::kNone}; ///< Stage the failure occurred in.
  std::vector<double> joint_solution;              ///< Solved joint positions; empty on failure.
};

/**
 * @struct CollisionRequest
 * @brief Input to Planner::collision(). Exactly one of joint_values,
 *        target_pose, trajectory_joint_values, or trajectory_poses should be
 *        populated -- the implementation rejects ambiguous combinations.
 */
struct CollisionRequest
{
  std::string frame_id;           ///< Reference frame for target_pose/trajectory_poses.
  std::string planning_group;     ///< MoveIt planning group to check collisions against.
  std::string end_effector_link;  ///< Link used when resolving target_pose via IK.
  std::vector<double> target_pose;   ///< Single-pose check input (flat xyz+rpy/xyz+quat).
  std::vector<double> joint_values;  ///< Single joint-state check input.
  std::vector<std::vector<double>> trajectory_joint_values;  ///< Joint-trajectory check input.
  std::vector<core_types::Pose> trajectory_poses;         ///< Pose-trajectory check input.
};

/**
 * @struct CollisionResult
 * @brief Outcome of Planner::collision().
 */
struct CollisionResult
{
  bool success{false};                             ///< True if the check itself ran successfully.
  bool collision_free{false};                      ///< True if no collision was detected.
  std::string message;                             ///< Additional detail/error message.
  std::string failure_reason_code{"none"};         ///< Machine-readable failure reason.
  FailureStage failure_stage{FailureStage::kNone}; ///< Stage the failure occurred in.
};

}  // namespace motion_planner
