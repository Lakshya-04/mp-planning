/**
 * @file moveit_planning_backend.hpp
 * @brief Shared MoveIt-backed CollisionPort/IkPort implementation.
 * @details Owns the single Node/MoveGroupInterface/PlanningSceneMonitor/
 *          PlanningSceneInterface that CartesianPlanner and OmplPlanner used
 *          to each construct independently -- confirmed live, that meant two
 *          fully separate mirrors of "what does the robot/world look like
 *          right now," with no shared ground truth (e.g. OmplPlanner's
 *          monitor could silently miss world geometry injected before its
 *          own monitor started, a gap CartesianPlanner's monitor didn't have
 *          -- see ensurePlanningSceneMonitor()). One MoveItPlanningBackend
 *          instance, injected into both planners' constructors, makes that
 *          structurally impossible: there is only one monitor to miss.
 *
 *          Implements both CollisionPort and IkPort (not two separate adapter
 *          classes) because IK validity-checking needs the exact same
 *          PlanningSceneMonitor collision-checking does -- splitting them
 *          would just recreate two copies of the ground truth this class
 *          exists to unify. Also exposes MoveIt-typed accessors/helpers
 *          (ensureMoveGroup(), the raw isStateCollisionFree() overload, node/
 *          moveGroup/planningSceneMonitor/planningSceneInterface accessors)
 *          for CartesianPlanner::plan()'s own internal needs (ACM diffs,
 *          time-parameterization, FK) that go beyond either port's contract.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"

#include "motion_planner/core/planners/collision_port.hpp"
#include "motion_planner/core/planners/ik_port.hpp"
#include "motion_planner/core/planners/seed_preferring_ik.hpp"

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
class JointModelGroup;
class RobotState;
}  // namespace moveit::core

namespace planning_scene_monitor
{
class PlanningSceneMonitor;
}  // namespace planning_scene_monitor

namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace motion_planner
{

class MoveItPlanningBackend : public CollisionPort, public IkPort
{
public:
  MoveItPlanningBackend();

  // CollisionPort / IkPort.
  CollisionResult collision(const CollisionRequest & request) override;
  /// With a seed and ik_extra_seeds > 0 (node parameter, or setIkSeeding) this retries the solve
  /// from perturbed seeds and returns the solution closest to the seed, wrapped by whole turns
  /// and inside the joint limits (SeedPreferringIk). Otherwise one plain solve, as before.
  IkResult ik(const IkRequest & request) override;

  /// One metre of a prismatic joint counts as this many radians when IK solutions are compared
  /// (the 0.3 rad / 0.05 m jump thresholds of docs/design_principles.md give 6). Parameter
  /// `ik_prismatic_factor` overrides it.
  static constexpr double kDefaultPrismaticIkFactor = 6.0;

  /// Fills limits, the revolute mask and the distance weights of `config` from the group's active
  /// joints, in group order. Prismatic joints are not wrapped and get weight factor^2 (the weight
  /// multiplies the squared difference, so 1 m weighs `prismatic_factor` rad); other joints weigh
  /// 1. A joint without position bounds (continuous) gets infinite limits. False for a group
  /// containing a multi-variable joint. Public so it is testable with a bare RobotModel, no live node.
  static bool seedingConfigFromGroup(
    const moveit::core::JointModelGroup * group, SeedPreferringIkConfig & config,
    double prismatic_factor = kDefaultPrismaticIkFactor);

  /// Overrides the ik_extra_seeds / ik_seed_perturbation parameters. 0 extra seeds = off.
  void setIkSeeding(int extra_seeds, double perturbation);

  /// Lazily constructs the shared Node + MoveGroupInterface for
  /// planning_group (or reuses the existing one if the group hasn't
  /// changed). Always sets the "ompl" planning pipeline -- harmless to a
  /// caller that never invokes MoveGroupInterface::plan() directly (e.g.
  /// CartesianPlanner, which plans locally via CartesianInterpolator).
  bool ensureMoveGroup(const std::string & planning_group, std::string & error_message);
  /// Lazily constructs the shared PlanningSceneMonitor, including one
  /// synchronous requestPlanningSceneState() pull so world geometry injected
  /// before this call is never silently invisible to subsequent collision
  /// checks (the actual fix this class exists to provide -- see the file
  /// comment above).
  bool ensurePlanningSceneMonitor(const std::string & planning_group, std::string & error_message);
  bool ensurePlanningSceneInterface(std::string & error_message);
  /// Raw MoveIt-typed collision check, used both by collision()/ik()
  /// (through CollisionRequest/IkRequest) and directly by CartesianPlanner's
  /// own GroupStateValidityCallbackFn lambdas during its local Cartesian
  /// walk, which need a per-step validity callback this port's PoD-typed
  /// contract doesn't expose.
  bool isStateCollisionFree(
    moveit::core::RobotState & state,
    const std::string & planning_group,
    bool & collision_free,
    std::string & error_message);
  /// Resolves `pose_in_source` (expressed in `source_frame`) into MoveIt's actual planning frame
  /// via TF, so PlanningRequest::frame_id / IkRequest::frame_id / CollisionRequest::frame_id are
  /// honored instead of silently ignored -- confirmed live: this was a real, total gap (this
  /// class never resolved frame_id at all), so a caller naming a TF-anchored frame numerically
  /// far from the planning frame (e.g. a wall mounted near base_link but expressed in its own
  /// captured coordinates) got those raw numbers treated as literal planning-frame offsets.
  /// No-op (returns `pose_in_source` unchanged) when `source_frame` is empty or already equals
  /// the planning frame -- the common case, and the only case every existing caller exercised
  /// before this was added. Requires ensureMoveGroup() to have already succeeded. Lazily
  /// constructs a tf2_ros::Buffer/TransformListener on node() the first time a real transform is
  /// needed.
  bool resolvePoseToPlanningFrame(
    const std::string & source_frame,
    const geometry_msgs::msg::Pose & pose_in_source,
    geometry_msgs::msg::Pose & pose_in_planning_frame,
    std::string & error_message);

  std::shared_ptr<rclcpp::Node> node() const;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup() const;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planningSceneMonitor() const;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planningSceneInterface() const;
  const std::string & activeMoveGroupName() const;

  /// Converts an IkRequest/PlanningRequest's flat target_pose vector into a geometry_msgs Pose --
  /// 7 values (xyz + xyzw quaternion) pass through directly, 6 values (xyz + rpy) convert via
  /// tf2::Quaternion::setRPY(). Public (rather than the private helper it started as) so both
  /// branches are directly unit-testable without needing a live rclcpp node/MoveGroupInterface,
  /// which ik()'s own ensureMoveGroup() precheck would otherwise require first.
  static bool parseTargetPose(
    const std::vector<double> & pose_values,
    geometry_msgs::msg::Pose & pose,
    std::string & error_message);

private:
  /// One MoveIt solve from the request's seed; what ik() did before seed preferring.
  IkResult solveIkOnce(const IkRequest & request);
  /// Limits / revolute mask of the active group, for SeedPreferringIk. False if unavailable.
  bool seedingConfigForGroup(SeedPreferringIkConfig & config, double prismatic_factor);

  std::shared_ptr<rclcpp::Node> node_;
  int ik_extra_seeds_{0};
  double ik_seed_perturbation_{0.3};
  bool ik_seeding_overridden_{false};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::string move_group_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace motion_planner
