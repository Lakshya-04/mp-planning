// StandVerifier over the motion_planner ports: IK (IkPort), collision (CollisionPort) and the obstacle scene
// (ScenePort). Works with any backend that implements them (MoveIt today); ROS-free itself.
//
// How a stand becomes a backend query. The backend's robot sits where its scene says, so the stand is
// expressed by moving the TARGET instead of the base:
//   world target -> base_footprint frame of the stand -> IK target for the arm group with the lift at 0
// (the arm group's lift is locked at its default, 0). The lift height then enters twice: the IK target is
// shifted down by it, and the collision check runs on the whole-body group with the lift joint set to it,
// so lift parts and the arm are checked where they really are. World obstacles (boxes, a wall scan) are
// re-posed into the stand's base frame and pushed through the ScenePort whenever the stand changes, so
// the scene the collision check sees is the one the robot would see from that stand.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "motion_planner/core/planners/collision_port.hpp"
#include "motion_planner/core/planners/ik_port.hpp"
#include "motion_planner/core/scene_port.hpp"
#include "reachability/core/stand_verifier.hpp"

namespace reachability_moveit
{

struct PortVerifierConfig
{
  std::string ik_group{"arm"};                 ///< Arm joints only, lift locked (profile groups.arm).
  std::string ik_link{"tcp_link"};             ///< Frame the target pose is for (profile frames.tcp).
  std::string ik_frame;                        ///< Passed as IkRequest::frame_id; "" = the backend's planning frame.
  std::string collision_group{"lift_arm"};     ///< Whole-body group (profile groups.whole_body).
  /// true: collision joint values are [lift, arm joints...] (a lift_arm group); false: arm joints only.
  bool collision_includes_lift{true};
  /// Arm base link in the planning frame (base_footprint) with the lift at 0. Its x, y is the arm mount
  /// (base arm_mount_xy); its z is the height reachability calls arm_mount_offset_z; the rotation is the arm
  /// base's yaw relative to the base. The planning frame is taken to be the base_footprint.
  Eigen::Isometry3d arm_base_in_plan{Eigen::Isometry3d::Identity()};
  std::vector<double> joint_lower;             ///< Arm joint limits for the out_of_limits check (empty = not checked).
  std::vector<double> joint_upper;
  double lift_min{0.0};                        ///< Lift travel; heights outside are out_of_limits.
  double lift_max{1.5};
  /// IK seeds tried in turn until one gives a collision-free solution (an empty list = one solve, no seed).
  std::vector<std::vector<double>> seeds;
  /// World obstacles and wall scan (world frame), re-posed per stand through the ScenePort. Null scene = the
  /// backend's own scene is used as is (valid for one fixed stand only).
  std::vector<motion_planner::WorldGeometryBox> boxes;
  std::vector<motion_planner::WorldGeometryPoint> points;
  std::string scene_frame{"base_footprint"};   ///< frame_id of the pushed geometry.
};

class PortStandVerifier : public reachability::StandVerifier
{
public:
  /// `scene` may be null. The ports must outlive the verifier.
  PortStandVerifier(
    PortVerifierConfig config, std::shared_ptr<motion_planner::IkPort> ik,
    std::shared_ptr<motion_planner::CollisionPort> collision, std::shared_ptr<motion_planner::ScenePort> scene = nullptr);

  std::vector<reachability::TargetVerdict> verify(
    const reachability::Stand & stand, double lift_height, const std::vector<reachability::Pose> & targets_world) override;

  /// The IK target (xyz + quaternion x y z w) for a world target, exposed for tests.
  static std::vector<double> ikTarget(
    const PortVerifierConfig & config, const reachability::Stand & stand, double lift_height,
    const reachability::Pose & target_world);

  size_t ikCalls() const {return ik_calls_;}
  size_t collisionCalls() const {return collision_calls_;}
  size_t scenePushes() const {return scene_pushes_;}

private:
  bool poseScene(const reachability::Stand & stand, std::string & error);

  PortVerifierConfig cfg_;
  std::shared_ptr<motion_planner::IkPort> ik_;
  std::shared_ptr<motion_planner::CollisionPort> collision_;
  std::shared_ptr<motion_planner::ScenePort> scene_;
  bool have_scene_stand_{false};
  reachability::Stand scene_stand_;
  size_t ik_calls_{0};
  size_t collision_calls_{0};
  size_t scene_pushes_{0};
};

}  // namespace reachability_moveit
