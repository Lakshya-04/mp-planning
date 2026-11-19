#include "reachability_moveit/port_stand_verifier.hpp"

#include <cmath>

namespace reachability_moveit
{
using reachability::Pose;
using reachability::Stand;
using reachability::TargetVerdict;
using reachability::VerifyReason;

namespace
{
/// Base_footprint pose of a stand in the world: yaw about z, origin so the arm base lands on stand.arm_xy.
Eigen::Isometry3d baseInWorld(const PortVerifierConfig & c, const Stand & s)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(s.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector2d mount = c.arm_base_in_plan.translation().head<2>();
  const Eigen::Vector2d origin = s.arm_xy - T.linear().topLeftCorner<2, 2>() * mount;
  T.translation() = Eigen::Vector3d(origin.x(), origin.y(), 0.0);
  return T;
}

TargetVerdict failure(VerifyReason r) {TargetVerdict v; v.reason = r; return v;}
}  // namespace

PortStandVerifier::PortStandVerifier(
  PortVerifierConfig config, std::shared_ptr<motion_planner::IkPort> ik,
  std::shared_ptr<motion_planner::CollisionPort> collision, std::shared_ptr<motion_planner::ScenePort> scene)
: cfg_(std::move(config)), ik_(std::move(ik)), collision_(std::move(collision)), scene_(std::move(scene))
{
}

std::vector<double> PortStandVerifier::ikTarget(
  const PortVerifierConfig & c, const Stand & stand, double lift, const Pose & target_world)
{
  const Eigen::Isometry3d plan_target = baseInWorld(c, stand).inverse() * target_world;
  Eigen::Isometry3d arm_at_lift = c.arm_base_in_plan;
  arm_at_lift.translation().z() += lift;
  // the arm group keeps the lift at 0: the target moves down by the lift height instead
  const Eigen::Isometry3d ik = c.arm_base_in_plan * arm_at_lift.inverse() * plan_target;
  const Eigen::Quaterniond q(ik.linear());
  return {ik.translation().x(), ik.translation().y(), ik.translation().z(), q.x(), q.y(), q.z(), q.w()};
}

bool PortStandVerifier::poseScene(const Stand & stand, std::string & error)
{
  if (!scene_) {return true;}
  if (have_scene_stand_ && (scene_stand_.arm_xy - stand.arm_xy).norm() < 1e-9 && std::abs(scene_stand_.yaw - stand.yaw) < 1e-9) {
    return true;
  }
  const Eigen::Isometry3d to_base = baseInWorld(cfg_, stand).inverse();
  motion_planner::WorldGeometryUpdate u;
  u.frame_id = cfg_.scene_frame;
  for (const auto & p : cfg_.points) {
    const Eigen::Vector3d q = to_base * Eigen::Vector3d(p.x, p.y, p.z);
    u.points.push_back({q.x(), q.y(), q.z()});
  }
  for (auto b : cfg_.boxes) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(b.center_x, b.center_y, b.center_z);
    T.linear() = Eigen::Quaterniond(b.orientation_w, b.orientation_x, b.orientation_y, b.orientation_z).normalized().toRotationMatrix();
    const Eigen::Isometry3d R = to_base * T;
    const Eigen::Quaterniond q(R.linear());
    b.center_x = R.translation().x();
    b.center_y = R.translation().y();
    b.center_z = R.translation().z();
    b.orientation_x = q.x();
    b.orientation_y = q.y();
    b.orientation_z = q.z();
    b.orientation_w = q.w();
    u.boxes.push_back(b);
  }
  if (!scene_->updateWorldGeometry(u, error)) {return false;}
  ++scene_pushes_;
  have_scene_stand_ = true;
  scene_stand_ = stand;
  return true;
}

std::vector<TargetVerdict> PortStandVerifier::verify(
  const Stand & stand, double lift_height, const std::vector<Pose> & targets_world)
{
  std::vector<TargetVerdict> out(targets_world.size(), failure(VerifyReason::kError));
  if (!ik_ || !collision_) {return out;}
  if (lift_height < cfg_.lift_min - 1e-9 || lift_height > cfg_.lift_max + 1e-9) {
    return std::vector<TargetVerdict>(targets_world.size(), failure(VerifyReason::kOutOfLimits));
  }
  std::string error;
  if (!poseScene(stand, error)) {return out;}   // no honest answer without the scene: error, never ok

  const std::vector<std::vector<double>> no_seed{{}};
  const auto & seeds = cfg_.seeds.empty() ? no_seed : cfg_.seeds;
  for (size_t i = 0; i < targets_world.size(); ++i) {
    motion_planner::IkRequest req;
    req.frame_id = cfg_.ik_frame;
    req.planning_group = cfg_.ik_group;
    req.end_effector_link = cfg_.ik_link;
    req.target_pose = ikTarget(cfg_, stand, lift_height, targets_world[i]);
    VerifyReason worst = VerifyReason::kIkFailed;
    bool answered = false;
    for (const auto & seed : seeds) {
      req.joint_seed = seed;
      ++ik_calls_;
      const auto r = ik_->ik(req);
      if (!r.success) {
        if (r.failure_reason_code != "ik_failed") {worst = answered ? worst : VerifyReason::kError;}
        continue;
      }
      answered = true;
      bool limits_ok = true;
      for (size_t j = 0; j < r.joint_solution.size(); ++j) {
        if (j < cfg_.joint_lower.size() && r.joint_solution[j] < cfg_.joint_lower[j] - 1e-6) {limits_ok = false;}
        if (j < cfg_.joint_upper.size() && r.joint_solution[j] > cfg_.joint_upper[j] + 1e-6) {limits_ok = false;}
      }
      if (!limits_ok) {
        if (worst != VerifyReason::kInCollision) {worst = VerifyReason::kOutOfLimits;}
        continue;
      }
      motion_planner::CollisionRequest cr;
      cr.planning_group = cfg_.collision_group;
      cr.end_effector_link = cfg_.ik_link;
      if (cfg_.collision_includes_lift) {cr.joint_values.push_back(lift_height);}
      cr.joint_values.insert(cr.joint_values.end(), r.joint_solution.begin(), r.joint_solution.end());
      ++collision_calls_;
      const auto c = collision_->collision(cr);
      if (!c.success) {worst = VerifyReason::kError; continue;}
      if (!c.collision_free) {worst = VerifyReason::kInCollision; continue;}
      out[i].reason = VerifyReason::kOk;
      out[i].joints = r.joint_solution;
      break;
    }
    if (out[i].reason != VerifyReason::kOk) {out[i].reason = worst;}
  }
  return out;
}

}  // namespace reachability_moveit
