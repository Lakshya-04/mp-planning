#include "reachability/ros/reachability_node.hpp"

#include <exception>
#include <vector>

#include "reachability/core/io.hpp"
#include "reachability/core/lift_bands.hpp"

namespace reachability
{
namespace
{
Pose toIso(const geometry_msgs::msg::Pose & p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (q.norm() < 1e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  Pose t = Pose::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = Vec3(p.position.x, p.position.y, p.position.z);
  return t;
}
}  // namespace

ReachabilityNode::ReachabilityNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("reachability", options)
{
  const auto file = declare_parameter<std::string>("map_file", "");
  const auto fingerprint = declare_parameter<std::string>("robot_fingerprint", "");
  try {
    map_ = std::make_shared<Map4D>(loadMap(file, fingerprint));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cannot load map '%s': %s", file.c_str(), e.what());
    return;
  }
  RCLCPP_INFO(get_logger(), "loaded map %s (%zu non-zero cells)", file.c_str(), map_->nonzeroCells());

  using namespace reachability_msgs::srv;
  is_reachable_ = create_service<IsReachable>(
    "~/is_reachable",
    [this](const IsReachable::Request::SharedPtr req, IsReachable::Response::SharedPtr res) {
      const Pose t = toIso(req->tool_in_arm_base);
      res->count = map_->cellCount(t);
      res->reachable = res->count > 0;
      if (!map_->keyOf(t).valid) {
        res->message = "pose is outside the map window";
      }
    });
  base_candidates_ = create_service<GetBaseCandidates>(
    "~/get_base_candidates",
    [this](const GetBaseCandidates::Request::SharedPtr req, GetBaseCandidates::Response::SharedPtr res) {
      const int ring = req->ring_samples > 0 ? static_cast<int>(req->ring_samples) : 24;
      for (const auto & c : map_->inverseQuery(toIso(req->tool_in_world), req->arm_base_z, ring)) {
        reachability_msgs::msg::BaseCandidate m;
        m.x = c.x;
        m.y = c.y;
        m.count = c.count;
        res->candidates.push_back(m);
      }
      res->success = !res->candidates.empty();
      if (!res->success) {
        res->message = "no base position reaches this pose";
      }
    });
  lift_bands_ = create_service<PlanLiftBands>(
    "~/plan_lift_bands",
    [this](const PlanLiftBands::Request::SharedPtr req, PlanLiftBands::Response::SharedPtr res) {
      if (req->step <= 0.0 || req->lift_max < req->lift_min) {
        res->message = "need step > 0 and lift_max >= lift_min";
        return;
      }
      std::vector<Pose> targets;
      targets.reserve(req->targets_world.size());
      for (const auto & p : req->targets_world) {
        targets.push_back(toIso(p));
      }
      const auto plan = planLiftBands(
        *map_, targets, req->base_x, req->base_y,
        heightRange(req->lift_min, req->lift_max, req->step), req->arm_mount_offset_z);
      for (const auto & b : plan.bands) {
        reachability_msgs::msg::LiftBand m;
        m.lift_height = b.lift_height;
        m.pose_indices.assign(b.pose_indices.begin(), b.pose_indices.end());
        res->bands.push_back(m);
      }
      res->unreachable.assign(plan.unreachable.begin(), plan.unreachable.end());
      res->success = plan.unreachable.empty();
      if (!res->success) {
        res->message = std::to_string(plan.unreachable.size()) + " target(s) unreachable from this station";
      }
    });
}

}  // namespace reachability
