#include "reachability/ros/reachability_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include "reachability/core/io.hpp"
#include "reachability/core/lift_bands.hpp"
#include "reachability/core/stands.hpp"

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

constexpr double kMinGridStep = 0.02;    // m
constexpr double kMinLiftStep = 0.01;    // m
constexpr double kMaxStandWork = 5e7;    // station x height x target evaluations per greedy round
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
  const auto base_file = declare_parameter<std::string>("base_profile", "");
  if (!base_file.empty()) {
    try {
      base_ = loadBaseProfile(base_file);
      RCLCPP_INFO(get_logger(), "base profile %s: %s", base_file.c_str(), base_->name.c_str());
    } catch (const std::exception & e) {
      // planning without the footprint would silently place the base into the wall: stay not ready
      RCLCPP_ERROR(get_logger(), "cannot load base profile '%s': %s; node not ready", base_file.c_str(), e.what());
      map_.reset();
      return;
    }
  }

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
  stands_ = create_service<PlanStands>(
    "~/plan_stands",
    [this](const PlanStands::Request::SharedPtr req, PlanStands::Response::SharedPtr res) {
      if (req->targets_world.empty()) {
        res->message = "no targets";
        return;
      }
      const Eigen::Vector3d n(req->wall_normal.x, req->wall_normal.y, req->wall_normal.z);
      if (n.norm() < 1e-9) {
        res->message = "wall_normal must be non-zero";
        return;
      }
      StationRequest r;
      for (const auto & p : req->targets_world) {r.targets_world.push_back(toIso(p));}
      r.wall_point = Vec3(req->wall_point.x, req->wall_point.y, req->wall_point.z);
      r.wall_normal = n.normalized();
      auto pos = [](double v, double dflt) {return v > 0.0 ? v : dflt;};
      const StationRequest d;
      r.min_clearance = pos(req->min_clearance, d.min_clearance);
      r.max_standoff = pos(req->max_standoff, d.max_standoff);
      r.grid_step = pos(req->grid_step, d.grid_step);
      r.lateral_margin = pos(req->lateral_margin, d.lateral_margin);
      r.lift_min = req->lift_min;
      r.lift_max = req->lift_max != 0.0 ? req->lift_max : d.lift_max;
      r.lift_step = pos(req->lift_step, d.lift_step);
      if (r.lift_max < r.lift_min) {
        res->message = "lift_max (default " + std::to_string(d.lift_max) + ") must be >= lift_min";
        return;
      }
      if (r.grid_step < kMinGridStep || r.lift_step < kMinLiftStep) {
        res->message = "grid_step must be >= " + std::to_string(kMinGridStep) + " m and lift_step >= " +
          std::to_string(kMinLiftStep) + " m";
        return;
      }
      // the search is stations x lift heights x targets per round: refuse requests far beyond a real wall
      double x_lo = 1e18, x_hi = -1e18, y_lo = 1e18, y_hi = -1e18;
      for (const auto & t : r.targets_world) {
        x_lo = std::min(x_lo, t.translation().x());
        x_hi = std::max(x_hi, t.translation().x());
        y_lo = std::min(y_lo, t.translation().y());
        y_hi = std::max(y_hi, t.translation().y());
      }
      const double span = std::hypot(x_hi - x_lo, y_hi - y_lo) + 2.0 * r.lateral_margin;
      const double stations = ((r.max_standoff - r.min_clearance) / r.grid_step + 1.0) * (span / r.grid_step + 1.0);
      const double heights = (r.lift_max - r.lift_min) / r.lift_step + 1.0;
      const double work = stations * heights * static_cast<double>(r.targets_world.size());
      if (!std::isfinite(work) || work > kMaxStandWork) {
        res->message = "search too large (" + std::to_string(work) + " station-height-target evaluations per stand, "
          "limit " + std::to_string(kMaxStandWork) + "): raise grid_step or lift_step, or narrow the search";
        return;
      }
      r.arm_mount_offset_z = req->arm_mount_offset_z;
      r.base = base_;
      std::optional<Stand> start;
      if (req->has_start) {
        start = Stand{Eigen::Vector2d(req->start_x, req->start_y), req->start_yaw};
      }
      const StandPlan plan = req->max_stands > 0 ? planStands(*map_, r, start, req->max_stands) :
                                                 planStands(*map_, r, start);
      for (const auto & a : plan.stands) {
        reachability_msgs::msg::Stand m;
        m.x = a.stand.arm_xy.x();
        m.y = a.stand.arm_xy.y();
        m.yaw = a.stand.yaw;
        m.standoff = a.station.standoff;
        m.footprint_clearance = a.station.footprint_clearance;
        m.travel_s = a.travel_s;
        m.target_indices.assign(a.targets.begin(), a.targets.end());
        for (const auto & b : a.station.plan.bands) {
          reachability_msgs::msg::LiftBand lb;
          lb.lift_height = b.lift_height;
          lb.pose_indices.assign(b.pose_indices.begin(), b.pose_indices.end());
          m.bands.push_back(lb);
        }
        res->stands.push_back(m);
      }
      res->unreachable.assign(plan.unreachable.begin(), plan.unreachable.end());
      res->not_planned.assign(plan.not_planned.begin(), plan.not_planned.end());
      res->total_travel_s = plan.total_travel_s;
      res->success = plan.unreachable.empty() && plan.not_planned.empty() && !plan.stands.empty();
      if (!res->success) {
        res->message = std::to_string(plan.unreachable.size()) + " target(s) no stand reaches, " +
          std::to_string(plan.not_planned.size()) + " not planned (stand cap)";
      }
    });
}

}  // namespace reachability
