// ROS shell: loads a map file and serves the reachability queries as services.
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <reachability_msgs/srv/get_base_candidates.hpp>
#include <reachability_msgs/srv/is_reachable.hpp>
#include <reachability_msgs/srv/plan_lift_bands.hpp>

#include "reachability/core/map4d.hpp"

namespace reachability
{

/// Parameters: `map_file` (required), `robot_fingerprint` (optional, must match the file's).
/// Services: ~/is_reachable, ~/get_base_candidates, ~/plan_lift_bands.
class ReachabilityNode : public rclcpp::Node
{
public:
  explicit ReachabilityNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool ready() const {return map_ != nullptr;}

private:
  std::shared_ptr<Map4D> map_;
  rclcpp::Service<reachability_msgs::srv::IsReachable>::SharedPtr is_reachable_;
  rclcpp::Service<reachability_msgs::srv::GetBaseCandidates>::SharedPtr base_candidates_;
  rclcpp::Service<reachability_msgs::srv::PlanLiftBands>::SharedPtr lift_bands_;
};

}  // namespace reachability
