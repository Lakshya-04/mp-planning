#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "reachability/ros/reachability_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<reachability::ReachabilityNode>();
  if (!node->ready()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
