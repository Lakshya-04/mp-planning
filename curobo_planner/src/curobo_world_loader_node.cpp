/**
 * @file curobo_world_loader_node.cpp
 * @brief One-shot tool: loads a .pcd/.ply file and sets it as the cuRobo planner node's world
 *        (clear:=true empties it). The counterpart of world_model_loader_node for the cuRobo
 *        backend, so a pass whose scene comes from a file lets cuRobo see the same obstacles
 *        MoveIt sees. Run it with the same frame_id as the MoveIt loader (the robot base frame
 *        the planner node was started with).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "curobo_planner/curobo_planner.hpp"
#include "motion_planner/core/point_cloud_file_loader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("curobo_world_loader");
  const std::string path = node->declare_parameter<std::string>("point_cloud_path", "");
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "base_link");
  const bool clear = node->declare_parameter<bool>("clear", false);
  curobo_planner::Options options;
  options.node_name = node->declare_parameter<std::string>("curobo_node", "/curobo_planner");
  options.base_frame = frame_id;
  if (path.empty() && !clear) {
    RCLCPP_ERROR(node->get_logger(), "'point_cloud_path' is required (or clear:=true)");
    rclcpp::shutdown();
    return 1;
  }
  // The client waits on service futures, so the node is spun by another thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  curobo_planner::CuroboClientPlanner client(node, options);
  std::string message;
  bool ok = false;
  try {
    if (clear) {
      ok = client.clearWorld(&message);
    } else {
      const auto update = motion_planner::loadPointCloudFile(path, frame_id);
      RCLCPP_INFO(node->get_logger(), "Loaded %zu points from %s", update.points.size(), path.c_str());
      ok = client.setWorld(update, &message);
    }
  } catch (const std::exception & e) {
    message = e.what();
  }
  if (ok) {
    RCLCPP_INFO(node->get_logger(), "cuRobo world %s", clear ? "cleared" : "set");
  } else {
    RCLCPP_ERROR(node->get_logger(), "cuRobo world update failed: %s", message.c_str());
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
