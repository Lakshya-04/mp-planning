/**
 * @file world_model_loader_node.cpp
 * @brief Live test/authoring tool: loads a static .pcd/.ply file once and
 *        injects it into move_group's planning scene via MoveItSceneAdapter,
 *        then exits. One-shot by design -- the injected octomap is a
 *        synchronous, persistent mutation of move_group's own scene (the
 *        same idiom CartesianPlanner's ACM-diff push already relies on), so
 *        nothing about it depends on this process staying alive. This also
 *        makes the node usable as a launch-sequencing step (e.g. via
 *        OnProcessExit) ahead of whatever should see the injected geometry.
 * @details Stands in for a live perception feed until one is merged --
 *          not a production node, a synthetic-obstacle authoring/test tool.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "motion_planner_moveit/moveit_scene_adapter.hpp"
#include "motion_planner/core/point_cloud_file_loader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("world_model_loader");

  const std::string point_cloud_path =
    node->declare_parameter<std::string>("point_cloud_path", "");
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "base_link");
  const double octomap_resolution =
    node->declare_parameter<double>("octomap_resolution", 0.05);

  if (point_cloud_path.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "world_model_loader: 'point_cloud_path' parameter is required.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for 'move_group' node to start...");
  bool move_group_ready = false;
  while (rclcpp::ok()) {
    for (const auto & name : node->get_node_names()) {
      if (name.find("move_group") != std::string::npos) {
        move_group_ready = true;
        break;
      }
    }
    if (move_group_ready) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  int exit_code = 1;

  if (move_group_ready && rclcpp::ok()) {
    try {
      const motion_planner::WorldGeometryUpdate update =
        motion_planner::loadPointCloudFile(point_cloud_path, frame_id);
      RCLCPP_INFO(
        node->get_logger(), "Loaded %zu points from %s",
        update.points.size(), point_cloud_path.c_str());

      motion_planner::MoveItSceneAdapter scene_adapter(octomap_resolution);
      std::string error_message;
      if (scene_adapter.updateWorldGeometry(update, error_message)) {
        RCLCPP_INFO(node->get_logger(), "World geometry injected into the planning scene.");
        exit_code = 0;
      } else {
        RCLCPP_ERROR(
          node->get_logger(), "Failed to inject world geometry: %s", error_message.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node->get_logger(), "world_model_loader failed to load point cloud: %s",
        e.what());
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "Shutdown requested before move_group became ready.");
  }

  rclcpp::shutdown();
  return exit_code;
}
