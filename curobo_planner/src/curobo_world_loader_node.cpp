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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "curobo_planner/curobo_planner.hpp"
#include "motion_planner/core/point_cloud_file_loader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("curobo_world_loader");
  const std::string path = node->declare_parameter<std::string>("point_cloud_path", "");
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "base_link");
  // Pose of the file's frame in frame_id (translation only): a cloud written in base_footprint, with
  // the planner's base frame base_link 0.31 m above it, is loaded with offset_z:=-0.31.
  const double offset_x = node->declare_parameter<double>("offset_x", 0.0);
  const double offset_y = node->declare_parameter<double>("offset_y", 0.0);
  const double offset_z = node->declare_parameter<double>("offset_z", 0.0);
  // source_frame set: the file is in that fixed frame (world or odom) and its pose in frame_id is looked up from TF
  // now, so a base that has moved to a new stand gets the wall where it is relative to the arm. The offsets above
  // are then ignored. Empty: the file is in frame_id, shifted by the offsets.
  const std::string source_frame = node->declare_parameter<std::string>("source_frame", "");
  const double tf_timeout_sec = node->declare_parameter<double>("tf_timeout_sec", 5.0);
  const bool clear = node->declare_parameter<bool>("clear", false);
  curobo_planner::Options options;
  options.node_name = node->declare_parameter<std::string>("curobo_node", "/curobo_planner");
  options.base_frame = frame_id;
  // A request can get no reply when the reply path is not matched yet (a ROS 2 service discovery race, seen while a
  // simulator starts on the same cores): try again with a short deadline instead of waiting out the planner default.
  options.timeout_sec = node->declare_parameter<double>("attempt_timeout_sec", 10.0);
  const int attempts = static_cast<int>(node->declare_parameter<int>("attempts", 5));
  if (path.empty() && !clear) {
    RCLCPP_ERROR(node->get_logger(), "'point_cloud_path' is required (or clear:=true)");
    rclcpp::shutdown();
    return 1;
  }
  // The client waits on service futures, so the node is spun by another thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, node, false);
  // the node is spun by the thread above: tell the buffer, or lookupTransform's timeout is ignored and it fails at once
  tf_buffer.setUsingDedicatedThread(true);
  curobo_planner::CuroboClientPlanner client(node, options);
  std::string message;
  bool ok = false;
  try {
    if (clear) {
      for (int k = 0; k < attempts && !ok; ++k) {
        ok = client.clearWorld(&message);
        if (!ok) {RCLCPP_WARN(node->get_logger(), "clear attempt %d failed: %s", k + 1, message.c_str());}
      }
    } else {
      auto update = motion_planner::loadPointCloudFile(path, frame_id);
      if (!source_frame.empty()) {
        // pose of source_frame expressed in frame_id; the points' own coordinates are in source_frame
        const auto tf = tf_buffer.lookupTransform(
          frame_id, source_frame, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec));
        update.points_origin.x = tf.transform.translation.x;
        update.points_origin.y = tf.transform.translation.y;
        update.points_origin.z = tf.transform.translation.z;
        update.points_origin.qx = tf.transform.rotation.x;
        update.points_origin.qy = tf.transform.rotation.y;
        update.points_origin.qz = tf.transform.rotation.z;
        update.points_origin.qw = tf.transform.rotation.w;
      } else {
        update.points_origin.x = offset_x;
        update.points_origin.y = offset_y;
        update.points_origin.z = offset_z;
      }
      RCLCPP_INFO(node->get_logger(), "Loaded %zu points from %s", update.points.size(), path.c_str());
      for (int k = 0; k < attempts && !ok; ++k) {
        ok = client.setWorld(update, &message);
        if (!ok) {RCLCPP_WARN(node->get_logger(), "set_world attempt %d failed: %s", k + 1, message.c_str());}
      }
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
