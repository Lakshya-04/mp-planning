#include "motion_planner_moveit/moveit_scene_adapter.hpp"

#include <chrono>
#include <exception>

#include "octomap/octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap_with_pose.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group/capability_names.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace motion_planner
{

namespace
{
constexpr std::chrono::seconds kClearOctomapServiceTimeout{5};

moveit_msgs::msg::CollisionObject makeRemoveObject(
  const std::string & frame_id, const std::string & id)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  return object;
}

moveit_msgs::msg::CollisionObject makeAddObject(
  const std::string & frame_id, const WorldGeometryBox & box)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = box.id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {box.size_x, box.size_y, box.size_z};
  object.primitives.push_back(primitive);

  geometry_msgs::msg::Pose primitive_pose;
  primitive_pose.position.x = box.center_x;
  primitive_pose.position.y = box.center_y;
  primitive_pose.position.z = box.center_z;
  primitive_pose.orientation.x = box.orientation_x;
  primitive_pose.orientation.y = box.orientation_y;
  primitive_pose.orientation.z = box.orientation_z;
  primitive_pose.orientation.w = box.orientation_w;
  object.primitive_poses.push_back(primitive_pose);

  return object;
}

}  // namespace

MoveItSceneAdapter::MoveItSceneAdapter(double octomap_resolution)
: octomap_resolution_(octomap_resolution)
{
}

bool MoveItSceneAdapter::ensureNode(std::string & error_message)
{
  if (node_) {
    return true;
  }
  if (!rclcpp::ok()) {
    error_message = "rclcpp_not_initialized";
    return false;
  }
  try {
    node_ = std::make_shared<rclcpp::Node>("motion_planner_scene_adapter");
    return true;
  } catch (const std::exception & e) {
    error_message = e.what();
    return false;
  }
}

bool MoveItSceneAdapter::ensurePlanningSceneInterface(std::string & error_message)
{
  if (planning_scene_interface_) {
    return true;
  }
  try {
    planning_scene_interface_ =
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    return true;
  } catch (const std::exception & e) {
    error_message = e.what();
    return false;
  }
}

bool MoveItSceneAdapter::updateWorldGeometry(
  const WorldGeometryUpdate & update,
  std::string & error_message)
{
  if (!ensurePlanningSceneInterface(error_message)) {
    return false;
  }

  // Direct occupancy marking (not insertPointCloud()'s raycasting API, which
  // requires a sensor origin to carve free space along each ray) --
  // appropriate for synthetic/authored geometry with no real sensor behind
  // it: each point is simply marked occupied, nothing is assumed about free
  // space.
  octomap::OcTree octree(octomap_resolution_);
  for (const auto & point : update.points) {
    octree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
  }
  octree.updateInnerOccupancy();

  octomap_msgs::msg::Octomap octomap_msg;
  if (!octomap_msgs::fullMapToMsg(octree, octomap_msg)) {
    error_message = "Failed to serialize octree into an Octomap message.";
    return false;
  }

  octomap_msgs::msg::OctomapWithPose octomap_with_pose;
  octomap_with_pose.header.frame_id = update.frame_id;
  // Identity by default (update.points_origin's own default) -- unchanged behavior for every
  // caller that hasn't set this. A non-identity pose here is how a caller voxelizing points in
  // a fitted plane's own local basis (see WorldGeometryUpdate::points_origin's doc comment)
  // tells MoveIt how to compose that local octree back into frame_id's real space.
  octomap_with_pose.origin.position.x = update.points_origin.x;
  octomap_with_pose.origin.position.y = update.points_origin.y;
  octomap_with_pose.origin.position.z = update.points_origin.z;
  octomap_with_pose.origin.orientation.x = update.points_origin.qx;
  octomap_with_pose.origin.orientation.y = update.points_origin.qy;
  octomap_with_pose.origin.orientation.z = update.points_origin.qz;
  octomap_with_pose.origin.orientation.w = update.points_origin.qw;
  octomap_with_pose.octomap = octomap_msg;

  // Same idiom as CartesianPlanner::setWallContactAcmEntry() -- a partial
  // scene diff, applied synchronously to move_group's real planning scene
  // (a locally-mutated PlanningSceneMonitor mirror would have no effect,
  // since collision checking happens inside that separate process).
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.world.octomap = octomap_with_pose;

  // Named box obstacles are collision objects, deliberately never folded into the octomap
  // above -- see scene_port.hpp's doc comment on why a wall-contact ACM exclusion (always
  // scoped to kWallOctomapCollisionObjectId) must never reach these. REMOVE whatever this
  // adapter added last call before ADDing the new set, so this call is a true replacement
  // rather than an accumulation of stale boxes.
  for (const auto & stale_id : previous_box_ids_) {
    diff.world.collision_objects.push_back(makeRemoveObject(update.frame_id, stale_id));
  }
  for (const auto & box : update.boxes) {
    diff.world.collision_objects.push_back(makeAddObject(update.frame_id, box));
  }

  if (!planning_scene_interface_->applyPlanningScene(diff)) {
    error_message = "applyPlanningScene() rejected the world-geometry diff.";
    return false;
  }

  previous_box_ids_.clear();
  previous_box_ids_.reserve(update.boxes.size());
  for (const auto & box : update.boxes) {
    previous_box_ids_.push_back(box.id);
  }
  return true;
}

bool MoveItSceneAdapter::clearWorldGeometry(std::string & error_message)
{
  if (!ensureNode(error_message)) {
    return false;
  }

  if (!previous_box_ids_.empty()) {
    if (!ensurePlanningSceneInterface(error_message)) {
      return false;
    }
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    for (const auto & stale_id : previous_box_ids_) {
      diff.world.collision_objects.push_back(makeRemoveObject("", stale_id));
    }
    if (!planning_scene_interface_->applyPlanningScene(diff)) {
      error_message = "applyPlanningScene() rejected the box-obstacle removal diff.";
      return false;
    }
    previous_box_ids_.clear();
  }

  // move_group's own capability for this (CLEAR_OCTOMAP_SERVICE_NAME =
  // "clear_octomap") is a relative name resolved against move_group's node
  // *namespace* (root, "/"), not its node *name* -- so it is globally
  // advertised as "/clear_octomap", confirmed via `ros2 node info
  // /move_group`, not "/move_group/clear_octomap".
  const std::string service_name = "/" + move_group::CLEAR_OCTOMAP_SERVICE_NAME;
  auto client = node_->create_client<std_srvs::srv::Empty>(service_name);
  if (!client->wait_for_service(kClearOctomapServiceTimeout)) {
    error_message = "Timed out waiting for " + service_name + " to become available.";
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, kClearOctomapServiceTimeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    error_message = "Failed to call " + service_name + " (timed out or interrupted).";
    return false;
  }
  return true;
}

}  // namespace motion_planner
