/**
 * @file moveit_scene_adapter.hpp
 * @brief MoveIt-backed ScenePort implementation.
 * @details Injects world/obstacle collision geometry into move_group's real
 *          planning scene as TWO separate representations, matching
 *          WorldGeometryUpdate's own split (see scene_port.hpp):
 *            - `points` -> one octomap, the same representation the
 *              production point-cloud-fed occupancy_map_monitor pipeline
 *              already produces (7dof_serum_moveit.launch.py's live
 *              PointCloudOctomapUpdater), so a static/offline injection here
 *              is a structural stand-in for that pipeline.
 *            - `boxes` -> named moveit_msgs::CollisionObject BOX primitives,
 *              one per box id, kept independent of the octomap specifically
 *              so a wall-contact ACM exclusion (which only ever targets
 *              kWallOctomapCollisionObjectId) can never also silently exclude
 *              them.
 *          Both push via
 *          moveit::planning_interface::PlanningSceneInterface::applyPlanningScene(),
 *          the exact idiom CartesianPlanner::setWallContactAcmEntry() already
 *          uses for ACM diffs -- see cartesian_planner.cpp -- just diffing
 *          world.octomap / world.collision_objects instead of
 *          allowed_collision_matrix.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "motion_planner/scene_port.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace moveit::planning_interface
{
class PlanningSceneInterface;
}  // namespace moveit::planning_interface

namespace motion_planner
{

class MoveItSceneAdapter : public ScenePort
{
public:
  /**
   * @param[in] octomap_resolution Voxel size (m) for the octree built from
   *            each updateWorldGeometry() call. Defaults to 0.05, matching
   *            the production bringup's own octomap_resolution.
   */
  explicit MoveItSceneAdapter(double octomap_resolution = 0.05);

  bool updateWorldGeometry(
    const WorldGeometryUpdate & update,
    std::string & error_message) override;
  bool clearWorldGeometry(std::string & error_message) override;

private:
  bool ensurePlanningSceneInterface(std::string & error_message);
  bool ensureNode(std::string & error_message);

  double octomap_resolution_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  /// IDs pushed by the previous updateWorldGeometry() call, so the next one can REMOVE them
  /// first -- otherwise a box dropped from one update to the next (or renamed) would linger in
  /// move_group's scene forever, breaking the port's "full replacement" contract.
  std::vector<std::string> previous_box_ids_;
};

}  // namespace motion_planner
