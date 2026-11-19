// The motion_planner Planner port over cuRobo: a client of the curobo_bridge planner node.
//
// The node does the GPU work in its own process; this class turns PlanningRequest / world updates
// into service calls and the answers back into PlanningResult. The node passed in must be spun by
// someone else (an executor thread) because calls wait on the service future.
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "core_types/logger.hpp"
#include "motion_planner/core/planners/planner.hpp"
#include "motion_planner/core/scene_port.hpp"
#include "scene_model/core/types.hpp"

namespace curobo_planner
{

struct Options
{
  std::string node_name{"/curobo_planner"};  ///< Name of the cuRobo planner node (its services' prefix).
  double wait_for_service_sec{5.0};
  /// Per-call deadline. Above the worst case measured for one plan_joint on the node (first pass up to
  /// 20 attempts, 6 to 33 s, plus the 0.3 to 0.4 s seed fallback; docs/planner_benchmark.md). A shorter
  /// deadline abandons a plan the single-threaded node still finishes, and later calls queue behind it.
  double timeout_sec{45.0};
  /// The robot's base frame. When set, setWorld() / updateWorldGeometry() reject an update whose
  /// frame_id is set to anything else (the node cannot transform it). Empty: the update's frame is
  /// trusted to be the base frame.
  std::string base_frame;
};

class CuroboClientPlanner : public motion_planner::Planner, public motion_planner::ScenePort
{
public:
  CuroboClientPlanner(
    rclcpp::Node::SharedPtr node, const Options & options = Options{},
    std::shared_ptr<core_types::Logger> logger = nullptr);
  ~CuroboClientPlanner() override;

  std::string name() const override {return "curobo";}

  /// Sets Options::base_frame after construction (the plugins share one client, built by whichever
  /// plugin initialised first). Call before planning.
  void setBaseFrame(const std::string & frame);

  /// Replaces the planner's boxes and points (`points_origin` applied). The distance field set by
  /// setEsdf() is kept unless `world` carries points, which replace it. False on failure; the node
  /// then still holds its PREVIOUS world (for example when the update exceeds its obstacle cache), so
  /// plan(), ik() and collision() answer failure_reason_code "world_stale" until an update succeeds.
  bool setWorld(const motion_planner::WorldGeometryUpdate & world, std::string * message = nullptr);
  bool clearWorld(std::string * message = nullptr);

  /// Replaces only the distance field (the world's boxes stay). The node must run with an ESDF grid
  /// of the same shape and voxel size; its centre may differ per call. Our x-fastest layout and
  /// voxel-(0,0,0)-centre origin are converted to cuRobo's x-major layout and grid centre.
  bool setEsdf(const scene_model::EsdfGrid & grid, std::string * message = nullptr);

  /// ScenePort: the same two operations, so a SceneModel (via scene_adapters) can feed cuRobo like
  /// any other backend.
  bool updateWorldGeometry(
    const motion_planner::WorldGeometryUpdate & update, std::string & error_message) override;
  bool clearWorldGeometry(std::string & error_message) override;

  /// Joint-space move start_joint_state -> target_joint_state. With no target joint state and one
  /// target pose, plans to that pose (position + quaternion, robot base frame).
  motion_planner::PlanningResult plan(const motion_planner::PlanningRequest & request) override;
  /// Collision-free IK for `target_pose` (xyz + quaternion x y z w, or xyz + roll pitch yaw), robot
  /// base frame; `joint_seed` optional. `end_effector_link` / `planning_group` are ignored: cuRobo
  /// solves for its configured tool frame.
  motion_planner::IkResult ik(const motion_planner::IkRequest & request) override;
  /// Validity (joint limits, self-collision, world) of `joint_values` and/or the states and sampled
  /// motions of `trajectory_joint_values`. Pose-based inputs are not supported.
  motion_planner::CollisionResult collision(const motion_planner::CollisionRequest & request) override;

private:
  bool sendWorld(const motion_planner::WorldGeometryUpdate & world, std::string * message);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace curobo_planner
