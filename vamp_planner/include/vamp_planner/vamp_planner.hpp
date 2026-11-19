// VAMP behind the motion_planner ports.
//
// VAMP plans in joint space with vectorized collision checking: free-space moves and validity
// checks, no IK, no Cartesian paths. Everything VAMP-specific stays in the .cpp files; this header
// carries only plain C++ types so it can be included by code built with other compiler flags.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core_types/logger.hpp"
#include "motion_planner/core/planners/planner.hpp"
#include "motion_planner/core/scene_port.hpp"

namespace vamp_planner
{

struct Options
{
  std::string robot{"ur5"};          ///< One of VampPlanner::availableRobots().
  uint64_t max_iterations{100000};   ///< RRT-Connect iteration budget.
  double range{2.0};                 ///< RRT-Connect step range (VAMP units).
  double point_radius{0.01};         ///< Radius of each world point (m).
  double nominal_joint_speed{1.0};   ///< rad/s used to stamp the result; real timing is a later stage.
  bool simplify{true};               ///< Shortcut + B-spline the raw path.
  bool require_world{false};         ///< Refuse to plan until setWorld() was called.
  /// Frame the robot is planned in. When set, updateWorldGeometry() rejects an update whose
  /// frame_id is set to anything else (no transform is available here, so it cannot be converted).
  /// Empty: the update's frame is trusted to be the planning base frame.
  std::string base_frame;
};

/// Free-space planner and collision checker over one compiled-in VAMP robot.
/// Not thread-safe: use one instance per thread.
class VampPlanner : public motion_planner::Planner, public motion_planner::ScenePort
{
public:
  explicit VampPlanner(const Options & options, std::shared_ptr<core_types::Logger> logger = nullptr);
  ~VampPlanner() override;

  /// Names of the robots compiled into this build.
  static std::vector<std::string> availableRobots();

  /// Sets Options::base_frame after construction. The plugins share one planner per robot, so a
  /// plugin whose params name a base_frame applies it to the planner it got. Call before planning.
  void setBaseFrame(const std::string & frame);

  /// False when `options.robot` is not compiled in.
  bool ok() const;
  size_t dimension() const;
  std::vector<std::string> jointNames() const;

  /// Replaces the world. Points become a point cloud, boxes become cuboids (a point's
  /// `points_origin` is applied). Everything is taken as already in the robot base frame.
  void setWorld(const motion_planner::WorldGeometryUpdate & world);
  void clearWorld();
  bool hasWorld() const;

  /// ScenePort: the same operations, so a SceneModel (via scene_adapters) can feed this backend
  /// like any other. Geometry must already be in the robot base frame; these never fail.
  bool updateWorldGeometry(
    const motion_planner::WorldGeometryUpdate & update, std::string & error_message) override;
  bool clearWorldGeometry(std::string & error_message) override;

  std::string name() const override {return "vamp";}

  /// Plans start_joint_state -> target_joint_state (both required, robot joint order).
  motion_planner::PlanningResult plan(const motion_planner::PlanningRequest & request) override;
  /// VAMP has no IK: always fails with failure_reason_code "unsupported".
  motion_planner::IkResult ik(const motion_planner::IkRequest & request) override;
  /// Checks `joint_values` and/or every motion of `trajectory_joint_values`.
  motion_planner::CollisionResult collision(const motion_planner::CollisionRequest & request) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vamp_planner
