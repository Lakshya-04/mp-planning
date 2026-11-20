/**
 * @file planning_pipeline.hpp
 * @brief Builds the selected planning backends (ik, collision, scene, cartesian, free_space) from a
 *        PlanningSelection and a PluginRegistry, and assembles the MotionPlanner from them.
 * @details ROS-free. The selection normally comes from YAML (planning_selection.hpp in the ros/
 *          layer) and the registry from pluginlib; tests pass a FactoryRegistry. Failures are
 *          structured (PipelineError), never exceptions. See docs/backend_selection.md.
 */

#pragma once

#include <map>
#include <memory>
#include <string>

#include "motion_planner/core/motion_planner.hpp"
#include "motion_planner/core/plugins.hpp"

namespace motion_planner
{

/// Which plugin fills each role. The defaults are the MoveIt backend: Cartesian first, OMPL for
/// free space, which is what the manager hard-wired before backends were selectable.
struct PlanningSelection
{
  std::string ik{"moveit_ik"};
  std::string free_space{"moveit_ompl"};
  std::string cartesian{"moveit_cartesian"};
  std::string collision{"moveit_collision"};
  std::string scene;  ///< Optional: empty means no scene plugin.
  /// Optional second free-space planner: tried when `free_space` fails (e.g. cuRobo unavailable or no path), see
  /// FallbackPlanner. Empty means no fallback.
  std::string free_space_fallback;

  /// Per-segment override: segment kind ("contact" = allow_target_contact, "transition" = every
  /// other segment) -> "cartesian" | "free_space". Empty means no override.
  std::map<std::string, std::string> segments;

  /// Parameters per plugin name (`plugin_params:` in the YAML).
  std::map<std::string, PluginParams> plugin_params;
};

struct PipelineError
{
  std::string code;     ///< "unknown_plugin" | "load_failed" | "missing_dependency" | "unavailable" |
                        ///< "invalid_parameter" | "invalid_selection"
  std::string role;     ///< "ik" | "collision" | "scene" | "cartesian" | "free_space" | "segments"
  std::string plugin;   ///< The configured name.
  std::string message;  ///< Human-readable, includes the available names for "unknown_plugin".
};

class PlanningPipeline
{
public:
  /// Builds every selected plugin and initializes it. False with `error` set on the first failure
  /// (the pipeline is then unusable).
  bool build(
    const PlanningSelection & selection, PluginRegistry & registry, PluginContext & context,
    PipelineError & error);

  std::shared_ptr<IkPort> ik() const {return ik_;}
  std::shared_ptr<CollisionPort> collision() const {return collision_;}
  std::shared_ptr<ScenePort> scene() const {return scene_;}

  /// Hands the cartesian and free-space planners to a MotionPlanner configured with the
  /// selection's per-segment overrides. Callable once after a successful build(); null otherwise.
  std::unique_ptr<MotionPlanner> makeMotionPlanner();

private:
  PlanningSelection selection_;
  std::shared_ptr<IkPort> ik_;
  std::shared_ptr<CollisionPort> collision_;
  std::shared_ptr<ScenePort> scene_;
  std::unique_ptr<PlannerPlugin> cartesian_;
  std::unique_ptr<PlannerPlugin> free_space_;
  std::unique_ptr<PlannerPlugin> free_space_fallback_;
};

}  // namespace motion_planner
