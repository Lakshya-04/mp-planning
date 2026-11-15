// pluginlib registration of the MoveIt backend: Cartesian planner, OMPL planner, IK, collision and
// scene. All of them share one MoveItPlanningBackend through the PluginContext ("moveit_backend"),
// created by whichever plugin is initialized first.
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "motion_planner/core/plugins.hpp"
#include "motion_planner_moveit/cartesian_planner.hpp"
#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "motion_planner_moveit/moveit_scene_adapter.hpp"
#include "motion_planner_moveit/ompl_planner.hpp"

namespace motion_planner
{
namespace
{

std::shared_ptr<MoveItPlanningBackend> sharedBackend(PluginContext & context)
{
  return context.getOrCreate<MoveItPlanningBackend>(
    "moveit_backend", [] {return std::make_shared<MoveItPlanningBackend>();});
}

/// Wraps a MoveIt planner (built on the shared backend in initialize()) as a PlannerPlugin.
template<typename PlannerT>
class MoveItPlannerPlugin : public PlannerPlugin
{
public:
  PluginStatus initialize(PluginContext & context, const PluginParams &) override
  {
    planner_ = std::make_unique<PlannerT>(sharedBackend(context));
    return PluginStatus::success();
  }
  std::string name() const override {return planner_ ? planner_->name() : "uninitialized";}
  PlanningResult plan(const PlanningRequest & request) override {return planner_->plan(request);}
  IkResult ik(const IkRequest & request) override {return planner_->ik(request);}
  CollisionResult collision(const CollisionRequest & request) override
  {
    return planner_->collision(request);
  }

private:
  std::unique_ptr<PlannerT> planner_;
};

class MoveItIkPlugin : public IkPlugin
{
public:
  PluginStatus initialize(PluginContext & context, const PluginParams & params) override
  {
    backend_ = sharedBackend(context);
    if (params.count("ik_extra_seeds") != 0) {
      backend_->setIkSeeding(
        static_cast<int>(paramOr(params, "ik_extra_seeds", 0.0)),
        paramOr(params, "ik_seed_perturbation", 0.3));
    }
    return PluginStatus::success();
  }
  IkResult ik(const IkRequest & request) override {return backend_->ik(request);}

private:
  std::shared_ptr<MoveItPlanningBackend> backend_;
};

class MoveItCollisionPlugin : public CollisionPlugin
{
public:
  PluginStatus initialize(PluginContext & context, const PluginParams &) override
  {
    backend_ = sharedBackend(context);
    return PluginStatus::success();
  }
  CollisionResult collision(const CollisionRequest & request) override
  {
    return backend_->collision(request);
  }

private:
  std::shared_ptr<MoveItPlanningBackend> backend_;
};

class MoveItScenePlugin : public ScenePlugin
{
public:
  PluginStatus initialize(PluginContext &, const PluginParams & params) override
  {
    adapter_ = std::make_unique<MoveItSceneAdapter>(paramOr(params, "octomap_resolution", 0.05));
    return PluginStatus::success();
  }
  bool updateWorldGeometry(const WorldGeometryUpdate & update, std::string & error) override
  {
    return adapter_->updateWorldGeometry(update, error);
  }
  bool clearWorldGeometry(std::string & error) override {return adapter_->clearWorldGeometry(error);}

private:
  std::unique_ptr<MoveItSceneAdapter> adapter_;
};

using MoveItCartesianPlugin = MoveItPlannerPlugin<CartesianPlanner>;
using MoveItOmplPlugin = MoveItPlannerPlugin<OmplPlanner>;

}  // namespace
}  // namespace motion_planner

PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItCartesianPlugin, motion_planner::PlannerPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItOmplPlugin, motion_planner::PlannerPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItIkPlugin, motion_planner::IkPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItCollisionPlugin, motion_planner::CollisionPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItScenePlugin, motion_planner::ScenePlugin)
