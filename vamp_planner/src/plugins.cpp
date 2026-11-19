// pluginlib registration of VAMP: planner, collision and scene. They share one VampPlanner per
// robot through the PluginContext. Initialization reports "unavailable" when the requested robot
// is not compiled into this build (robots are compiled in; see VAMP_PLANNER_ROBOT_DIR). VAMP has
// no IK, so there is no IK plugin.
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "motion_planner/core/plugins.hpp"
#include "vamp_planner/vamp_planner.hpp"

namespace vamp_planner
{
namespace
{

namespace mp = motion_planner;

std::shared_ptr<VampPlanner> sharedPlanner(
  mp::PluginContext & context, const mp::PluginParams & params, mp::PluginStatus & status)
{
  Options options;
  options.robot = mp::paramOr(params, "robot", options.robot);
  options.max_iterations = static_cast<uint64_t>(
    mp::paramOr(params, "max_iterations", static_cast<double>(options.max_iterations)));
  options.range = mp::paramOr(params, "range", options.range);
  options.point_radius = mp::paramOr(params, "point_radius", options.point_radius);
  options.nominal_joint_speed = mp::paramOr(params, "nominal_joint_speed", options.nominal_joint_speed);
  options.simplify = mp::paramOr(params, "simplify", options.simplify);
  options.require_world = mp::paramOr(params, "require_world", options.require_world);
  options.base_frame = mp::paramOr(params, "base_frame", options.base_frame);

  auto planner = context.getOrCreate<VampPlanner>(
    "vamp_planner:" + options.robot,
    [&] {return std::make_shared<VampPlanner>(options, context.logger);});
  // The planner is shared per robot and the first plugin to ask built it: a later plugin's base_frame
  // still has to take effect (the scene plugin is the one that needs it).
  if (params.count("base_frame")) {planner->setBaseFrame(options.base_frame);}
  if (!planner->ok()) {
    std::string robots;
    for (const auto & r : VampPlanner::availableRobots()) {
      robots += (robots.empty() ? "" : ", ") + r;
    }
    status = mp::PluginStatus::fail(
      "unavailable",
      "VAMP robot '" + options.robot + "' is not compiled into this build (compiled: " +
      (robots.empty() ? "none" : robots) + ")");
    return nullptr;
  }
  status = mp::PluginStatus::success();
  return planner;
}

class VampPlannerPlugin : public mp::PlannerPlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    mp::PluginStatus status;
    planner_ = sharedPlanner(context, params, status);
    return status;
  }
  std::string name() const override {return "vamp";}
  mp::PlanningResult plan(const mp::PlanningRequest & r) override {return planner_->plan(r);}
  mp::IkResult ik(const mp::IkRequest & r) override {return planner_->ik(r);}
  mp::CollisionResult collision(const mp::CollisionRequest & r) override
  {
    return planner_->collision(r);
  }

private:
  std::shared_ptr<VampPlanner> planner_;
};

class VampCollisionPlugin : public mp::CollisionPlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    mp::PluginStatus status;
    planner_ = sharedPlanner(context, params, status);
    return status;
  }
  mp::CollisionResult collision(const mp::CollisionRequest & r) override
  {
    return planner_->collision(r);
  }

private:
  std::shared_ptr<VampPlanner> planner_;
};

class VampScenePlugin : public mp::ScenePlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    mp::PluginStatus status;
    planner_ = sharedPlanner(context, params, status);
    return status;
  }
  bool updateWorldGeometry(const mp::WorldGeometryUpdate & u, std::string & error) override
  {
    return planner_->updateWorldGeometry(u, error);
  }
  bool clearWorldGeometry(std::string & error) override {return planner_->clearWorldGeometry(error);}

private:
  std::shared_ptr<VampPlanner> planner_;
};

}  // namespace
}  // namespace vamp_planner

PLUGINLIB_EXPORT_CLASS(vamp_planner::VampPlannerPlugin, motion_planner::PlannerPlugin)
PLUGINLIB_EXPORT_CLASS(vamp_planner::VampCollisionPlugin, motion_planner::CollisionPlugin)
PLUGINLIB_EXPORT_CLASS(vamp_planner::VampScenePlugin, motion_planner::ScenePlugin)
