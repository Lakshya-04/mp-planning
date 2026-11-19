// pluginlib registration of the cuRobo client: planner, IK, collision and scene. All four share one
// CuroboClientPlanner through the PluginContext ("curobo_client"). The ROS node it needs is taken
// from the context ("node", a std::shared_ptr<rclcpp::Node> that the composition root spins in an
// executor thread); no node is created here. Without the cuRobo planner node running, calls
// return failure_reason_code "unavailable".
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "curobo_planner/curobo_planner.hpp"
#include "motion_planner/core/plugins.hpp"

namespace curobo_planner
{
namespace
{

namespace mp = motion_planner;

/// Gets or creates the shared client; sets `status` and returns null when the node is missing.
std::shared_ptr<CuroboClientPlanner> sharedClient(
  mp::PluginContext & context, const mp::PluginParams & params, mp::PluginStatus & status)
{
  status = mp::PluginStatus::success();
  auto node = context.get<rclcpp::Node>("node");
  if (!node) {
    status = mp::PluginStatus::fail(
      "missing_dependency", "the cuRobo plugins need a spun rclcpp node under the context key 'node'");
    return nullptr;
  }
  Options options;
  options.node_name = mp::paramOr(params, "node_name", options.node_name);
  options.wait_for_service_sec = mp::paramOr(params, "wait_for_service_sec", options.wait_for_service_sec);
  options.timeout_sec = mp::paramOr(params, "timeout_sec", options.timeout_sec);
  options.base_frame = mp::paramOr(params, "base_frame", options.base_frame);
  auto client = context.getOrCreate<CuroboClientPlanner>(
    "curobo_client", [&] {return std::make_shared<CuroboClientPlanner>(node, options, context.logger);});
  // shared client: a later plugin's base_frame still takes effect
  if (params.count("base_frame")) {client->setBaseFrame(options.base_frame);}
  return client;
}

class CuroboPlannerPlugin : public mp::PlannerPlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    mp::PluginStatus status;
    client_ = sharedClient(context, params, status);
    return status;
  }
  std::string name() const override {return "curobo";}
  mp::PlanningResult plan(const mp::PlanningRequest & r) override {return client_->plan(r);}
  mp::IkResult ik(const mp::IkRequest & r) override {return client_->ik(r);}
  mp::CollisionResult collision(const mp::CollisionRequest & r) override
  {
    return client_->collision(r);
  }

private:
  std::shared_ptr<CuroboClientPlanner> client_;
};

class CuroboIkPlugin : public mp::IkPlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    mp::PluginStatus status;
    client_ = sharedClient(context, params, status);
    return status;
  }
  mp::IkResult ik(const mp::IkRequest & r) override {return client_->ik(r);}

private:
  std::shared_ptr<CuroboClientPlanner> client_;
};

class CuroboCollisionPlugin : public mp::CollisionPlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    mp::PluginStatus status;
    client_ = sharedClient(context, params, status);
    return status;
  }
  mp::CollisionResult collision(const mp::CollisionRequest & r) override
  {
    return client_->collision(r);
  }

private:
  std::shared_ptr<CuroboClientPlanner> client_;
};

class CuroboScenePlugin : public mp::ScenePlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    mp::PluginStatus status;
    client_ = sharedClient(context, params, status);
    return status;
  }
  bool updateWorldGeometry(const mp::WorldGeometryUpdate & u, std::string & error) override
  {
    return client_->updateWorldGeometry(u, error);
  }
  bool clearWorldGeometry(std::string & error) override {return client_->clearWorldGeometry(error);}

private:
  std::shared_ptr<CuroboClientPlanner> client_;
};

}  // namespace
}  // namespace curobo_planner

PLUGINLIB_EXPORT_CLASS(curobo_planner::CuroboPlannerPlugin, motion_planner::PlannerPlugin)
PLUGINLIB_EXPORT_CLASS(curobo_planner::CuroboIkPlugin, motion_planner::IkPlugin)
PLUGINLIB_EXPORT_CLASS(curobo_planner::CuroboCollisionPlugin, motion_planner::CollisionPlugin)
PLUGINLIB_EXPORT_CLASS(curobo_planner::CuroboScenePlugin, motion_planner::ScenePlugin)
