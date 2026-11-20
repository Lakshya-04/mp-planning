// pluginlib registration of the cuRobo client: planner, IK, collision and scene. All four share one
// CuroboClientPlanner through the PluginContext ("curobo_client"). The ROS node it needs is taken
// from the context ("node", a std::shared_ptr<rclcpp::Node> that the composition root spins in an
// executor thread); no node is created here. Without the cuRobo planner node running, calls
// return failure_reason_code "unavailable".
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "curobo_planner/curobo_planner.hpp"
#include "curobo_planner/joint_limit_guard.hpp"
#include "motion_planner/core/plugins.hpp"

namespace curobo_planner
{
namespace
{

namespace mp = motion_planner;

/// "package://<pkg>/<relative path>" resolves through the ament index; anything else is a path.
std::string resolvePath(const std::string & path)
{
  const std::string prefix = "package://";
  if (path.compare(0, prefix.size(), prefix) != 0) {return path;}
  const auto slash = path.find('/', prefix.size());
  const std::string pkg = path.substr(prefix.size(), slash == std::string::npos ? std::string::npos : slash - prefix.size());
  return ament_index_cpp::get_package_share_directory(pkg) + (slash == std::string::npos ? "" : path.substr(slash));
}

/// Reads the limit params. `present` is false when none is given (no guard); false return = error.
bool limitPolicyFromParams(
  const mp::PluginParams & params, JointLimitPolicy & policy, bool & present, std::string & error)
{
  present = params.count("joint_limits_file") || params.count("joint_limits");
  if (!present) {return true;}
  policy = JointLimitPolicy{};
  policy.safety_factor = mp::paramOr(params, "limit_safety_factor", policy.safety_factor);
  const std::string mode = mp::paramOr(params, "limit_mode", std::string("retime"));
  if (mode != "retime" && mode != "reject") {
    error = "limit_mode must be 'retime' or 'reject', got '" + mode + "'";
    return false;
  }
  policy.mode = mode == "reject" ? LimitMode::kReject : LimitMode::kRetime;
  const std::string file = mp::paramOr(params, "joint_limits_file", std::string());
  if (!file.empty()) {
    double vs = 1.0, as = 1.0;
    try {
      if (!loadMoveItJointLimits(resolvePath(file), policy.limits, error, &vs, &as)) {return false;}
    } catch (const std::exception & e) {
      error = std::string("cannot resolve joint_limits_file '") + file + "': " + e.what();
      return false;
    }
    // MoveIt multiplies by default_*_scaling_factor when it retimes; opt in to match that
    if (mp::paramOr(params, "limit_use_file_scaling", false)) {
      for (auto & kv : policy.limits) {
        kv.second.velocity *= vs;
        kv.second.acceleration *= as;
      }
    }
  }
  // inline entries add to or override the file's
  if (!parseInlineJointLimits(mp::paramOr(params, "joint_limits", std::string()), policy.limits, error)) {
    return false;
  }
  if (policy.limits.empty()) {
    error = "joint_limits_file / joint_limits given but no joint has both a velocity and an acceleration limit";
    return false;
  }
  return true;
}

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
  JointLimitPolicy limits;
  bool limits_present = false;
  std::string limit_error;
  if (!limitPolicyFromParams(params, limits, limits_present, limit_error)) {
    status = mp::PluginStatus::fail("invalid_param", limit_error);
    return nullptr;
  }
  options.joint_limits = limits;
  auto client = context.getOrCreate<CuroboClientPlanner>(
    "curobo_client", [&] {return std::make_shared<CuroboClientPlanner>(node, options, context.logger);});
  // shared client: a later plugin's base_frame still takes effect
  if (params.count("base_frame")) {client->setBaseFrame(options.base_frame);}
  if (limits_present) {client->setJointLimits(limits);}
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
