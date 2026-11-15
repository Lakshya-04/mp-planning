#include "motion_planner/ros/planning_config.hpp"

#include <set>
#include <utility>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

namespace motion_planner
{

bool parsePlanningSelection(
  const std::string & yaml_text, PlanningSelection & selection, std::string & error)
{
  try {
    YAML::Node root = YAML::Load(yaml_text);
    if (root["pipeline"] && root["pipeline"].IsMap()) {
      root = root["pipeline"];
    }
    const YAML::Node planning = root["planning"];
    if (!planning) {
      error = "no 'planning' map in the selection YAML";
      return false;
    }
    if (!planning.IsMap()) {
      error = "'planning' must be a map";
      return false;
    }
    PlanningSelection out;
    const std::set<std::string> known = {
      "ik", "free_space", "cartesian", "collision", "scene", "segments", "plugin_params"};
    for (const auto & kv : planning) {
      const std::string key = kv.first.as<std::string>();
      if (known.count(key) == 0) {
        error = "unknown key 'planning." + key + "' (known: ik, free_space, cartesian, collision, "
          "scene, segments, plugin_params)";
        return false;
      }
    }
    auto read = [&](const char * key, std::string & dst) {
        if (planning[key]) {dst = planning[key].as<std::string>();}
      };
    read("ik", out.ik);
    read("free_space", out.free_space);
    read("cartesian", out.cartesian);
    read("collision", out.collision);
    read("scene", out.scene);
    if (planning["segments"]) {
      for (const auto & kv : planning["segments"]) {
        out.segments[kv.first.as<std::string>()] = kv.second.as<std::string>();
      }
    }
    if (planning["plugin_params"]) {
      for (const auto & plugin : planning["plugin_params"]) {
        PluginParams params;
        for (const auto & kv : plugin.second) {
          params[kv.first.as<std::string>()] = kv.second.as<std::string>();
        }
        out.plugin_params[plugin.first.as<std::string>()] = std::move(params);
      }
    }
    selection = std::move(out);
    return true;
  } catch (const YAML::Exception & e) {
    error = std::string("invalid selection YAML: ") + e.what();
    return false;
  }
}

bool loadPlanningSelection(
  const std::string & path, PlanningSelection & selection, std::string & error)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    return parsePlanningSelection(YAML::Dump(root), selection, error);
  } catch (const YAML::Exception & e) {
    error = "cannot read planning config '" + path + "': " + e.what();
    return false;
  }
}

namespace
{

struct Loaders
{
  pluginlib::ClassLoader<PlannerPlugin> planner{"motion_planner", "motion_planner::PlannerPlugin"};
  pluginlib::ClassLoader<IkPlugin> ik{"motion_planner", "motion_planner::IkPlugin"};
  pluginlib::ClassLoader<CollisionPlugin> collision{
    "motion_planner", "motion_planner::CollisionPlugin"};
  pluginlib::ClassLoader<ScenePlugin> scene{"motion_planner", "motion_planner::ScenePlugin"};
};

class PluginlibRegistry : public PluginRegistry
{
public:
  PluginlibRegistry()
  {
    static Loaders * const shared = new Loaders;  // intentionally never freed, see the header
    loaders_ = shared;
  }

  std::unique_ptr<PlannerPlugin> createPlanner(const std::string & n, PluginStatus & s) override
  {
    return create(loaders_->planner, "planner", n, s);
  }
  std::unique_ptr<IkPlugin> createIk(const std::string & n, PluginStatus & s) override
  {
    return create(loaders_->ik, "ik", n, s);
  }
  std::unique_ptr<CollisionPlugin> createCollision(const std::string & n, PluginStatus & s) override
  {
    return create(loaders_->collision, "collision", n, s);
  }
  std::unique_ptr<ScenePlugin> createScene(const std::string & n, PluginStatus & s) override
  {
    return create(loaders_->scene, "scene", n, s);
  }
  std::vector<std::string> names(const std::string & kind) const override
  {
    if (kind == "planner") {return loaders_->planner.getDeclaredClasses();}
    if (kind == "ik") {return loaders_->ik.getDeclaredClasses();}
    if (kind == "collision") {return loaders_->collision.getDeclaredClasses();}
    return loaders_->scene.getDeclaredClasses();
  }

private:
  template<typename T>
  static std::unique_ptr<T> create(
    pluginlib::ClassLoader<T> & loader, const std::string & kind, const std::string & name,
    PluginStatus & status)
  {
    status = PluginStatus::success();
    if (!loader.isClassAvailable(name)) {
      status = PluginStatus::fail("unknown_plugin", "no " + kind + " plugin named '" + name + "'");
      return nullptr;
    }
    try {
      return std::unique_ptr<T>(loader.createUnmanagedInstance(name));
    } catch (const pluginlib::PluginlibException & e) {
      status = PluginStatus::fail("load_failed", e.what());
      return nullptr;
    }
  }

  Loaders * loaders_;
};

}  // namespace

std::unique_ptr<PluginRegistry> makePluginlibRegistry()
{
  return std::make_unique<PluginlibRegistry>();
}

}  // namespace motion_planner
