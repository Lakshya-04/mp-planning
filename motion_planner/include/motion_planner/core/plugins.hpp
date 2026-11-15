/**
 * @file plugins.hpp
 * @brief Plugin base classes for the planning ports, the dependency context handed to a plugin,
 *        and the registry interface the planning pipeline builds plugins from.
 * @details A plugin is a port implementation selected by name at runtime. pluginlib needs a
 *          default-constructible class, so the dependencies a plugin needs (a ROS node, a shared
 *          backend, a logger, its YAML parameters) arrive through initialize() instead of the
 *          constructor. Nothing here includes ROS: the ROS node is just an entry in the
 *          PluginContext that the composition root puts there. See docs/backend_selection.md.
 */

#pragma once

#include <any>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core_types/logger.hpp"
#include "core_types/null_logger.hpp"
#include "motion_planner/core/planners/collision_port.hpp"
#include "motion_planner/core/planners/ik_port.hpp"
#include "motion_planner/core/planners/planner.hpp"
#include "motion_planner/core/scene_port.hpp"

namespace motion_planner
{

/// Result of PluginRegistry creation and Plugin::initialize(). `code` is machine-readable:
/// "unknown_plugin", "load_failed", "missing_dependency", "unavailable", "invalid_parameter".
struct PluginStatus
{
  bool ok{true};
  std::string code;
  std::string message;

  static PluginStatus success() {return PluginStatus{};}
  static PluginStatus fail(std::string code, std::string message)
  {
    return PluginStatus{false, std::move(code), std::move(message)};
  }
};

/// The plugin's own parameters, as written under `plugin_params:<name>` in the selection YAML.
using PluginParams = std::map<std::string, std::string>;

inline std::string paramOr(const PluginParams & p, const std::string & key, const std::string & dflt)
{
  const auto it = p.find(key);
  return it == p.end() ? dflt : it->second;
}
inline double paramOr(const PluginParams & p, const std::string & key, double dflt)
{
  const auto it = p.find(key);
  return it == p.end() ? dflt : std::stod(it->second);
}
inline bool paramOr(const PluginParams & p, const std::string & key, bool dflt)
{
  const auto it = p.find(key);
  return it == p.end() ? dflt : (it->second == "true" || it->second == "1");
}

/**
 * @brief Dependencies shared by every plugin of one pipeline.
 * @details Entries are looked up by name and type. The composition root registers lazy factories
 *          (e.g. "node": create and spin a ROS node only if some plugin asks for it); plugins that
 *          share one backend object use getOrCreate(). The pipeline also publishes the selected
 *          ports as "ik_port" and "collision_port" before it builds the planners.
 */
class PluginContext
{
public:
  std::shared_ptr<core_types::Logger> logger{std::make_shared<core_types::NullLogger>()};

  template<typename T>
  void set(const std::string & key, std::shared_ptr<T> value) {entries_[key] = std::move(value);}

  template<typename T>
  void setFactory(const std::string & key, std::function<std::shared_ptr<T>()> factory)
  {
    factories_[key] = [factory = std::move(factory)]() {return std::any(factory());};
  }

  /// Returns the entry, creating it through its factory on first use; null when absent or of
  /// another type.
  template<typename T>
  std::shared_ptr<T> get(const std::string & key)
  {
    auto it = entries_.find(key);
    if (it == entries_.end()) {
      const auto f = factories_.find(key);
      if (f == factories_.end()) {
        return nullptr;
      }
      it = entries_.emplace(key, f->second()).first;
    }
    const auto * p = std::any_cast<std::shared_ptr<T>>(&it->second);
    return p ? *p : nullptr;
  }

  template<typename T>
  std::shared_ptr<T> getOrCreate(
    const std::string & key, const std::function<std::shared_ptr<T>()> & make)
  {
    if (auto existing = get<T>(key)) {
      return existing;
    }
    auto created = make();
    set<T>(key, created);
    return created;
  }

private:
  std::map<std::string, std::any> entries_;
  std::map<std::string, std::function<std::any()>> factories_;
};

/// Plugin base classes: the ports plus initialize(). These are the types pluginlib loads.
class PlannerPlugin : public Planner
{
public:
  virtual PluginStatus initialize(PluginContext & context, const PluginParams & params) = 0;
};

class IkPlugin : public IkPort
{
public:
  virtual PluginStatus initialize(PluginContext & context, const PluginParams & params) = 0;
};

class CollisionPlugin : public CollisionPort
{
public:
  virtual PluginStatus initialize(PluginContext & context, const PluginParams & params) = 0;
};

class ScenePlugin : public ScenePort
{
public:
  virtual PluginStatus initialize(PluginContext & context, const PluginParams & params) = 0;
};

/// Creates (does not initialize) plugins by name. A null result comes with a failed status.
class PluginRegistry
{
public:
  virtual ~PluginRegistry() = default;
  virtual std::unique_ptr<PlannerPlugin> createPlanner(
    const std::string & name, PluginStatus & status) = 0;
  virtual std::unique_ptr<IkPlugin> createIk(const std::string & name, PluginStatus & status) = 0;
  virtual std::unique_ptr<CollisionPlugin> createCollision(
    const std::string & name, PluginStatus & status) = 0;
  virtual std::unique_ptr<ScenePlugin> createScene(
    const std::string & name, PluginStatus & status) = 0;
  /// Registered names per kind ("planner" | "ik" | "collision" | "scene"), for error messages.
  virtual std::vector<std::string> names(const std::string & kind) const = 0;
};

/// In-process registry of factory functions: for tests and for code that links its plugins
/// directly instead of loading them through pluginlib.
class FactoryRegistry : public PluginRegistry
{
public:
  void addPlanner(const std::string & name, std::function<std::unique_ptr<PlannerPlugin>()> f)
  {
    planners_[name] = std::move(f);
  }
  void addIk(const std::string & name, std::function<std::unique_ptr<IkPlugin>()> f)
  {
    iks_[name] = std::move(f);
  }
  void addCollision(const std::string & name, std::function<std::unique_ptr<CollisionPlugin>()> f)
  {
    collisions_[name] = std::move(f);
  }
  void addScene(const std::string & name, std::function<std::unique_ptr<ScenePlugin>()> f)
  {
    scenes_[name] = std::move(f);
  }

  std::unique_ptr<PlannerPlugin> createPlanner(const std::string & n, PluginStatus & s) override
  {
    return make(planners_, "planner", n, s);
  }
  std::unique_ptr<IkPlugin> createIk(const std::string & n, PluginStatus & s) override
  {
    return make(iks_, "ik", n, s);
  }
  std::unique_ptr<CollisionPlugin> createCollision(const std::string & n, PluginStatus & s) override
  {
    return make(collisions_, "collision", n, s);
  }
  std::unique_ptr<ScenePlugin> createScene(const std::string & n, PluginStatus & s) override
  {
    return make(scenes_, "scene", n, s);
  }
  std::vector<std::string> names(const std::string & kind) const override
  {
    if (kind == "planner") {return keys(planners_);}
    if (kind == "ik") {return keys(iks_);}
    if (kind == "collision") {return keys(collisions_);}
    return keys(scenes_);
  }

private:
  template<typename Map>
  static std::vector<std::string> keys(const Map & m)
  {
    std::vector<std::string> out;
    for (const auto & kv : m) {out.push_back(kv.first);}
    return out;
  }
  template<typename Map>
  static auto make(
    const Map & m, const std::string & kind, const std::string & name, PluginStatus & status)
  -> decltype(m.begin()->second())
  {
    const auto it = m.find(name);
    if (it == m.end()) {
      status = PluginStatus::fail("unknown_plugin", "no " + kind + " plugin named '" + name + "'");
      return nullptr;
    }
    status = PluginStatus::success();
    return it->second();
  }

  std::map<std::string, std::function<std::unique_ptr<PlannerPlugin>()>> planners_;
  std::map<std::string, std::function<std::unique_ptr<IkPlugin>()>> iks_;
  std::map<std::string, std::function<std::unique_ptr<CollisionPlugin>()>> collisions_;
  std::map<std::string, std::function<std::unique_ptr<ScenePlugin>()>> scenes_;
};

}  // namespace motion_planner
