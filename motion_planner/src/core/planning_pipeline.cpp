#include "motion_planner/core/planning_pipeline.hpp"

#include "motion_planner/core/planners/fallback_planner.hpp"

#include <utility>

#include <vector>

namespace motion_planner
{
namespace
{

std::string join(const std::vector<std::string> & v)
{
  std::string out;
  for (const auto & s : v) {
    out += (out.empty() ? "" : ", ") + s;
  }
  return out.empty() ? "(none)" : out;
}

PipelineError toError(
  const PluginStatus & s, const std::string & role, const std::string & plugin,
  const std::vector<std::string> & available)
{
  PipelineError e;
  e.code = s.code;
  e.role = role;
  e.plugin = plugin;
  e.message = role + " '" + plugin + "': " + s.message;
  if (s.code == "unknown_plugin") {
    e.message += " (available: " + join(available) + ")";
  }
  return e;
}

PluginParams paramsFor(const PlanningSelection & sel, const std::string & name)
{
  const auto it = sel.plugin_params.find(name);
  return it == sel.plugin_params.end() ? PluginParams{} : it->second;
}

}  // namespace

bool PlanningPipeline::build(
  const PlanningSelection & selection, PluginRegistry & registry, PluginContext & context,
  PipelineError & error)
{
  selection_ = selection;
  ik_.reset();
  collision_.reset();
  scene_.reset();
  cartesian_.reset();
  free_space_.reset();
  free_space_fallback_.reset();

  for (const auto & kv : selection.segments) {
    const bool kind_ok = kv.first == "contact" || kv.first == "transition";
    const bool role_ok = kv.second == "cartesian" || kv.second == "free_space";
    if (!kind_ok || !role_ok) {
      error = {"invalid_selection", "segments", kv.first + ": " + kv.second,
        "segments override must map contact|transition to cartesian|free_space"};
      return false;
    }
  }

  // Order matters: ik and collision are published to the context before the planners are built,
  // so a planner can reuse them instead of creating its own.
  PluginStatus status;
  {
    auto p = registry.createCollision(selection.collision, status);
    if (p) {status = p->initialize(context, paramsFor(selection, selection.collision));}
    if (!status.ok) {
      error = toError(status, "collision", selection.collision, registry.names("collision"));
      return false;
    }
    collision_ = std::move(p);
    context.set<CollisionPort>("collision_port", collision_);
  }
  {
    auto p = registry.createIk(selection.ik, status);
    if (p) {status = p->initialize(context, paramsFor(selection, selection.ik));}
    if (!status.ok) {
      error = toError(status, "ik", selection.ik, registry.names("ik"));
      return false;
    }
    ik_ = std::move(p);
    context.set<IkPort>("ik_port", ik_);
  }
  if (!selection.scene.empty()) {
    auto p = registry.createScene(selection.scene, status);
    if (p) {status = p->initialize(context, paramsFor(selection, selection.scene));}
    if (!status.ok) {
      error = toError(status, "scene", selection.scene, registry.names("scene"));
      return false;
    }
    scene_ = std::move(p);
    context.set<ScenePort>("scene_port", scene_);
  }

  const std::pair<const char *, const std::string *> planner_roles[] = {
    {"cartesian", &selection.cartesian}, {"free_space", &selection.free_space}};
  for (const auto & role : planner_roles) {
    auto p = registry.createPlanner(*role.second, status);
    if (p) {status = p->initialize(context, paramsFor(selection, *role.second));}
    if (!status.ok) {
      error = toError(status, role.first, *role.second, registry.names("planner"));
      return false;
    }
    (std::string(role.first) == "cartesian" ? cartesian_ : free_space_) = std::move(p);
  }
  if (!selection.free_space_fallback.empty()) {
    auto p = registry.createPlanner(selection.free_space_fallback, status);
    if (p) {status = p->initialize(context, paramsFor(selection, selection.free_space_fallback));}
    if (!status.ok) {
      error = toError(status, "free_space_fallback", selection.free_space_fallback, registry.names("planner"));
      return false;
    }
    free_space_fallback_ = std::move(p);
  }
  return true;
}

std::unique_ptr<MotionPlanner> PlanningPipeline::makeMotionPlanner()
{
  if (!cartesian_ || !free_space_) {
    return nullptr;
  }
  std::unique_ptr<Planner> free_space = std::move(free_space_);
  if (free_space_fallback_) {
    free_space = std::make_unique<FallbackPlanner>(std::move(free_space), std::move(free_space_fallback_));
  }
  auto planner = std::make_unique<MotionPlanner>(std::move(cartesian_), std::move(free_space));
  planner->setSegmentKindModes(selection_.segments);
  return planner;
}

}  // namespace motion_planner
