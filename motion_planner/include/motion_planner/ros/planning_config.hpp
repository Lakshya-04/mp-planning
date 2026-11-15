/**
 * @file planning_config.hpp
 * @brief The ros/ half of backend selection: the YAML loader for PlanningSelection and the
 *        pluginlib-backed PluginRegistry. See docs/backend_selection.md for the schema.
 */

#pragma once

#include <memory>
#include <string>

#include "motion_planner/core/planning_pipeline.hpp"
#include "motion_planner/core/plugins.hpp"

namespace motion_planner
{

/// Parses a selection from YAML text: a top-level `planning:` map (also accepted under a
/// `pipeline:` map, as in docs/architecture.md). Missing keys keep the defaults of
/// PlanningSelection; unknown keys are an error so a typo does not silently select a default.
bool parsePlanningSelection(
  const std::string & yaml_text, PlanningSelection & selection, std::string & error);

bool loadPlanningSelection(
  const std::string & path, PlanningSelection & selection, std::string & error);

/// Registry that loads plugins with pluginlib: packages export `plugin="..."` XML files under the
/// `motion_planner` package for the base classes PlannerPlugin, IkPlugin, CollisionPlugin and
/// ScenePlugin. The ClassLoaders live for the whole process (a plugin library is never unloaded
/// while its instances may exist).
std::unique_ptr<PluginRegistry> makePluginlibRegistry();

}  // namespace motion_planner
