// Stands for a horizontal surface (ceiling or floor): which base positions reach a patch of tool poses.
//
// A wall has a free side, so chooseStation searches a strip in front of it. A ceiling or floor has
// none: the base drives under (or over) the patch, so the search grid covers the patch footprint
// plus a margin. The tool approach axis is vertical (up for a ceiling, down for a floor), where the
// RM4D map has no azimuth: the cell depends only on the horizontal distance between base and
// target, so a station reaches a ring around itself. The lift decides which heights are reachable
// (planLiftBands over the lift range), and targets above or below every reachable height are
// reported separately from targets that are merely out of the horizontal reach.
//
// A patch wider than one stand's reach is covered greedily, like planStands: the station reaching
// the most remaining targets each round, ties to the stand nearest the base (with a base model and a
// start) or to the middle of the remaining targets. Base footprint and obstacles are not checked: a
// stand is any arm base position the map says reaches; the base yaw is free and reported as 0.
#pragma once

#include <optional>
#include <vector>

#include "reachability/core/stands.hpp"

namespace reachability
{

struct HorizontalStationRequest
{
  std::vector<Pose> targets_world;   ///< Tool poses on the surface (tool z axis vertical).
  double margin{0.5};                ///< Search this far beyond the targets' footprint (m).
  double grid_step{0.2};             ///< Search grid over the footprint (m).
  double lift_min{0.0};
  double lift_max{1.5};
  double lift_step{0.05};
  double arm_mount_offset_z{0.0};
  /// With a base model and `from`, ties go to the stand the base reaches soonest (travelTime).
  std::optional<BaseModel> base;
  std::optional<Stand> from;
};

struct HorizontalStandPlan
{
  StandPlan plan;   ///< `stands` in visiting order, `unreachable` (all targets no stand reaches),
                    ///< `not_planned` (cap hit); station.standoff is 0, station.yaw is 0.
  /// Subset of `plan.unreachable` whose height no lift position can bring within the map's
  /// vertical window: too high or too low for lift range + arm, whatever the base does.
  std::vector<size_t> out_of_lift_range;
};

/// False when a target at world height `z` is outside the map's vertical window for every lift
/// position: no base position helps, the target is out of lift + arm reach.
bool heightWithinLiftReach(
  const Map4D & map, double z, double lift_min, double lift_max, double arm_mount_offset_z);

/// Best single station for the patch. `found` is false only if nothing reaches any target.
Station chooseHorizontalStation(const Map4D & map, const HorizontalStationRequest & request);

/// Several stands for a patch larger than one stand's reach. Targets are indexed as in the request.
HorizontalStandPlan planHorizontalStands(
  const Map4D & map, const HorizontalStationRequest & request, size_t max_stands = 32);

}  // namespace reachability
