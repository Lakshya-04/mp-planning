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
// stand is any arm base position the map says reaches. The base yaw is free: without a base model
// and start it is reported as 0; with them each stand takes the heading the base arrives with (a
// strafing base keeps its yaw, one that cannot strafe drives there forward or reversing), so
// travel times contain no turn the base would not make.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "reachability/core/stands.hpp"

namespace reachability
{

struct HorizontalStationRequest
{
  std::vector<Pose> targets_world;   ///< Tool poses on the surface (tool z axis vertical).
  /// Search at least this far beyond the targets' footprint (m). A station reaches a ring up to the
  /// map's xy_extent around itself, so the search always extends by max(margin, xy_extent).
  double margin{0.5};
  double grid_step{0.2};             ///< Search grid over the footprint (m).
  double lift_min{0.0};
  double lift_max{1.5};
  double lift_step{0.05};
  double arm_mount_offset_z{0.0};
  /// With a base model and `from`, ties go to the stand the base reaches soonest (travelTime).
  std::optional<BaseModel> base;
  std::optional<Stand> from;
};

enum class HorizontalStandCode
{
  kOk,
  kInvalidRequest,   ///< Bad grid step, margin or lift range; nothing was planned (all targets are in `not_planned`).
};

struct HorizontalStandPlan
{
  HorizontalStandCode code{HorizontalStandCode::kOk};
  std::string message;   ///< Why the request was refused (empty when kOk).
  StandPlan plan;   ///< `stands` in visiting order, `unreachable` (all targets no stand reaches),
                    ///< `not_planned` (cap hit); station.standoff is 0, station.yaw is 0.
  /// Subset of `plan.unreachable` whose height no lift position can bring within the map's
  /// vertical window: too high or too low for lift range + arm, whatever the base does.
  std::vector<size_t> out_of_lift_range;
};

/// False when a target at world height `z` is outside the map's vertical window for every lift
/// position: no base position helps, the target is out of lift + arm reach. The window is the map's
/// real one: z_min up to z_min + nz() * resolution (the map rounds z_max up to whole cells).
bool heightWithinLiftReach(
  const Map4D & map, double z, double lift_min, double lift_max, double arm_mount_offset_z);

/// Empty when the request's grid and lift range are usable (finite positive grid_step and
/// lift_step, lift_max >= lift_min, finite margin, at most kMaxGridPoints stations and 100000
/// lift heights); otherwise the reason. planHorizontalStands reports it as kInvalidRequest.
std::string horizontalRequestProblem(const Map4D & map, const HorizontalStationRequest & request);

/// Best single station for the patch. `found` is false only if nothing reaches any target.
Station chooseHorizontalStation(const Map4D & map, const HorizontalStationRequest & request);

/// Several stands for a patch larger than one stand's reach. Targets are indexed as in the request.
HorizontalStandPlan planHorizontalStands(
  const Map4D & map, const HorizontalStationRequest & request, size_t max_stands = 32);

}  // namespace reachability
