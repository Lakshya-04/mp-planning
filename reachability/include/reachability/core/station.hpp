// Base stationing: where should the base stand to work a wall?
//
// The map says which base positions reach a pose but not which side of the wall is free, so this
// layer takes the wall plane, searches base positions on the free side only, and ranks them by how
// many targets stay unreachable, then how many lift bands the wall needs, then how close the base
// is to the wall's work area.
#pragma once

#include <vector>

#include "reachability/core/lift_bands.hpp"
#include "reachability/core/map4d.hpp"

namespace reachability
{

struct StationRequest
{
  std::vector<Pose> targets_world;   ///< Tool poses on the wall (tool z axis = approach axis).
  Vec3 wall_point{Vec3::Zero()};     ///< Any point on the wall plane.
  Vec3 wall_normal{Vec3::UnitX()};   ///< Unit normal pointing into the free (robot) side.
  double min_clearance{0.3};         ///< Base must stand at least this far from the wall (m).
  double max_standoff{1.6};          ///< ... and at most this far (m).
  double grid_step{0.1};             ///< Search grid along and away from the wall (m).
  double lateral_margin{0.5};        ///< Search this far beyond the targets' extent along the wall.
  double lift_min{0.0};
  double lift_max{1.5};
  double lift_step{0.05};
  double arm_mount_offset_z{0.0};
};

struct Station
{
  bool found{false};
  double x{0.0};
  double y{0.0};
  double standoff{0.0};          ///< Distance from the wall plane.
  LiftBandPlan plan;             ///< Lift bands for this station.
  size_t evaluated{0};           ///< Stations tried.
};

/// Best station on the free side of the wall. `found` is false only if nothing reaches any target.
Station chooseStation(const Map4D & map, const StationRequest & request);

}  // namespace reachability
