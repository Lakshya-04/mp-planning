// Lift band design: which lift heights does a wall need, given where the base stands?
//
// With the arm's map and a base station (x, y), every target pose is reachable at some set of
// lift heights. Choosing the fewest heights that together reach all targets gives the "bands":
// the lift is indexed once per band and all in-band work runs with the lift locked
// (see docs/design_principles.md, section 4).
#pragma once

#include <cstddef>
#include <vector>

#include "reachability/core/map4d.hpp"

namespace reachability
{

struct LiftBand
{
  double lift_height{0.0};            ///< Lift position of this band (m).
  std::vector<size_t> pose_indices;   ///< Targets first covered by this band.
};

struct LiftBandPlan
{
  std::vector<LiftBand> bands;          ///< In the order the greedy cover chose them.
  std::vector<size_t> unreachable;      ///< Targets no candidate height reaches from this station.
  size_t reachable_count{0};
};

/// Fraction-of-targets view of one station: for each target, the candidate heights that reach it.
/// `arm_mount_offset_z` is the arm base's height above the lift's zero position, so the arm base
/// sits at `lift_height + arm_mount_offset_z` in the world.
std::vector<std::vector<bool>> reachableByHeight(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & heights, double arm_mount_offset_z);

/// Greedy set cover over the candidate heights.
LiftBandPlan planLiftBands(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & candidate_heights, double arm_mount_offset_z);

/// Evenly spaced candidate heights, inclusive of both ends.
std::vector<double> heightRange(double lo, double hi, double step);

}  // namespace reachability
