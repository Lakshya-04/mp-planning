// Base stationing: where should the base stand to work a wall?
//
// The map says which base positions reach a pose but not which side of the wall is free, so this
// layer takes the wall plane, searches base positions on the free side only, and ranks them by how
// many targets stay unreachable, then how many lift bands the wall needs, then how close the base
// is to the wall's work area.
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "reachability/core/base_model.hpp"
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
  /// With a base model the clearance applies to the whole footprint (not the arm base point), the
  /// station reports the base yaw that gives it, and stands where the footprint would not fit are
  /// skipped. `max_standoff` and `standoff` still refer to the arm base.
  std::optional<BaseModel> base;
  /// Where the base is now. With a base model, ties between equally good stations go to the one
  /// the base reaches soonest (travelTime), instead of the one nearest the middle of the work area.
  std::optional<Stand> from;
};

struct Station
{
  bool found{false};
  double x{0.0};
  double y{0.0};
  double standoff{0.0};          ///< Distance from the wall plane (arm base).
  double yaw{0.0};               ///< Base yaw (only set when the request has a base model).
  double footprint_clearance{0.0};  ///< Footprint to wall distance (only with a base model).
  LiftBandPlan plan;             ///< Lift bands for this station.
  size_t evaluated{0};           ///< Stations tried.

  /// A station was found and it reaches every target.
  bool complete() const {return found && plan.unreachable.empty();}
};

/// Most search-grid points (standoff rings x positions along the wall) a request may ask for.
constexpr size_t kMaxGridPoints = 250000;

/// Empty when the request's search grid is usable: finite, positive `grid_step`, finite standoff
/// and margins, and at most kMaxGridPoints points. Otherwise the reason. chooseStation,
/// makeCandidateStands and selectStands(request, ...) refuse a request with a problem instead of
/// looping on it (an infinite max_standoff) or running out of memory (a tiny grid_step).
std::string stationGridProblem(const StationRequest & request);

/// Best station on the free side of the wall. `found` is false only if nothing reaches any target.
Station chooseStation(const Map4D & map, const StationRequest & request);

}  // namespace reachability
