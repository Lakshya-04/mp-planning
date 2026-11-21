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
#include "reachability/core/free_space.hpp"
#include "reachability/core/surface_frame.hpp"
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
  /// Stations (arm base x, y) the search must not pick: any grid point within `avoid_radius` of one is
  /// skipped (chooseStation, and the set-cover candidate grid). planStandsVerified fills this with the
  /// stands a verifier refused. Empty = no change.
  std::vector<Eigen::Vector2d> avoid_stations;
  double avoid_radius{0.05};
  /// Obstacles and walls other than the work plane (see free_space.hpp). A candidate stand is kept
  /// only when at least one of the base's yaw samples puts the whole footprint, inflated by
  /// `free_space_margin`, in free space; the stand then takes the best such yaw. Without a base
  /// model the arm-base point alone is tested. Empty / unset = no change.
  std::optional<FreeSpace> free_space;
  double free_space_margin{0.05};
};

/// True when `xy` lies within the request's avoid radius of an avoided station.
inline bool stationAvoided(const StationRequest & q, const Eigen::Vector2d & xy)
{
  for (const auto & a : q.avoid_stations) {
    if ((a - xy).norm() <= q.avoid_radius) {return true;}
  }
  return false;
}

/// Why chooseStation found nothing (or kOk). Stable codes.
enum class StationCode
{
  kOk,                 ///< A station was found.
  kInvalidRequest,     ///< No targets, a zero normal or an unusable grid.
  kNoFreeStand,        ///< Every grid point was filtered out by the footprint (wall clearance or free space).
  kNoReachableStand,   ///< Free stands exist but none reaches any target.
};

const char * toString(StationCode code);

struct Station
{
  StationCode code{StationCode::kInvalidRequest};
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

/// One kept grid point of the stand search (shared by chooseStation and the candidate generators).
struct StationGridPoint
{
  Eigen::Vector2d arm_xy{0.0, 0.0};
  double yaw{0.0};         ///< 0 without a base model.
  double clearance{0.0};   ///< Footprint to work-plane distance (0 without a base model or on a horizontal plane).
  double standoff{0.0};    ///< Arm base distance from the plane (0 on a horizontal plane).
  double along{0.0};       ///< Position along the plane frame's u axis, from wall_point.
};

struct StationGridStats
{
  size_t visited{0};               ///< Grid points generated.
  size_t avoided{0};               ///< Skipped by avoid_stations.
  size_t plane_rejected{0};        ///< No yaw leaves the footprint `min_clearance` from the work plane.
  size_t free_space_rejected{0};   ///< Footprint fits the work plane for some yaw but no yaw is in free space.
  size_t kept{0};
};

/// The search grid with the avoid list, the work-plane footprint test and the free-space test applied,
/// in a fixed order (standoff rings outer, then along u; on a horizontal plane v outer, u inner). Empty
/// for a zero normal or no targets. Does not validate the grid size: see stationGridProblem.
std::vector<StationGridPoint> stationGrid(const StationRequest & request, StationGridStats * stats = nullptr);

/// Best station on the free side of the wall (or under / over a ceiling or floor patch: the search
/// area is then the patch's footprint plus `lateral_margin`, and `standoff` is 0). `found` is false only if nothing reaches any target.
Station chooseStation(const Map4D & map, const StationRequest & request);

}  // namespace reachability
