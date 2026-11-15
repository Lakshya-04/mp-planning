// Several stands for one wall: which base stations, in what order, cover every target.
//
// A wall wider than the arm's reach needs more than one stand. Greedy cover: take the station that
// reaches the most of the remaining targets (chooseStation), among equally good ones the one the
// base reaches soonest from where it is (drive type and footprint via BaseModel), repeat on what is
// left. The stands come out in visiting order with the time spent driving between them.
#pragma once

#include <optional>
#include <vector>

#include "reachability/core/station.hpp"

namespace reachability
{

struct StandAssignment
{
  Station station;                 ///< Arm base position, standoff, lift bands (pose indices are into
                                   ///< request.targets_world, like `targets`).
  Stand stand;                     ///< Arm base position and base yaw (yaw 0 without a base model).
  std::vector<size_t> targets;     ///< Indices into request.targets_world covered from this stand.
  double travel_s{0.0};            ///< Driving time from the previous stand (0 without a base model).
};

struct StandPlan
{
  std::vector<StandAssignment> stands;   ///< In visiting order.
  std::vector<size_t> unreachable;       ///< Targets no stand reaches.
  std::vector<size_t> not_planned;       ///< Targets left when `max_stands` was hit; they may be reachable.
  double total_travel_s{0.0};
};

/// `start` is where the base begins (only used with request.base); it replaces request.from, which
/// planStands sets itself for every round. Stops after `max_stands`.
StandPlan planStands(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start = std::nullopt,
  size_t max_stands = 32);

}  // namespace reachability
