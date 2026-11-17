// Several stands for one wall: which base stations, in what order, cover every target.
//
// A wall wider than the arm's reach needs more than one stand. Greedy cover: take the station that
// reaches the most of the remaining targets (chooseStation), among equally good ones the one the
// base reaches soonest from where it is (drive type and footprint via BaseModel), repeat on what is
// left. The stands come out in visiting order with the time spent driving between them.
//
// StandStrategy::SetCover is the alternative: every station of the search grid is scored once for the
// targets it reaches (the RM4D map is the prefilter), then a weighted greedy set cover is improved by a
// bounded exact search for fewer stands, the stands are ordered by nearest-next plus 2-opt over the
// drive-type travel time, and each stand is moved to the cheapest station that still reaches what only
// it reaches. Never more stands than the greedy start of that search; the default stays Greedy.
#pragma once

#include <optional>
#include <string>
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

enum class StandStrategy
{
  Greedy,     ///< Station reaching the most remaining targets each round (the default).
  SetCover,   ///< Set cover over station-to-target feasibility with a travel term.
};

/// "greedy" or "set_cover"; anything else gives `fallback`.
StandStrategy parseStandStrategy(const std::string & name, StandStrategy fallback = StandStrategy::Greedy);

struct SetCoverOptions
{
  /// Weight of base travel in the greedy cover: a station costs 1 + travel_weight * travel_s.
  double travel_weight{0.05};
  /// Search nodes the exact stand-count reduction may spend (0 skips it: greedy only).
  size_t node_budget{200000};
};

/// `start` is where the base begins (only used with request.base); it replaces request.from, which
/// planStands sets itself for every round. Stops after `max_stands`.
StandPlan planStands(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start = std::nullopt,
  size_t max_stands = 32, StandStrategy strategy = StandStrategy::Greedy,
  const SetCoverOptions & cover = {});

}  // namespace reachability
