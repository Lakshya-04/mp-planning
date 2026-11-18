// Global stand selection: set cover over candidate stands, then a tour over the chosen ones.
//
// Given every wall target and a set of candidate stands (a grid over the free side of the wall, with the
// footprint and clearance rules of chooseStation), selectStands
//  1. scores each candidate against every target with the RM4D map (planLiftBands per candidate);
//     the share of candidate lift heights that reach a target is that pair's reachability margin;
//  2. runs the standard weighted greedy set cover: pick the candidate with the lowest
//     cost / newly covered targets, cost = 1 + margin_penalty * (1 - mean margin of what it reaches).
//     Costs are fixed per candidate, so the classical H(n) <= 1 + ln(n) guarantee holds against the
//     cheapest cover in these costs (the stand count is then within (1 + margin_penalty) * H(n) of the
//     fewest stands). Redundant stands are then dropped;
//  3. orders the chosen stands as an open tour from the current base pose over the travel time
//     (travelTime for the drive type; plain distance in metres without a base model): nearest
//     neighbour, then 2-opt, under a solve-time budget. The tour is never longer than visiting the
//     stands in the order the cover picked them.
// Targets nothing reaches are reported in `unreachable`, never dropped. Deterministic: no random
// numbers; ties go to the lower index. The one exception is the wall-clock budget: if it expires the
// 2-opt stops early (`tour_budget_hit`), so the tour may differ between machines; set
// `time_budget_s` to 0 for a pure work-count limit (`max_two_opt_passes`).
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "reachability/core/station.hpp"

namespace reachability
{

struct CandidateStand
{
  Stand stand;                     ///< Arm base position and base yaw.
  double standoff{0.0};            ///< Arm base distance from the wall plane (m).
  double footprint_clearance{0.0}; ///< Footprint to wall distance (m; 0 without a base model).
};

enum class StandSelectionCode
{
  kOk,                  ///< Every target is covered.
  kNoTargets,           ///< Nothing to cover; success with an empty tour.
  kInvalidRequest,      ///< Bad grid step, wall normal, lift range or options.
  kNoCandidates,        ///< No candidate stand fits (footprint clearance) or reaches any target.
  kUnreachableTargets,  ///< Some targets are reachable from no candidate; the rest are covered.
};

const char * toString(StandSelectionCode code);

struct StandSelectionOptions
{
  double margin_penalty{0.5};     ///< >= 0. 0 = plain greedy by count; larger prefers stands with more lift slack.
  double time_budget_s{2.0};      ///< Wall-clock limit for the tour step; <= 0 disables the clock.
  size_t max_two_opt_passes{100}; ///< Deterministic limit on 2-opt sweeps.
};

struct SelectedStand
{
  Stand stand;
  double standoff{0.0};
  double footprint_clearance{0.0};
  size_t candidate_index{0};        ///< Index into the candidate list.
  size_t pick_index{0};             ///< Position in the order the cover picked it (before the tour).
  std::vector<size_t> targets;      ///< Indices into request.targets_world done from this stand.
  LiftBandPlan lift_bands;          ///< Bands over `targets` (pose indices are into targets_world).
  double mean_margin{0.0};          ///< Mean reachability margin of `targets` here, in (0, 1].
  double travel_cost{0.0};          ///< Leg from the previous stop (or from the start for the first).
};

struct StandSelection
{
  bool success{false};              ///< True when every target is covered (also for no targets).
  StandSelectionCode code{StandSelectionCode::kInvalidRequest};
  std::string message;
  std::vector<SelectedStand> stands;   ///< In tour order.
  std::vector<size_t> unreachable;     ///< Targets no candidate reaches (sorted).
  double tour_cost{0.0};               ///< Path cost of `stands` in order, from the start.
  double input_order_cost{0.0};        ///< Same stands in the order the cover picked them.
  size_t greedy_stands{0};             ///< Stands the greedy picked before redundant ones were dropped.
  size_t candidates{0};                ///< Candidates given.
  size_t useful_candidates{0};         ///< ... that reach at least one target.
  bool tour_budget_hit{false};         ///< 2-opt stopped by the clock or the pass limit before converging.
  double solve_time_s{0.0};
};

/// The candidate grid of chooseStation (same free side, search window, footprint and clearance
/// tests), every point that passes, in a fixed order (away from the wall, then along it).
std::vector<CandidateStand> makeCandidateStands(const StationRequest & request);

/// `start` is where the base is now (used only with request.base; without a base model the
/// first stop is the one at the lowest position along the wall and costs are metres).
StandSelection selectStands(
  const Map4D & map, const StationRequest & request, const std::vector<CandidateStand> & candidates,
  const std::optional<Stand> & start = std::nullopt, const StandSelectionOptions & options = {});

/// Same with the candidates from makeCandidateStands(request).
StandSelection selectStands(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start = std::nullopt,
  const StandSelectionOptions & options = {});

}  // namespace reachability
