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
// numbers; ties go to the lower index. The one exception is the wall-clock budget (it covers the
// 2-opt step only, not scoring or the cover): if it expires the 2-opt stops early (`tour_budget_hit`), so the tour may differ between machines; set
// `time_budget_s` to 0 for a pure work-count limit (`max_two_opt_passes`).
#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "reachability/core/stand_verifier.hpp"
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
  double time_budget_s{2.0};      ///< Wall-clock limit for the 2-opt step alone (the clock starts when the tour step
                                  ///< starts, not at selectStands); <= 0 disables the clock.
  size_t max_two_opt_passes{100}; ///< Deterministic limit on 2-opt sweeps.
  /// Opt-in second pass. With a verifier, every chosen stand's targets are confirmed by IK and collision
  /// checking (heights from the RM4D map, VerifyOptions): a (stand, target) pair the verifier refuses is
  /// dropped from the cover, the margins of confirmed pairs become the IK margins, and the cover and tour are
  /// recomputed until every chosen stand is fully confirmed (or `verify.max_rounds`). Null = RM4D only.
  std::shared_ptr<StandVerifier> verifier;
  VerifyOptions verify;
};

/// One (candidate stand, target) pair RM4D promised and the verifier refused.
struct RefusedPair
{
  size_t candidate_index{0};
  size_t target{0};
  VerifyReason reason{VerifyReason::kIkFailed};
};

/// What the verification pass found; all zero / empty without a verifier. The refused pairs measure how
/// optimistic RM4D is: refused_pairs / verified_pairs.
struct VerificationSummary
{
  bool enabled{false};
  size_t rounds{0};
  bool converged{true};                 ///< False when `max_rounds` stopped it with unverified stands left.
  size_t verified_pairs{0};             ///< (candidate, target) pairs sent to the verifier.
  size_t confirmed_pairs{0};
  std::vector<RefusedPair> refused;     ///< RM4D promised, IK / collision refused (in the order found).
  size_t verifier_calls{0};
  size_t verifier_poses{0};             ///< Target poses in those batches.
  bool budget_hit{false};               ///< `verify.max_verifier_calls` stopped it (then also converged = false).
  double optimism() const
  {
    return verified_pairs ? static_cast<double>(refused.size()) / static_cast<double>(verified_pairs) : 0.0;
  }
};

struct SelectedStand
{
  Stand stand;
  double standoff{0.0};
  double footprint_clearance{0.0};
  size_t candidate_index{0};        ///< Index into the candidate list.
  size_t pick_index{0};             ///< Position in the cover's pick order (redundant picks dropped, before the tour).
  std::vector<size_t> targets;      ///< Indices into request.targets_world done from this stand.
  LiftBandPlan lift_bands;          ///< Bands over `targets` (pose indices are into targets_world).
  double mean_margin{0.0};          ///< Mean reachability margin of `targets` here, in (0, 1].
  double travel_cost{0.0};          ///< Leg from the previous stop (or from the start for the first).
  bool verified{false};             ///< `verification` is filled (a verifier was given).
  StandVerification verification;   ///< Per-target IK results, margins and the confirmed lift bands.
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
  VerificationSummary verification;    ///< Empty unless StandSelectionOptions::verifier is set.
  /// Targets that were reachable by RM4D from some candidate but that no candidate confirmed (sorted; a
  /// subset of `unreachable` when verifying). The reasons are in verification.refused.
  std::vector<size_t> rejected_by_verifier;
};

/// The candidate grid of chooseStation (same free side, search window, footprint and clearance
/// tests), every point that passes, in a fixed order (away from the wall, then along it).
std::vector<CandidateStand> makeCandidateStands(const StationRequest & request);

/// `start` is where the base is now (used only with request.base). Without a base model costs are
/// metres and the nearest-neighbour seed starts at the stand lowest along the wall; 2-opt may then
/// reorder, so the first stop is not guaranteed to be that one.
StandSelection selectStands(
  const Map4D & map, const StationRequest & request, const std::vector<CandidateStand> & candidates,
  const std::optional<Stand> & start = std::nullopt, const StandSelectionOptions & options = {});

/// Same with the candidates from makeCandidateStands(request).
StandSelection selectStands(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start = std::nullopt,
  const StandSelectionOptions & options = {});

}  // namespace reachability
