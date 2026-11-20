#include "reachability/core/stands.hpp"

#include <algorithm>
#include <set>

namespace reachability
{

StandStrategy parseStandStrategy(const std::string & name, StandStrategy fallback)
{
  if (name == "greedy") {return StandStrategy::Greedy;}
  if (name == "set_cover") {return StandStrategy::SetCover;}
  return fallback;
}

StandPlan planStandsSetCover(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start,
  size_t max_stands, const SetCoverOptions & cover);

StandPlan planStands(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start,
  size_t max_stands, StandStrategy strategy, const SetCoverOptions & cover)
{
  if (strategy == StandStrategy::SetCover) {
    return planStandsSetCover(map, request, start, max_stands, cover);
  }
  StandPlan plan;
  std::vector<size_t> left(request.targets_world.size());
  for (size_t i = 0; i < left.size(); ++i) {left[i] = i;}
  std::optional<Stand> at = request.base ? start : std::nullopt;

  while (!left.empty() && plan.stands.size() < max_stands) {
    StationRequest r = request;
    r.targets_world.clear();
    for (const size_t i : left) {r.targets_world.push_back(request.targets_world[i]);}
    r.from = at;
    const Station st = chooseStation(map, r);
    if (!st.found) {break;}

    StandAssignment a;
    a.station = st;
    for (const auto & band : st.plan.bands) {
      for (const size_t k : band.pose_indices) {a.targets.push_back(left[k]);}
    }
    if (a.targets.empty()) {break;}
    std::sort(a.targets.begin(), a.targets.end());
    // chooseStation numbered targets within the remaining set; make every index global
    for (auto & band : a.station.plan.bands) {
      for (auto & k : band.pose_indices) {k = left[k];}
    }
    for (auto & k : a.station.plan.unreachable) {k = left[k];}
    a.stand = Stand{Eigen::Vector2d(st.x, st.y), st.yaw};
    if (request.base && at) {
      a.travel_s = travelTime(*request.base, *at, a.stand);
      plan.total_travel_s += a.travel_s;
    }
    if (request.base) {at = a.stand;}

    left.erase(std::remove_if(left.begin(), left.end(), [&](size_t i) {
        return std::binary_search(a.targets.begin(), a.targets.end(), i);
      }), left.end());
    plan.stands.push_back(std::move(a));
  }
  // left over because the cap stopped the loop: not known to be unreachable
  (plan.stands.size() >= max_stands ? plan.not_planned : plan.unreachable) = left;
  return plan;
}

VerifiedStandPlan planStandsVerified(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start,
  size_t max_stands, StandStrategy strategy, const SetCoverOptions & cover, const VerifiedPlanOptions & options)
{
  VerifiedStandPlan out;
  if (!options.verifier) {
    out.plan = planStands(map, request, start, max_stands, strategy, cover);
    return out;
  }
  const auto heights = heightRange(request.lift_min, request.lift_max, request.lift_step);
  const std::optional<Stand> origin = request.base ? start : std::nullopt;
  std::vector<size_t> left(request.targets_world.size());
  for (size_t i = 0; i < left.size(); ++i) {left[i] = i;}
  std::set<size_t> refused_ever;
  std::set<size_t> unreachable;
  std::vector<size_t> not_planned;
  StationRequest r = request;   // keeps its own avoid list, extended as stands are refused

  const size_t max_rounds = std::max<size_t>(1, options.max_rounds);
  for (size_t round = 1; round <= max_rounds && !left.empty(); ++round) {
    out.rounds = round;
    StationRequest rr = r;
    rr.targets_world.clear();
    for (const size_t i : left) {rr.targets_world.push_back(request.targets_world[i]);}
    const std::optional<Stand> at = out.plan.stands.empty() ? origin : std::optional<Stand>(out.plan.stands.back().stand);
    const size_t room = max_stands > out.plan.stands.size() ? max_stands - out.plan.stands.size() : 0;
    if (room == 0) {
      not_planned = left;
      left.clear();
      break;
    }
    StandPlan p = planStands(map, rr, at, room, strategy, cover);
    // plan indices are into `left`; make them global
    auto global = [&](size_t k) {return left[k];};
    std::vector<size_t> next;
    for (const size_t k : p.unreachable) {
      // round 1: RM4D rejects it outright. Later: a refused target that lost its stations to the avoid list
      // stays refused; anything else has no station left.
      if (round > 1 && refused_ever.count(global(k))) {next.push_back(global(k));} else {unreachable.insert(global(k));}
    }
    for (const size_t k : p.not_planned) {next.push_back(global(k));}

    bool any_refusal = false;
    for (auto & a : p.stands) {
      std::vector<size_t> todo;
      for (const size_t k : a.targets) {todo.push_back(global(k));}
      std::vector<std::vector<double>> hs;
      const auto reach = reachableByHeight(
        map, request.targets_world, a.stand.arm_xy.x(), a.stand.arm_xy.y(), heights, request.arm_mount_offset_z);
      for (const size_t t : todo) {
        std::vector<double> ok;
        for (size_t h = 0; h < heights.size(); ++h) {if (reach[t][h]) {ok.push_back(heights[h]);}}
        hs.push_back(std::move(ok));
      }
      StandVerification v = verifyStand(*options.verifier, a.stand, request.targets_world, todo, hs, options.verify);
      out.verifier_calls += v.verifier_calls;
      std::vector<size_t> kept;
      std::vector<TargetReport> kept_reports;
      for (const auto & rep : v.targets) {
        if (rep.confirmed) {
          kept.push_back(rep.target);
          kept_reports.push_back(rep);
        } else {
          any_refusal = true;
          refused_ever.insert(rep.target);
          next.push_back(rep.target);
          out.refused.push_back({a.stand, rep.target, rep.reason});
        }
      }
      if (kept.size() < todo.size()) {r.avoid_stations.push_back(a.stand.arm_xy);}
      if (kept.empty()) {continue;}
      std::sort(kept.begin(), kept.end());
      a.targets = kept;
      // station plan: global indices, the lift bands IK confirmed
      a.station.plan.bands = v.bands;
      a.station.plan.unreachable.clear();
      v.targets = kept_reports;
      v.confirmed = kept_reports.size();
      v.coverage = todo.empty() ? 1.0 : static_cast<double>(v.confirmed) / static_cast<double>(todo.size());
      out.plan.stands.push_back(std::move(a));
      out.verification.push_back(std::move(v));
    }
    std::sort(next.begin(), next.end());
    next.erase(std::unique(next.begin(), next.end()), next.end());
    left = std::move(next);
    if (!any_refusal && left.empty()) {break;}
    if (!any_refusal && !left.empty() && p.stands.empty()) {break;}   // nothing planned, nothing more to try
    if (round == max_rounds && !left.empty()) {out.converged = false;}
  }
  for (const size_t t : left) {
    if (refused_ever.count(t)) {out.rejected_by_verifier.push_back(t);} else {not_planned.push_back(t);}
  }
  std::sort(not_planned.begin(), not_planned.end());
  not_planned.erase(std::unique(not_planned.begin(), not_planned.end()), not_planned.end());
  out.plan.unreachable.assign(unreachable.begin(), unreachable.end());
  out.plan.not_planned = not_planned;

  // the stand order is the order they were confirmed in: recompute the legs
  std::optional<Stand> at = origin;
  for (auto & a : out.plan.stands) {
    a.travel_s = 0.0;
    if (request.base && at) {
      a.travel_s = travelTime(*request.base, *at, a.stand);
      out.plan.total_travel_s += a.travel_s;
    }
    if (request.base) {at = a.stand;}
  }
  return out;
}

}  // namespace reachability
