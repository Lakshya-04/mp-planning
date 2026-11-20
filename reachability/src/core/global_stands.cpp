// Global stand selection: see core/global_stands.hpp.
#include "reachability/core/global_stands.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>

namespace reachability
{
namespace
{
using Clock = std::chrono::steady_clock;

struct Frame
{
  Vec3 n;
  Vec3 u;
  bool ok{false};
};

Frame wallFrame(const StationRequest & q)
{
  Frame f;
  if (q.wall_normal.norm() < 1e-9) {return f;}
  f.n = q.wall_normal.normalized();
  f.u = Vec3::UnitZ().cross(f.n);
  if (f.u.norm() < 1e-6) {return f;}   // a horizontal wall plane has no base position on it
  f.u.normalize();
  f.ok = true;
  return f;
}

/// Open path over `ids` (indices into the chosen list): optional leg from the start, then the legs between stops.
struct Legs
{
  std::vector<double> from_start;              ///< Empty when the first stop is free.
  std::vector<std::vector<double>> between;

  double cost(const std::vector<size_t> & order) const
  {
    double t = 0.0;
    for (size_t i = 0; i < order.size(); ++i) {
      if (i == 0) {
        if (!from_start.empty()) {t += from_start[order[0]];}
      } else {
        t += between[order[i - 1]][order[i]];
      }
    }
    return t;
  }
};

/// 2-opt (segment reversal) to a local minimum, first improvement, fixed scan order. Returns true when it
/// converged, false when the clock or the pass limit stopped it.
bool twoOpt(
  const Legs & legs, std::vector<size_t> * order, const StandSelectionOptions & opt, const Clock::time_point & deadline,
  bool use_clock)
{
  double best = legs.cost(*order);
  for (size_t pass = 0; pass < opt.max_two_opt_passes; ++pass) {
    bool improved = false;
    for (size_t i = 0; i + 1 < order->size(); ++i) {
      for (size_t j = i + 1; j < order->size(); ++j) {
        if (use_clock && Clock::now() > deadline) {return false;}
        std::reverse(order->begin() + static_cast<long>(i), order->begin() + static_cast<long>(j) + 1);
        const double c = legs.cost(*order);
        if (c < best - 1e-9) {
          best = c;
          improved = true;
        } else {
          std::reverse(order->begin() + static_cast<long>(i), order->begin() + static_cast<long>(j) + 1);
        }
      }
    }
    if (!improved) {return true;}
  }
  return false;
}
}  // namespace

const char * toString(StandSelectionCode code)
{
  switch (code) {
    case StandSelectionCode::kOk: return "ok";
    case StandSelectionCode::kNoTargets: return "no_targets";
    case StandSelectionCode::kInvalidRequest: return "invalid_request";
    case StandSelectionCode::kNoCandidates: return "no_candidates";
    case StandSelectionCode::kUnreachableTargets: return "unreachable_targets";
  }
  return "unknown";
}

std::vector<CandidateStand> makeCandidateStands(const StationRequest & q)
{
  std::vector<CandidateStand> out;
  const Frame f = wallFrame(q);
  if (q.targets_world.empty() || !f.ok || !stationGridProblem(q).empty()) {return out;}
  double u_lo = 1e18, u_hi = -1e18;
  for (const auto & t : q.targets_world) {
    const double a = (t.translation() - q.wall_point).dot(f.u);
    u_lo = std::min(u_lo, a);
    u_hi = std::max(u_hi, a);
  }
  u_lo -= q.lateral_margin;
  u_hi += q.lateral_margin;
  for (double d = q.min_clearance; d <= q.max_standoff + 1e-9; d += q.grid_step) {
    for (double a = u_lo; a <= u_hi + 1e-9; a += q.grid_step) {
      const Vec3 p = q.wall_point + f.u * a + f.n * d;
      CandidateStand c;
      c.stand.arm_xy = Eigen::Vector2d(p.x(), p.y());
      c.standoff = d;
      if (q.base) {
        c.stand.yaw = bestYaw(*q.base, c.stand.arm_xy, Eigen::Vector2d(q.wall_point.x(), q.wall_point.y()),
            Eigen::Vector2d(f.n.x(), f.n.y()), &c.footprint_clearance);
        if (c.footprint_clearance < q.min_clearance - 1e-9) {continue;}
      }
      out.push_back(c);
    }
  }
  return out;
}

namespace
{
using PairMargins = std::map<std::pair<size_t, size_t>, double>;   // (candidate, target) -> margin; 0 blocks
}

namespace
{
StandSelection selectOnce(
  const Map4D & map, const StationRequest & q, const std::vector<CandidateStand> & candidates,
  const std::optional<Stand> & start_in, const StandSelectionOptions & opt, const PairMargins & overrides)
{
  const auto t0 = Clock::now();
  StandSelection res;
  res.candidates = candidates.size();
  auto finish = [&](StandSelectionCode code, std::string msg) {
      res.code = code;
      res.success = code == StandSelectionCode::kOk || code == StandSelectionCode::kNoTargets;
      res.message = std::move(msg);
      res.solve_time_s = std::chrono::duration<double>(Clock::now() - t0).count();
      return res;
    };

  const size_t nt = q.targets_world.size();
  if (nt == 0) {return finish(StandSelectionCode::kNoTargets, "no targets: nothing to cover");}
  const Frame f = wallFrame(q);
  const auto heights = heightRange(q.lift_min, q.lift_max, q.lift_step);
  if (!f.ok) {return finish(StandSelectionCode::kInvalidRequest, "wall normal is zero or vertical");}
  if (heights.empty()) {return finish(StandSelectionCode::kInvalidRequest, "empty lift range or non-positive lift_step");}
  if (!std::isfinite(opt.margin_penalty) || opt.margin_penalty < 0.0) {
    return finish(StandSelectionCode::kInvalidRequest, "margin_penalty must be finite and >= 0");
  }
  if (candidates.empty()) {
    for (size_t t = 0; t < nt; ++t) {res.unreachable.push_back(t);}
    return finish(StandSelectionCode::kNoCandidates, "no candidate stands");
  }

  // 1. score every candidate: margin[c][t] = share of lift heights that reach target t (0 = not reachable)
  const size_t nc = candidates.size();
  std::vector<std::vector<double>> margin(nc, std::vector<double>(nt, 0.0));
  std::vector<double> along(nc, 0.0);
  std::vector<double> weight(nc, 1.0);
  std::vector<size_t> cover_count(nc, 0);
  std::vector<bool> coverable(nt, false);
  for (size_t c = 0; c < nc; ++c) {
    const auto & s = candidates[c].stand;
    const auto reach = reachableByHeight(map, q.targets_world, s.arm_xy.x(), s.arm_xy.y(), heights, q.arm_mount_offset_z);
    double sum = 0.0;
    for (size_t t = 0; t < nt; ++t) {
      const auto n_ok = static_cast<size_t>(std::count(reach[t].begin(), reach[t].end(), true));
      if (n_ok == 0) {continue;}
      margin[c][t] = static_cast<double>(n_ok) / static_cast<double>(heights.size());
      const auto ov = overrides.find({c, t});
      if (ov != overrides.end()) {
        margin[c][t] = ov->second;   // the verifier's margin; 0 = refused, so not covered from here
        if (margin[c][t] <= 0.0) {continue;}
      }
      sum += margin[c][t];
      ++cover_count[c];
      coverable[t] = true;
    }
    if (cover_count[c] > 0) {
      ++res.useful_candidates;
      weight[c] = 1.0 + opt.margin_penalty * (1.0 - sum / static_cast<double>(cover_count[c]));
    }
    const Vec3 p(s.arm_xy.x(), s.arm_xy.y(), q.wall_point.z());
    along[c] = (p - q.wall_point).dot(f.u);
  }
  for (size_t t = 0; t < nt; ++t) {if (!coverable[t]) {res.unreachable.push_back(t);}}
  if (res.useful_candidates == 0) {return finish(StandSelectionCode::kNoCandidates, "no candidate stand reaches any target");}

  // 2. weighted greedy cover: lowest weight per newly covered target, ties to the lower index
  std::vector<bool> covered(nt, false);
  size_t left = static_cast<size_t>(std::count(coverable.begin(), coverable.end(), true));
  std::vector<size_t> picks;
  while (left > 0) {
    size_t best = nc;
    double best_ratio = 0.0;
    for (size_t c = 0; c < nc; ++c) {
      size_t gain = 0;
      for (size_t t = 0; t < nt; ++t) {gain += (margin[c][t] > 0.0 && !covered[t]) ? 1 : 0;}
      if (gain == 0) {continue;}
      const double ratio = weight[c] / static_cast<double>(gain);
      if (best == nc || ratio < best_ratio - 1e-12) {best = c; best_ratio = ratio;}
    }
    picks.push_back(best);
    for (size_t t = 0; t < nt; ++t) {
      if (margin[best][t] > 0.0 && !covered[t]) {covered[t] = true; --left;}
    }
  }
  res.greedy_stands = picks.size();
  // drop stands the others make redundant, latest pick first
  for (size_t k = picks.size(); k-- > 0;) {
    bool redundant = true;
    for (size_t t = 0; t < nt && redundant; ++t) {
      if (margin[picks[k]][t] <= 0.0) {continue;}
      bool other = false;
      for (size_t j = 0; j < picks.size() && !other; ++j) {other = j != k && margin[picks[j]][t] > 0.0;}
      redundant = other;
    }
    if (redundant) {picks.erase(picks.begin() + static_cast<long>(k));}
  }

  // 3. each target goes to the chosen stand with the best margin (ties: earlier pick); empty stands drop out
  std::vector<std::vector<size_t>> mine(picks.size());
  for (size_t t = 0; t < nt; ++t) {
    if (!coverable[t]) {continue;}
    size_t best = 0;
    for (size_t k = 1; k < picks.size(); ++k) {
      if (margin[picks[k]][t] > margin[picks[best]][t] + 1e-12) {best = k;}
    }
    mine[best].push_back(t);
  }
  std::vector<size_t> chosen;   // positions into picks, in pick order (stands that kept a target)
  for (size_t k = 0; k < picks.size(); ++k) {if (!mine[k].empty()) {chosen.push_back(k);}}
  const size_t m = chosen.size();

  // 4. tour: nearest neighbour then 2-opt, from the start pose when there is one
  const bool has_start = q.base.has_value() && start_in.has_value();
  auto leg = [&](const Stand & a, const Stand & b) {
      return q.base ? travelTime(*q.base, a, b) : (a.arm_xy - b.arm_xy).norm();
    };
  Legs legs;
  legs.between.assign(m, std::vector<double>(m, 0.0));
  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < m; ++j) {
      if (i != j) {legs.between[i][j] = leg(candidates[picks[chosen[i]]].stand, candidates[picks[chosen[j]]].stand);}
    }
  }
  if (has_start) {
    for (size_t i = 0; i < m; ++i) {legs.from_start.push_back(leg(*start_in, candidates[picks[chosen[i]]].stand));}
  }
  std::vector<size_t> input(m);
  std::iota(input.begin(), input.end(), size_t{0});
  res.input_order_cost = legs.cost(input);

  std::vector<size_t> nn;
  std::vector<bool> used(m, false);
  for (size_t step = 0; step < m; ++step) {
    size_t best = m;
    double best_d = 0.0;
    for (size_t i = 0; i < m; ++i) {
      if (used[i]) {continue;}
      double d;
      if (step == 0) {
        d = has_start ? legs.from_start[i] : along[picks[chosen[i]]];
      } else {
        d = legs.between[nn.back()][i];
      }
      if (best == m || d < best_d - 1e-12) {best = i; best_d = d;}
    }
    used[best] = true;
    nn.push_back(best);
  }
  // The budget is for 2-opt alone: it starts here, so slow scoring or a big cover cannot use it up.
  const bool use_clock = opt.time_budget_s > 0.0;
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(opt.time_budget_s));
  bool converged = twoOpt(legs, &nn, opt, deadline, use_clock);
  std::vector<size_t> seeded = input;   // the cover's own order is a second seed, so the result can never be worse than it
  converged = twoOpt(legs, &seeded, opt, deadline, use_clock) && converged;
  const std::vector<size_t> & tour = legs.cost(seeded) < legs.cost(nn) - 1e-9 ? seeded : nn;
  res.tour_budget_hit = !converged;
  res.tour_cost = legs.cost(tour);

  // 5. result in tour order, lift bands rebuilt over each stand's own targets
  for (size_t pos = 0; pos < m; ++pos) {
    const size_t i = tour[pos];
    const size_t cand = picks[chosen[i]];
    SelectedStand s;
    s.stand = candidates[cand].stand;
    s.standoff = candidates[cand].standoff;
    s.footprint_clearance = candidates[cand].footprint_clearance;
    s.candidate_index = cand;
    s.pick_index = chosen[i];
    s.targets = mine[chosen[i]];
    std::vector<Pose> subset;
    double msum = 0.0;
    for (const size_t t : s.targets) {subset.push_back(q.targets_world[t]); msum += margin[cand][t];}
    s.mean_margin = msum / static_cast<double>(s.targets.size());
    s.lift_bands = planLiftBands(map, subset, s.stand.arm_xy.x(), s.stand.arm_xy.y(), heights, q.arm_mount_offset_z);
    for (auto & band : s.lift_bands.bands) {for (auto & k : band.pose_indices) {k = s.targets[k];}}
    for (auto & k : s.lift_bands.unreachable) {k = s.targets[k];}
    s.travel_cost = pos == 0 ? (has_start ? legs.from_start[i] : 0.0) : legs.between[tour[pos - 1]][i];
    res.stands.push_back(std::move(s));
  }
  if (!res.unreachable.empty()) {
    return finish(StandSelectionCode::kUnreachableTargets,
        std::to_string(res.unreachable.size()) + " of " + std::to_string(nt) + " targets are reachable from no candidate stand");
  }
  return finish(StandSelectionCode::kOk, "ok");
}

bool vv_all_cached(const SelectedStand & st, const std::map<std::pair<size_t, size_t>, TargetReport> & cache)
{
  for (const size_t t : st.targets) {
    if (!cache.count({st.candidate_index, t})) {return false;}
  }
  return true;
}
}  // namespace

StandSelection selectStands(
  const Map4D & map, const StationRequest & q, const std::vector<CandidateStand> & candidates,
  const std::optional<Stand> & start, const StandSelectionOptions & opt)
{
  if (!opt.verifier) {return selectOnce(map, q, candidates, start, opt, {});}

  // Verification pass: confirm every chosen stand's targets, block what is refused, re-rank by the IK
  // margins, recompute the cover. Each round verifies at least one new pair, so it ends.
  const auto heights = heightRange(q.lift_min, q.lift_max, q.lift_step);
  PairMargins overrides;
  std::map<std::pair<size_t, size_t>, TargetReport> cache;
  VerificationSummary sum;
  sum.enabled = true;
  StandSelection res;
  const size_t max_rounds = std::max<size_t>(1, opt.verify.max_rounds);
  for (size_t round = 1; round <= max_rounds; ++round) {
    res = selectOnce(map, q, candidates, start, opt, overrides);
    sum.rounds = round;
    bool fresh = false;
    for (auto & st : res.stands) {
      std::vector<size_t> todo;
      for (const size_t t : st.targets) {
        if (!cache.count({st.candidate_index, t})) {todo.push_back(t);}
      }
      if (todo.empty()) {continue;}   // every pair known from an earlier round: no map query, no verifier call
      if (opt.verify.max_verifier_calls > 0 && sum.verifier_calls >= opt.verify.max_verifier_calls) {
        sum.budget_hit = true;
        continue;
      }
      std::vector<std::vector<double>> hs;
      const auto reach = reachableByHeight(map, q.targets_world, st.stand.arm_xy.x(), st.stand.arm_xy.y(), heights,
          q.arm_mount_offset_z);
      for (const size_t t : todo) {
        std::vector<double> ok;
        for (size_t k = 0; k < heights.size(); ++k) {if (reach[t][k]) {ok.push_back(heights[k]);}}
        hs.push_back(std::move(ok));
      }
      fresh = true;
      const auto v = verifyStand(*opt.verifier, st.stand, q.targets_world, todo, hs, opt.verify);
      sum.verifier_calls += v.verifier_calls;
      sum.verifier_poses += v.verifier_poses;
      for (const auto & r : v.targets) {
        cache[{st.candidate_index, r.target}] = r;
        ++sum.verified_pairs;
        if (r.confirmed) {
          ++sum.confirmed_pairs;
          overrides[{st.candidate_index, r.target}] = r.margin;
        } else {
          overrides[{st.candidate_index, r.target}] = 0.0;
          sum.refused.push_back({st.candidate_index, r.target, r.reason});
        }
      }
    }
    if (sum.budget_hit) {sum.converged = false; break;}
    if (!fresh) {break;}
    if (round == max_rounds) {sum.converged = false;}
  }
  if (!sum.converged) {
    // The last cover was never fully verified: re-run it so the result is consistent with what is known.
    res = selectOnce(map, q, candidates, start, opt, overrides);
  }
  // Report: per stand, the verified targets only (confirmed), IK margins and the confirmed lift bands.
  for (auto & st : res.stands) {
    std::vector<size_t> verified_targets;
    StandVerification vv;
    vv.stand = st.stand;
    for (const size_t t : st.targets) {
      const auto it = cache.find({st.candidate_index, t});
      if (it != cache.end() && it->second.confirmed) {vv.targets.push_back(it->second);}
    }
    // Rebuild the aggregate from the cache: the bands over the confirmed targets at their best heights.
    vv.confirmed = vv.targets.size();
    vv.coverage = st.targets.empty() ? 1.0 : static_cast<double>(vv.confirmed) / static_cast<double>(st.targets.size());
    double msum = 0.0;
    std::map<double, std::vector<size_t>> at;
    for (const auto & r : vv.targets) {msum += r.margin; at[r.best_height].push_back(r.target);}
    vv.mean_margin = vv.confirmed ? msum / static_cast<double>(vv.confirmed) : 0.0;
    for (const auto & [h, ts] : at) {vv.bands.push_back({h, ts});}
    st.verified = vv_all_cached(st, cache);
    st.verification = std::move(vv);
    if (st.verification.confirmed > 0) {
      st.mean_margin = st.verification.mean_margin;
      st.lift_bands.bands = st.verification.bands;   // the bands IK confirmed, not the RM4D ones
      st.lift_bands.unreachable.clear();
    }
  }
  for (const size_t t : res.unreachable) {
    for (const auto & r : sum.refused) {
      if (r.target == t) {res.rejected_by_verifier.push_back(t); break;}
    }
  }
  res.verification = std::move(sum);
  if (!res.unreachable.empty() && !res.rejected_by_verifier.empty()) {
    res.message += "; " + std::to_string(res.rejected_by_verifier.size()) +
      " of them reachable by RM4D but refused by IK / collision checking";
  }
  return res;
}

StandSelection selectStands(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start,
  const StandSelectionOptions & options)
{
  if (const std::string problem = stationGridProblem(request); !problem.empty()) {
    StandSelection bad;
    bad.code = StandSelectionCode::kInvalidRequest;
    bad.message = problem;
    return bad;
  }
  return selectStands(map, request, makeCandidateStands(request), start, options);
}

}  // namespace reachability
