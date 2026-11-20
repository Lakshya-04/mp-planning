// StandStrategy::SetCover: see core/stands.hpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "reachability/core/stands.hpp"

namespace reachability
{
namespace
{
using Bits = std::vector<uint64_t>;

struct Candidate
{
  Stand stand;
  double standoff{0.0};
  double clearance{0.0};
  double along{0.0};     ///< Position along the wall (ordering without a base model).
  Bits covers;
  size_t count{0};
};

size_t countNew(const Bits & c, const Bits & covered)
{
  size_t n = 0;
  for (size_t i = 0; i < c.size(); ++i) {n += static_cast<size_t>(__builtin_popcountll(c[i] & ~covered[i]));}
  return n;
}

bool subsetOf(const Bits & a, const Bits & b)
{
  for (size_t i = 0; i < a.size(); ++i) {if (a[i] & ~b[i]) {return false;}}
  return true;
}

bool test(const Bits & b, size_t i) {return (b[i >> 6] >> (i & 63)) & 1U;}
void set(Bits & b, size_t i) {b[i >> 6] |= uint64_t{1} << (i & 63);}
void orInto(Bits & a, const Bits & b) {for (size_t i = 0; i < a.size(); ++i) {a[i] |= b[i];}}

/// Same search grid and footprint test as chooseStation, but every station that reaches anything is kept.
std::vector<Candidate> enumerate(const Map4D & map, const StationRequest & q, size_t words, Bits * coverable)
{
  std::vector<Candidate> out;
  if (q.targets_world.empty() || q.grid_step <= 0.0 || q.wall_normal.norm() < 1e-9) {return out;}
  const Vec3 n = q.wall_normal.normalized();
  Vec3 u = Vec3::UnitZ().cross(n);
  if (u.norm() < 1e-6) {return out;}
  u.normalize();
  double u_lo = 1e18, u_hi = -1e18;
  for (const auto & t : q.targets_world) {
    const double a = (t.translation() - q.wall_point).dot(u);
    u_lo = std::min(u_lo, a);
    u_hi = std::max(u_hi, a);
  }
  u_lo -= q.lateral_margin;
  u_hi += q.lateral_margin;
  const auto heights = heightRange(q.lift_min, q.lift_max, q.lift_step);
  for (double d = q.min_clearance; d <= q.max_standoff + 1e-9; d += q.grid_step) {
    for (double a = u_lo; a <= u_hi + 1e-9; a += q.grid_step) {
      const Vec3 p = q.wall_point + u * a + n * d;
      if (stationAvoided(q, Eigen::Vector2d(p.x(), p.y()))) {continue;}
      Candidate c;
      c.stand.arm_xy = Eigen::Vector2d(p.x(), p.y());
      if (q.base) {
        c.stand.yaw = bestYaw(*q.base, c.stand.arm_xy, Eigen::Vector2d(q.wall_point.x(), q.wall_point.y()),
          Eigen::Vector2d(n.x(), n.y()), &c.clearance);
        if (c.clearance < q.min_clearance - 1e-9) {continue;}
      }
      const auto plan = planLiftBands(map, q.targets_world, p.x(), p.y(), heights, q.arm_mount_offset_z);
      if (plan.reachable_count == 0) {continue;}
      c.covers.assign(words, 0);
      for (const auto & band : plan.bands) {for (const size_t k : band.pose_indices) {set(c.covers, k);}}
      c.count = plan.reachable_count;
      c.standoff = d;
      c.along = a;
      orInto(*coverable, c.covers);
      out.push_back(std::move(c));
    }
  }
  return out;
}

/// Bounded search for a cover with at most `k` stands over `pool` (maximal candidate sets only).
class CoverSearch
{
public:
  CoverSearch(const std::vector<Candidate> & all, const std::vector<size_t> & pool, const Bits & universe,
    size_t budget)
  : all_(all), pool_(pool), universe_(universe), budget_(budget) {}

  bool run(size_t k, std::vector<size_t> * out)
  {
    nodes_ = 0;
    chosen_.clear();
    Bits covered(universe_.size(), 0);
    if (!dfs(k, covered)) {return false;}
    *out = chosen_;
    return true;
  }

private:
  bool dfs(size_t left, const Bits & covered)
  {
    const size_t missing = countNew(universe_, covered);
    if (missing == 0) {return true;}
    if (left == 0 || ++nodes_ > budget_) {return false;}
    size_t max_new = 0;
    for (const size_t ci : pool_) {max_new = std::max(max_new, countNew(all_[ci].covers, covered));}
    if (max_new * left < missing) {return false;}
    // branch on the uncovered target that the fewest candidates reach
    size_t best_t = 0, best_n = ~size_t{0};
    for (size_t t = 0; t < universe_.size() * 64; ++t) {
      if (!test(universe_, t) || test(covered, t)) {continue;}
      size_t n = 0;
      for (const size_t ci : pool_) {n += test(all_[ci].covers, t) ? 1 : 0;}
      if (n < best_n) {best_n = n; best_t = t;}
    }
    std::vector<std::pair<size_t, size_t>> opts;   // (new targets, candidate)
    for (const size_t ci : pool_) {
      if (test(all_[ci].covers, best_t)) {opts.emplace_back(countNew(all_[ci].covers, covered), ci);}
    }
    std::stable_sort(opts.begin(), opts.end(), [](const auto & a, const auto & b) {return a.first > b.first;});
    for (const auto & o : opts) {
      Bits next = covered;
      orInto(next, all_[o.second].covers);
      chosen_.push_back(o.second);
      if (dfs(left - 1, next)) {return true;}
      chosen_.pop_back();
      if (nodes_ > budget_) {return false;}
    }
    return false;
  }

  const std::vector<Candidate> & all_;
  const std::vector<size_t> & pool_;
  const Bits & universe_;
  size_t budget_;
  size_t nodes_{0};
  std::vector<size_t> chosen_;
};

struct Router
{
  const StationRequest & q;
  const std::vector<Candidate> & c;
  const std::optional<Stand> & start;

  double leg(const Stand & a, const Stand & b) const
  {
    return q.base ? travelTime(*q.base, a, b) : (a.arm_xy - b.arm_xy).norm();
  }
  double cost(const std::vector<size_t> & order) const
  {
    double t = 0.0;
    for (size_t i = 0; i < order.size(); ++i) {
      if (i > 0) {
        t += leg(c[order[i - 1]].stand, c[order[i]].stand);
      } else if (start) {
        t += leg(*start, c[order[0]].stand);
      }
    }
    return t;
  }
  /// Nearest next from the start (or from the end of the wall), then 2-opt on the whole path cost.
  std::vector<size_t> order(std::vector<size_t> left) const
  {
    std::vector<size_t> out;
    if (left.empty()) {return out;}
    std::optional<Stand> at = start;
    if (!at) {
      const auto first = std::min_element(left.begin(), left.end(), [&](size_t a, size_t b) {
          return c[a].along < c[b].along;
        });
      out.push_back(*first);
      at = c[*first].stand;
      left.erase(first);
    }
    while (!left.empty()) {
      const auto next = std::min_element(left.begin(), left.end(), [&](size_t a, size_t b) {
          return leg(*at, c[a].stand) < leg(*at, c[b].stand);
        });
      out.push_back(*next);
      at = c[*next].stand;
      left.erase(next);
    }
    for (bool improved = true; improved;) {
      improved = false;
      for (size_t i = 0; i + 1 < out.size(); ++i) {
        for (size_t j = i + 1; j < out.size(); ++j) {
          auto cand = out;
          std::reverse(cand.begin() + static_cast<long>(i), cand.begin() + static_cast<long>(j) + 1);
          if (cost(cand) < cost(out) - 1e-9) {out = std::move(cand); improved = true;}
        }
      }
    }
    return out;
  }
};
}  // namespace

StandPlan planStandsSetCover(
  const Map4D & map, const StationRequest & request, const std::optional<Stand> & start_in,
  size_t max_stands, const SetCoverOptions & opt)
{
  StandPlan plan;
  const size_t n_targets = request.targets_world.size();
  const size_t words = (n_targets + 63) / 64;
  Bits coverable(words, 0);
  const auto cands = enumerate(map, request, words, &coverable);
  const std::optional<Stand> start = request.base ? start_in : std::nullopt;
  for (size_t i = 0; i < n_targets; ++i) {if (!test(coverable, i)) {plan.unreachable.push_back(i);}}
  if (cands.empty()) {return plan;}
  const Router router{request, cands, start};

  // 1. weighted greedy cover: new targets per unit of (1 + weight * travel from the last stand)
  std::vector<size_t> chosen;
  Bits covered(words, 0);
  std::optional<Stand> at = start;
  while (countNew(coverable, covered) > 0) {
    size_t best = 0;
    double best_score = -1.0;
    for (size_t i = 0; i < cands.size(); ++i) {
      const size_t gain = countNew(cands[i].covers, covered);
      if (gain == 0) {continue;}
      const double travel = (request.base && at) ? travelTime(*request.base, *at, cands[i].stand) : 0.0;
      const double score = static_cast<double>(gain) / (1.0 + opt.travel_weight * travel);
      if (score > best_score + 1e-12) {best_score = score; best = i;}
    }
    chosen.push_back(best);
    orInto(covered, cands[best].covers);
    if (request.base) {at = cands[best].stand;}
  }

  // 2. fewer stands by a bounded exact search over the candidates no other candidate dominates
  if (opt.node_budget > 0 && chosen.size() > 1) {
    std::vector<size_t> pool;
    for (size_t i = 0; i < cands.size(); ++i) {
      bool dominated = false;
      for (size_t j = 0; j < cands.size() && !dominated; ++j) {
        if (i == j || !subsetOf(cands[i].covers, cands[j].covers)) {continue;}
        dominated = cands[i].count < cands[j].count || j < i;    // equal sets: keep the first
      }
      if (!dominated) {pool.push_back(i);}
    }
    CoverSearch search(cands, pool, coverable, opt.node_budget);
    std::vector<size_t> better;
    while (chosen.size() > 1 && search.run(chosen.size() - 1, &better)) {chosen = better;}
  }

  // 3. drop stands the others make redundant, order them, then move each stand to the cheapest
  // station that still reaches the targets only it reaches
  for (bool again = true; again;) {
    again = false;
    for (size_t i = 0; i < chosen.size() && !again; ++i) {
      Bits rest(words, 0);
      for (size_t j = 0; j < chosen.size(); ++j) {if (j != i) {orInto(rest, cands[chosen[j]].covers);}}
      if (subsetOf(cands[chosen[i]].covers, rest)) {
        chosen.erase(chosen.begin() + static_cast<long>(i));
        again = true;
      }
    }
  }
  auto order = router.order(chosen);
  for (int pass = 0; pass < 5; ++pass) {
    bool moved = false;
    for (size_t i = 0; i < order.size(); ++i) {
      Bits rest(words, 0), only(words, 0);
      for (size_t j = 0; j < order.size(); ++j) {if (j != i) {orInto(rest, cands[order[j]].covers);}}
      for (size_t w = 0; w < words; ++w) {only[w] = coverable[w] & ~rest[w];}
      double best_cost = router.cost(order);
      size_t best = order[i];
      for (size_t ci = 0; ci < cands.size(); ++ci) {
        if (ci == order[i] || !subsetOf(only, cands[ci].covers)) {continue;}
        auto trial = order;
        trial[i] = ci;
        const double cst = router.cost(trial);
        if (cst < best_cost - 1e-9) {best_cost = cst; best = ci;}
      }
      if (best != order[i]) {order[i] = best; moved = true;}
    }
    if (!moved) {break;}
    order = router.order(order);
  }

  // 4. assign targets in visiting order, rebuild the lift bands for what each stand takes
  const auto heights = heightRange(request.lift_min, request.lift_max, request.lift_step);
  Bits done(words, 0);
  std::optional<Stand> from = start;
  for (const size_t ci : order) {
    if (plan.stands.size() >= max_stands) {break;}
    const Candidate & c = cands[ci];
    StandAssignment a;
    std::vector<Pose> subset;
    for (size_t t = 0; t < n_targets; ++t) {
      if (test(c.covers, t) && !test(done, t)) {
        a.targets.push_back(t);
        subset.push_back(request.targets_world[t]);
      }
    }
    if (a.targets.empty()) {continue;}
    for (const size_t t : a.targets) {set(done, t);}
    a.station.found = true;
    a.station.x = c.stand.arm_xy.x();
    a.station.y = c.stand.arm_xy.y();
    a.station.standoff = c.standoff;
    a.station.yaw = c.stand.yaw;
    a.station.footprint_clearance = c.clearance;
    a.station.evaluated = cands.size();
    a.station.plan = planLiftBands(map, subset, a.station.x, a.station.y, heights, request.arm_mount_offset_z);
    for (auto & band : a.station.plan.bands) {for (auto & k : band.pose_indices) {k = a.targets[k];}}
    for (auto & k : a.station.plan.unreachable) {k = a.targets[k];}
    a.stand = c.stand;
    if (request.base && from) {
      a.travel_s = travelTime(*request.base, *from, a.stand);
      plan.total_travel_s += a.travel_s;
    }
    if (request.base) {from = a.stand;}
    plan.stands.push_back(std::move(a));
  }
  for (size_t t = 0; t < n_targets; ++t) {
    if (test(coverable, t) && !test(done, t)) {plan.not_planned.push_back(t);}
  }
  return plan;
}

}  // namespace reachability
