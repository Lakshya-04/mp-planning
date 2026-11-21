#include "reachability/core/base_placement.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <functional>
#include <queue>

#include "reachability/core/global_stands.hpp"
#include "reachability/core/lift_bands.hpp"

namespace reachability
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
using V2 = Eigen::Vector2d;

double wrapPi(double a)
{
  while (a > kPi) {a -= 2.0 * kPi;}
  while (a < -kPi) {a += 2.0 * kPi;}
  return a;
}

double distToSegment(const V2 & p, const V2 & a, const V2 & b)
{
  const V2 ab = b - a;
  const double l2 = ab.squaredNorm();
  const double t = l2 < 1e-18 ? 0.0 : std::clamp((p - a).dot(ab) / l2, 0.0, 1.0);
  return (p - (a + t * ab)).norm();
}

/// Circumscribed and inscribed radius of the footprint about the base origin.
void footprintRadii(const std::optional<BaseModel> & base, double * circ, double * insc)
{
  *circ = 0.0;
  *insc = 0.0;
  if (!base || base->footprint.empty()) {return;}
  const auto & f = base->footprint;
  double mx = 0.0, mn = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < f.size(); ++i) {
    mx = std::max(mx, f[i].norm());
    mn = std::min(mn, distToSegment(V2(0.0, 0.0), f[i], f[(i + 1) % f.size()]));
  }
  *circ = mx;
  *insc = std::isfinite(mn) ? mn : 0.0;
}
}  // namespace

const char * toString(NavReason r)
{
  switch (r) {
    case NavReason::kOk: return "ok";
    case NavReason::kStartBlocked: return "start_blocked";
    case NavReason::kGoalBlocked: return "goal_blocked";
    case NavReason::kDisconnected: return "unreachable_by_nav";
    case NavReason::kOutsideMap: return "outside_nav_map";
  }
  return "unknown";
}

const char * toString(BpoReason r)
{
  switch (r) {
    case BpoReason::kOk: return "ok";
    case BpoReason::kSplitNoWholeStand: return "split_no_whole_stand";
    case BpoReason::kSkippedToAvoidSplit: return "skipped_to_avoid_split";
    case BpoReason::kOutOfReach: return "out_of_reach";
    case BpoReason::kLowMargin: return "low_margin";
    case BpoReason::kRefusedByVerifier: return "refused_by_verifier";
    case BpoReason::kNearSingular: return "near_singular";
    case BpoReason::kNearJointLimit: return "near_joint_limit";
    case BpoReason::kUnreachableByNav: return "unreachable_by_nav";
    case BpoReason::kYawWindow: return "yaw_window";
    case BpoReason::kNoFreeStand: return "no_free_stand";
    case BpoReason::kMaxStands: return "max_stands";
  }
  return "unknown";
}

const char * toString(BpoEscalation e)
{
  switch (e) {
    case BpoEscalation::kNone: return "none";
    case BpoEscalation::kLiftHeight: return "lift_height";
    case BpoEscalation::kOtherStand: return "other_stand";
    case BpoEscalation::kSplit: return "split";
    case BpoEscalation::kSkipped: return "skipped";
    case BpoEscalation::kUnreachable: return "unreachable";
  }
  return "unknown";
}

// ---------------------------------------------------------------- grid navigation

GridNavFeasibility::GridNavFeasibility(
  const FreeSpace & fs, const std::optional<BaseModel> & base, const V2 & lo, const V2 & hi,
  const GridNavParams & p)
: base_(base), lo_(lo), res_(p.resolution > 1e-3 ? p.resolution : 0.1)
{
  double circ = 0.0, insc = 0.0;
  footprintRadii(base_, &circ, &insc);
  snap_ = p.snap_m >= 0.0 ? p.snap_m : (circ - insc) + res_;
  const V2 span = (hi - lo).cwiseMax(V2(res_, res_));
  // Keep the grid bounded: coarsen the cell instead of running out of memory.
  while ((span.x() / res_ + 1.0) * (span.y() / res_ + 1.0) > 4.0e6) {res_ *= 1.5;}
  nx_ = static_cast<size_t>(std::floor(span.x() / res_)) + 1;
  ny_ = static_cast<size_t>(std::floor(span.y() / res_)) + 1;
  free_.assign(nx_ * ny_, 0);
  const double r = circ + p.margin;
  for (size_t j = 0; j < ny_; ++j) {
    for (size_t i = 0; i < nx_; ++i) {
      const V2 c = lo_ + V2((static_cast<double>(i) + 0.5) * res_, (static_cast<double>(j) + 0.5) * res_);
      free_[j * nx_ + i] = fs.pointFree(c, r) ? 1 : 0;
    }
  }
}

bool GridNavFeasibility::inside(const V2 & p) const
{
  const V2 q = p - lo_;
  return q.x() >= 0.0 && q.y() >= 0.0 && q.x() < static_cast<double>(nx_) * res_ && q.y() < static_cast<double>(ny_) * res_;
}

V2 GridNavFeasibility::centreOf(const Stand & s) const
{
  return base_ ? baseCentre(*base_, s) : s.arm_xy;
}

void GridNavFeasibility::flood(const V2 & c) const
{
  reached_.assign(nx_ * ny_, 0);
  have_flood_ = true;
  flood_start_ = c;
  std::queue<size_t> q;
  const int reach = static_cast<int>(std::ceil(snap_ / res_));
  const long ci = static_cast<long>(std::floor((c.x() - lo_.x()) / res_));
  const long cj = static_cast<long>(std::floor((c.y() - lo_.y()) / res_));
  for (long dj = -reach; dj <= reach; ++dj) {
    for (long di = -reach; di <= reach; ++di) {
      const long i = ci + di, j = cj + dj;
      if (i < 0 || j < 0 || i >= static_cast<long>(nx_) || j >= static_cast<long>(ny_)) {continue;}
      const V2 cc = lo_ + V2((static_cast<double>(i) + 0.5) * res_, (static_cast<double>(j) + 0.5) * res_);
      if ((cc - c).norm() > snap_ + 1e-9) {continue;}
      const size_t k = static_cast<size_t>(j) * nx_ + static_cast<size_t>(i);
      if (free_[k] && !reached_[k]) {reached_[k] = 1; q.push(k);}
    }
  }
  while (!q.empty()) {
    const size_t k = q.front();
    q.pop();
    const long i = static_cast<long>(k % nx_), j = static_cast<long>(k / nx_);
    for (long dj = -1; dj <= 1; ++dj) {
      for (long di = -1; di <= 1; ++di) {
        if (di == 0 && dj == 0) {continue;}
        const long a = i + di, b = j + dj;
        if (a < 0 || b < 0 || a >= static_cast<long>(nx_) || b >= static_cast<long>(ny_)) {continue;}
        const size_t n = static_cast<size_t>(b) * nx_ + static_cast<size_t>(a);
        if (free_[n] && !reached_[n]) {reached_[n] = 1; q.push(n);}
      }
    }
  }
}

bool GridNavFeasibility::nearComponent(const V2 & p) const
{
  const int reach = static_cast<int>(std::ceil(snap_ / res_));
  const long ci = static_cast<long>(std::floor((p.x() - lo_.x()) / res_));
  const long cj = static_cast<long>(std::floor((p.y() - lo_.y()) / res_));
  for (long dj = -reach; dj <= reach; ++dj) {
    for (long di = -reach; di <= reach; ++di) {
      const long i = ci + di, j = cj + dj;
      if (i < 0 || j < 0 || i >= static_cast<long>(nx_) || j >= static_cast<long>(ny_)) {continue;}
      const V2 cc = lo_ + V2((static_cast<double>(i) + 0.5) * res_, (static_cast<double>(j) + 0.5) * res_);
      if ((cc - p).norm() > snap_ + 1e-9) {continue;}
      if (reached_[static_cast<size_t>(j) * nx_ + static_cast<size_t>(i)]) {return true;}
    }
  }
  return false;
}

NavVerdict GridNavFeasibility::feasible(const Stand & start, const Stand & goal) const
{
  const V2 s = centreOf(start), g = centreOf(goal);
  if (!inside(s)) {return {false, NavReason::kStartBlocked};}
  if (!inside(g)) {return {false, NavReason::kOutsideMap};}
  if (!have_flood_ || (flood_start_ - s).norm() > 1e-9) {flood(s);}
  if (std::none_of(reached_.begin(), reached_.end(), [](unsigned char c) {return c != 0;})) {
    return {false, NavReason::kStartBlocked};
  }
  if (nearComponent(g)) {return {true, NavReason::kOk};}
  // Blocked (no free cell near the goal at all) or free but not connected.
  const int reach = static_cast<int>(std::ceil(snap_ / res_));
  const long ci = static_cast<long>(std::floor((g.x() - lo_.x()) / res_));
  const long cj = static_cast<long>(std::floor((g.y() - lo_.y()) / res_));
  for (long dj = -reach; dj <= reach; ++dj) {
    for (long di = -reach; di <= reach; ++di) {
      const long i = ci + di, j = cj + dj;
      if (i < 0 || j < 0 || i >= static_cast<long>(nx_) || j >= static_cast<long>(ny_)) {continue;}
      if (free_[static_cast<size_t>(j) * nx_ + static_cast<size_t>(i)]) {return {false, NavReason::kDisconnected};}
    }
  }
  return {false, NavReason::kGoalBlocked};
}

// ---------------------------------------------------------------- planner

namespace
{
int rank(ColumnStatus s) {return s == ColumnStatus::kWhole ? 2 : (s == ColumnStatus::kSplit ? 1 : 0);}

struct Eval
{
  ColumnStatus status{ColumnStatus::kUnreachable};
  bool ends_usable{false};
  size_t usable{0};
  double min_score{0.0};
  std::vector<size_t> strokes;
};

Eval evalAt(
  const Map4D & map, const ColumnPoseSequence & seq, const V2 & xy, const std::vector<double> & heights,
  double mount, const MarginConfig & margin)
{
  Eval e;
  if (seq.poses.empty()) {return e;}
  const ColumnFeasibility f = evaluateColumn(map, seq, xy, heights, mount, margin);
  e.status = f.status;
  e.ends_usable = f.best.ends_usable;
  e.min_score = f.best.min_score;
  for (const auto & p : f.best.poses) {
    if (p.role == PoseRole::kStroke && p.usable) {e.strokes.push_back(p.stroke);}
  }
  e.usable = e.strokes.size();
  return e;
}

bool betterEval(const Eval & a, const Eval & b)
{
  if (rank(a.status) != rank(b.status)) {return rank(a.status) > rank(b.status);}
  if (a.ends_usable != b.ends_usable) {return a.ends_usable;}
  return a.usable > b.usable;
}

/// Work wall as obstacles / walls for navigation and clearance.
FreeSpace withWorkWall(const FreeSpace & fs, const SurfaceFrame & f, const Vec3 & origin)
{
  FreeSpace out = fs;
  if (f.horizontal) {return out;}
  const V2 n(f.n.x(), f.n.y());
  const V2 p(origin.x(), origin.y());
  if (!fs.has_work_span) {
    out.walls.push_back({p, n});
  } else {
    const V2 u(f.u.x(), f.u.y());
    const V2 a = p + u * fs.work_span_lo, b = p + u * fs.work_span_hi;
    out.obstacles.push_back({a, b, b - n * 0.02, a - n * 0.02});
  }
  return out;
}

/// Work-plane clearance of a footprint (only points inside the work span when one is set).
double planeClearance(
  const BaseModel & base, const Stand & st, const FreeSpace & fs, const SurfaceFrame & f, const Vec3 & origin)
{
  const V2 n(f.n.x(), f.n.y());
  const V2 wp(origin.x(), origin.y());
  if (!fs.has_work_span) {return wallClearance(base, st, wp, n);}
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : footprintWorld(base, st)) {
    const double a = (Vec3(p.x(), p.y(), origin.z()) - origin).dot(f.u);
    if (a < fs.work_span_lo - 1e-9 || a > fs.work_span_hi + 1e-9) {continue;}
    best = std::min(best, (p - wp).dot(n));
  }
  return best;
}

double clearanceOf(const FreeSpace & fs, const std::optional<BaseModel> & base, const Stand & st)
{
  auto free_at = [&](double m) {return base ? fs.footprintFree(st, *base, m) : fs.pointFree(st.arm_xy, m);};
  constexpr double kCap = 2.0;
  if (free_at(kCap)) {return kCap;}
  double lo = 0.0, hi = kCap;
  if (!free_at(0.0)) {return 0.0;}
  for (int i = 0; i < 14; ++i) {
    const double mid = 0.5 * (lo + hi);
    (free_at(mid) ? lo : hi) = mid;
  }
  return lo;
}

/// Fewest sets (masks) covering `all`, by iterative deepening with a node budget; nullopt when the budget ran out.
std::optional<size_t> exactCover(std::vector<uint64_t> masks, uint64_t all, size_t budget)
{
  if (all == 0) {return 0;}
  std::sort(masks.begin(), masks.end());
  masks.erase(std::unique(masks.begin(), masks.end()), masks.end());
  std::vector<uint64_t> keep;
  for (size_t i = 0; i < masks.size(); ++i) {
    bool dominated = false;
    for (size_t j = 0; j < masks.size() && !dominated; ++j) {
      if (i != j && (masks[i] & masks[j]) == masks[i] && masks[i] != masks[j]) {dominated = true;}
    }
    if (!dominated && masks[i] != 0) {keep.push_back(masks[i]);}
  }
  size_t nodes = 0;
  bool exhausted = false;
  size_t limit = 0;
  std::function<bool(uint64_t, size_t)> dfs = [&](uint64_t left, size_t depth) -> bool {
      if (left == 0) {return true;}
      if (depth == limit) {return false;}
      if (++nodes > budget) {exhausted = true; return false;}
      const uint64_t bit = left & (~left + 1);
      for (const uint64_t m : keep) {
        if ((m & bit) == 0) {continue;}
        if (dfs(left & ~m, depth + 1)) {return true;}
        if (exhausted) {return false;}
      }
      return false;
    };
  for (limit = 1; limit <= keep.size(); ++limit) {
    if (dfs(all, 0)) {return limit;}
    if (exhausted) {return std::nullopt;}
  }
  return std::nullopt;
}

double legCost(const std::optional<BaseModel> & base, const Stand & a, const Stand & b)
{
  return base ? travelTime(*base, a, b) : (a.arm_xy - b.arm_xy).norm();
}
}  // namespace

BasePlacement planBasePlacement(
  const Map4D & map, const SurfaceFrame & frame, const std::vector<ColumnPlan> & columns,
  const FreeSpace & fs_in, const std::optional<Stand> & start, const BasePlacementOptions & opt)
{
  BasePlacement out;
  const size_t nc = columns.size();
  out.columns.resize(nc);
  for (size_t k = 0; k < nc; ++k) {
    out.columns[k].column = k;
    out.columns[k].label = columns[k].label;
    out.columns[k].strokes_total = columns[k].strokes_world.size();
    out.strokes_total += columns[k].strokes_world.size();
  }
  out.covered_fraction = out.reachable_fraction = 1.0;
  if (!frame.ok) {
    out.message = "the surface frame is not valid (zero normal)";
    out.covered_fraction = out.reachable_fraction = 0.0;
    for (auto & c : out.columns) {c.reason = BpoReason::kNoFreeStand;}
    out.unreachable = nc;
    return out;
  }
  if (out.strokes_total == 0) {
    out.success = true;
    out.message = "no strokes to cover";
    out.whole = nc;
    for (auto & c : out.columns) {c.status = ColumnStatus::kWhole; c.reason = BpoReason::kOk; c.escalation = BpoEscalation::kNone;}
    return out;
  }

  StationRequest station = opt.search;
  station.targets_world.clear();
  for (const auto & c : columns) {
    station.targets_world.insert(station.targets_world.end(), c.strokes_world.begin(), c.strokes_world.end());
  }
  station.wall_point = frame.origin;
  station.wall_normal = frame.n;
  station.free_space = fs_in.empty() ? std::nullopt : std::optional<FreeSpace>(fs_in);
  const std::optional<BaseModel> & base = station.base;
  const FreeSpace nav_fs = withWorkWall(fs_in, frame, frame.origin);
  if (const std::string problem = stationGridProblem(station); !problem.empty()) {
    out.message = problem;
    out.covered_fraction = out.reachable_fraction = 0.0;
    out.unreachable = nc;
    return out;
  }

  // ---- candidates, then navigation and yaw filters
  const std::vector<CandidateStand> cands0 = makeCandidateStands(station);
  out.candidates = cands0.size();
  const std::vector<double> heights = heightRange(station.lift_min, station.lift_max, station.lift_step);
  if (heights.empty()) {
    out.message = "lift range is not usable";
    out.covered_fraction = out.reachable_fraction = 0.0;
    out.unreachable = nc;
    return out;
  }

  double window = opt.yaw_window_rad;
  if (window < 0.0) {window = (base && !base->can_strafe) ? opt.diff_yaw_window_rad : kPi;}
  const bool use_window = base && window < kPi - 1e-9 && !frame.horizontal;
  const double facing = std::atan2(-frame.n.y(), -frame.n.x());
  const int steps = std::max(opt.yaw_steps, 4);

  std::unique_ptr<GridNavFeasibility> own_nav;
  const NavFeasibility * nav = opt.nav;
  if (opt.check_navigation && start && !nav && !cands0.empty()) {
    V2 lo = start->arm_xy, hi = start->arm_xy;
    for (const auto & c : cands0) {
      const V2 p = base ? baseCentre(*base, c.stand) : c.stand.arm_xy;
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
    const V2 sc = base ? baseCentre(*base, *start) : start->arm_xy;
    lo = lo.cwiseMin(sc);
    hi = hi.cwiseMax(sc);
    const V2 pad(opt.nav_padding_m, opt.nav_padding_m);
    own_nav = std::make_unique<GridNavFeasibility>(nav_fs, base, lo - pad, hi + pad, opt.nav_grid);
    nav = own_nav.get();
  }
  if (!start || !opt.check_navigation) {nav = nullptr;}

  std::vector<CandidateStand> survivors;
  std::vector<CandidateStand> rejected_nav, rejected_yaw;
  for (CandidateStand c : cands0) {
    if (use_window) {
      bool found = false;
      double best_d = std::numeric_limits<double>::infinity(), best_yaw = c.stand.yaw;
      for (int k = 0; k < steps; ++k) {
        const double y = wrapPi(facing + 2.0 * kPi * k / steps);
        const double d = std::fabs(wrapPi(y - facing));
        if (d > window + 1e-9) {continue;}
        const Stand st{c.stand.arm_xy, y};
        if (planeClearance(*base, st, fs_in, frame, frame.origin) < station.min_clearance - 1e-9) {continue;}
        if (!fs_in.footprintFree(st, *base, station.free_space_margin)) {continue;}
        if (d < best_d - 1e-9) {best_d = d; best_yaw = y; found = true;}
      }
      if (!found) {rejected_yaw.push_back(c); continue;}
      c.stand.yaw = best_yaw;
    }
    if (nav) {
      const NavVerdict v = nav->feasible(*start, c.stand);
      if (!v.ok) {
        rejected_nav.push_back(c);
        out.rejected.push_back({c.stand, BpoReason::kUnreachableByNav, v.reason});
        continue;
      }
    }
    survivors.push_back(c);
  }
  for (const auto & c : rejected_yaw) {out.rejected.push_back({c.stand, BpoReason::kYawWindow, NavReason::kOk});}
  out.nav_rejected = rejected_nav.size();
  out.yaw_rejected = rejected_yaw.size();

  // ---- sequences and per-(candidate, column) evaluation of the survivors
  SequenceParams sp = opt.sequence;
  sp.wall_normal = frame.n;
  std::vector<ColumnPoseSequence> seqs(nc);
  for (size_t k = 0; k < nc; ++k) {seqs[k] = buildColumnSequence(columns[k].strokes_world, sp);}
  const double mount = station.arm_mount_offset_z;

  std::vector<std::vector<Eval>> ev(survivors.size(), std::vector<Eval>(nc));
  std::vector<Eval> best_surv(nc);
  std::vector<bool> have_surv(nc, false);
  std::vector<std::vector<bool>> reach(nc);
  for (size_t k = 0; k < nc; ++k) {reach[k].assign(columns[k].strokes_world.size(), false);}
  for (size_t c = 0; c < survivors.size(); ++c) {
    for (size_t k = 0; k < nc; ++k) {
      ev[c][k] = evalAt(map, seqs[k], survivors[c].stand.arm_xy, heights, mount, opt.margin);
      if (!have_surv[k] || betterEval(ev[c][k], best_surv[k])) {best_surv[k] = ev[c][k]; have_surv[k] = true;}
      if (ev[c][k].ends_usable) {for (const size_t s : ev[c][k].strokes) {reach[k][s] = true;}}
    }
  }
  for (size_t k = 0; k < nc; ++k) {
    for (const bool b : reach[k]) {out.reachable_strokes += b ? 1 : 0;}
  }
  out.reachable_fraction = static_cast<double>(out.reachable_strokes) / static_cast<double>(out.strokes_total);

  // Exact whole-cover bound over the survivors.
  {
    std::vector<size_t> coverable;
    for (size_t k = 0; k < nc; ++k) {
      if (columns[k].strokes_world.empty()) {continue;}
      for (size_t c = 0; c < survivors.size(); ++c) {
        if (ev[c][k].status == ColumnStatus::kWhole) {coverable.push_back(k); break;}
      }
    }
    out.whole_coverable = coverable.size();
    if (coverable.size() <= std::min<size_t>(opt.exact_max_columns, 62)) {
      std::vector<uint64_t> masks;
      for (size_t c = 0; c < survivors.size(); ++c) {
        uint64_t m = 0;
        for (size_t i = 0; i < coverable.size(); ++i) {
          if (ev[c][coverable[i]].status == ColumnStatus::kWhole) {m |= (uint64_t{1} << i);}
        }
        masks.push_back(m);
      }
      const uint64_t all = coverable.empty() ? 0 : (coverable.size() == 64 ? ~uint64_t{0} : ((uint64_t{1} << coverable.size()) - 1));
      out.exact_min_stands = exactCover(masks, all, opt.exact_node_budget);
    }
  }

  // ---- plan, with the verifier loop
  ColumnStandOptions co;
  co.margin = opt.margin;
  co.sequence = sp;
  co.margin_penalty = opt.margin_penalty;
  co.allow_splits = opt.allow_splits;
  co.max_stands = opt.max_stands;
  co.verifier = opt.verifier;
  co.arm = opt.arm;

  // The survivors of the yaw and navigation filters go straight to the planner; the caller's avoid
  // list (and radius) was already applied when the candidates were made.
  std::vector<CandidateStand> pool = survivors;
  co.candidates = &pool;

  ColumnStandPlan best;
  bool have_plan = false;
  const size_t rounds = opt.verifier ? std::max<size_t>(opt.verifier_rounds, 1) : 1;
  for (size_t r = 0; r < rounds; ++r) {
    ColumnStandPlan plan = planColumnStands(map, station, columns, start, co);
    out.rounds = r + 1;
    const bool better = !have_plan || plan.strokes_covered > best.strokes_covered ||
      (plan.strokes_covered == best.strokes_covered && plan.whole > best.whole) ||
      (plan.strokes_covered == best.strokes_covered && plan.whole == best.whole && plan.stands.size() < best.stands.size());
    const bool refused = !plan.refused.empty();
    if (better) {best = std::move(plan); have_plan = true;}
    if (!refused) {break;}
    const ColumnStandPlan & last = better ? best : plan;
    bool added = false;
    for (const auto & rf : last.refused) {
      const size_t before = pool.size();
      pool.erase(std::remove_if(pool.begin(), pool.end(), [&](const CandidateStand & c) {
          return (c.stand.arm_xy - rf.stand.arm_xy).norm() < 1e-9;
        }), pool.end());
      if (pool.size() != before) {
        out.refused_stands.push_back(rf.stand);
        added = true;
      }
    }
    if (!added) {break;}
  }
  out.verified = best.verified;
  out.stands_if_splitting_allowed = best.stands_if_splitting_allowed;
  if (!have_plan) {return out;}

  // ---- stands: yaw inside the window, travel, clearance
  std::optional<Stand> at = start;
  for (size_t i = 0; i < best.stands.size(); ++i) {
    const auto & ps = best.stands[i];
    BpoStand bs;
    bs.stand = ps.stand;
    for (const auto & c : survivors) {
      if ((c.stand.arm_xy - ps.stand.arm_xy).norm() < 1e-6) {bs.stand.yaw = c.stand.yaw; break;}
    }
    bs.standoff = ps.standoff;
    bs.columns = ps.columns;
    if (at) {
      bs.travel_cost = legCost(base, *at, bs.stand);
      const V2 a = base ? baseCentre(*base, *at) : at->arm_xy;
      const V2 b = base ? baseCentre(*base, bs.stand) : bs.stand.arm_xy;
      bs.travel_m = (b - a).norm();
    }
    bs.clearance_m = clearanceOf(nav_fs, base, bs.stand);
    out.travel_cost += bs.travel_cost;
    out.travel_m += bs.travel_m;
    out.min_clearance_m = i == 0 ? bs.clearance_m : std::min(out.min_clearance_m, bs.clearance_m);
    at = bs.stand;
    out.stands.push_back(std::move(bs));
  }
  out.stand_count = out.stands.size();

  // ---- per-column status, reason, escalation
  for (size_t k = 0; k < nc; ++k) {
    BpoColumn & bc = out.columns[k];
    const ColumnResult & cr = best.columns[k];
    bc.status = cr.status;
    bc.strokes_covered = cr.strokes_covered;
    bc.pieces = cr.pieces;
    bc.limited = cr.limited;
    bc.limiting_role = cr.limiting_role;
    bc.limiting_stroke = cr.limiting_stroke;
    bc.limiting_failure = cr.limiting_failure;
    bc.limiting_score = cr.limiting_score;
    out.strokes_covered += cr.strokes_covered;
    if (cr.status == ColumnStatus::kWhole) {++out.whole;}
    else if (cr.status == ColumnStatus::kSplit) {++out.split;}
    else {++out.unreachable;}
    if (columns[k].strokes_world.empty()) {bc.reason = BpoReason::kOk; bc.escalation = BpoEscalation::kNone; continue;}

    // Escalation follows the final status: a whole column reports the cheapest step at which it is whole;
    // any other column reports split / skipped / unreachable, never 'none' (the reason says why).
    bool whole_exists = false;
    for (size_t c = 0; c < survivors.size() && !whole_exists; ++c) {whole_exists = ev[c][k].status == ColumnStatus::kWhole;}
    if (cr.status == ColumnStatus::kWhole && whole_exists) {
      double mz = 0.0;
      V2 mxy(0.0, 0.0);
      for (const auto & p : columns[k].strokes_world) {
        mz += p.translation().z();
        mxy += V2(p.translation().x(), p.translation().y());
      }
      const double n = static_cast<double>(columns[k].strokes_world.size());
      mz /= n;
      mxy /= n;
      const double d0 = frame.horizontal ? 0.0 : 0.5 * (station.min_clearance + station.max_standoff);
      size_t direct = 0;
      double best_d = std::numeric_limits<double>::infinity();
      for (size_t c = 0; c < survivors.size(); ++c) {
        const double d = (survivors[c].stand.arm_xy - (mxy + V2(frame.n.x(), frame.n.y()) * d0)).norm();
        if (d < best_d - 1e-12) {best_d = d; direct = c;}
      }
      double nominal = heights.front();
      for (const double h : heights) {
        if (std::fabs(h - (mz - mount)) < std::fabs(nominal - (mz - mount)) - 1e-12) {nominal = h;}
      }
      const V2 dxy = survivors[direct].stand.arm_xy;
      if (evalAt(map, seqs[k], dxy, {nominal}, mount, opt.margin).status == ColumnStatus::kWhole) {
        bc.escalation = BpoEscalation::kNone;
      } else if (ev[direct][k].status == ColumnStatus::kWhole) {
        bc.escalation = BpoEscalation::kLiftHeight;
      } else {
        bc.escalation = BpoEscalation::kOtherStand;
      }
    } else if (cr.skipped_to_avoid_split) {
      bc.escalation = BpoEscalation::kSkipped;
    } else if (cr.status == ColumnStatus::kSplit) {
      bc.escalation = BpoEscalation::kSplit;
    } else {
      bc.escalation = BpoEscalation::kUnreachable;
    }

    if (cr.status == ColumnStatus::kWhole) {bc.reason = BpoReason::kOk; continue;}

    // Would a candidate the filters removed have done better?
    Eval best_rej;
    bool rej_nav = false, have_rej = false;
    auto scan = [&](const std::vector<CandidateStand> & set, bool is_nav) {
        for (const auto & c : set) {
          const Eval e = evalAt(map, seqs[k], c.stand.arm_xy, heights, mount, opt.margin);
          if (!have_rej || betterEval(e, best_rej)) {best_rej = e; rej_nav = is_nav; have_rej = true;}
        }
      };
    scan(rejected_nav, true);
    scan(rejected_yaw, false);
    const bool rescued = have_rej && (!have_surv[k] || betterEval(best_rej, best_surv[k])) &&
      (best_rej.status != ColumnStatus::kUnreachable || best_rej.usable > 0);
    if (survivors.empty() && !rescued) {bc.reason = BpoReason::kNoFreeStand; continue;}
    if (rescued) {bc.reason = rej_nav ? BpoReason::kUnreachableByNav : BpoReason::kYawWindow; continue;}
    if (whole_exists && best.stands.size() >= opt.max_stands) {bc.reason = BpoReason::kMaxStands; continue;}
    if (cr.skipped_to_avoid_split) {bc.reason = BpoReason::kSkippedToAvoidSplit; continue;}
    if (cr.status == ColumnStatus::kSplit && bc.limiting_failure == PoseFailure::kNone) {
      bc.reason = BpoReason::kSplitNoWholeStand;
      continue;
    }
    switch (bc.limiting_failure) {
      case PoseFailure::kLowMargin: bc.reason = BpoReason::kLowMargin; break;
      case PoseFailure::kRefused: bc.reason = BpoReason::kRefusedByVerifier; break;
      case PoseFailure::kSingular: bc.reason = BpoReason::kNearSingular; break;
      case PoseFailure::kJointLimit: bc.reason = BpoReason::kNearJointLimit; break;
      case PoseFailure::kOutOfReach: bc.reason = BpoReason::kOutOfReach; break;
      case PoseFailure::kNone:
        bc.reason = cr.status == ColumnStatus::kSplit ? BpoReason::kSplitNoWholeStand : BpoReason::kOutOfReach;
        break;
    }
    if (cr.status == ColumnStatus::kSplit && bc.limiting_failure != PoseFailure::kRefused &&
      bc.limiting_failure != PoseFailure::kSingular && bc.limiting_failure != PoseFailure::kJointLimit)
    {
      bc.reason = BpoReason::kSplitNoWholeStand;
    }
  }
  out.split_count = out.split;
  out.covered_fraction = static_cast<double>(out.strokes_covered) / static_cast<double>(out.strokes_total);
  out.success = out.whole == nc;
  out.message = out.success ? "every column whole" :
    (std::to_string(out.split) + " split, " + std::to_string(out.unreachable) + " unreachable");
  return out;
}

}  // namespace reachability
