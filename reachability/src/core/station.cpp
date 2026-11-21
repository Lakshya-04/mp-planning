#include "reachability/core/station.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace reachability
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kYawSteps = 24;

double wrapAngle(double a)
{
  while (a > kPi) {a -= 2.0 * kPi;}
  while (a < -kPi) {a += 2.0 * kPi;}
  return a;
}

/// Work-plane clearance of a footprint; with a finite work span only the points inside it count.
double planeClearance(const StationRequest & q, const SurfaceFrame & f, const Stand & st)
{
  const Eigen::Vector2d n(f.n.x(), f.n.y());
  const Eigen::Vector2d wp(q.wall_point.x(), q.wall_point.y());
  if (!q.free_space || !q.free_space->has_work_span) {return wallClearance(*q.base, st, wp, n);}
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : footprintWorld(*q.base, st)) {
    const double a = (Eigen::Vector3d(p.x(), p.y(), q.wall_point.z()) - q.wall_point).dot(f.u);
    if (a < q.free_space->work_span_lo - 1e-9 || a > q.free_space->work_span_hi + 1e-9) {continue;}
    best = std::min(best, (p - wp).dot(n));
  }
  return best;
}

/// In-plane grid axes of the stand search. A wall uses the frame's u and v. A horizontal plane (level or
/// tilted within the tolerance) is gridded in the floor's x-y plane along the frame's axes projected onto it,
/// so a ceiling off level by a fraction of a degree has exactly the grid of a level one.
struct GridAxes
{
  Eigen::Vector3d u, v;
};

GridAxes gridAxes(const SurfaceFrame & f)
{
  if (!f.horizontal) {return {f.u, f.v};}
  const Eigen::Vector3d u = Eigen::Vector3d(f.u.x(), f.u.y(), 0.0).normalized();
  const Eigen::Vector3d v = Eigen::Vector3d(f.v.x(), f.v.y(), 0.0).normalized();
  return {u, v};
}

enum class YawResult { kOk, kPlaneRejected, kFreeSpaceRejected };

/// Picks the base yaw at an arm base position. Without a free space this is exactly bestYaw (vertical
/// plane) or yaw 0 (horizontal plane). With one, every yaw sample is tested and the best feasible
/// one wins (most work-plane clearance, ties to the first sample).
YawResult pickYaw(const StationRequest & q, const SurfaceFrame & f, const Eigen::Vector2d & xy, double * yaw, double * clearance)
{
  *yaw = 0.0;
  *clearance = 0.0;
  if (!q.base && !q.free_space) {return YawResult::kOk;}
  const bool use_fs = q.free_space && !q.free_space->empty();
  if (!q.base) {
    return q.free_space->pointFree(xy, q.free_space_margin) ? YawResult::kOk : YawResult::kFreeSpaceRejected;
  }
  if (!use_fs) {
    if (f.horizontal) {return YawResult::kOk;}
    *yaw = bestYaw(*q.base, xy, Eigen::Vector2d(q.wall_point.x(), q.wall_point.y()),
        Eigen::Vector2d(f.n.x(), f.n.y()), clearance);
    return *clearance < q.min_clearance - 1e-9 ? YawResult::kPlaneRejected : YawResult::kOk;
  }
  const double facing = f.horizontal ? 0.0 : std::atan2(-f.n.y(), -f.n.x());
  bool any_plane_ok = false, found = false;
  double best_c = -std::numeric_limits<double>::infinity(), best_yaw = facing;
  for (int k = 0; k < kYawSteps; ++k) {
    const double y = wrapAngle(facing + 2.0 * kPi * k / kYawSteps);
    const Stand st{xy, y};
    double c = 0.0;
    if (!f.horizontal) {
      c = planeClearance(q, f, st);
      if (c < q.min_clearance - 1e-9) {continue;}
    }
    any_plane_ok = true;
    if (!q.free_space->footprintFree(st, *q.base, q.free_space_margin)) {continue;}
    if (!found || c > best_c + 1e-9) {found = true; best_c = c; best_yaw = y;}
  }
  if (!found) {return any_plane_ok ? YawResult::kFreeSpaceRejected : YawResult::kPlaneRejected;}
  *yaw = best_yaw;
  *clearance = std::isfinite(best_c) ? best_c : 0.0;
  return YawResult::kOk;
}
}  // namespace

const char * toString(StationCode code)
{
  switch (code) {
    case StationCode::kOk: return "ok";
    case StationCode::kInvalidRequest: return "invalid_request";
    case StationCode::kNoFreeStand: return "no_free_stand";
    case StationCode::kNoReachableStand: return "no_reachable_stand";
  }
  return "unknown";
}

std::string stationGridProblem(const StationRequest & q)
{
  if (!std::isfinite(q.grid_step) || q.grid_step <= 0.0) {return "grid_step must be finite and positive";}
  if (!std::isfinite(q.min_clearance) || !std::isfinite(q.max_standoff) || !std::isfinite(q.lateral_margin)) {
    return "min_clearance, max_standoff and lateral_margin must be finite";
  }
  if (!surfaceFrame(q.wall_normal).ok || q.targets_world.empty()) {return "";}   // reported elsewhere
  const double points = stationGridPointCount(q);
  if (!std::isfinite(points)) {return "a target position is not finite";}
  if (!(points <= static_cast<double>(kMaxGridPoints))) {
    return "the search grid would have more than " + std::to_string(kMaxGridPoints) +
           " points: raise grid_step or lower max_standoff / lateral_margin";
  }
  return "";
}

double stationGridPointCount(const StationRequest & q)
{
  const SurfaceFrame f = surfaceFrame(q.wall_normal);
  if (!f.ok || q.targets_world.empty() || !(q.grid_step > 0.0)) {return 0.0;}
  const GridAxes ax = gridAxes(f);
  double lo = 1e18, hi = -1e18, vlo = 1e18, vhi = -1e18;
  for (const auto & t : q.targets_world) {
    const double a = (t.translation() - q.wall_point).dot(ax.u);
    const double b = (t.translation() - q.wall_point).dot(ax.v);
    lo = std::min(lo, a);
    hi = std::max(hi, a);
    vlo = std::min(vlo, b);
    vhi = std::max(vhi, b);
  }
  if (!std::isfinite(lo) || !std::isfinite(hi) || !std::isfinite(vlo) || !std::isfinite(vhi)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double along = std::floor(std::max(0.0, hi - lo + 2.0 * q.lateral_margin) / q.grid_step) + 1.0;
  const double rings = f.horizontal ?
    std::floor(std::max(0.0, vhi - vlo + 2.0 * q.lateral_margin) / q.grid_step) + 1.0 :
    std::floor(std::max(0.0, q.max_standoff - q.min_clearance) / q.grid_step) + 1.0;
  return rings * along;
}

std::vector<StationGridPoint> stationGrid(const StationRequest & q, StationGridStats * stats)
{
  std::vector<StationGridPoint> out;
  StationGridStats local;
  StationGridStats & st = stats ? *stats : local;
  st = StationGridStats{};
  const SurfaceFrame f = surfaceFrame(q.wall_normal);
  if (q.targets_world.empty() || !f.ok || !(q.grid_step > 0.0)) {return out;}
  const GridAxes ax = gridAxes(f);
  double u_lo = 1e18, u_hi = -1e18, v_lo = 1e18, v_hi = -1e18;
  for (const auto & t : q.targets_world) {
    const double a = (t.translation() - q.wall_point).dot(ax.u);
    u_lo = std::min(u_lo, a);
    u_hi = std::max(u_hi, a);
    const double b = (t.translation() - q.wall_point).dot(ax.v);
    v_lo = std::min(v_lo, b);
    v_hi = std::max(v_hi, b);
  }
  u_lo -= q.lateral_margin;
  u_hi += q.lateral_margin;
  v_lo -= q.lateral_margin;
  v_hi += q.lateral_margin;

  auto visit = [&](const Vec3 & p, double a, double d) {
      ++st.visited;
      if (stationAvoided(q, Eigen::Vector2d(p.x(), p.y()))) {++st.avoided; return;}
      StationGridPoint g;
      g.arm_xy = Eigen::Vector2d(p.x(), p.y());
      g.standoff = d;
      g.along = a;
      switch (pickYaw(q, f, g.arm_xy, &g.yaw, &g.clearance)) {
        case YawResult::kPlaneRejected: ++st.plane_rejected; return;
        case YawResult::kFreeSpaceRejected: ++st.free_space_rejected; return;
        case YawResult::kOk: break;
      }
      ++st.kept;
      out.push_back(g);
    };

  if (f.horizontal) {
    for (double b = v_lo; b <= v_hi + 1e-9; b += q.grid_step) {
      for (double a = u_lo; a <= u_hi + 1e-9; a += q.grid_step) {
        visit(q.wall_point + ax.u * a + ax.v * b, a, 0.0);
      }
    }
  } else {
    for (double d = q.min_clearance; d <= q.max_standoff + 1e-9; d += q.grid_step) {
      for (double a = u_lo; a <= u_hi + 1e-9; a += q.grid_step) {
        visit(q.wall_point + ax.u * a + f.n * d, a, d);
      }
    }
  }
  return out;
}

Station chooseStation(const Map4D & map, const StationRequest & q)
{
  Station best;
  if (q.targets_world.empty() || !surfaceFrame(q.wall_normal).ok || !stationGridProblem(q).empty()) {
    return best;   // code stays kInvalidRequest
  }
  const auto heights = heightRange(q.lift_min, q.lift_max, q.lift_step);
  std::vector<StationGridPoint> grid = stationGrid(q);
  if (grid.empty()) {
    best.code = StationCode::kNoFreeStand;
    return best;
  }
  best.code = StationCode::kNoReachableStand;
  const SurfaceFrame f = surfaceFrame(q.wall_normal);
  const GridAxes ax = gridAxes(f);
  double u_lo = 1e18, u_hi = -1e18, v_lo = 1e18, v_hi = -1e18;
  for (const auto & t : q.targets_world) {
    const double a = (t.translation() - q.wall_point).dot(ax.u);
    u_lo = std::min(u_lo, a);
    u_hi = std::max(u_hi, a);
    const double b = (t.translation() - q.wall_point).dot(ax.v);
    v_lo = std::min(v_lo, b);
    v_hi = std::max(v_hi, b);
  }
  u_lo -= q.lateral_margin;
  u_hi += q.lateral_margin;
  const double u_mid = 0.5 * (u_lo + u_hi);
  const double v_mid = 0.5 * (v_lo + v_hi);
  const Vec3 centre = q.wall_point + ax.u * u_mid + ax.v * v_mid;

  // Rank: fewer unreachable targets, then fewer bands, then a base near the middle of the work
  // area, then the smaller standoff.
  using Key = std::tuple<size_t, size_t, double, double>;
  Key best_key{~size_t{0}, ~size_t{0}, 0.0, 0.0};

  for (const auto & g : grid) {
    const auto plan = planLiftBands(map, q.targets_world, g.arm_xy.x(), g.arm_xy.y(), heights, q.arm_mount_offset_z);
    ++best.evaluated;
    if (plan.reachable_count == 0) {
      continue;
    }
    // A strafing base can have every nearby station cost the same (the final turn dominates),
    // so the distance breaks that tie.
    double tie = f.horizontal ? (g.arm_xy - Eigen::Vector2d(centre.x(), centre.y())).norm() : std::abs(g.along - u_mid);
    if (q.base && q.from) {
      const Stand to{g.arm_xy, g.yaw};
      tie = travelTime(*q.base, *q.from, to) + 1e-3 * (to.arm_xy - q.from->arm_xy).norm();
    }
    const Key key{plan.unreachable.size(), plan.bands.size(), tie, g.standoff};
    if (!best.found || key < best_key) {
      best_key = key;
      best.found = true;
      best.x = g.arm_xy.x();
      best.y = g.arm_xy.y();
      best.standoff = g.standoff;
      best.yaw = g.yaw;
      best.footprint_clearance = g.clearance;
      best.plan = plan;
    }
  }
  if (best.found) {best.code = StationCode::kOk;}
  return best;
}

}  // namespace reachability
