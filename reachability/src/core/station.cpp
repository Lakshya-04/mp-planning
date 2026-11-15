#include "reachability/core/station.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace reachability
{

Station chooseStation(const Map4D & map, const StationRequest & q)
{
  Station best;
  if (q.targets_world.empty() || q.grid_step <= 0.0 || q.wall_normal.norm() < 1e-9) {
    return best;
  }
  const Vec3 n = q.wall_normal.normalized();
  // In-plane search frame: u along the wall's horizontal direction, away from the wall along n.
  Vec3 u = Vec3::UnitZ().cross(n);
  if (u.norm() < 1e-6) {
    return best;  // a horizontal wall plane has no sensible base position on it
  }
  u.normalize();

  double u_lo = 1e18, u_hi = -1e18;
  for (const auto & t : q.targets_world) {
    const double a = (t.translation() - q.wall_point).dot(u);
    u_lo = std::min(u_lo, a);
    u_hi = std::max(u_hi, a);
  }
  u_lo -= q.lateral_margin;
  u_hi += q.lateral_margin;
  const Vec3 on_wall = q.wall_point;
  const auto heights = heightRange(q.lift_min, q.lift_max, q.lift_step);
  const double u_mid = 0.5 * (u_lo + u_hi);

  // Rank: fewer unreachable targets, then fewer bands, then a base near the middle of the work
  // area, then the smaller standoff.
  using Key = std::tuple<size_t, size_t, double, double>;
  Key best_key{~size_t{0}, ~size_t{0}, 0.0, 0.0};

  for (double d = q.min_clearance; d <= q.max_standoff + 1e-9; d += q.grid_step) {
    for (double a = u_lo; a <= u_hi + 1e-9; a += q.grid_step) {
      const Vec3 p = on_wall + u * a + n * d;
      double yaw = 0.0, clearance = 0.0;
      if (q.base) {
        yaw = bestYaw(*q.base, Eigen::Vector2d(p.x(), p.y()), Eigen::Vector2d(on_wall.x(), on_wall.y()),
          Eigen::Vector2d(n.x(), n.y()), &clearance);
        if (clearance < q.min_clearance - 1e-9) {continue;}   // the footprint would not fit here
      }
      const auto plan = planLiftBands(map, q.targets_world, p.x(), p.y(), heights, q.arm_mount_offset_z);
      ++best.evaluated;
      if (plan.reachable_count == 0) {
        continue;
      }
      // A strafing base can have every nearby station cost the same (the final turn dominates),
      // so the distance breaks that tie.
      double tie = std::abs(a - u_mid);
      if (q.base && q.from) {
        const Stand to{Eigen::Vector2d(p.x(), p.y()), yaw};
        tie = travelTime(*q.base, *q.from, to) + 1e-3 * (to.arm_xy - q.from->arm_xy).norm();
      }
      const Key key{plan.unreachable.size(), plan.bands.size(), tie, d};
      if (!best.found || key < best_key) {
        best_key = key;
        best.found = true;
        best.x = p.x();
        best.y = p.y();
        best.standoff = d;
        best.yaw = yaw;
        best.footprint_clearance = clearance;
        best.plan = plan;
      }
    }
  }
  return best;
}

}  // namespace reachability
