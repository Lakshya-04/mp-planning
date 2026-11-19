#include "reachability/core/horizontal_stands.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace reachability
{

bool heightWithinLiftReach(
  const Map4D & map, double z, double lift_min, double lift_max, double arm_mount_offset_z)
{
  const double lo = lift_min + arm_mount_offset_z + map.params().z_min;
  // Map4D rounds the window up to whole cells: the top cell ends at z_min + nz * resolution.
  const double hi = lift_max + arm_mount_offset_z + map.params().z_min + map.nz() * map.params().resolution;
  return z >= lo - 1e-9 && z <= hi + 1e-9;
}

namespace
{
// Where the base ends up when it drives to `xy` and takes whatever yaw it arrives with. A strafing
// base keeps its yaw; the others arrive along their heading, forward or reversing, whichever is
// quicker (the stand's own yaw is free, so no turn to a fixed final yaw belongs in the leg).
Stand arrivalStand(const BaseModel & base, const Stand & from, const Eigen::Vector2d & xy)
{
  Stand best{xy, from.yaw};
  if (base.can_strafe) {return best;}
  const Eigen::Vector2d d = xy - from.arm_xy;
  if (d.norm() < 1e-3) {return best;}
  const double heading = std::atan2(d.y(), d.x());
  double best_t = travelTime(base, from, best);
  for (const double h : {heading, heading + M_PI}) {
    const Stand s{xy, std::atan2(std::sin(h), std::cos(h))};
    const double t = travelTime(base, from, s);
    if (t < best_t) {best_t = t; best = s;}
  }
  return best;
}
}  // namespace

std::string horizontalRequestProblem(const Map4D & map, const HorizontalStationRequest & q)
{
  (void)map;
  if (!std::isfinite(q.grid_step) || q.grid_step <= 0.0) {return "grid_step must be finite and positive";}
  if (!std::isfinite(q.margin)) {return "margin must be finite";}
  if (!std::isfinite(q.lift_min) || !std::isfinite(q.lift_max) || !std::isfinite(q.lift_step) ||
    q.lift_step <= 0.0)
  {
    return "lift_min and lift_max must be finite and lift_step positive";
  }
  if (q.lift_max < q.lift_min) {return "lift_max is below lift_min";}
  if (heightRange(q.lift_min, q.lift_max, q.lift_step).empty()) {return "too many lift heights: raise lift_step";}
  if (q.targets_world.empty()) {return "";}
  double x_lo = 1e18, x_hi = -1e18, y_lo = 1e18, y_hi = -1e18;
  for (const auto & t : q.targets_world) {
    x_lo = std::min(x_lo, t.translation().x());
    x_hi = std::max(x_hi, t.translation().x());
    y_lo = std::min(y_lo, t.translation().y());
    y_hi = std::max(y_hi, t.translation().y());
  }
  if (!std::isfinite(x_lo + x_hi + y_lo + y_hi)) {return "a target position is not finite";}
  const double reach = std::max({0.0, q.margin, map.params().xy_extent});
  const double nx = std::floor((x_hi - x_lo + 2.0 * reach) / q.grid_step) + 1.0;
  const double ny = std::floor((y_hi - y_lo + 2.0 * reach) / q.grid_step) + 1.0;
  if (!(nx * ny <= static_cast<double>(kMaxGridPoints))) {
    return "the search grid would have more than " + std::to_string(kMaxGridPoints) + " points: raise grid_step";
  }
  return "";
}

Station chooseHorizontalStation(const Map4D & map, const HorizontalStationRequest & q)
{
  Station best;
  if (q.targets_world.empty() || !horizontalRequestProblem(map, q).empty()) {
    return best;
  }
  double x_lo = 1e18, x_hi = -1e18, y_lo = 1e18, y_hi = -1e18;
  for (const auto & t : q.targets_world) {
    x_lo = std::min(x_lo, t.translation().x());
    x_hi = std::max(x_hi, t.translation().x());
    y_lo = std::min(y_lo, t.translation().y());
    y_hi = std::max(y_hi, t.translation().y());
  }
  const double mx = 0.5 * (x_lo + x_hi), my = 0.5 * (y_lo + y_hi);
  // a station reaches a ring out to the map's window, so a lone target needs stations that far away
  const double margin = std::max({0.0, q.margin, map.params().xy_extent});
  x_lo -= margin; x_hi += margin; y_lo -= margin; y_hi += margin;
  const auto heights = heightRange(q.lift_min, q.lift_max, q.lift_step);

  // Fewer unreachable targets, then fewer bands, then the base's travel (or the distance to the
  // middle of the patch). The scan order is fixed, so the first of equal stations wins.
  using Key = std::tuple<size_t, size_t, double>;
  Key best_key{~size_t{0}, ~size_t{0}, 0.0};
  const int nx = static_cast<int>(std::floor((x_hi - x_lo) / q.grid_step + 1e-9));
  const int ny = static_cast<int>(std::floor((y_hi - y_lo) / q.grid_step + 1e-9));
  for (int i = 0; i <= nx; ++i) {
    for (int j = 0; j <= ny; ++j) {
      const double x = x_lo + i * q.grid_step, y = y_lo + j * q.grid_step;
      const auto plan = planLiftBands(map, q.targets_world, x, y, heights, q.arm_mount_offset_z);
      ++best.evaluated;
      if (plan.reachable_count == 0) {continue;}
      double tie = std::hypot(x - mx, y - my);
      if (q.base && q.from) {
        const Stand to = arrivalStand(*q.base, *q.from, Eigen::Vector2d(x, y));
        tie = travelTime(*q.base, *q.from, to) + 1e-3 * (to.arm_xy - q.from->arm_xy).norm();
      }
      const Key key{plan.unreachable.size(), plan.bands.size(), tie};
      if (!best.found || key < best_key) {
        best_key = key;
        best.found = true;
        best.x = x;
        best.y = y;
        best.plan = plan;
      }
    }
  }
  return best;
}

HorizontalStandPlan planHorizontalStands(
  const Map4D & map, const HorizontalStationRequest & request, size_t max_stands)
{
  HorizontalStandPlan out;
  StandPlan & plan = out.plan;
  if (const std::string problem = horizontalRequestProblem(map, request); !problem.empty()) {
    out.code = HorizontalStandCode::kInvalidRequest;
    out.message = problem;
    for (size_t i = 0; i < request.targets_world.size(); ++i) {plan.not_planned.push_back(i);}
    return out;
  }
  std::vector<size_t> left;
  for (size_t i = 0; i < request.targets_world.size(); ++i) {
    const double z = request.targets_world[i].translation().z();
    if (heightWithinLiftReach(map, z, request.lift_min, request.lift_max, request.arm_mount_offset_z)) {
      left.push_back(i);
    } else {
      out.out_of_lift_range.push_back(i);
    }
  }
  std::optional<Stand> at = request.base ? request.from : std::nullopt;

  while (!left.empty() && plan.stands.size() < max_stands) {
    HorizontalStationRequest r = request;
    r.targets_world.clear();
    for (const size_t i : left) {r.targets_world.push_back(request.targets_world[i]);}
    r.from = at;
    const Station st = chooseHorizontalStation(map, r);
    if (!st.found) {break;}

    StandAssignment a;
    a.station = st;
    for (const auto & band : st.plan.bands) {
      for (const size_t k : band.pose_indices) {a.targets.push_back(left[k]);}
    }
    if (a.targets.empty()) {break;}
    std::sort(a.targets.begin(), a.targets.end());
    // the station numbered targets within the remaining set; make every index global
    for (auto & band : a.station.plan.bands) {
      for (auto & k : band.pose_indices) {k = left[k];}
    }
    for (auto & k : a.station.plan.unreachable) {k = left[k];}
    a.stand = request.base && at ? arrivalStand(*request.base, *at, Eigen::Vector2d(st.x, st.y)) :
      Stand{Eigen::Vector2d(st.x, st.y), 0.0};
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
  plan.unreachable.insert(
    plan.unreachable.end(), out.out_of_lift_range.begin(), out.out_of_lift_range.end());
  std::sort(plan.unreachable.begin(), plan.unreachable.end());
  return out;
}

}  // namespace reachability
