#include "reachability/core/lift_bands.hpp"

#include <algorithm>
#include <cmath>

namespace reachability
{

std::vector<double> heightRange(double lo, double hi, double step)
{
  std::vector<double> out;
  // Non-finite input or a huge count would loop (or overflow the int below): no heights.
  constexpr double kMaxHeights = 100000.0;
  if (!std::isfinite(lo) || !std::isfinite(hi) || !std::isfinite(step) || step <= 0.0 || hi < lo ||
    !((hi - lo) / step < kMaxHeights))
  {
    return out;
  }
  const int n = static_cast<int>(std::floor((hi - lo) / step + 1e-9));
  for (int i = 0; i <= n; ++i) {
    out.push_back(lo + i * step);
  }
  if (out.back() < hi - 1e-9) {
    out.push_back(hi);
  }
  return out;
}

std::vector<std::vector<bool>> reachableByHeight(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & heights, double arm_mount_offset_z)
{
  std::vector<std::vector<bool>> out(targets_world.size(), std::vector<bool>(heights.size(), false));
  for (size_t i = 0; i < targets_world.size(); ++i) {
    for (size_t h = 0; h < heights.size(); ++h) {
      // Base yaw is free (the arm's first joint absorbs it), so a base at yaw 0 is enough.
      Pose in_base = targets_world[i];
      in_base.translation() -= Vec3(base_x, base_y, heights[h] + arm_mount_offset_z);
      out[i][h] = map.isReachable(in_base);
    }
  }
  return out;
}

LiftBandPlan planLiftBands(
  const Map4D & map, const std::vector<Pose> & targets_world, double base_x, double base_y,
  const std::vector<double> & candidate_heights, double arm_mount_offset_z)
{
  LiftBandPlan plan;
  const auto reach = reachableByHeight(map, targets_world, base_x, base_y, candidate_heights, arm_mount_offset_z);

  std::vector<bool> covered(targets_world.size(), false);
  for (size_t i = 0; i < targets_world.size(); ++i) {
    bool any = false;
    for (bool r : reach[i]) {any = any || r;}
    if (!any) {
      plan.unreachable.push_back(i);
      covered[i] = true;  // nothing can cover it: leave it out of the cover problem
    }
  }
  plan.reachable_count = targets_world.size() - plan.unreachable.size();

  while (true) {
    size_t best_h = 0, best_gain = 0;
    for (size_t h = 0; h < candidate_heights.size(); ++h) {
      size_t gain = 0;
      for (size_t i = 0; i < targets_world.size(); ++i) {
        if (!covered[i] && reach[i][h]) {++gain;}
      }
      // Ties go to the lower height: the slow axis travels less.
      if (gain > best_gain) {
        best_gain = gain;
        best_h = h;
      }
    }
    if (best_gain == 0) {
      break;
    }
    LiftBand band;
    band.lift_height = candidate_heights[best_h];
    for (size_t i = 0; i < targets_world.size(); ++i) {
      if (!covered[i] && reach[i][best_h]) {
        covered[i] = true;
        band.pose_indices.push_back(i);
      }
    }
    plan.bands.push_back(std::move(band));
  }
  std::sort(plan.bands.begin(), plan.bands.end(),
    [](const LiftBand & a, const LiftBand & b) {return a.lift_height < b.lift_height;});
  return plan;
}

}  // namespace reachability
