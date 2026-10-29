#include "motion_planner/core/planners/reachability_tilt.hpp"

namespace motion_planner
{

namespace
{
constexpr int kReachabilityTiltMagnitudeSteps = 3;
}  // namespace

EigenIsometry3dVector unconsumedWaypoints(
  const Eigen::Isometry3d & seed_transform,
  const EigenIsometry3dVector & waypoints,
  double achieved_fraction)
{
  EigenIsometry3dVector all_points;
  all_points.reserve(waypoints.size() + 1);
  all_points.push_back(seed_transform);
  all_points.insert(all_points.end(), waypoints.begin(), waypoints.end());

  std::vector<double> cumulative_length(all_points.size(), 0.0);
  for (size_t i = 1; i < all_points.size(); ++i) {
    cumulative_length[i] = cumulative_length[i - 1] +
      (all_points[i].translation() - all_points[i - 1].translation()).norm();
  }
  const double total_length = cumulative_length.back();
  const double achieved_length = achieved_fraction * total_length;
  size_t next_waypoint_index = 0;
  while (next_waypoint_index + 1 < cumulative_length.size() &&
    cumulative_length[next_waypoint_index + 1] <= achieved_length + 1e-9)
  {
    ++next_waypoint_index;
  }
  return EigenIsometry3dVector(
    waypoints.begin() + static_cast<std::ptrdiff_t>(next_waypoint_index), waypoints.end());
}

EigenIsometry3dVector generateReachabilityTiltCandidates(
  const Eigen::Isometry3d & ideal_target, double max_tilt_rad)
{
  EigenIsometry3dVector candidates;
  if (max_tilt_rad <= 0.0) {
    return candidates;
  }
  candidates.reserve(static_cast<size_t>(kReachabilityTiltMagnitudeSteps) * 4);
  for (int step = 1; step <= kReachabilityTiltMagnitudeSteps; ++step) {
    const double angle =
      max_tilt_rad * static_cast<double>(step) /
      static_cast<double>(kReachabilityTiltMagnitudeSteps);
    for (int axis_index = 0; axis_index < 2; ++axis_index) {
      const Eigen::Vector3d tangent_axis = ideal_target.linear().col(axis_index);
      for (const double signed_angle : {angle, -angle}) {
        Eigen::Isometry3d candidate = ideal_target;
        candidate.linear() =
          Eigen::AngleAxisd(signed_angle, tangent_axis).toRotationMatrix() * ideal_target.linear();
        candidates.push_back(candidate);
      }
    }
  }
  return candidates;
}

}  // namespace motion_planner
