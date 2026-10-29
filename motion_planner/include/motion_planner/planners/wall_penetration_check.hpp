/**
 * @file wall_penetration_check.hpp
 * @brief Shared signed-distance-past-a-plane helper for wall-contact bounds.
 * @details Used by both OmplPlanner (per-state, during FSP planning) and
 *          CartesianPlanner (post-hoc, over a computed path) to bound how far
 *          a point may sit past a known wall plane -- the independent check
 *          that stops PlanningRequest::allow_target_contact / an Allowed
 *          Collision Matrix relaxation from being exploited to travel
 *          arbitrarily far through the wall. See
 *          implementation_plan_manipulation_architecture.md §6.3 and
 *          Cartesian_Planner_Architecture.md ("Closing the exclusion's blast
 *          radius", Part B step 5, Part B.5).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cmath>

#include <Eigen/Geometry>

#include "core_types/pose.hpp"

namespace motion_planner
{

/**
 * @brief Whether wall_reference_pose carries real plane info, vs. a
 *        default-constructed sentinel (an untouched core_types::Pose{}).
 * @details Callers gate the checks below on this -- requests that don't
 *          supply wall-plane info (e.g. non-wall-cleaning callers, joint-space
 *          home/stow segments) skip wall-penetration checking entirely rather
 *          than being rejected by a degenerate zero-length normal.
 *          Quaternion-norm alone is NOT a reliable "unset" signal:
 *          core_types::Pose defaults qw to 1.0 (identity rotation, a
 *          sane default for any OTHER consumer of an untouched Pose), which
 *          has norm 1 -- indistinguishable from a genuinely-set identity-
 *          oriented wall plane. Confirmed live: an unset home segment's
 *          default Pose{} (origin position + identity quaternion) passed the
 *          old quaternion-only check and then failed wall-penetration
 *          against a bogus "wall at the world origin" plane. Requiring the
 *          position ALSO be at the origin narrows this to the one case that
 *          actually matters (the untouched default) -- a real wall's own
 *          captured/fitted reference point coincidentally landing exactly at
 *          (0,0,0) in whatever frame it's expressed in is not a realistic
 *          perception-pipeline outcome.
 */
inline bool hasWallReference(const core_types::Pose & wall_reference_pose)
{
  constexpr double kQuatNormSqEpsilon = 1e-9;
  constexpr double kOriginEpsilonSq = 1e-9;
  const double quat_norm_sq = wall_reference_pose.qx * wall_reference_pose.qx +
    wall_reference_pose.qy * wall_reference_pose.qy +
    wall_reference_pose.qz * wall_reference_pose.qz +
    wall_reference_pose.qw * wall_reference_pose.qw;
  const double position_norm_sq = wall_reference_pose.x * wall_reference_pose.x +
    wall_reference_pose.y * wall_reference_pose.y +
    wall_reference_pose.z * wall_reference_pose.z;
  const bool is_untouched_default = position_norm_sq <= kOriginEpsilonSq &&
    std::abs(quat_norm_sq - 1.0) <= kQuatNormSqEpsilon &&
    std::abs(wall_reference_pose.qw - 1.0) <= 1e-9;
  return quat_norm_sq > kQuatNormSqEpsilon && !is_untouched_default;
}

/**
 * @brief Signed depth of a point past the wall plane defined by
 *        wall_reference_pose (position = a point on the plane, local +Z =
 *        outward wall normal -- the same normal convention
 *        makeToolOrientationFromWallNormal already uses upstream, where the
 *        tool's own Z axis is set to -wall_normal to point into the wall).
 * @return Positive when the point is behind the plane (through/into the
 *         wall, along -normal) -- i.e. penetration depth. Negative when the
 *         point is in front of the plane (the expected standoff/free-space
 *         side). Zero at the surface.
 */
inline double wallPenetrationDepth(
  const core_types::Pose & wall_reference_pose,
  double point_x, double point_y, double point_z)
{
  const Eigen::Quaterniond orientation(
    wall_reference_pose.qw, wall_reference_pose.qx,
    wall_reference_pose.qy, wall_reference_pose.qz);
  const Eigen::Vector3d wall_normal = orientation.normalized() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d plane_point(
    wall_reference_pose.x, wall_reference_pose.y, wall_reference_pose.z);
  const Eigen::Vector3d point(point_x, point_y, point_z);
  return wall_normal.dot(plane_point - point);
}

/**
 * @brief True if the point's penetration depth past wall_reference_pose's
 *        plane does not exceed max_wall_penetration.
 * @details A point still in front of the wall (negative depth) always
 *          passes. max_wall_penetration <= 0 means "no tolerance" -- any
 *          penetration at all fails -- not "check disabled"; callers that
 *          want the check skipped entirely gate on hasWallReference() first
 *          (see PlanningRequest::wall_reference_pose /
 *          PlanningRequest::max_wall_penetration).
 */
inline bool isWithinWallPenetrationBound(
  const core_types::Pose & wall_reference_pose,
  double point_x, double point_y, double point_z,
  double max_wall_penetration)
{
  return wallPenetrationDepth(wall_reference_pose, point_x, point_y, point_z) <=
         max_wall_penetration;
}

}  // namespace motion_planner
