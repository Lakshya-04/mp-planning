/**
 * @file reachability_tilt.hpp
 * @brief Pure geometry helpers for CartesianPlanner's reachability-tilt retry tier.
 * @details Extracted out of cartesian_planner.cpp so this math -- which has no MoveIt dependency
 *          at all -- is independently unit-testable without a live move_group/robot model (unlike
 *          the rest of CartesianPlanner::plan(), which requires one from the very first
 *          precheck). See cartesian_planner.cpp's plan() for how these are used: unconsumedWaypoints()
 *          re-derives "what's left" after a partial resumed walk (mirroring Step 4's own initial
 *          derivation); generateReachabilityTiltCandidates() produces the candidate tilted
 *          orientations tried, smallest tilt first, when the ideal target pose has no nearby IK
 *          solution at all (CartesianTruncationReason::kIkFailure).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace motion_planner
{

using EigenIsometry3dVector = std::vector<Eigen::Isometry3d,
    Eigen::aligned_allocator<Eigen::Isometry3d>>;

/**
 * @brief Given a seed transform, a waypoint list, and the fraction of that combined path actually
 *        achieved (walking from the seed through the waypoints, in order), returns the suffix of
 *        `waypoints` not yet reached -- the cumulative-arc-length computation
 *        CartesianPlanner::plan() uses (both at its own Step 4 and inside the reachability-tilt
 *        retry tier, which needs to redo this after every partial resumed leg against its own
 *        locally-substituted waypoint list).
 */
EigenIsometry3dVector unconsumedWaypoints(
  const Eigen::Isometry3d & seed_transform,
  const EigenIsometry3dVector & waypoints,
  double achieved_fraction);

/**
 * @brief Candidate tilted orientations for the reachability-tilt retry tier, smallest tilt first
 *        (first success wins, matching CartesianPlanner's existing collision-recovery loop's
 *        "bounded attempts, first success wins" style). Tilts about the ideal target's OWN local
 *        X/Y (tangent) axes -- CartesianPlanner has no notion of "wall normal" of its own, only
 *        the target pose's local frame, so pinning one tangent axis while rotating the tool's Z
 *        axis toward/away from it is the natural geometry-agnostic way to perturb "how the tool
 *        points" without introducing an arbitrary twist. Position is never changed -- this is an
 *        orientation-only perturbation.
 * @param ideal_target The pose that had no reachable IK solution.
 * @param max_tilt_rad Maximum tilt magnitude to try, in radians (0 = no candidates returned).
 */
EigenIsometry3dVector generateReachabilityTiltCandidates(
  const Eigen::Isometry3d & ideal_target, double max_tilt_rad);

}  // namespace motion_planner
