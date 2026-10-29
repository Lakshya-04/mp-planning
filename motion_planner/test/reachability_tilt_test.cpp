#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "motion_planner/planners/reachability_tilt.hpp"

namespace motion_planner
{
namespace
{

Eigen::Isometry3d makePose(double x, double y, double z)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

}  // namespace

TEST(UnconsumedWaypoints, ZeroFractionReturnsEveryWaypoint)
{
  const Eigen::Isometry3d seed = makePose(0.0, 0.0, 0.0);
  const EigenIsometry3dVector waypoints = {
    makePose(1.0, 0.0, 0.0), makePose(2.0, 0.0, 0.0), makePose(3.0, 0.0, 0.0)};

  const auto remaining = unconsumedWaypoints(seed, waypoints, 0.0);

  ASSERT_EQ(remaining.size(), 3u);
  EXPECT_NEAR(remaining[0].translation().x(), 1.0, 1e-9);
}

TEST(UnconsumedWaypoints, FullFractionReturnsEmpty)
{
  const Eigen::Isometry3d seed = makePose(0.0, 0.0, 0.0);
  const EigenIsometry3dVector waypoints = {
    makePose(1.0, 0.0, 0.0), makePose(2.0, 0.0, 0.0), makePose(3.0, 0.0, 0.0)};

  const auto remaining = unconsumedWaypoints(seed, waypoints, 1.0);

  EXPECT_TRUE(remaining.empty());
}

TEST(UnconsumedWaypoints, PartialFractionReturnsCorrectSuffix)
{
  // Seed at origin; 3 equally-spaced waypoints 1m apart -> total length 3m. Achieving 0.5
  // (1.5m) means the first waypoint (at 1m) is fully behind, the second (at 2m) is not yet
  // reached -- so the suffix should start at the second waypoint.
  const Eigen::Isometry3d seed = makePose(0.0, 0.0, 0.0);
  const EigenIsometry3dVector waypoints = {
    makePose(1.0, 0.0, 0.0), makePose(2.0, 0.0, 0.0), makePose(3.0, 0.0, 0.0)};

  const auto remaining = unconsumedWaypoints(seed, waypoints, 0.5);

  ASSERT_EQ(remaining.size(), 2u);
  EXPECT_NEAR(remaining[0].translation().x(), 2.0, 1e-9);
  EXPECT_NEAR(remaining[1].translation().x(), 3.0, 1e-9);
}

TEST(GenerateReachabilityTiltCandidates, ZeroMaxTiltProducesNoCandidates)
{
  const Eigen::Isometry3d ideal = makePose(0.0, 0.0, 0.0);
  const auto candidates = generateReachabilityTiltCandidates(ideal, 0.0);
  EXPECT_TRUE(candidates.empty());
}

TEST(GenerateReachabilityTiltCandidates, AllCandidatesPreservePositionAndAreWithinMaxTilt)
{
  const Eigen::Isometry3d ideal = makePose(1.0, 2.0, 3.0);
  const double max_tilt_rad = 30.0 * M_PI / 180.0;
  const auto candidates = generateReachabilityTiltCandidates(ideal, max_tilt_rad);

  ASSERT_FALSE(candidates.empty());
  for (const auto & candidate : candidates) {
    EXPECT_NEAR(candidate.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(candidate.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(candidate.translation().z(), 3.0, 1e-9);

    // Angle between candidate's Z axis and ideal's Z axis must never exceed max_tilt_rad.
    const Eigen::Vector3d ideal_z = ideal.linear().col(2);
    const Eigen::Vector3d candidate_z = candidate.linear().col(2);
    const double cos_angle = std::clamp(ideal_z.dot(candidate_z), -1.0, 1.0);
    const double angle = std::acos(cos_angle);
    EXPECT_LE(angle, max_tilt_rad + 1e-6);

    // Rotation matrix must stay orthonormal (a real rotation, not a shear/scale).
    EXPECT_NEAR(
      (candidate.linear().transpose() * candidate.linear() - Eigen::Matrix3d::Identity())
      .norm(), 0.0, 1e-9);
  }
}

TEST(GenerateReachabilityTiltCandidates, SmallestMagnitudeCandidatesComeFirst)
{
  const Eigen::Isometry3d ideal = Eigen::Isometry3d::Identity();
  const double max_tilt_rad = 30.0 * M_PI / 180.0;
  const auto candidates = generateReachabilityTiltCandidates(ideal, max_tilt_rad);

  ASSERT_GE(candidates.size(), 4u);
  const Eigen::Vector3d ideal_z = ideal.linear().col(2);
  auto tiltAngle = [&](const Eigen::Isometry3d & candidate) {
      const double cos_angle = std::clamp(ideal_z.dot(candidate.linear().col(2)), -1.0, 1.0);
      return std::acos(cos_angle);
    };

  // The first 4 candidates (one magnitude step, 2 axes x 2 signs) must all have a strictly
  // smaller tilt angle than the last 4 (the largest magnitude step).
  const double first_step_angle = tiltAngle(candidates.front());
  const double last_step_angle = tiltAngle(candidates.back());
  EXPECT_LT(first_step_angle, last_step_angle);
}

}  // namespace motion_planner
