#include <gtest/gtest.h>

#include "motion_planner/core/planners/wall_penetration_check.hpp"

namespace motion_planner
{
namespace
{

core_types::Pose makeWallReference(double x, double y, double z)
{
  core_types::Pose pose;
  pose.x = x;
  pose.y = y;
  pose.z = z;
  // Identity orientation -- local +Z (the wall normal) points along the world +Z axis.
  return pose;
}

TEST(WallPenetrationCheckTest, HasWallReferenceIsFalseForUntouchedDefault)
{
  EXPECT_FALSE(hasWallReference(core_types::Pose{}));
}

TEST(WallPenetrationCheckTest, HasWallReferenceIsTrueForNonOriginPlane)
{
  EXPECT_TRUE(hasWallReference(makeWallReference(1.0, 0.0, 0.0)));
}

TEST(WallPenetrationCheckTest, HasWallReferenceIsTrueForOriginPlaneWithNonIdentityOrientation)
{
  core_types::Pose pose = makeWallReference(0.0, 0.0, 0.0);
  pose.qw = 0.70710678;
  pose.qz = 0.70710678;

  EXPECT_TRUE(hasWallReference(pose));
}

TEST(WallPenetrationCheckTest, PenetrationDepthIsZeroAtThePlane)
{
  const core_types::Pose wall_reference = makeWallReference(0.0, 0.0, 1.0);

  EXPECT_NEAR(wallPenetrationDepth(wall_reference, 0.0, 0.0, 1.0), 0.0, 1e-9);
}

TEST(WallPenetrationCheckTest, PenetrationDepthIsNegativeInFrontOfThePlane)
{
  // Wall plane at z=1, normal +Z; a point at z=1.5 is on the free-space side (further along
  // +normal from the plane), which wallPenetrationDepth defines as negative depth.
  const core_types::Pose wall_reference = makeWallReference(0.0, 0.0, 1.0);

  EXPECT_LT(wallPenetrationDepth(wall_reference, 0.0, 0.0, 1.5), 0.0);
}

TEST(WallPenetrationCheckTest, PenetrationDepthIsPositiveBehindThePlane)
{
  // A point at z=0.5 is behind the plane (through the wall, along -normal) -- positive depth.
  const core_types::Pose wall_reference = makeWallReference(0.0, 0.0, 1.0);

  EXPECT_GT(wallPenetrationDepth(wall_reference, 0.0, 0.0, 0.5), 0.0);
}

TEST(WallPenetrationCheckTest, WithinBoundAllowsAnyStandoffInFrontOfThePlane)
{
  const core_types::Pose wall_reference = makeWallReference(0.0, 0.0, 1.0);

  EXPECT_TRUE(isWithinWallPenetrationBound(wall_reference, 0.0, 0.0, 1.5, /*max_wall_penetration=*/0.0));
}

TEST(WallPenetrationCheckTest, ZeroToleranceRejectsAnyPenetration)
{
  const core_types::Pose wall_reference = makeWallReference(0.0, 0.0, 1.0);

  // max_wall_penetration <= 0 means "no tolerance", not "check disabled" -- even a hair's-width
  // penetration at z=0.999 (depth=0.001) must fail.
  EXPECT_FALSE(isWithinWallPenetrationBound(wall_reference, 0.0, 0.0, 0.999, /*max_wall_penetration=*/0.0));
}

TEST(WallPenetrationCheckTest, PositiveToleranceAllowsBoundedPenetration)
{
  const core_types::Pose wall_reference = makeWallReference(0.0, 0.0, 1.0);

  EXPECT_TRUE(isWithinWallPenetrationBound(wall_reference, 0.0, 0.0, 0.99, /*max_wall_penetration=*/0.02));
  EXPECT_FALSE(isWithinWallPenetrationBound(wall_reference, 0.0, 0.0, 0.95, /*max_wall_penetration=*/0.02));
}

}  // namespace
}  // namespace motion_planner
