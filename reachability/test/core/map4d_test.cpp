#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "reachability/core/map4d.hpp"

using namespace reachability;  // NOLINT

namespace
{
Pose randomPose(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> pos(-1.0, 1.0), ang(-M_PI, M_PI), unit(-1.0, 1.0);
  Pose T = Pose::Identity();
  T.translation() = Vec3(pos(rng), pos(rng), 0.5 * pos(rng));
  Vec3 axis(unit(rng), unit(rng), unit(rng));
  if (axis.norm() < 0.1) {axis = Vec3::UnitX();}
  T.linear() = Eigen::AngleAxisd(ang(rng), axis.normalized()).toRotationMatrix();
  return T;
}
}  // namespace

TEST(Map4D, KeyIsInvariantUnderRotationAboutTheBaseAxis)
{
  Map4D map;
  std::mt19937 rng(1);
  int checked = 0;
  for (int i = 0; i < 300; ++i) {
    const Pose T = randomPose(rng);
    if (std::hypot(T.linear().col(2).x(), T.linear().col(2).y()) < 0.05) {continue;}
    const CellKey a = map.keyOf(T);
    const Pose R(Eigen::AngleAxisd(0.37 * (i % 17) - 2.0, Vec3::UnitZ()));
    const CellKey b = map.keyOf(R * T);
    if (!a.valid || !b.valid) {continue;}
    ++checked;
    // Rotating position and orientation together about z changes nothing about reachability.
    EXPECT_NEAR(a.iz, b.iz, 0);
    EXPECT_NEAR(a.itheta, b.itheta, 0);
    EXPECT_LE(std::abs(a.ix - b.ix), 1) << "boundary rounding only";
    EXPECT_LE(std::abs(a.iy - b.iy), 1);
  }
  EXPECT_GT(checked, 150);
}

TEST(Map4D, KeyIsInvariantUnderRotationAboutTheToolAxis)
{
  Map4D map;
  std::mt19937 rng(2);
  for (int i = 0; i < 200; ++i) {
    const Pose T = randomPose(rng);
    Pose spun = T;
    spun.linear() = T.linear() * Eigen::AngleAxisd(0.9 + 0.1 * i, Vec3::UnitZ()).toRotationMatrix();
    const CellKey a = map.keyOf(T), b = map.keyOf(spun);
    ASSERT_EQ(a.valid, b.valid);
    if (a.valid) {
      EXPECT_EQ(a.ix, b.ix);
      EXPECT_EQ(a.iy, b.iy);
      EXPECT_EQ(a.iz, b.iz);
      EXPECT_EQ(a.itheta, b.itheta);
    }
  }
}

TEST(Map4D, PolarAngleIsMeasuredFromVertical)
{
  Map4D map;
  Pose up = Pose::Identity();       // tool z up
  Pose side = Pose::Identity();     // tool z along +x
  side.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  Pose down = Pose::Identity();     // tool z down
  down.linear() = Eigen::AngleAxisd(M_PI, Vec3::UnitY()).toRotationMatrix();
  for (Pose * p : {&up, &side, &down}) {p->translation() = Vec3(0.5, 0, 0.2);}
  EXPECT_EQ(map.keyOf(up).itheta, 0);
  EXPECT_EQ(map.keyOf(side).itheta, map.ntheta() / 2);
  EXPECT_EQ(map.keyOf(down).itheta, map.ntheta() - 1);
}

TEST(Map4D, RecordSaturatesAndQueriesWork)
{
  Map4D map;
  Pose T = Pose::Identity();
  T.translation() = Vec3(0.4, 0.1, 0.3);
  EXPECT_FALSE(map.isReachable(T));
  for (int i = 0; i < 300; ++i) {map.record(T);}
  EXPECT_TRUE(map.isReachable(T));
  EXPECT_EQ(map.cellCount(T), 255);
  EXPECT_EQ(map.nonzeroCells(), 1U);
}

TEST(Map4D, OutsideTheWindowIsNeverReachableAndNeverRecorded)
{
  Map4D map;
  Pose far = Pose::Identity();
  far.translation() = Vec3(10.0, 0.0, 0.0);
  EXPECT_FALSE(map.keyOf(far).valid);
  map.record(far);
  EXPECT_EQ(map.nonzeroCells(), 0U);
  EXPECT_FALSE(map.isReachable(far));
  Pose high = Pose::Identity();
  high.translation() = Vec3(0.3, 0.0, 50.0);
  EXPECT_FALSE(map.keyOf(high).valid);
}

TEST(Map4D, MergeAddsCountsAndRejectsMismatchedParameters)
{
  Map4D a, b;
  Pose T = Pose::Identity();
  T.translation() = Vec3(0.4, 0.0, 0.3);
  a.record(T);
  b.record(T);
  b.record(T);
  a.mergeFrom(b);
  EXPECT_EQ(a.cellCount(T), 3);
  MapParams other;
  other.resolution = 0.1;
  EXPECT_THROW(a.mergeFrom(Map4D(other)), std::invalid_argument);
}

TEST(Map4D, InvalidParametersThrow)
{
  MapParams p;
  p.resolution = 0.0;
  EXPECT_THROW(Map4D{p}, std::invalid_argument);
  MapParams q;
  q.z_max = q.z_min;
  EXPECT_THROW(Map4D{q}, std::invalid_argument);
}

TEST(Map4D, InverseQueryReturnsTheBaseForARecordedPose)
{
  Map4D map;
  // A tool 0.6 m in front of the arm base, pointing horizontally along +x, 0.3 m up.
  Pose T = Pose::Identity();
  T.translation() = Vec3(0.6, 0.0, 0.3);
  T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  map.record(T);

  // The same pose placed in the world, tool at (3.0, 2.0, 1.3) pointing +x: the arm base must
  // be 0.6 m behind it along x and 1.0 m below.
  Pose world = T;
  world.translation() = Vec3(3.0, 2.0, 1.3);
  const auto c = map.inverseQuery(world, /*arm_base_z=*/1.0);
  ASSERT_EQ(c.size(), 1U);
  EXPECT_NEAR(c[0].x, 2.4, 0.05);
  EXPECT_NEAR(c[0].y, 2.0, 0.05);
}

TEST(Map4D, InverseQueryOutsideTheHeightRangeIsEmpty)
{
  Map4D map;
  Pose T = Pose::Identity();
  T.translation() = Vec3(0.6, 0.0, 0.3);
  map.record(T);
  Pose world = T;
  world.translation() = Vec3(1.0, 0.0, 100.0);
  EXPECT_TRUE(map.inverseQuery(world, 0.0).empty());
}

namespace
{
Pose verticalPose(double radius, double azimuth, double z)
{
  Pose T = Pose::Identity();   // tool z axis points straight up
  T.translation() = Vec3(radius * std::cos(azimuth), radius * std::sin(azimuth), z);
  return T;
}
}  // namespace

TEST(Map4D, InverseQueryEmitsEachRingOnceForAVerticalAxis)
{
  struct Case {double extent, res;};
  // default window (inverseQuery used to read a row below the recorded one), 0.1 m cells (it
  // returned nothing) and a window where two rows passed its filter (every ring came out twice)
  for (const Case c : {Case{1.8, 0.05}, Case{1.8, 0.1}, Case{1.6, 0.05}}) {
    MapParams p;
    p.xy_extent = c.extent;
    p.resolution = c.res;
    Map4D map(p);
    const double radius = 0.62;
    // the same pose seen from many azimuths: all of them must land in one cell (the rounding noise of
    // y* used to split them over two rows, which inverseQuery then emitted twice)
    for (int i = 0; i < 60; ++i) {map.record(verticalPose(radius, 0.7 + 0.1 * i, 0.3));}
    ASSERT_EQ(map.nonzeroCells(), 1U);
    const auto cands = map.inverseQuery(verticalPose(radius, 0.7, 0.3), 0.0, 8);
    EXPECT_EQ(cands.size(), 8U) << "extent " << c.extent << " res " << c.res;   // one ring
    for (const auto & b : cands) {
      EXPECT_NEAR(std::hypot(b.x - radius * std::cos(0.7), b.y - radius * std::sin(0.7)), radius, c.res);
    }
  }
}

TEST(Map4D, VerticalAxisReachabilityDoesNotDependOnTheTargetAzimuth)
{
  for (const double extent : {1.8, 1.6}) {
    MapParams p;
    p.xy_extent = extent;
    Map4D map(p);
    std::vector<double> radii;
    for (double r = 0.31; r < 1.55; r += 0.0731) {radii.push_back(r);}
    for (const double r : radii) {map.record(verticalPose(r, 0.0, 0.3));}
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> az(-M_PI, M_PI);
    for (const double r : radii) {
      for (int i = 0; i < 200; ++i) {
        EXPECT_TRUE(map.isReachable(verticalPose(r, az(rng), 0.3))) << "extent " << extent << " radius " << r;
      }
    }
  }
}
