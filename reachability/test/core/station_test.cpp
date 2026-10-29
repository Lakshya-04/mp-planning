#include <gtest/gtest.h>

#include "reachability/core/builder.hpp"
#include "reachability/core/station.hpp"
#include "support.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

namespace
{
const Map4D & sharedMap()
{
  static const Map4D map = [] {
      static const ChainModel arm = genericArm();
      BuildOptions o;
      o.samples = 1'500'000;
      o.threads = 4;
      return buildMap(arm, coarse(), o);
    }();
  return map;
}

// Wall at x = 3 (plane normal pointing back toward the robot, -x), tool pointing into it (+x).
std::vector<Pose> wallTargets(const std::vector<double> & ys, const std::vector<double> & zs)
{
  std::vector<Pose> out;
  for (double y : ys) {
    for (double z : zs) {
      Pose T = Pose::Identity();
      T.translation() = Vec3(3.0, y, z);
      T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
      out.push_back(T);
    }
  }
  return out;
}

StationRequest baseRequest(std::vector<Pose> targets)
{
  StationRequest r;
  r.targets_world = std::move(targets);
  r.wall_point = Vec3(3.0, 0.0, 0.0);
  r.wall_normal = Vec3(-1.0, 0.0, 0.0);
  r.arm_mount_offset_z = 0.2;
  r.lift_max = 1.5;
  r.grid_step = 0.2;
  r.lateral_margin = 0.2;
  return r;
}
}  // namespace

TEST(Station, StandsOnTheFreeSideAndReachesTheWall)
{
  const auto s = chooseStation(sharedMap(), baseRequest(wallTargets({0.0}, {0.8, 1.2})));
  ASSERT_TRUE(s.found);
  EXPECT_LT(s.x, 3.0);                      // never behind the wall
  EXPECT_GE(s.standoff, 0.3 - 1e-9);
  EXPECT_LE(s.standoff, 1.6 + 1e-9);
  EXPECT_TRUE(s.plan.unreachable.empty());
  EXPECT_GT(s.evaluated, 10U);
}

TEST(Station, NothingReachableMeansNotFound)
{
  auto r = baseRequest(wallTargets({0.0}, {5.0}));  // far above any lift + arm reach
  EXPECT_FALSE(chooseStation(sharedMap(), r).found);
}

TEST(Station, WideWallPrefersAStationInTheMiddle)
{
  const auto s = chooseStation(sharedMap(), baseRequest(wallTargets({-0.3, 0.0, 0.3}, {1.0})));
  ASSERT_TRUE(s.found);
  EXPECT_TRUE(s.plan.unreachable.empty());
  EXPECT_NEAR(s.y, 0.0, 0.35);
}

TEST(Station, RejectsDegenerateRequests)
{
  auto r = baseRequest(wallTargets({0.0}, {1.0}));
  r.grid_step = 0.0;
  EXPECT_FALSE(chooseStation(sharedMap(), r).found);
  r = baseRequest({});
  EXPECT_FALSE(chooseStation(sharedMap(), r).found);
  r = baseRequest(wallTargets({0.0}, {1.0}));
  r.wall_normal = Vec3::UnitZ();  // horizontal plane
  EXPECT_FALSE(chooseStation(sharedMap(), r).found);
}
