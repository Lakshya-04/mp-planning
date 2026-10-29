#include <gtest/gtest.h>

#include <set>

#include "reachability/core/builder.hpp"
#include "reachability/core/lift_bands.hpp"
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

// Tool poses on a wall at x = `wall_x`, pointing into it (+x), at the given heights.
std::vector<Pose> wallTargets(double wall_x, const std::vector<double> & heights)
{
  std::vector<Pose> out;
  for (double z : heights) {
    Pose T = Pose::Identity();
    T.translation() = Vec3(wall_x, 0.0, z);
    T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
    out.push_back(T);
  }
  return out;
}
}  // namespace

TEST(LiftBands, HeightRangeIncludesBothEnds)
{
  const auto h = heightRange(0.0, 1.5, 0.25);
  ASSERT_EQ(h.size(), 7U);
  EXPECT_DOUBLE_EQ(h.front(), 0.0);
  EXPECT_DOUBLE_EQ(h.back(), 1.5);
  EXPECT_EQ(heightRange(0.0, 1.0, 0.4).back(), 1.0);  // last step clipped to the end
  EXPECT_TRUE(heightRange(1.0, 0.0, 0.1).empty());
}

TEST(LiftBands, ATallWallNeedsSeveralBandsAndEveryReachablePoseIsCoveredOnce)
{
  // Wall 0.6 m in front of the base, targets every 0.1 m from 0.1 to 3.0 m: more than the arm
  // covers from any one lift height (the lowest band reaches about 0.1 to 2.6 m).
  std::vector<double> zs;
  for (double z = 0.1; z <= 3.0; z += 0.1) {zs.push_back(z);}
  const auto targets = wallTargets(0.6, zs);
  const auto plan = planLiftBands(sharedMap(), targets, 0.0, 0.0, heightRange(0.0, 1.5, 0.05), 0.2);

  EXPECT_GE(plan.bands.size(), 2U);
  EXPECT_LE(plan.bands.size(), 3U);
  EXPECT_TRUE(plan.unreachable.empty());
  std::set<size_t> covered;
  for (const auto & b : plan.bands) {
    for (size_t i : b.pose_indices) {
      EXPECT_TRUE(covered.insert(i).second) << "target " << i << " assigned to two bands";
    }
  }
  EXPECT_EQ(covered.size() + plan.unreachable.size(), targets.size());
  EXPECT_EQ(plan.reachable_count, covered.size());
  for (size_t k = 0; k + 1 < plan.bands.size(); ++k) {
    EXPECT_LT(plan.bands[k].lift_height, plan.bands[k + 1].lift_height);  // ordered bottom up
  }
}

TEST(LiftBands, AShortWallNeedsOneBand)
{
  const auto targets = wallTargets(0.6, {0.5, 0.6, 0.7, 0.8});
  const auto plan = planLiftBands(sharedMap(), targets, 0.0, 0.0, heightRange(0.0, 1.5, 0.05), 0.2);
  EXPECT_EQ(plan.bands.size(), 1U);
  EXPECT_TRUE(plan.unreachable.empty());
}

TEST(LiftBands, PosesNoHeightReachesAreReportedNotHidden)
{
  // 3 m out is beyond the arm from any height; 8 m up is beyond the lift's range.
  auto targets = wallTargets(0.6, {0.6});
  auto far = wallTargets(3.0, {0.6});
  auto high = wallTargets(0.6, {8.0});
  targets.push_back(far[0]);
  targets.push_back(high[0]);
  const auto plan = planLiftBands(sharedMap(), targets, 0.0, 0.0, heightRange(0.0, 1.5, 0.05), 0.2);
  ASSERT_EQ(plan.unreachable.size(), 2U);
  EXPECT_EQ(plan.unreachable[0], 1U);
  EXPECT_EQ(plan.unreachable[1], 2U);
  EXPECT_EQ(plan.reachable_count, 1U);
}

TEST(LiftBands, StandingTooFarMakesTheWallUnreachable)
{
  const auto targets = wallTargets(0.6, {0.5, 0.6});
  const auto plan = planLiftBands(sharedMap(), targets, -2.5, 0.0, heightRange(0.0, 1.5, 0.05), 0.2);
  EXPECT_TRUE(plan.bands.empty());
  EXPECT_EQ(plan.unreachable.size(), 2U);
}

TEST(LiftBands, ReachableByHeightMatchesDirectQueries)
{
  const auto targets = wallTargets(0.6, {1.0});
  const std::vector<double> heights = {0.0, 0.5, 1.0, 1.5};
  const auto table = reachableByHeight(sharedMap(), targets, 0.0, 0.0, heights, 0.2);
  ASSERT_EQ(table.size(), 1U);
  for (size_t h = 0; h < heights.size(); ++h) {
    Pose in_base = targets[0];
    in_base.translation() -= Vec3(0.0, 0.0, heights[h] + 0.2);
    EXPECT_EQ(static_cast<bool>(table[0][h]), sharedMap().isReachable(in_base));
  }
}
