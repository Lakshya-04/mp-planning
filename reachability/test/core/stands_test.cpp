#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "reachability/core/builder.hpp"
#include "reachability/core/stands.hpp"
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

std::vector<Pose> wall(const std::vector<double> & ys, const std::vector<double> & zs)
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

StationRequest request(std::vector<Pose> targets)
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

BaseModel box(bool strafe)
{
  BaseModel b;
  b.drive = strafe ? DriveType::kOmni : DriveType::kDiff;
  b.can_strafe = strafe;
  b.footprint = {{-0.4, -0.25}, {0.4, -0.25}, {0.4, 0.25}, {-0.4, 0.25}};
  return b;
}
}  // namespace

TEST(Stands, AWideWallNeedsSeveralStandsAndEveryTargetIsAssignedOnce)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const auto one = chooseStation(sharedMap(), r);
  ASSERT_TRUE(one.found);
  ASSERT_FALSE(one.plan.unreachable.empty());      // one stand cannot do it: the wall is 6 m wide

  const auto plan = planStands(sharedMap(), r);
  EXPECT_GE(plan.stands.size(), 2U);
  std::multiset<size_t> seen(plan.unreachable.begin(), plan.unreachable.end());
  for (const auto & s : plan.stands) {seen.insert(s.targets.begin(), s.targets.end());}
  ASSERT_EQ(seen.size(), r.targets_world.size());
  for (size_t i = 0; i < r.targets_world.size(); ++i) {EXPECT_EQ(seen.count(i), 1U) << i;}
  size_t covered = 0;
  for (const auto & s : plan.stands) {covered += s.targets.size();}
  EXPECT_GT(covered, one.plan.reachable_count);     // more than the best single stand
}

TEST(Stands, DrivingTimeIsSummedAndSlowerWithoutStrafing)
{
  auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const Stand start{Eigen::Vector2d(1.0, -3.0), 0.0};

  r.base = box(true);
  const auto omni = planStands(sharedMap(), r, start);
  r.base = box(false);
  const auto diff = planStands(sharedMap(), r, start);
  ASSERT_GE(omni.stands.size(), 2U);
  ASSERT_GE(diff.stands.size(), 2U);

  double sum = 0.0;
  for (const auto & s : diff.stands) {sum += s.travel_s;}
  EXPECT_NEAR(sum, diff.total_travel_s, 1e-9);
  EXPECT_GT(diff.total_travel_s, 0.0);
  // same wall, same speeds: turning, driving and turning back costs more than strafing
  EXPECT_GE(diff.total_travel_s, omni.total_travel_s - 1e-9);
}

TEST(Stands, WithoutABaseModelThereIsNoTravelTime)
{
  const auto plan = planStands(sharedMap(), request(wall({-3.0, 0.0, 3.0}, {0.8})));
  EXPECT_FALSE(plan.stands.empty());
  EXPECT_EQ(plan.total_travel_s, 0.0);
  for (const auto & s : plan.stands) {EXPECT_EQ(s.travel_s, 0.0);}
}

TEST(Stands, EquallyGoodStationsGoToTheOneTheBaseReachesSoonest)
{
  auto r = request(wall({0.0}, {0.8}));            // one target: many stations tie on coverage
  r.base = box(true);
  r.grid_step = 0.1;
  r.lateral_margin = 0.6;                          // room on both sides of the target
  const auto middle = chooseStation(sharedMap(), r);
  ASSERT_TRUE(middle.found);
  r.from = Stand{Eigen::Vector2d(1.0, 1.0), 0.0};  // the base is off to the +y side
  const auto near_from = chooseStation(sharedMap(), r);
  ASSERT_TRUE(near_from.found);
  EXPECT_GT(near_from.y, middle.y + 0.05);          // strictly moved towards `from`
  EXPECT_NEAR(near_from.y, 0.6, 0.11);              // the +y end of the search window
}

TEST(Stands, TheStandCapLeavesTargetsNotPlannedNotUnreachable)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const auto plan = planStands(sharedMap(), r, std::nullopt, 1);
  ASSERT_EQ(plan.stands.size(), 1U);
  EXPECT_TRUE(plan.unreachable.empty());        // every one of these targets is reachable from some stand
  EXPECT_EQ(plan.not_planned.size() + plan.stands[0].targets.size(), r.targets_world.size());
}

TEST(Stands, AStrafingBaseStillPrefersTheNearerStationWhenTurningDominates)
{
  auto r = request(wall({0.0}, {0.8}));
  r.base = box(true);
  r.grid_step = 0.1;
  r.lateral_margin = 0.6;
  // the best yaw for the box is side-on (90 deg): the turn (1.57 s) outlasts every drive, so travel time alone ties
  r.from = Stand{Eigen::Vector2d(2.7, -0.3), 0.0};
  const auto st = chooseStation(sharedMap(), r);
  ASSERT_TRUE(st.found);
  EXPECT_NEAR(st.y, -0.3, 0.11);                 // beside `from`, not the end the scan starts at
}

TEST(Stands, LiftBandIndicesAreGlobalAndMatchTheStandsTargets)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const auto plan = planStands(sharedMap(), r);
  ASSERT_GE(plan.stands.size(), 2U);
  for (const auto & s : plan.stands) {
    const std::set<size_t> mine(s.targets.begin(), s.targets.end());
    size_t in_bands = 0;
    for (const auto & b : s.station.plan.bands) {
      for (const size_t k : b.pose_indices) {
        EXPECT_EQ(mine.count(k), 1U) << "band index " << k << " is not one of this stand's targets";
        ++in_bands;
      }
    }
    EXPECT_EQ(in_bands, s.targets.size());
  }
}

TEST(Stands, UnreachableTargetsAreReportedNotLooped)
{
  const auto plan = planStands(sharedMap(), request(wall({0.0}, {5.0})));   // far above reach
  EXPECT_TRUE(plan.stands.empty());
  ASSERT_EQ(plan.unreachable.size(), 1U);
  EXPECT_EQ(plan.unreachable[0], 0U);
}
