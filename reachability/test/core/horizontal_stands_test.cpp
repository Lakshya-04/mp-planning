#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "reachability/core/builder.hpp"
#include "reachability/core/horizontal_stands.hpp"
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

// Grid of tool poses on a horizontal surface at height z. Ceiling: tool points up; floor: down.
std::vector<Pose> patch(double x0, double x1, double y0, double y1, double step, double z, bool up)
{
  std::vector<Pose> out;
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    for (double y = y0; y <= y1 + 1e-9; y += step) {
      Pose T = Pose::Identity();
      T.translation() = Vec3(x, y, z);
      if (!up) {T.linear() = Eigen::AngleAxisd(M_PI, Vec3::UnitX()).toRotationMatrix();}
      out.push_back(T);
    }
  }
  return out;
}

HorizontalStationRequest request(std::vector<Pose> targets)
{
  HorizontalStationRequest r;
  r.targets_world = std::move(targets);
  r.arm_mount_offset_z = 0.2;
  r.lift_max = 1.5;
  r.grid_step = 0.2;
  r.margin = 0.4;
  return r;
}

std::set<size_t> covered(const HorizontalStandPlan & p)
{
  std::set<size_t> s;
  for (const auto & st : p.plan.stands) {s.insert(st.targets.begin(), st.targets.end());}
  return s;
}
}  // namespace

TEST(HorizontalStands, CeilingPatchWithinOneStand)
{
  const auto t = patch(-0.2, 0.2, -0.2, 0.2, 0.2, 2.2, true);
  const auto p = planHorizontalStands(sharedMap(), request(t));
  ASSERT_EQ(p.plan.stands.size(), 1U);
  EXPECT_TRUE(p.plan.unreachable.empty());
  EXPECT_TRUE(p.out_of_lift_range.empty());
  EXPECT_EQ(p.plan.stands[0].targets.size(), t.size());
  // the stand is under the patch, not off to one side
  EXPECT_NEAR(p.plan.stands[0].stand.arm_xy.x(), 0.0, 0.8);
  EXPECT_NEAR(p.plan.stands[0].stand.arm_xy.y(), 0.0, 0.8);
}

TEST(HorizontalStands, WidePatchNeedsSeveralStands)
{
  const auto t = patch(-3.0, 3.0, -0.2, 0.2, 0.5, 2.2, true);
  const auto p = planHorizontalStands(sharedMap(), request(t));
  EXPECT_GE(p.plan.stands.size(), 2U);
  EXPECT_TRUE(p.plan.unreachable.empty());
  EXPECT_EQ(covered(p).size(), t.size());
  size_t total = 0;   // each target goes to exactly one stand
  for (const auto & s : p.plan.stands) {total += s.targets.size();}
  EXPECT_EQ(total, t.size());
}

TEST(HorizontalStands, CeilingAboveLiftAndArmReachIsUnreachable)
{
  const auto t = patch(-0.2, 0.2, -0.2, 0.2, 0.2, 5.0, true);
  const auto p = planHorizontalStands(sharedMap(), request(t));
  EXPECT_TRUE(p.plan.stands.empty());
  EXPECT_EQ(p.plan.unreachable.size(), t.size());
  EXPECT_EQ(p.out_of_lift_range.size(), t.size());
  EXPECT_FALSE(heightWithinLiftReach(sharedMap(), 5.0, 0.0, 1.5, 0.2));
  EXPECT_FALSE(chooseHorizontalStation(sharedMap(), request(t)).found);
}

TEST(HorizontalStands, MixedPatchReportsTheHighTargetsOnly)
{
  auto t = patch(-0.2, 0.2, 0.0, 0.0, 0.2, 2.2, true);
  const size_t n_ok = t.size();
  const auto high = patch(0.0, 0.0, 0.0, 0.0, 0.2, 5.0, true);
  t.insert(t.end(), high.begin(), high.end());
  const auto p = planHorizontalStands(sharedMap(), request(t));
  ASSERT_EQ(p.out_of_lift_range.size(), 1U);
  EXPECT_EQ(p.out_of_lift_range[0], n_ok);
  EXPECT_EQ(p.plan.unreachable, p.out_of_lift_range);
  EXPECT_EQ(covered(p).size(), n_ok);
}

TEST(HorizontalStands, FloorPatch)
{
  const auto t = patch(-0.3, 0.3, -0.3, 0.3, 0.3, 0.0, false);
  const auto p = planHorizontalStands(sharedMap(), request(t));
  ASSERT_GE(p.plan.stands.size(), 1U);
  EXPECT_EQ(covered(p).size(), t.size());
  EXPECT_TRUE(p.plan.unreachable.empty());
  // every target is either covered or reported, never lost
  EXPECT_EQ(covered(p).size() + p.plan.unreachable.size(), t.size());
}

TEST(HorizontalStands, FloorBelowTheMapWindowIsUnreachable)
{
  const auto t = patch(0.0, 0.0, 0.0, 0.0, 0.2, -3.0, false);
  const auto p = planHorizontalStands(sharedMap(), request(t));
  EXPECT_EQ(p.out_of_lift_range.size(), 1U);
  EXPECT_TRUE(p.plan.stands.empty());
}

TEST(HorizontalStands, Deterministic)
{
  const auto t = patch(-2.0, 2.0, -0.2, 0.2, 0.5, 2.2, true);
  const auto a = planHorizontalStands(sharedMap(), request(t));
  const auto b = planHorizontalStands(sharedMap(), request(t));
  ASSERT_EQ(a.plan.stands.size(), b.plan.stands.size());
  for (size_t i = 0; i < a.plan.stands.size(); ++i) {
    EXPECT_EQ(a.plan.stands[i].stand.arm_xy, b.plan.stands[i].stand.arm_xy);
    EXPECT_EQ(a.plan.stands[i].targets, b.plan.stands[i].targets);
  }
}

TEST(HorizontalStands, EmptyInput)
{
  const auto p = planHorizontalStands(sharedMap(), request({}));
  EXPECT_TRUE(p.plan.stands.empty());
  EXPECT_TRUE(p.plan.unreachable.empty());
  EXPECT_TRUE(p.out_of_lift_range.empty());
  EXPECT_FALSE(chooseHorizontalStation(sharedMap(), request({})).found);
}

TEST(HorizontalStands, SingleTargetCeiling)
{
  const auto t = patch(1.0, 1.0, 1.0, 1.0, 0.2, 2.2, true);
  const auto p = planHorizontalStands(sharedMap(), request(t));
  ASSERT_EQ(p.plan.stands.size(), 1U);
  EXPECT_EQ(p.plan.stands[0].targets, std::vector<size_t>{0});
  EXPECT_FALSE(p.plan.stands[0].station.plan.bands.empty());
}
