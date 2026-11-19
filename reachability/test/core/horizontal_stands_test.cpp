#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
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
  // the stand is under the patch: the grid (0.2 m, aligned to 0) has the patch centre on it and that
  // station reaches every target, so the tie-break to the middle picks it exactly
  EXPECT_NEAR(p.plan.stands[0].stand.arm_xy.x(), 0.0, 0.1);
  EXPECT_NEAR(p.plan.stands[0].stand.arm_xy.y(), 0.0, 0.1);
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

namespace
{
// A map where a tool pointing up at height 0.3 is reachable only at horizontal radii r_lo..r_hi.
Map4D ringMap(double r_lo, double r_hi, MapParams p = MapParams{})
{
  Map4D map(p);
  for (double r = r_lo; r <= r_hi + 1e-9; r += 0.05) {
    Pose T = Pose::Identity();
    T.translation() = Vec3(r + 0.025, 0.0, 0.3);
    map.record(T);
  }
  return map;
}

Pose up(double x, double y, double z)
{
  Pose T = Pose::Identity();
  T.translation() = Vec3(x, y, z);
  return T;
}

HorizontalStationRequest ringRequest(std::vector<Pose> targets)
{
  HorizontalStationRequest r;
  r.targets_world = std::move(targets);
  r.lift_min = 0.0;
  r.lift_max = 0.0;
  r.lift_step = 0.05;
  r.grid_step = 0.1;
  return r;
}
}  // namespace

TEST(HorizontalStands, ALoneTargetReachableOnlyBeyondTheMarginIsStillFound)
{
  const Map4D map = ringMap(0.80, 1.00);   // nothing closer than 0.8 m
  auto r = ringRequest({up(5.0, 5.0, 0.3)});
  ASSERT_DOUBLE_EQ(r.margin, 0.5);          // the search used to stop 0.5 m (0.7 m diagonally) out
  const auto st = chooseHorizontalStation(map, r);
  ASSERT_TRUE(st.found);
  const double d = std::hypot(st.x - 5.0, st.y - 5.0);
  EXPECT_GE(d, 0.8);
  EXPECT_LE(d, 1.05);
  const auto p = planHorizontalStands(map, r);
  EXPECT_TRUE(p.plan.unreachable.empty());
  ASSERT_EQ(p.plan.stands.size(), 1U);
  EXPECT_EQ(p.plan.stands[0].targets, std::vector<size_t>{0});
}

TEST(HorizontalStands, TheLiftWindowIsTheMapsRoundedUpWindow)
{
  MapParams mp;
  mp.z_min = -1.0;
  mp.z_max = 1.8;
  mp.resolution = 0.06;   // 2.8 / 0.06 -> 47 cells: the window really ends at 1.82
  const Map4D map(mp);
  ASSERT_EQ(map.nz(), 47);
  EXPECT_TRUE(heightWithinLiftReach(map, 1.81, 0.0, 0.0, 0.0));
  EXPECT_FALSE(heightWithinLiftReach(map, 1.83, 0.0, 0.0, 0.0));
  Map4D rec(mp);
  for (double r = 0.5; r <= 0.8; r += 0.05) {rec.record(up(r, 0.0, 1.81));}
  ASSERT_TRUE(rec.isReachable(up(0.5, 0.0, 1.81)));   // the cell is queryable ...
  const auto p = planHorizontalStands(rec, ringRequest({up(5.0, 5.0, 1.81)}));
  EXPECT_TRUE(p.out_of_lift_range.empty());          // ... so the target is not out of lift range
  EXPECT_EQ(p.plan.stands.size(), 1U);
}

TEST(HorizontalStands, AnUnusableRequestIsRefusedWithACode)
{
  const Map4D map = ringMap(0.5, 0.7);
  const auto t = up(1.0, 1.0, 0.3);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int k = 0; k < 6; ++k) {
    auto r = ringRequest({t, up(2.0, 2.0, 0.3)});
    if (k == 0) {r.lift_step = 0.0;}
    if (k == 1) {r.lift_max = -1.0;}
    if (k == 2) {r.grid_step = nan;}
    if (k == 3) {r.grid_step = 1e-6;}
    if (k == 4) {r.margin = std::numeric_limits<double>::infinity();}
    if (k == 5) {r.lift_step = nan;}
    EXPECT_FALSE(horizontalRequestProblem(map, r).empty()) << k;
    const auto p = planHorizontalStands(map, r);
    EXPECT_EQ(p.code, HorizontalStandCode::kInvalidRequest) << k;
    EXPECT_FALSE(p.message.empty());
    EXPECT_TRUE(p.plan.stands.empty());
    EXPECT_EQ(p.plan.not_planned.size(), 2U) << k;     // refused, not claimed unreachable
    EXPECT_TRUE(p.plan.unreachable.empty());
    EXPECT_FALSE(chooseHorizontalStation(map, r).found);
  }
  EXPECT_EQ(planHorizontalStands(map, ringRequest({t})).code, HorizontalStandCode::kOk);
}

TEST(HorizontalStands, TheStandCapLeavesTargetsNotPlannedNotUnreachable)
{
  const auto t = patch(-3.0, 3.0, -0.2, 0.2, 0.5, 2.2, true);
  const auto full = planHorizontalStands(sharedMap(), request(t));
  ASSERT_GE(full.plan.stands.size(), 2U);
  const auto capped = planHorizontalStands(sharedMap(), request(t), 1);
  ASSERT_EQ(capped.plan.stands.size(), 1U);
  EXPECT_TRUE(capped.plan.unreachable.empty());      // reachable, just not planned
  EXPECT_FALSE(capped.plan.not_planned.empty());
  std::multiset<size_t> seen(capped.plan.not_planned.begin(), capped.plan.not_planned.end());
  seen.insert(capped.plan.stands[0].targets.begin(), capped.plan.stands[0].targets.end());
  ASSERT_EQ(seen.size(), t.size());
  for (size_t i = 0; i < t.size(); ++i) {EXPECT_EQ(seen.count(i), 1U) << i;}
}

TEST(HorizontalStands, ABaseThatCannotStrafeIsNotChargedATurnToAYawItDoesNotNeed)
{
  const Map4D map = ringMap(0.5, 0.7);
  BaseModel base;
  base.drive = DriveType::kDiff;
  base.can_strafe = false;
  base.footprint = {{-0.4, -0.25}, {0.4, -0.25}, {0.4, 0.25}, {-0.4, 0.25}};
  base.max_linear = 1.0;
  base.max_angular = 1.0;
  auto r = ringRequest({up(0.0, 3.0, 0.3)});
  r.base = base;
  const Stand from{Eigen::Vector2d(0.0, 0.0), M_PI / 2};   // already facing the target
  r.from = from;
  const auto p = planHorizontalStands(map, r);
  ASSERT_EQ(p.plan.stands.size(), 1U);
  const Stand & to = p.plan.stands[0].stand;
  const double dist = (to.arm_xy - from.arm_xy).norm();
  const double heading = std::atan2(to.arm_xy.y(), to.arm_xy.x());
  auto wrap = [](double a) {return std::abs(std::atan2(std::sin(a), std::cos(a)));};
  const double turn = std::min(wrap(heading - from.yaw), wrap(heading + M_PI - from.yaw));
  // turn toward the stand, drive: no second turn to yaw 0 (the stand's yaw is free)
  EXPECT_NEAR(p.plan.stands[0].travel_s, dist / base.max_linear + turn / base.max_angular, 1e-9);
  EXPECT_NEAR(p.plan.total_travel_s, p.plan.stands[0].travel_s, 1e-12);
  EXPECT_NEAR(travelTime(base, from, to), p.plan.stands[0].travel_s, 1e-9);   // the reported yaw is the arrival yaw
}
