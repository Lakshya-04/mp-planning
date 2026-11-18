#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "reachability/core/builder.hpp"
#include "reachability/core/global_stands.hpp"
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

StationRequest request(std::vector<Pose> targets, bool with_base = true)
{
  StationRequest r;
  r.targets_world = std::move(targets);
  r.wall_point = Vec3(3.0, 0.0, 0.0);
  r.wall_normal = Vec3(-1.0, 0.0, 0.0);
  r.arm_mount_offset_z = 0.2;
  r.lift_max = 1.5;
  r.grid_step = 0.2;
  r.lateral_margin = 0.2;
  if (with_base) {
    BaseModel b;
    b.drive = DriveType::kDiff;
    b.can_strafe = false;
    b.footprint = {{-0.4, -0.25}, {0.4, -0.25}, {0.4, 0.25}, {-0.4, 0.25}};
    r.base = b;
  }
  return r;
}

// Every target index appears exactly once across the stands and `unreachable`.
void expectPartition(const StandSelection & s, size_t n)
{
  std::multiset<size_t> seen(s.unreachable.begin(), s.unreachable.end());
  for (const auto & st : s.stands) {seen.insert(st.targets.begin(), st.targets.end());}
  ASSERT_EQ(seen.size(), n);
  for (size_t i = 0; i < n; ++i) {EXPECT_EQ(seen.count(i), 1U) << "target " << i;}
}
}  // namespace

TEST(GlobalStands, AWallTooWideForOneStandNeedsSeveralAndEveryTargetIsCovered)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  ASSERT_FALSE(chooseStation(sharedMap(), r).plan.unreachable.empty());   // one stand cannot do it
  const Stand start{Eigen::Vector2d(0.0, -3.5), 0.0};
  const auto sel = selectStands(sharedMap(), r, start);
  EXPECT_TRUE(sel.success) << sel.message;
  EXPECT_EQ(sel.code, StandSelectionCode::kOk);
  EXPECT_TRUE(sel.unreachable.empty());
  EXPECT_GE(sel.stands.size(), 2U);
  expectPartition(sel, r.targets_world.size());
  for (const auto & s : sel.stands) {
    EXPECT_TRUE(s.lift_bands.unreachable.empty());
    EXPECT_GT(s.mean_margin, 0.0);
    EXPECT_LE(s.mean_margin, 1.0);
    EXPECT_GE(s.footprint_clearance, r.min_clearance - 1e-9);
  }
}

TEST(GlobalStands, StandCountIsWithinTheSetCoverBoundOfTheExistingBoundedSearch)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const Stand start{Eigen::Vector2d(0.0, -3.5), 0.0};
  StandSelectionOptions o;
  const auto sel = selectStands(sharedMap(), r, start, o);
  const auto ref = planStands(sharedMap(), r, start, 32, StandStrategy::SetCover);
  ASSERT_TRUE(sel.success);
  const double n = static_cast<double>(r.targets_world.size());
  double h = 0.0;
  for (int k = 1; k <= static_cast<int>(n); ++k) {h += 1.0 / k;}
  EXPECT_LE(static_cast<double>(sel.stands.size()), (1.0 + o.margin_penalty) * h * static_cast<double>(ref.stands.size()));
  EXPECT_LE(sel.stands.size(), sel.greedy_stands);
}

TEST(GlobalStands, AnUnreachableTargetIsReportedNotDropped)
{
  auto targets = wall({-1.0, 0.0, 1.0}, {0.8, 1.4});
  targets.push_back(wall({0.0}, {6.0})[0]);      // 6 m up: no lift height reaches it
  const auto r = request(targets);
  const auto sel = selectStands(sharedMap(), r, Stand{Eigen::Vector2d(0.0, -1.0), 0.0});
  EXPECT_FALSE(sel.success);
  EXPECT_EQ(sel.code, StandSelectionCode::kUnreachableTargets);
  ASSERT_EQ(sel.unreachable.size(), 1U);
  EXPECT_EQ(sel.unreachable[0], targets.size() - 1);
  EXPECT_FALSE(sel.message.empty());
  EXPECT_FALSE(sel.stands.empty());               // the reachable ones are still planned
  expectPartition(sel, targets.size());
}

TEST(GlobalStands, AllTargetsUnreachableGivesNoCandidates)
{
  const auto r = request(wall({0.0}, {6.0}));
  const auto sel = selectStands(sharedMap(), r);
  EXPECT_FALSE(sel.success);
  EXPECT_EQ(sel.code, StandSelectionCode::kNoCandidates);
  EXPECT_TRUE(sel.stands.empty());
  ASSERT_EQ(sel.unreachable.size(), 1U);
}

TEST(GlobalStands, EmptyInputsAreHandled)
{
  StationRequest r = request({});
  const auto none = selectStands(sharedMap(), r);
  EXPECT_TRUE(none.success);
  EXPECT_EQ(none.code, StandSelectionCode::kNoTargets);
  EXPECT_TRUE(none.stands.empty());
  EXPECT_EQ(none.tour_cost, 0.0);

  const auto r2 = request(wall({0.0}, {1.0}));
  const auto no_cands = selectStands(sharedMap(), r2, std::vector<CandidateStand>{});
  EXPECT_FALSE(no_cands.success);
  EXPECT_EQ(no_cands.code, StandSelectionCode::kNoCandidates);
  EXPECT_EQ(no_cands.unreachable.size(), 1U);

  auto bad = r2;
  bad.lift_step = 0.0;
  EXPECT_EQ(selectStands(sharedMap(), bad).code, StandSelectionCode::kInvalidRequest);
  bad = r2;
  bad.wall_normal = Vec3::UnitZ();
  EXPECT_EQ(selectStands(sharedMap(), bad).code, StandSelectionCode::kInvalidRequest);
  StandSelectionOptions o;
  o.margin_penalty = -1.0;
  EXPECT_EQ(selectStands(sharedMap(), r2, std::nullopt, o).code, StandSelectionCode::kInvalidRequest);
}

TEST(GlobalStands, ASingleTargetNeedsOneStandAndOneLiftBand)
{
  const auto r = request(wall({0.0}, {1.0}));
  const Stand start{Eigen::Vector2d(0.0, 0.0), 0.0};
  const auto sel = selectStands(sharedMap(), r, start);
  ASSERT_TRUE(sel.success) << sel.message;
  ASSERT_EQ(sel.stands.size(), 1U);
  EXPECT_EQ(sel.stands[0].targets, std::vector<size_t>{0});
  EXPECT_EQ(sel.stands[0].lift_bands.bands.size(), 1U);
  EXPECT_NEAR(sel.tour_cost, sel.stands[0].travel_cost, 1e-12);
  EXPECT_NEAR(sel.tour_cost, sel.input_order_cost, 1e-12);
}

TEST(GlobalStands, TheTourIsNeverLongerThanTheCoverOrderAndStartsFromTheBasePose)
{
  const std::vector<std::vector<double>> walls = {
    {-3.0, -1.5, 0.0, 1.5, 3.0}, {-4.5, -3.0, -1.5, 0.0, 1.5, 3.0, 4.5}, {-2.2, -0.9, 0.0, 0.9, 2.2}};
  const std::vector<Stand> starts = {
    {Eigen::Vector2d(0.0, 0.0), 0.0}, {Eigen::Vector2d(0.0, 3.5), 1.0}, {Eigen::Vector2d(1.0, -4.0), -2.0}};
  for (const auto & ys : walls) {
    for (const auto & st : starts) {
      for (bool with_base : {true, false}) {
        const auto r = request(wall(ys, {0.8, 1.4}), with_base);
        const auto sel = selectStands(sharedMap(), r, st);
        ASSERT_TRUE(sel.success) << sel.message;
        EXPECT_LE(sel.tour_cost, sel.input_order_cost + 1e-9);
        double sum = 0.0;
        for (const auto & s : sel.stands) {sum += s.travel_cost;}
        EXPECT_NEAR(sum, sel.tour_cost, 1e-9);
        if (with_base) {
          ASSERT_FALSE(sel.stands.empty());
          EXPECT_NEAR(sel.stands[0].travel_cost, travelTime(*r.base, st, sel.stands[0].stand), 1e-9);
        }
      }
    }
  }
}

TEST(GlobalStands, AStartAtOneEndVisitsTheStandsAlongTheWall)
{
  const auto r = request(wall({-4.5, -3.0, -1.5, 0.0, 1.5, 3.0, 4.5}, {0.8, 1.4}));
  const auto sel = selectStands(sharedMap(), r, Stand{Eigen::Vector2d(0.0, -6.0), 0.0});
  ASSERT_TRUE(sel.success);
  ASSERT_GE(sel.stands.size(), 3U);
  for (size_t i = 1; i < sel.stands.size(); ++i) {
    EXPECT_GT(sel.stands[i].stand.arm_xy.y(), sel.stands[i - 1].stand.arm_xy.y() - 1e-9);
  }
}

TEST(GlobalStands, ResultsAreDeterministic)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const Stand start{Eigen::Vector2d(0.5, 2.0), 0.3};
  StandSelectionOptions o;
  o.time_budget_s = 0.0;   // no clock: bit-for-bit repeatable
  const auto a = selectStands(sharedMap(), r, start, o);
  const auto b = selectStands(sharedMap(), r, start, o);
  ASSERT_EQ(a.stands.size(), b.stands.size());
  for (size_t i = 0; i < a.stands.size(); ++i) {
    EXPECT_EQ(a.stands[i].candidate_index, b.stands[i].candidate_index);
    EXPECT_EQ(a.stands[i].targets, b.stands[i].targets);
    EXPECT_EQ(a.stands[i].stand.arm_xy, b.stands[i].stand.arm_xy);
    EXPECT_EQ(a.stands[i].stand.yaw, b.stands[i].stand.yaw);
    EXPECT_EQ(a.stands[i].travel_cost, b.stands[i].travel_cost);
  }
  EXPECT_EQ(a.tour_cost, b.tour_cost);
  EXPECT_EQ(a.unreachable, b.unreachable);
}

TEST(GlobalStands, ATinyTimeBudgetStillGivesACompleteTourNoLongerThanTheCoverOrder)
{
  const auto r = request(wall({-4.5, -3.0, -1.5, 0.0, 1.5, 3.0, 4.5}, {0.8, 1.4}));
  StandSelectionOptions o;
  o.time_budget_s = 1e-9;
  const auto sel = selectStands(sharedMap(), r, Stand{Eigen::Vector2d(0.0, 4.0), 0.0}, o);
  ASSERT_TRUE(sel.success);
  expectPartition(sel, r.targets_world.size());
  EXPECT_LE(sel.tour_cost, sel.input_order_cost + 1e-9);
}

TEST(GlobalStands, ExplicitCandidatesAreUsedAsGiven)
{
  const auto r = request(wall({-2.0, 0.0, 2.0}, {0.8, 1.4}));
  auto all = makeCandidateStands(r);
  ASSERT_GT(all.size(), 10U);
  for (const auto & c : all) {EXPECT_GE(c.footprint_clearance, r.min_clearance - 1e-9);}
  std::vector<CandidateStand> few(all.begin(), all.begin() + 3);   // three near-wall stands at one end
  const auto sel = selectStands(sharedMap(), r, few);
  for (const auto & s : sel.stands) {EXPECT_LT(s.candidate_index, few.size());}
  expectPartition(sel, r.targets_world.size());
}

TEST(GlobalStands, MarginPenaltyKeepsCoverageComplete)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  for (double w : {0.0, 0.5, 5.0}) {
    StandSelectionOptions o;
    o.margin_penalty = w;
    const auto sel = selectStands(sharedMap(), r, std::nullopt, o);
    EXPECT_TRUE(sel.success) << w;
    expectPartition(sel, r.targets_world.size());
  }
}
