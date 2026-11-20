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

TargetVerdict ok()
{
  TargetVerdict v;
  v.reason = VerifyReason::kOk;
  v.joints = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  return v;
}
TargetVerdict no(VerifyReason r) {TargetVerdict v; v.reason = r; return v;}

VerifiedPlanOptions with(FunctionStandVerifier::Fn fn)
{
  VerifiedPlanOptions o;
  o.verifier = std::make_shared<FunctionStandVerifier>(std::move(fn));
  return o;
}

std::set<size_t> covered(const StandPlan & p)
{
  std::set<size_t> s;
  for (const auto & a : p.stands) {s.insert(a.targets.begin(), a.targets.end());}
  return s;
}
}  // namespace

TEST(AvoidStations, ChooseStationSkipsAvoidedGridPoints)
{
  auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto first = chooseStation(sharedMap(), r);
  ASSERT_TRUE(first.found);
  r.avoid_stations.push_back(Eigen::Vector2d(first.x, first.y));
  const auto second = chooseStation(sharedMap(), r);
  ASSERT_TRUE(second.found);
  EXPECT_GE((Eigen::Vector2d(second.x, second.y) - Eigen::Vector2d(first.x, first.y)).norm(), r.avoid_radius);
}

TEST(PlanStandsVerified, AnAcceptingVerifierGivesTheSameStandsAndTargets)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const auto plain = planStands(sharedMap(), r);
  const auto v = planStandsVerified(
    sharedMap(), r, std::nullopt, 32, StandStrategy::Greedy, {},
    with([](const Stand &, double, const Pose &) {return ok();}));
  ASSERT_EQ(v.plan.stands.size(), plain.stands.size());
  for (size_t i = 0; i < plain.stands.size(); ++i) {
    EXPECT_EQ(v.plan.stands[i].targets, plain.stands[i].targets) << i;
    EXPECT_DOUBLE_EQ(v.verification[i].coverage, 1.0);
  }
  EXPECT_EQ(v.plan.unreachable, plain.unreachable);
  EXPECT_TRUE(v.refused.empty());
  EXPECT_TRUE(v.rejected_by_verifier.empty());
  EXPECT_TRUE(v.converged);
  EXPECT_EQ(v.rounds, 1U);
  EXPECT_GT(v.verifier_calls, 0U);
}

TEST(PlanStandsVerified, ARefusedStandIsReplacedAndEveryTargetStillCovered)
{
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto plain = planStands(sharedMap(), r);
  ASSERT_EQ(plain.stands.size(), 1U);
  const Eigen::Vector2d liked = plain.stands[0].stand.arm_xy;
  const auto v = planStandsVerified(
    sharedMap(), r, std::nullopt, 32, StandStrategy::Greedy, {},
    with([liked](const Stand & s, double, const Pose &) {
      return (s.arm_xy - liked).norm() < 0.35 ? no(VerifyReason::kInCollision) : ok();}));
  ASSERT_FALSE(v.plan.stands.empty());
  for (const auto & a : v.plan.stands) {EXPECT_GE((a.stand.arm_xy - liked).norm(), 0.35);}
  EXPECT_EQ(covered(v.plan).size(), r.targets_world.size());
  EXPECT_TRUE(v.rejected_by_verifier.empty());
  EXPECT_TRUE(v.plan.unreachable.empty());
  ASSERT_FALSE(v.refused.empty());
  EXPECT_EQ(v.refused[0].reason, VerifyReason::kInCollision);
  EXPECT_GE(v.rounds, 2U);
  ASSERT_EQ(v.verification.size(), v.plan.stands.size());
}

TEST(PlanStandsVerified, TargetsNoStandConfirmsAreReportedNotCovered)
{
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  // z = 1.4 fails IK everywhere, (y = 0.3, z = 0.8) collides everywhere; index = 2 * iy + iz
  const auto v = planStandsVerified(
    sharedMap(), r, std::nullopt, 32, StandStrategy::Greedy, {},
    with([](const Stand &, double, const Pose & t) {
      if (t.translation().z() > 1.0) {return no(VerifyReason::kIkFailed);}
      if (t.translation().y() > 0.2) {return no(VerifyReason::kInCollision);}
      return ok();}));
  EXPECT_EQ(covered(v.plan), (std::set<size_t>{0, 2}));
  EXPECT_EQ(v.rejected_by_verifier, (std::vector<size_t>{1, 3, 4, 5}));
  EXPECT_TRUE(v.plan.unreachable.empty());   // RM4D reached them; the verifier is why they are out
  std::set<size_t> refused_targets;
  for (const auto & p : v.refused) {refused_targets.insert(p.target);}
  EXPECT_TRUE(refused_targets.count(1) && refused_targets.count(4));
}

TEST(PlanStandsVerified, EverythingRefusedCoversNothingAndTerminates)
{
  const auto r = request(wall({0.0}, {1.0}));
  VerifiedPlanOptions o = with([](const Stand &, double, const Pose &) {return no(VerifyReason::kIkFailed);});
  o.max_rounds = 4;
  const auto v = planStandsVerified(sharedMap(), r, std::nullopt, 32, StandStrategy::Greedy, {}, o);
  EXPECT_TRUE(v.plan.stands.empty());
  EXPECT_EQ(v.rejected_by_verifier, (std::vector<size_t>{0}));
  EXPECT_LE(v.rounds, 4U);
}

TEST(PlanStandsVerified, WorksWithSetCoverAndKeepsTheConfirmedBands)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  const auto v = planStandsVerified(
    sharedMap(), r, std::nullopt, 32, StandStrategy::SetCover, {},
    with([](const Stand &, double h, const Pose &) {return h > 0.0 ? ok() : no(VerifyReason::kOutOfLimits);}));
  ASSERT_FALSE(v.plan.stands.empty());
  EXPECT_EQ(covered(v.plan).size() + v.plan.unreachable.size() + v.rejected_by_verifier.size(), r.targets_world.size());
  for (const auto & a : v.plan.stands) {
    std::set<size_t> in_bands;
    for (const auto & b : a.station.plan.bands) {
      EXPECT_GT(b.lift_height, 0.0);
      in_bands.insert(b.pose_indices.begin(), b.pose_indices.end());
    }
    EXPECT_EQ(in_bands, std::set<size_t>(a.targets.begin(), a.targets.end()));
  }
}

TEST(PlanStandsVerified, TravelIsRecomputedForTheFinalOrderWithABase)
{
  auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  BaseModel b;
  b.drive = DriveType::kOmni;
  b.can_strafe = true;
  b.footprint = {{-0.4, -0.25}, {0.4, -0.25}, {0.4, 0.25}, {-0.4, 0.25}};
  r.base = b;
  const Stand start{Eigen::Vector2d(0.0, 0.0), 0.0};
  const auto v = planStandsVerified(
    sharedMap(), r, start, 32, StandStrategy::Greedy, {},
    with([](const Stand &, double, const Pose &) {return ok();}));
  ASSERT_GE(v.plan.stands.size(), 2U);
  double total = 0.0;
  Stand at = start;
  for (const auto & a : v.plan.stands) {
    EXPECT_NEAR(a.travel_s, travelTime(b, at, a.stand), 1e-12);
    total += a.travel_s;
    at = a.stand;
  }
  EXPECT_NEAR(v.plan.total_travel_s, total, 1e-9);
}

TEST(PlanStandsVerified, NoVerifierIsPlanStands)
{
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto plain = planStands(sharedMap(), r);
  const auto v = planStandsVerified(sharedMap(), r, std::nullopt, 32, StandStrategy::Greedy, {}, {});
  ASSERT_EQ(v.plan.stands.size(), plain.stands.size());
  EXPECT_TRUE(v.verification.empty());
}
