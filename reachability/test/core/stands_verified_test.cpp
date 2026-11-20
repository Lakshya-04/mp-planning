#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <tuple>

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

// ---- cost: counting fake verifiers ------------------------------------------------------------------------

namespace
{
// A verifier that counts the poses it is asked about (the IK cost) next to the batches FunctionStandVerifier counts.
struct Counting
{
  std::shared_ptr<FunctionStandVerifier> verifier;
  std::shared_ptr<size_t> poses = std::make_shared<size_t>(0);
};

Counting counting(const Eigen::Vector2d & refused_center, double refused_radius)
{
  Counting c;
  auto poses = c.poses;
  c.verifier = std::make_shared<FunctionStandVerifier>(
    [poses, refused_center, refused_radius](const Stand & s, double h, const Pose &) {
      ++*poses;
      if ((s.arm_xy - refused_center).norm() < refused_radius) {return no(VerifyReason::kInCollision);}
      return h < 0.2 ? no(VerifyReason::kIkFailed) : ok();   // the lowest heights fail whatever the stand
    });
  return c;
}
}  // namespace

TEST(PlanStandsVerifiedCost, StopAtFirstOkSendsFewerPosesAndGivesTheSameCover)
{
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto plain = planStands(sharedMap(), r);
  ASSERT_FALSE(plain.stands.empty());
  const Eigen::Vector2d liked = plain.stands[0].stand.arm_xy;

  auto run = [&](bool early, size_t * poses, size_t * batches) {
      auto c = counting(liked, 0.35);
      VerifiedPlanOptions o;
      o.verifier = c.verifier;
      o.verify.max_lift_probes = 4;
      o.verify.stop_at_first_ok = early;
      auto v = planStandsVerified(sharedMap(), r, std::nullopt, 32, StandStrategy::Greedy, {}, o);
      *poses = *c.poses;
      *batches = c.verifier->calls();
      EXPECT_EQ(v.verifier_poses, *c.poses);   // the plan's own count agrees with the fake's
      EXPECT_EQ(v.verifier_calls, *batches);
      return v;
    };
  size_t full_poses = 0, full_batches = 0, fast_poses = 0, fast_batches = 0;
  const auto full = run(false, &full_poses, &full_batches);
  const auto fast = run(true, &fast_poses, &fast_batches);

  EXPECT_LT(fast_poses, full_poses) << "full " << full_poses << " fast " << fast_poses;
  EXPECT_LE(fast_batches, full_batches);
  // the same result: stands, their targets, the reported rejections and the confirmed lift bands' heights
  ASSERT_EQ(fast.plan.stands.size(), full.plan.stands.size());
  for (size_t i = 0; i < full.plan.stands.size(); ++i) {
    EXPECT_EQ(fast.plan.stands[i].targets, full.plan.stands[i].targets) << i;
    EXPECT_EQ(fast.plan.stands[i].stand.arm_xy, full.plan.stands[i].stand.arm_xy) << i;
    std::set<double> hf, hl;
    for (const auto & b : fast.plan.stands[i].station.plan.bands) {hf.insert(b.lift_height);}
    for (const auto & b : full.plan.stands[i].station.plan.bands) {hl.insert(b.lift_height);}
    EXPECT_FALSE(hf.empty());
    EXPECT_GE(*hf.begin(), 0.2 - 1e-9);   // the failing low heights are never reported as confirmed
  }
  EXPECT_EQ(fast.rejected_by_verifier, full.rejected_by_verifier);
  EXPECT_EQ(fast.refused.size(), full.refused.size());
  EXPECT_EQ(covered(fast.plan), covered(full.plan));
  std::printf("[ cost ] poses sent: %zu full -> %zu stop_at_first_ok; batches %zu -> %zu\n",
    full_poses, fast_poses, full_batches, fast_batches);
}

TEST(PlanStandsVerifiedCost, TheVerifierCallBudgetStopsTheRunAndSaysSo)
{
  const auto r = request(wall({-3.0, -1.5, 0.0, 1.5, 3.0}, {0.8, 1.4}));
  VerifiedPlanOptions o;
  auto c = counting(Eigen::Vector2d(1e6, 1e6), 0.1);   // refuses nothing
  o.verifier = c.verifier;
  o.verify.max_verifier_calls = 2;
  const auto v = planStandsVerified(sharedMap(), r, std::nullopt, 32, StandStrategy::Greedy, {}, o);
  EXPECT_FALSE(v.converged);
  // the budget is checked before each stand, so a stand in flight may overshoot it by its own batches only
  EXPECT_LE(v.verifier_calls, 2U + 3U * o.verify.max_lift_probes);
  // stands that were not verified are not reported as covering anything
  EXPECT_LT(covered(v.plan).size(), r.targets_world.size());
  EXPECT_FALSE(v.plan.not_planned.empty());
}

TEST(SelectStandsCost, StopAtFirstOkSendsFewerPosesAndCoversTheSameTargets)
{
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto first = selectStands(sharedMap(), r, std::nullopt, {});
  ASSERT_FALSE(first.stands.empty());
  const Eigen::Vector2d liked = first.stands[0].stand.arm_xy;
  auto run = [&](bool early, size_t * poses) {
      auto c = counting(liked, 0.35);
      StandSelectionOptions o;
      o.verifier = c.verifier;
      o.verify.max_lift_probes = 4;
      o.verify.stop_at_first_ok = early;
      auto s = selectStands(sharedMap(), r, std::nullopt, o);
      *poses = *c.poses;
      EXPECT_EQ(s.verification.verifier_poses, *c.poses);
      return s;
    };
  size_t full_poses = 0, fast_poses = 0;
  const auto full = run(false, &full_poses);
  const auto fast = run(true, &fast_poses);
  EXPECT_LT(fast_poses, full_poses);
  EXPECT_EQ(fast.unreachable, full.unreachable);
  EXPECT_EQ(fast.rejected_by_verifier, full.rejected_by_verifier);
  auto cover = [](const StandSelection & s) {
      std::set<size_t> t;
      for (const auto & st : s.stands) {t.insert(st.targets.begin(), st.targets.end());}
      return t;
    };
  EXPECT_EQ(cover(fast), cover(full));
  EXPECT_EQ(cover(fast).size(), r.targets_world.size());
  EXPECT_TRUE(fast.verification.converged);
  std::printf("[ cost ] selectStands poses sent: %zu full -> %zu stop_at_first_ok\n", full_poses, fast_poses);
}

TEST(SelectStandsCost, EveryPairIsVerifiedAtMostOnceAcrossRounds)
{
  // a stand refused for everything forces several rounds; no (stand, target) pair may be sent twice
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto first = selectStands(sharedMap(), r, std::nullopt, {});
  ASSERT_FALSE(first.stands.empty());
  std::map<std::tuple<double, double, double, double>, int> seen;
  size_t repeats = 0;
  const Eigen::Vector2d liked = first.stands[0].stand.arm_xy;
  StandSelectionOptions o;
  o.verify.max_lift_probes = 1;   // one height per pair, so a repeated key is a repeated pair
  o.verifier = std::make_shared<FunctionStandVerifier>(
    [&](const Stand & s, double h, const Pose & t) {
      if (++seen[{s.arm_xy.x(), s.arm_xy.y(), h, t.translation().y() * 1000.0 + t.translation().z()}] > 1) {++repeats;}
      return (s.arm_xy - liked).norm() < 0.35 ? no(VerifyReason::kInCollision) : ok();
    });
  const auto s = selectStands(sharedMap(), r, std::nullopt, o);
  EXPECT_GE(s.verification.rounds, 2U);
  EXPECT_EQ(repeats, 0U);
}
