#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "reachability/core/builder.hpp"
#include "reachability/core/global_stands.hpp"
#include "reachability/core/stand_verifier.hpp"
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

std::shared_ptr<FunctionStandVerifier> always(VerifyReason r = VerifyReason::kOk)
{
  return std::make_shared<FunctionStandVerifier>(
    [r](const Stand &, double, const Pose &) {return r == VerifyReason::kOk ? ok() : no(r);});
}
}  // namespace

TEST(StandVerifier, ReasonsAreNamed)
{
  EXPECT_STREQ(toString(VerifyReason::kOk), "ok");
  EXPECT_STREQ(toString(VerifyReason::kIkFailed), "ik_failed");
  EXPECT_STREQ(toString(VerifyReason::kInCollision), "in_collision");
  EXPECT_STREQ(toString(VerifyReason::kOutOfLimits), "out_of_limits");
}

TEST(StandVerifier, MarginIsTheShareOfLiftHeightsThatSolve)
{
  FunctionStandVerifier v([](const Stand &, double h, const Pose &) {return h <= 0.5 ? ok() : no(VerifyReason::kIkFailed);});
  const auto targets = wall({0.0}, {1.0});
  const auto res = verifyStand(v, Stand{}, targets, {0}, {{0.2, 0.4, 0.6, 0.8}});
  ASSERT_EQ(res.targets.size(), 1U);
  const auto & r = res.targets[0];
  EXPECT_TRUE(r.confirmed);
  EXPECT_EQ(r.reason, VerifyReason::kOk);
  EXPECT_EQ(r.height_probes, 4U);
  EXPECT_EQ(r.height_ok, 2U);
  EXPECT_DOUBLE_EQ(r.lift_margin, 0.5);
  EXPECT_DOUBLE_EQ(r.margin, 0.5);
  EXPECT_DOUBLE_EQ(r.best_height, 0.2);
  EXPECT_EQ(r.joints.size(), 6U);
  EXPECT_EQ(res.confirmed, 1U);
  ASSERT_EQ(res.bands.size(), 1U);
  EXPECT_DOUBLE_EQ(res.bands[0].lift_height, 0.2);
  EXPECT_EQ(v.calls(), 4U);   // one batch per distinct height
}

TEST(StandVerifier, OffsetJitterLowersTheMargin)
{
  const Stand at{Eigen::Vector2d(1.0, 2.0), 0.0};
  auto near = [&at](double tol) {
      return [at, tol](const Stand & s, double, const Pose &) {
               return (s.arm_xy - at.arm_xy).norm() <= tol ? ok() : no(VerifyReason::kInCollision);
             };
    };
  const auto targets = wall({0.0}, {1.0});
  VerifyOptions o;
  o.jitter_m = 0.05;
  FunctionStandVerifier tight(near(0.02));   // fine at the stand, nowhere 5 cm away
  auto a = verifyStand(tight, at, targets, {0}, {{0.5}}, o);
  EXPECT_TRUE(a.targets[0].confirmed);
  EXPECT_DOUBLE_EQ(a.targets[0].offset_margin, 0.0);
  EXPECT_GT(a.targets[0].margin, 0.0);
  EXPECT_LT(a.targets[0].margin, 0.01);
  FunctionStandVerifier loose(near(0.2));
  auto b = verifyStand(loose, at, targets, {0}, {{0.5}}, o);
  EXPECT_DOUBLE_EQ(b.targets[0].offset_margin, 1.0);
  EXPECT_DOUBLE_EQ(b.targets[0].margin, 1.0);
}

TEST(StandVerifier, RefusalsKeepTheirTypedReasonAndTheWorstOneWins)
{
  FunctionStandVerifier v([](const Stand &, double h, const Pose &) {
      return h < 0.5 ? no(VerifyReason::kIkFailed) : no(VerifyReason::kInCollision);});
  const auto targets = wall({0.0}, {1.0});
  const auto res = verifyStand(v, Stand{}, targets, {0}, {{0.2, 0.8}});
  EXPECT_FALSE(res.targets[0].confirmed);
  EXPECT_EQ(res.targets[0].reason, VerifyReason::kInCollision);
  EXPECT_EQ(res.confirmed, 0U);
  EXPECT_DOUBLE_EQ(res.coverage, 0.0);
  EXPECT_DOUBLE_EQ(res.targets[0].margin, 0.0);
  // no heights at all: nothing to try, so ik_failed
  const auto none = verifyStand(v, Stand{}, targets, {0}, {{}});
  EXPECT_EQ(none.targets[0].reason, VerifyReason::kIkFailed);
}

TEST(StandVerifier, ShortAnswerFromABackendIsAnErrorNeverAnOk)
{
  struct Short : StandVerifier
  {
    std::vector<TargetVerdict> verify(const Stand &, double, const std::vector<Pose> &) override {return {};}
  } v;
  const auto targets = wall({0.0}, {1.0});
  const auto res = verifyStand(v, Stand{}, targets, {0}, {{0.5}});
  EXPECT_FALSE(res.targets[0].confirmed);
  EXPECT_EQ(res.targets[0].reason, VerifyReason::kError);
}

TEST(StandVerifier, EmptyInputsAreHandled)
{
  FunctionStandVerifier v([](const Stand &, double, const Pose &) {return ok();});
  const auto res = verifyStand(v, Stand{}, {}, {}, {});
  EXPECT_TRUE(res.targets.empty());
  EXPECT_DOUBLE_EQ(res.coverage, 1.0);
  EXPECT_EQ(v.calls(), 0U);
  EXPECT_TRUE(pickProbeHeights({}, 4).empty());
  EXPECT_EQ(pickProbeHeights({1, 2, 3, 4, 5, 6, 7}, 3), (std::vector<double>{1, 4, 7}));
}

TEST(StandSelectionVerified, AnAcceptingVerifierChangesNothingButAddsTheReport)
{
  const auto r = request(wall({-1.0, 0.0, 1.0}, {0.8, 1.4}));
  const auto base = selectStands(sharedMap(), r);
  StandSelectionOptions o;
  o.verifier = always();
  const auto ver = selectStands(sharedMap(), r, std::nullopt, o);
  ASSERT_TRUE(ver.success) << ver.message;
  ASSERT_EQ(ver.stands.size(), base.stands.size());
  for (size_t i = 0; i < ver.stands.size(); ++i) {
    EXPECT_EQ(ver.stands[i].candidate_index, base.stands[i].candidate_index);
    EXPECT_TRUE(ver.stands[i].verified);
    EXPECT_DOUBLE_EQ(ver.stands[i].verification.coverage, 1.0);
    EXPECT_FALSE(ver.stands[i].verification.bands.empty());
  }
  EXPECT_TRUE(ver.verification.enabled);
  EXPECT_TRUE(ver.verification.refused.empty());
  EXPECT_DOUBLE_EQ(ver.verification.optimism(), 0.0);
  EXPECT_GT(ver.verification.verified_pairs, 0U);
  EXPECT_TRUE(ver.verification.converged);
  EXPECT_FALSE(base.stands[0].verified);
}

TEST(StandSelectionVerified, AStandRm4dLikesButTheVerifierRejectsIsDroppedAndTheCoverRecomputed)
{
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto base = selectStands(sharedMap(), r);
  ASSERT_TRUE(base.success);
  ASSERT_EQ(base.stands.size(), 1U);
  const Eigen::Vector2d liked = base.stands[0].stand.arm_xy;
  // an obstacle makes every stand within 0.35 m of the liked one collide
  auto v = std::make_shared<FunctionStandVerifier>([liked](const Stand & s, double, const Pose &) {
      return (s.arm_xy - liked).norm() < 0.35 ? no(VerifyReason::kInCollision) : ok();});
  StandSelectionOptions o;
  o.verifier = v;
  const auto sel = selectStands(sharedMap(), r, std::nullopt, o);
  ASSERT_TRUE(sel.success) << sel.message;
  ASSERT_FALSE(sel.stands.empty());
  std::set<size_t> seen;
  for (const auto & st : sel.stands) {
    EXPECT_GE((st.stand.arm_xy - liked).norm(), 0.35) << "a rejected stand was kept";
    EXPECT_DOUBLE_EQ(st.verification.coverage, 1.0);
    seen.insert(st.targets.begin(), st.targets.end());
  }
  EXPECT_EQ(seen.size(), r.targets_world.size());   // the cover was recomputed over the confirmed pairs
  EXPECT_TRUE(sel.unreachable.empty());
  ASSERT_FALSE(sel.verification.refused.empty());
  EXPECT_EQ(sel.verification.refused[0].reason, VerifyReason::kInCollision);
  EXPECT_GT(sel.verification.optimism(), 0.0);
  EXPECT_GE(sel.verification.rounds, 2U);
}

TEST(StandSelectionVerified, TargetsIkRefusesEverywhereAreReportedWithTheirReason)
{
  const auto targets = wall({-0.3, 0.0, 0.3}, {0.8, 1.4});
  const auto r = request(targets);
  // z = 1.4 is refused by IK, y = 0.3 at z = 0.8 collides: neither at any stand
  auto v = std::make_shared<FunctionStandVerifier>([](const Stand &, double, const Pose & t) {
      if (t.translation().z() > 1.0) {return no(VerifyReason::kIkFailed);}
      if (t.translation().y() > 0.2) {return no(VerifyReason::kInCollision);}
      return ok();});
  StandSelectionOptions o;
  o.verifier = v;
  const auto sel = selectStands(sharedMap(), r, std::nullopt, o);
  EXPECT_FALSE(sel.success);
  EXPECT_EQ(sel.code, StandSelectionCode::kUnreachableTargets);
  // targets: y-major, so index = 2 * iy + iz; z = 1.4 are the odd ones, and index 4 is (0.3, 0.8)
  EXPECT_EQ(sel.unreachable, (std::vector<size_t>{1, 3, 4, 5}));
  EXPECT_EQ(sel.rejected_by_verifier, sel.unreachable);
  std::map<size_t, VerifyReason> why;
  for (const auto & p : sel.verification.refused) {why[p.target] = p.reason;}
  EXPECT_EQ(why[1], VerifyReason::kIkFailed);
  EXPECT_EQ(why[3], VerifyReason::kIkFailed);
  EXPECT_EQ(why[4], VerifyReason::kInCollision);
  EXPECT_NE(sel.message.find("refused by IK"), std::string::npos);
  std::set<size_t> covered;
  for (const auto & st : sel.stands) {covered.insert(st.targets.begin(), st.targets.end());}
  EXPECT_EQ(covered, (std::set<size_t>{0, 2}));
}

TEST(StandSelectionVerified, EverythingRefusedGivesNoCandidates)
{
  const auto r = request(wall({0.0}, {1.0}));
  StandSelectionOptions o;
  o.verifier = always(VerifyReason::kOutOfLimits);
  const auto sel = selectStands(sharedMap(), r, std::nullopt, o);
  EXPECT_FALSE(sel.success);
  EXPECT_EQ(sel.code, StandSelectionCode::kNoCandidates);
  EXPECT_TRUE(sel.stands.empty());
  EXPECT_EQ(sel.unreachable.size(), 1U);
  EXPECT_EQ(sel.rejected_by_verifier.size(), 1U);
  EXPECT_FALSE(sel.verification.refused.empty());
  for (const auto & p : sel.verification.refused) {EXPECT_EQ(p.reason, VerifyReason::kOutOfLimits);}
}

TEST(StandSelectionVerified, IsDeterministic)
{
  const auto r = request(wall({-1.0, 0.0, 1.0}, {0.8, 1.4}));
  auto make = [] {
      return std::make_shared<FunctionStandVerifier>([](const Stand & s, double h, const Pose &) {
          return (std::fmod(std::abs(s.arm_xy.y()) * 7.0 + h, 1.0) < 0.3) ? no(VerifyReason::kIkFailed) : ok();});
    };
  StandSelectionOptions o;
  o.verifier = make();
  const auto a = selectStands(sharedMap(), r, std::nullopt, o);
  o.verifier = make();
  const auto b = selectStands(sharedMap(), r, std::nullopt, o);
  ASSERT_EQ(a.stands.size(), b.stands.size());
  for (size_t i = 0; i < a.stands.size(); ++i) {
    EXPECT_EQ(a.stands[i].candidate_index, b.stands[i].candidate_index);
    EXPECT_EQ(a.stands[i].targets, b.stands[i].targets);
    EXPECT_DOUBLE_EQ(a.stands[i].mean_margin, b.stands[i].mean_margin);
  }
  EXPECT_EQ(a.unreachable, b.unreachable);
  EXPECT_EQ(a.verification.refused.size(), b.verification.refused.size());
  EXPECT_EQ(a.verification.verifier_calls, b.verification.verifier_calls);
}

TEST(StandSelectionVerified, EmptyInputNeverCallsTheVerifier)
{
  auto v = always();
  StandSelectionOptions o;
  o.verifier = v;
  const auto sel = selectStands(sharedMap(), request({}), std::nullopt, o);
  EXPECT_TRUE(sel.success);
  EXPECT_EQ(sel.code, StandSelectionCode::kNoTargets);
  EXPECT_EQ(v->calls(), 0U);
}

TEST(StandSelectionVerified, TheRoundLimitIsReportedNotHidden)
{
  const auto r = request(wall({-0.3, 0.0, 0.3}, {0.8, 1.4}));
  const auto base = selectStands(sharedMap(), r);
  const Eigen::Vector2d liked = base.stands[0].stand.arm_xy;
  StandSelectionOptions o;
  o.verifier = std::make_shared<FunctionStandVerifier>([liked](const Stand & s, double, const Pose &) {
      return (s.arm_xy - liked).norm() < 0.35 ? no(VerifyReason::kIkFailed) : ok();});
  o.verify.max_rounds = 1;
  const auto sel = selectStands(sharedMap(), r, std::nullopt, o);
  EXPECT_FALSE(sel.verification.converged);
  EXPECT_EQ(sel.verification.rounds, 1U);
}
