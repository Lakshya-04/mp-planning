#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "reachability/core/column_ik_chain.hpp"

using namespace reachability;  // NOLINT

namespace
{
// 2-link planar arm, both links 1 m. The branch (elbow sign) is q2 > 0 or < 0.
JointVec planar(double x, double y, int branch)
{
  const double c = (x * x + y * y - 2.0) / 2.0;
  const double q2 = branch * std::acos(std::clamp(c, -1.0, 1.0));
  const double q1 = std::atan2(y, x) - std::atan2(std::sin(q2), 1.0 + c);
  JointVec q(2);
  q << q1, q2;
  return q;
}

Pose at(double x, double y, double z = 0.0)
{
  Pose p = Pose::Identity();
  p.translation() = Vec3(x, y, z);
  return p;
}

// Reaches the branch of the seed's elbow sign. `blocked_x`: that branch "+" is not available for x past it
// (a joint limit), so the IK answers with the other branch. `quirk_q1`: a seed with q1 above it makes the
// solver converge to "-" (a basin effect that a different seed avoids).
struct FakePlanarIk
{
  double blocked_x{1e9};
  double quirk_q1{1e9};
  std::optional<JointVec> operator()(const Pose & t, const JointVec & seed) const
  {
    const double x = t.translation().x(), y = t.translation().y();
    if (x * x + y * y > 3.99) {return std::nullopt;}
    int branch = seed[1] >= 0.0 ? 1 : -1;
    if (branch == 1 && (x > blocked_x || seed[0] > quirk_q1)) {branch = -1;}
    return planar(x, y, branch);
  }
};

ColumnIkChainParams planarParams()
{
  ColumnIkChainParams p;
  p.joint_kinds = {JointKind::kRevolute, JointKind::kRevolute};
  p.ratio_limit_revolute = 10.0;
  p.branch_joints = {1};
  p.initial_seed = planar(0.8, 0.5, 1);
  return p;
}

std::vector<Pose> alongX(double x0, double x1, double y)
{
  std::vector<Pose> v;
  for (double x = x0; x <= x1 + 1e-9; x += 0.05) {v.push_back(at(x, y));}
  return v;
}

std::vector<Pose> alongY(double x, double y0, double y1)
{
  std::vector<Pose> v;
  for (double y = y0; y <= y1 + 1e-9; y += 0.05) {v.push_back(at(x, y));}
  return v;
}
}  // namespace

TEST(ColumnIkChain, ContinuousColumnIsWhole)
{
  ColumnIkChain chain(planarParams(), FakePlanarIk{});
  const auto r = chain.solve(alongX(0.8, 1.5, 0.5));
  ASSERT_TRUE(r.complete);
  EXPECT_TRUE(r.continuous());
  EXPECT_EQ(r.split_count, 0u);
  EXPECT_EQ(r.branch_changes, 0u);
  EXPECT_EQ(r.reseeds, 0u);
  EXPECT_EQ(r.failure, PoseFailure::kNone);
  EXPECT_FALSE(r.failure_pose.has_value());
  EXPECT_GT(r.max_continuity_ratio, 0.0);
  EXPECT_LT(r.max_continuity_ratio, 10.0);
  EXPECT_EQ(r.steps.size(), 15u);
}

TEST(ColumnIkChain, ElbowFlipIsABranchBreak)
{
  FakePlanarIk ik;
  ik.blocked_x = 1.22;   // past x = 1.22 only the other elbow is available
  ColumnIkChain chain(planarParams(), ik);
  const auto r = chain.solve(alongX(0.8, 1.5, 0.5));
  ASSERT_TRUE(r.complete);
  EXPECT_FALSE(r.continuous());
  EXPECT_EQ(r.split_count, 1u);          // one flip; after it the seed follows the new branch
  EXPECT_EQ(r.branch_changes, 1u);
  EXPECT_EQ(r.failure, PoseFailure::kBranchBreak);
  ASSERT_TRUE(r.failure_pose.has_value());
  const auto & s = r.steps[*r.failure_pose];
  EXPECT_TRUE(s.branch_break);
  EXPECT_TRUE(s.branch_changed);
  EXPECT_GT(s.normalized_ratio, 1.0);
  EXPECT_GT(r.max_continuity_ratio, 30.0);   // dq2 of about 3.4 rad over a 0.05 m step
  EXPECT_EQ(*r.failure_pose, 9u);   // x = 0.8 + 9 * 0.05 = 1.25, the first pose past the block
}

TEST(ColumnIkChain, ReseedRescuesASeedDependentFlip)
{
  FakePlanarIk ik;
  ik.quirk_q1 = -0.1;   // the previous solution's q1 passes -0.1 around y = 0.85
  const auto poses = alongY(0.6, 0.2, 1.0);

  ColumnIkChain plain(planarParams(), ik);
  const auto broken = plain.solve(poses);
  ASSERT_TRUE(broken.complete);
  EXPECT_EQ(broken.split_count, 1u);

  auto p = planarParams();
  p.folded_seeds = {planar(0.6, 0.5, 1)};   // "+" branch with q1 under the quirk threshold
  p.folded_seeds[0][0] = -0.5;
  ColumnIkChain rescued(p, ik);
  const auto r = rescued.solve(poses);
  ASSERT_TRUE(r.complete);
  EXPECT_TRUE(r.continuous());
  EXPECT_EQ(r.split_count, 0u);
  EXPECT_EQ(r.branch_changes, 0u);
  EXPECT_GE(r.reseeds, 1u);
  EXPECT_LT(r.max_continuity_ratio, 10.0);
}

TEST(ColumnIkChain, ReseedCannotRescueAnInfeasibleBranch)
{
  FakePlanarIk ik;
  ik.blocked_x = 1.22;
  auto p = planarParams();
  p.folded_seeds = {planar(0.8, 0.5, 1)};
  ColumnIkChain chain(p, ik);
  const auto r = chain.solve(alongX(0.8, 1.5, 0.5));
  ASSERT_TRUE(r.complete);
  EXPECT_EQ(r.split_count, 1u);
}

TEST(ColumnIkChain, UnreachablePoseStopsTheChain)
{
  ColumnIkChain chain(planarParams(), FakePlanarIk{});
  auto poses = alongX(0.8, 1.2, 0.5);
  poses.push_back(at(3.0, 0.0));   // out of reach
  poses.push_back(at(1.0, 0.5));
  const auto r = chain.solve(poses);
  EXPECT_FALSE(r.complete);
  EXPECT_FALSE(r.continuous());
  EXPECT_EQ(r.failure, PoseFailure::kRefused);
  ASSERT_TRUE(r.failure_pose.has_value());
  EXPECT_EQ(*r.failure_pose, poses.size() - 2);
  EXPECT_EQ(r.steps.size(), poses.size() - 1);   // the chain stops there
  EXPECT_FALSE(r.steps.back().solved);
}

namespace
{
// [revolute, prismatic lift, continuous wrist]; the lift follows z and the wrist follows x, by gains.
struct FakeLiftIk
{
  double lift_gain{1.0};
  double wrist_gain{1.0};
  std::optional<JointVec> operator()(const Pose & t, const JointVec &) const
  {
    JointVec q(3);
    q << 0.0, lift_gain * t.translation().z(), wrist_gain * t.translation().x();
    return q;
  }
};

ColumnIkChainParams liftParams()
{
  ColumnIkChainParams p;
  p.joint_kinds = {JointKind::kRevolute, JointKind::kPrismatic, JointKind::kContinuous};
  p.ratio_limit_revolute = 6.0;
  p.ratio_limit_prismatic = 2.0;
  p.ratio_limit_continuous = 8.0;
  return p;
}

std::vector<Pose> diagonal(int n, double step)
{
  std::vector<Pose> v;
  for (int i = 0; i < n; ++i) {v.push_back(at(step * i, 0.0, step * i));}
  return v;
}
}  // namespace

TEST(ColumnIkChain, LiftTravelIsAccounted)
{
  FakeLiftIk ik;
  ik.lift_gain = 1.5;     // 1.5 m per m, under the prismatic limit
  ik.wrist_gain = 0.0;
  ColumnIkChain chain(liftParams(), ik);
  // Up 0.2 m, then back down 0.1 m: travel 0.3 m of lift (the poses are 0.1 m apart in z only).
  std::vector<Pose> poses = {at(0, 0, 0.0), at(0, 0, 0.1), at(0, 0, 0.2), at(0, 0, 0.1)};
  const auto r = chain.solve(poses);
  ASSERT_TRUE(r.complete);
  EXPECT_TRUE(r.continuous());
  EXPECT_NEAR(r.lift_travel_m, 1.5 * 0.3, 1e-9);
  EXPECT_NEAR(r.max_continuity_ratio, 1.5, 1e-9);
}

TEST(ColumnIkChain, ThresholdsDependOnTheJointType)
{
  const auto poses = diagonal(5, 0.1);   // jump of 0.1 * sqrt(2) per step; compare per joint below
  // The same raw ratio of 5 per unit jump: fine for a continuous joint (limit 8), a break for a prismatic
  // one (limit 2).
  const double jump = 0.1 * std::sqrt(2.0);
  {
    FakeLiftIk ik;
    ik.lift_gain = 0.0;
    ik.wrist_gain = 5.0 * jump / 0.1;   // dq = wrist_gain * 0.1 = 5 * jump
    ColumnIkChain chain(liftParams(), ik);
    const auto r = chain.solve(poses);
    ASSERT_TRUE(r.complete);
    EXPECT_TRUE(r.continuous());
    EXPECT_NEAR(r.max_continuity_ratio, 5.0, 1e-9);
  }
  {
    FakeLiftIk ik;
    ik.lift_gain = 5.0 * jump / 0.1;
    ik.wrist_gain = 0.0;
    ColumnIkChain chain(liftParams(), ik);
    const auto r = chain.solve(poses);
    ASSERT_TRUE(r.complete);
    EXPECT_FALSE(r.continuous());
    EXPECT_EQ(r.split_count, 4u);
    EXPECT_EQ(r.failure, PoseFailure::kBranchBreak);
    EXPECT_EQ(r.steps[1].worst_joint, 1u);   // the lift
    EXPECT_NEAR(r.steps[1].normalized_ratio, 5.0 / 2.0, 1e-9);
  }
  {
    // Over the continuous limit too: 10 per unit jump on the wrist.
    FakeLiftIk ik;
    ik.lift_gain = 0.0;
    ik.wrist_gain = 10.0 * jump / 0.1;
    ColumnIkChain chain(liftParams(), ik);
    const auto r = chain.solve(poses);
    EXPECT_EQ(r.split_count, 4u);
    EXPECT_EQ(r.steps[1].worst_joint, 2u);
  }
}

TEST(ColumnIkChain, ContinuousJointStepIsWrapped)
{
  // A wrist that answers 2*pi - 0.01 then 0.01 is a 0.02 rad step, not a flip.
  ColumnIkChainParams p;
  p.joint_kinds = {JointKind::kContinuous};
  int n = 0;
  ColumnIkChain chain(p, [&n](const Pose &, const JointVec &) -> std::optional<JointVec> {
      JointVec q(1);
      q << (n++ == 0 ? 2.0 * M_PI - 0.01 : 0.01);
      return q;
    });
  const auto r = chain.solve(std::vector<Pose>{at(0, 0), at(0.1, 0)});
  ASSERT_TRUE(r.complete);
  EXPECT_TRUE(r.continuous());
  EXPECT_NEAR(r.max_continuity_ratio, 0.2, 1e-9);
}

TEST(ColumnIkChain, FailureNameAndEmptyInput)
{
  EXPECT_STREQ(toString(PoseFailure::kBranchBreak), "branch_break");
  ColumnIkChain chain(planarParams(), FakePlanarIk{});
  const auto r = chain.solve(std::vector<Pose>{});
  EXPECT_FALSE(r.complete);
  EXPECT_TRUE(r.steps.empty());
}
