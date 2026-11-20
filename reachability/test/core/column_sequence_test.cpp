#include <gtest/gtest.h>

#include "reachability/core/column_sequence.hpp"
#include "support.hpp"
#include "synthetic_map.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

namespace
{
// A vertical column on the wall x = 2, strokes every 0.1 m from z0 to z1, tool pointing at the wall.
std::vector<Pose> column(double y, double z0, double z1)
{
  std::vector<Pose> s;
  for (double z = z0; z <= z1 + 1e-9; z += 0.1) {s.push_back(towardPlusX(2.0, y, z));}
  return s;
}

SequenceParams params(double standoff, double extra, double press)
{
  SequenceParams p;
  p.wall_normal = Vec3(-1, 0, 0);
  p.standoff_m = standoff;
  p.approach_extra_m = extra;
  p.press_depth_m = press;
  return p;
}
}  // namespace

TEST(ColumnSequence, OrderAndPositions)
{
  const auto strokes = column(0.0, 1.0, 1.4);   // 5 strokes
  const auto seq = buildColumnSequence(strokes, params(0.12, 0.10, 0.03));
  ASSERT_EQ(seq.poses.size(), 8u);   // approach, contact, 5 strokes, retract
  EXPECT_EQ(seq.roles.front(), PoseRole::kApproach);
  EXPECT_EQ(seq.roles[1], PoseRole::kContact);
  EXPECT_EQ(seq.roles.back(), PoseRole::kRetract);
  EXPECT_EQ(seq.strokes, 5u);
  // Away from the wall (wall at +x, normal -x): approach 0.22 short of the dock stroke (the middle one).
  EXPECT_NEAR(seq.poses[0].translation().x(), 2.0 - 0.22, 1e-12);
  EXPECT_NEAR(seq.poses[0].translation().z(), strokes[2].translation().z(), 1e-12);
  // Contact: 3 cm past the plan surface.
  EXPECT_NEAR(seq.poses[1].translation().x(), 2.03, 1e-12);
  // Retract defaults to the approach distance, from the last stroke.
  EXPECT_NEAR(seq.poses.back().translation().x(), 1.78, 1e-12);
  EXPECT_NEAR(seq.poses.back().translation().z(), strokes.back().translation().z(), 1e-12);
  EXPECT_EQ(seq.stroke_index[2], 0u);
  EXPECT_EQ(seq.stroke_index[0], kNoStroke);
}

TEST(ColumnSequence, DockAtFirstStrokeAndOmittedPoses)
{
  const auto strokes = column(0.0, 1.0, 1.4);
  SequenceParams p = params(0.0, 0.0, 0.0);   // no standoff, no press: only the strokes
  EXPECT_EQ(buildColumnSequence(strokes, p).poses.size(), 5u);
  p = params(0.12, 0.0, 0.0);
  p.dock_at_first_stroke = true;
  p.retract_m = 0.0;   // explicit: no retract pose
  const auto seq = buildColumnSequence(strokes, p);
  ASSERT_EQ(seq.poses.size(), 6u);
  EXPECT_NEAR(seq.poses[0].translation().z(), strokes[0].translation().z(), 1e-12);
  EXPECT_EQ(seq.roles.back(), PoseRole::kStroke);
  EXPECT_TRUE(buildColumnSequence({}, p).poses.empty());
}

// The reason for this layer: the strokes are reachable, the approach is not.
TEST(ColumnSequence, UnreachableApproachMakesColumnInfeasibleThoughStrokesReachable)
{
  const Map4D map = syntheticMap();   // x* in [0.30, 0.90)
  const auto strokes = column(0.0, 0.2, 0.6);
  const Eigen::Vector2d base(2.0 - 0.45, 0.0);   // strokes at x* = 0.45, approach at 0.45 - 0.22 = 0.23
  MarginConfig m;

  const auto bare = buildColumnSequence(strokes, params(0.0, 0.0, 0.0));
  const auto b = evaluateColumn(map, bare, base, {0.0}, 0.0, m);
  EXPECT_EQ(b.status, ColumnStatus::kWhole) << "the strokes alone are reachable";

  const auto full = buildColumnSequence(strokes, params(0.12, 0.10, 0.0));
  const auto f = evaluateColumn(map, full, base, {0.0}, 0.0, m);
  EXPECT_EQ(f.status, ColumnStatus::kUnreachable);
  EXPECT_EQ(f.best.strokes_usable, f.best.strokes_total);
  EXPECT_FALSE(f.best.ends_usable);
  ASSERT_TRUE(f.best.limiting.has_value());
  EXPECT_EQ(f.best.poses[*f.best.limiting].role, PoseRole::kApproach);
  EXPECT_EQ(f.best.poses[*f.best.limiting].failure, PoseFailure::kOutOfReach);

  // Further from the wall everything fits (approach at 0.60 - 0.22 = 0.38).
  const auto g = evaluateColumn(map, full, Eigen::Vector2d(2.0 - 0.60, 0.0), {0.0}, 0.0, m);
  EXPECT_EQ(g.status, ColumnStatus::kWhole);
}

TEST(ColumnSequence, RetractAlsoGatesTheColumn)
{
  const Map4D map = syntheticMap();
  SequenceParams p = params(0.0, 0.0, 0.0);
  p.retract_m = 0.5;   // from x* = 0.60 back to 0.10: outside [0.30, 0.90)
  const auto seq = buildColumnSequence(column(0.0, 0.2, 0.6), p);
  const auto f = evaluateColumn(map, seq, Eigen::Vector2d(2.0 - 0.60, 0.0), {0.0}, 0.0, MarginConfig{});
  EXPECT_EQ(f.status, ColumnStatus::kUnreachable);
  EXPECT_EQ(f.best.strokes_usable, f.best.strokes_total);
  EXPECT_EQ(f.best.poses[*f.best.limiting].role, PoseRole::kRetract);
}

TEST(ColumnSequence, LowMarginPoseIsNotUsable)
{
  const Map4D map = syntheticMap();   // the outer 5 cm of x* hold count 1: score 1/8 < 0.15
  const auto strokes = column(0.0, 0.2, 0.6);
  const Eigen::Vector2d base(2.0 - 0.87, 0.0);   // strokes at x* = 0.87: reachable cell, weak
  MarginConfig m;
  const auto seq = buildColumnSequence(strokes, params(0.0, 0.0, 0.0));

  const auto strict = evaluateColumn(map, seq, base, {0.0}, 0.0, m);
  EXPECT_EQ(strict.status, ColumnStatus::kUnreachable);
  EXPECT_EQ(strict.best.strokes_usable, 0u);
  ASSERT_TRUE(strict.best.limiting.has_value());
  EXPECT_EQ(strict.best.poses[*strict.best.limiting].failure, PoseFailure::kLowMargin);
  EXPECT_NEAR(strict.best.poses[*strict.best.limiting].score.count_score, 1.0 / 8.0, 1e-12);

  m.min_score = 0.0;   // margin off: the cell is reachable, so the column is
  EXPECT_EQ(evaluateColumn(map, seq, base, {0.0}, 0.0, m).status, ColumnStatus::kWhole);
}

TEST(ColumnSequence, PartiallyReachableStrokesGiveASplit)
{
  const Map4D map = syntheticMap();   // z in [-0.5, 0.7]
  SequenceParams sp = params(0.12, 0.0, 0.0);
  sp.retract_m = 0.0;   // a retract from the top stroke (z = 1.0) would itself be out of the window
  const auto seq = buildColumnSequence(column(0.0, 0.2, 1.0), sp);
  const auto f = evaluateColumn(map, seq, Eigen::Vector2d(2.0 - 0.60, 0.0), {0.0}, 0.0, MarginConfig{});
  EXPECT_EQ(f.status, ColumnStatus::kSplit);
  EXPECT_LT(f.best.strokes_usable, f.best.strokes_total);
  EXPECT_GT(f.best.strokes_usable, 0u);
  EXPECT_TRUE(f.best.ends_usable);
  // A lift height that brings the top strokes into the window is chosen when offered.
  const auto g = evaluateColumn(map, seq, Eigen::Vector2d(2.0 - 0.60, 0.0), {0.0, 0.2, 0.4}, 0.0, MarginConfig{});
  EXPECT_EQ(g.status, ColumnStatus::kWhole);
  EXPECT_GT(g.best.lift_height, 0.0);

  // With the retract from the top stroke in the sequence the same column cannot even leave: not a split.
  const auto with_retract = buildColumnSequence(column(0.0, 0.2, 1.0), params(0.12, 0.0, 0.0));
  const auto h = evaluateColumn(map, with_retract, Eigen::Vector2d(2.0 - 0.60, 0.0), {0.0}, 0.0, MarginConfig{});
  EXPECT_EQ(h.status, ColumnStatus::kUnreachable);
  EXPECT_EQ(h.best.poses[*h.best.limiting].role, PoseRole::kRetract);
}

TEST(ColumnSequence, VerifierRefusalOfAnEndPoseGatesTheColumn)
{
  const Map4D map = syntheticMap();
  const auto strokes = column(0.0, 0.2, 0.6);
  const auto seq = buildColumnSequence(strokes, params(0.12, 0.10, 0.0));
  const Stand stand{Eigen::Vector2d(2.0 - 0.80, 0.0), 0.0};
  const auto feas = evaluateColumn(map, seq, stand.arm_xy, {0.0}, 0.0, MarginConfig{});
  ASSERT_EQ(feas.status, ColumnStatus::kWhole);

  // Fake IK: refuses anything closer to the wall than the strokes' own plane minus 0.15 (the approach).
  FunctionStandVerifier refuse_approach([](const Stand &, double, const Pose & t) {
      TargetVerdict v;
      v.reason = t.translation().x() < 1.85 ? VerifyReason::kIkFailed : VerifyReason::kOk;
      v.joints = {0.0};
      return v;
    });
  const auto v = confirmColumnWithVerifier(refuse_approach, nullptr, stand, seq, feas, MarginConfig{});
  EXPECT_TRUE(v.verified);
  EXPECT_EQ(v.status, ColumnStatus::kUnreachable);
  ASSERT_TRUE(v.best.limiting.has_value());
  EXPECT_EQ(v.best.poses[*v.best.limiting].role, PoseRole::kApproach);
  EXPECT_EQ(v.best.poses[*v.best.limiting].failure, PoseFailure::kRefused);
  EXPECT_EQ(v.best.strokes_usable, v.best.strokes_total);
}

TEST(ColumnSequence, SingularSolutionIsRejectedWhenAnArmIsGiven)
{
  const ChainModel arm = genericArm();
  const std::vector<double> straight = {0, 0, 0, 0, 0, 0};
  const std::vector<double> bent = {0.3, -1.0, 1.6, -0.6, 0.4, 0.8};
  const double s0 = minSingularValue(arm, straight);
  const double s1 = minSingularValue(arm, bent);
  EXPECT_GE(s0, 0.0);
  EXPECT_LT(s0, 1e-3);   // arm stretched up: the elbow is singular
  EXPECT_GT(s1, 0.02);
  EXPECT_GT(s1, 100 * s0 + 1e-9);

  const Map4D map = syntheticMap();
  const auto seq = buildColumnSequence(column(0.0, 0.3, 0.6), params(0.0, 0.0, 0.0));
  const Stand stand{Eigen::Vector2d(2.0 - 0.60, 0.0), 0.0};
  const auto feas = evaluateColumn(map, seq, stand.arm_xy, {0.0}, 0.0, MarginConfig{});
  MarginConfig m;
  m.min_singular_value = 0.02;
  FunctionStandVerifier at_singularity([&](const Stand &, double, const Pose &) {
      TargetVerdict v;
      v.reason = VerifyReason::kOk;
      v.joints = straight;
      return v;
    });
  const auto bad = confirmColumnWithVerifier(at_singularity, &arm, stand, seq, feas, m);
  EXPECT_EQ(bad.status, ColumnStatus::kUnreachable);
  EXPECT_EQ(bad.best.poses[*bad.best.limiting].failure, PoseFailure::kSingular);
  FunctionStandVerifier fine([&](const Stand &, double, const Pose &) {
      TargetVerdict v;
      v.reason = VerifyReason::kOk;
      v.joints = bent;
      return v;
    });
  EXPECT_EQ(confirmColumnWithVerifier(fine, &arm, stand, seq, feas, m).status, ColumnStatus::kWhole);
}
