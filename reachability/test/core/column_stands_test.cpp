#include <gtest/gtest.h>

#include "reachability/core/column_analysis.hpp"
#include "reachability/core/column_stands.hpp"
#include "support.hpp"
#include "synthetic_map.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

namespace
{
ColumnPlan vertical(const std::string & label, double y, double z0, double z1)
{
  ColumnPlan c;
  c.label = label;
  for (double z = z0; z <= z1 + 1e-9; z += 0.1) {c.strokes_world.push_back(towardPlusX(2.0, y, z));}
  return c;
}

// A stroke line along the wall: one "column" that is wider than one stand sees (the map's y window is +-0.8).
ColumnPlan lateral(const std::string & label, double y0, double y1, double z)
{
  ColumnPlan c;
  c.label = label;
  for (double y = y0; y <= y1 + 1e-9; y += 0.1) {c.strokes_world.push_back(towardPlusX(2.0, y, z));}
  return c;
}

StationRequest wall()
{
  StationRequest r;
  r.wall_point = Vec3(2.0, 0.0, 0.0);
  r.wall_normal = Vec3(-1.0, 0.0, 0.0);
  r.min_clearance = 0.3;
  r.max_standoff = 1.0;
  r.grid_step = 0.1;
  r.lateral_margin = 0.3;
  r.lift_min = 0.0;
  r.lift_max = 0.6;
  r.lift_step = 0.1;
  return r;
}

ColumnStandOptions opts()
{
  ColumnStandOptions o;
  o.sequence.standoff_m = 0.10;
  o.sequence.approach_extra_m = 0.10;
  o.sequence.press_depth_m = 0.02;
  return o;
}
}  // namespace

TEST(ColumnStands, ColumnsThatFitOneStandStayWhole)
{
  const Map4D map = syntheticMap();
  const std::vector<ColumnPlan> cols = {vertical("a", -0.3, 0.2, 0.6), vertical("b", 0.0, 0.2, 0.6), vertical("c", 0.3, 0.2, 0.6)};
  const auto plan = planColumnStands(map, wall(), cols, std::nullopt, opts());
  ASSERT_TRUE(plan.success) << plan.message;
  EXPECT_EQ(plan.whole, 3u);
  EXPECT_EQ(plan.split, 0u);
  EXPECT_EQ(plan.stands.size(), 1u);
  for (const auto & r : plan.columns) {
    EXPECT_EQ(r.status, ColumnStatus::kWhole);
    ASSERT_EQ(r.pieces.size(), 1u);
    EXPECT_EQ(r.strokes_covered, r.strokes_total);
  }
  EXPECT_EQ(plan.stands_if_splitting_allowed, 1u);
}

TEST(ColumnStands, ApproachOutOfReachExcludesStandsThatOnlyReachTheStrokes)
{
  // x* window [0.30, 0.90); standoff + extra = 0.45 puts the approach 0.45 closer: the stand must be
  // at least 0.30 + 0.45 = 0.75 from the wall for the approach, though the strokes alone fit from 0.30.
  SyntheticReach r;
  r.x_lo = 0.30;
  r.x_hi = 1.30;
  const Map4D map = syntheticMap(r);
  ColumnStandOptions o = opts();
  o.sequence.standoff_m = 0.25;
  o.sequence.approach_extra_m = 0.20;
  const auto plan = planColumnStands(map, wall(), {vertical("a", 0.0, 0.2, 0.6)}, std::nullopt, o);
  ASSERT_TRUE(plan.success) << plan.message;
  ASSERT_EQ(plan.stands.size(), 1u);
  const double standoff = plan.stands[0].standoff;
  EXPECT_GE(standoff, 0.75 - 1e-9);

  // With the grid capped at 0.6 m no stand has an approach in reach, though the strokes are: unreachable.
  StationRequest near = wall();
  near.max_standoff = 0.6;
  const auto bad = planColumnStands(map, near, {vertical("a", 0.0, 0.2, 0.6)}, std::nullopt, o);
  EXPECT_FALSE(bad.success);
  EXPECT_EQ(bad.unreachable, 1u);
  EXPECT_EQ(bad.columns[0].strokes_covered, 0u);
  ASSERT_TRUE(bad.columns[0].limited);
  EXPECT_EQ(bad.columns[0].limiting_role, PoseRole::kApproach);
  // Strokes alone: the same wall is fine from the same grid.
  ColumnStandOptions strokes_only = o;
  strokes_only.sequence.standoff_m = 0.0;
  strokes_only.sequence.approach_extra_m = 0.0;
  strokes_only.sequence.press_depth_m = 0.0;
  EXPECT_TRUE(planColumnStands(map, near, {vertical("a", 0.0, 0.2, 0.6)}, std::nullopt, strokes_only).success);
}

TEST(ColumnStands, ColumnWiderThanOneStandIsSplitAndReported)
{
  const Map4D map = syntheticMap();
  // 2.4 m of strokes along the wall (25 poses) against a lateral window of 1.6 m per stand.
  const std::vector<ColumnPlan> cols = {lateral("long", -1.2, 1.2, 0.5), vertical("short", 0.9, 0.2, 0.6)};
  const auto plan = planColumnStands(map, wall(), cols, std::nullopt, opts());
  EXPECT_FALSE(plan.success);
  ASSERT_EQ(plan.columns.size(), 2u);
  const ColumnResult & s = plan.columns[0];
  EXPECT_EQ(s.status, ColumnStatus::kSplit);
  EXPECT_EQ(s.pieces.size(), 2u);
  EXPECT_EQ(s.strokes_covered, s.strokes_total);   // every stroke is done, by two stands
  size_t sum = 0;
  for (const auto & p : s.pieces) {sum += p.strokes.size();}
  EXPECT_EQ(sum, s.strokes_total);
  EXPECT_NE(s.pieces[0].stand, s.pieces[1].stand);
  EXPECT_TRUE(s.limited);   // the report says why it is not whole
  // The column that fits one stand stays whole and in one piece.
  EXPECT_EQ(plan.columns[1].status, ColumnStatus::kWhole);
  EXPECT_EQ(plan.columns[1].pieces.size(), 1u);
  EXPECT_EQ(plan.split, 1u);
  EXPECT_EQ(plan.whole, 1u);
  EXPECT_LE(plan.stands_if_splitting_allowed, plan.stands.size());
}

TEST(ColumnStands, AllowSplitsFalseLeavesTheWideColumnOut)
{
  const Map4D map = syntheticMap();
  ColumnStandOptions o = opts();
  o.allow_splits = false;
  const auto plan = planColumnStands(map, wall(), {lateral("long", -1.2, 1.2, 0.5), vertical("short", 0.0, 0.2, 0.6)}, std::nullopt, o);
  EXPECT_EQ(plan.columns[0].status, ColumnStatus::kUnreachable);
  EXPECT_TRUE(plan.columns[0].skipped_to_avoid_split);
  EXPECT_EQ(plan.columns[0].strokes_covered, 0u);
  EXPECT_EQ(plan.columns[1].status, ColumnStatus::kWhole);
}

TEST(ColumnStands, WholeCoverNeverCutsAColumnThatSomeStandDoesWhole)
{
  // Two columns 1.4 m apart: a stand between them reaches both only partly, so the stroke-by-stroke cover
  // could mix stands; here each column fits whole from a stand of its own or one shared stand.
  const Map4D map = syntheticMap();
  const auto plan = planColumnStands(map, wall(), {vertical("l", -0.7, 0.2, 0.6), vertical("r", 0.7, 0.2, 0.6)}, std::nullopt, opts());
  ASSERT_TRUE(plan.success) << plan.message;
  for (const auto & r : plan.columns) {EXPECT_EQ(r.pieces.size(), 1u);}
}

TEST(ColumnStands, AnalysisTotalsAndJson)
{
  const Map4D map = syntheticMap();
  const std::vector<ColumnPlan> cols = {
    vertical("ok", 0.0, 0.2, 0.6),            // 5 strokes
    lateral("long", -1.2, 1.2, 0.5),          // 25 strokes, needs a split
    vertical("high", 0.0, 3.0, 3.4)};         // 5 strokes, far above any lift + arm reach
  const auto a = analyzeColumnCoverage(map, wall(), cols, std::nullopt, opts());
  EXPECT_EQ(a.columns_total, 3u);
  EXPECT_EQ(a.columns_whole, 1u);
  EXPECT_EQ(a.columns_split, 1u);
  EXPECT_EQ(a.columns_unreachable, 1u);
  EXPECT_EQ(a.points_total, 35u);
  EXPECT_EQ(a.points_covered, 30u);
  EXPECT_NEAR(a.covered_fraction, 30.0 / 35.0, 1e-12);
  EXPECT_EQ(a.stands, a.per_stand.size());
  size_t pts = 0;
  for (const auto & s : a.per_stand) {pts += s.points_covered;}
  EXPECT_EQ(pts, a.points_covered);
  EXPECT_TRUE(a.plan.columns[2].limited);
  EXPECT_EQ(a.plan.columns[2].limiting_failure, PoseFailure::kOutOfReach);

  const std::string j = toJson(a);
  EXPECT_NE(j.find("\"columns_whole\": 1"), std::string::npos);
  EXPECT_NE(j.find("\"status\": \"split\""), std::string::npos);
  EXPECT_NE(j.find("\"status\": \"unreachable\""), std::string::npos);
  EXPECT_NE(j.find("\"limiting\": {\"pose\": "), std::string::npos);
  EXPECT_NE(j.find("\"points_covered\": 30"), std::string::npos);
  EXPECT_EQ(j.front(), '{');
}

TEST(ColumnStands, VerifierRefusalDowngradesTheColumnAndTheTotals)
{
  const Map4D map = syntheticMap();
  FunctionStandVerifier refuse_high([](const Stand &, double, const Pose & t) {
      TargetVerdict v;
      v.reason = (t.translation().z() > 0.45 && t.translation().x() > 1.95) ? VerifyReason::kInCollision : VerifyReason::kOk;
      return v;
    });
  ColumnStandOptions o = opts();
  o.verifier = &refuse_high;
  const auto plan = planColumnStands(map, wall(), {vertical("a", 0.0, 0.2, 0.6)}, std::nullopt, o);
  EXPECT_TRUE(plan.verified);
  EXPECT_EQ(plan.columns[0].status, ColumnStatus::kSplit);   // strokes above 0.45 were refused
  EXPECT_LT(plan.columns[0].strokes_covered, plan.columns[0].strokes_total);
  EXPECT_GT(plan.columns[0].strokes_covered, 0u);
}

TEST(ColumnStands, EmptyAndInvalidRequests)
{
  const Map4D map = syntheticMap();
  EXPECT_TRUE(planColumnStands(map, wall(), {}).success);
  StationRequest bad = wall();
  bad.grid_step = 0.0;
  const auto p = planColumnStands(map, bad, {vertical("a", 0.0, 0.2, 0.6)});
  EXPECT_FALSE(p.success);
  EXPECT_FALSE(p.message.empty());
}
