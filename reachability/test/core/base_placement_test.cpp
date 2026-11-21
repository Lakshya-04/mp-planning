/**
 * @file base_placement_test.cpp
 * @brief Scenario and property harness for planBasePlacement over a catalogue of geometries (inside and
 *        outside corner, wall end, opening, protrusion, alcove, corridor, partition, plain wall) and
 *        randomised cases with fixed seeds. Hand-made reach maps only: no arm, no ROS. A metrics table is
 *        printed at the end of the run.
 *
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "reachability/core/base_placement.hpp"
#include "synthetic_map.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT
using V2 = Eigen::Vector2d;

namespace
{
// ---------------------------------------------------------------- metrics table

struct Row
{
  std::string name;
  size_t columns{0}, whole{0}, split{0}, unreachable{0};
  double covered{0.0}, reachable{0.0};
  size_t stands{0};
  long exact{-1};
  double travel_m{0.0}, clearance{0.0};
  size_t nav_rej{0}, yaw_rej{0};
};
std::vector<Row> & table() {static std::vector<Row> t; return t;}

void record(const std::string & name, const BasePlacement & p)
{
  Row r;
  r.name = name;
  r.columns = p.columns.size();
  r.whole = p.whole;
  r.split = p.split;
  r.unreachable = p.unreachable;
  r.covered = p.covered_fraction;
  r.reachable = p.reachable_fraction;
  r.stands = p.stand_count;
  r.exact = p.exact_min_stands ? static_cast<long>(*p.exact_min_stands) : -1;
  r.travel_m = p.travel_m;
  r.clearance = p.stands.empty() ? 0.0 : p.min_clearance_m;
  r.nav_rej = p.nav_rejected;
  r.yaw_rej = p.yaw_rejected;
  table().push_back(r);
}

class TablePrinter : public ::testing::Environment
{
public:
  void TearDown() override
  {
    std::printf("\nBase placement metrics\n");
    std::printf("%-34s %4s %5s %5s %5s %7s %7s %6s %5s %8s %9s %4s %4s\n", "case", "cols", "whole", "split", "unrch",
      "cover%", "reach%", "stands", "exact", "travel_m", "clear_m", "nav", "yaw");
    for (const auto & r : table()) {
      std::printf("%-34s %4zu %5zu %5zu %5zu %7.1f %7.1f %6zu %5ld %8.2f %9.2f %4zu %4zu\n", r.name.c_str(), r.columns,
        r.whole, r.split, r.unreachable, 100.0 * r.covered, 100.0 * r.reachable, r.stands, r.exact, r.travel_m,
        r.clearance, r.nav_rej, r.yaw_rej);
    }
    std::fflush(stdout);
  }
};
const ::testing::Environment * const kEnv = ::testing::AddGlobalTestEnvironment(new TablePrinter);

// ---------------------------------------------------------------- fixtures

BaseModel rect(double len, double wid, bool strafe)
{
  BaseModel b;
  b.name = strafe ? "omni" : "diff";
  b.drive = strafe ? DriveType::kOmni : DriveType::kDiff;
  b.can_strafe = strafe;
  b.footprint = {{-len / 2, -wid / 2}, {len / 2, -wid / 2}, {len / 2, wid / 2}, {-len / 2, wid / 2}};
  return b;
}

// Vertical work wall at x = 2 (free side x < 2). Columns run along z at the given y.
ColumnPlan vcol(const std::string & label, double y, double z0, double z1)
{
  ColumnPlan c;
  c.label = label;
  for (double z = z0; z <= z1 + 1e-9; z += 0.1) {c.strokes_world.push_back(towardPlusX(2.0, y, z));}
  return c;
}

std::vector<ColumnPlan> vcols(double y0, double y1, double dy = 0.2)
{
  std::vector<ColumnPlan> out;
  int i = 0;
  for (double y = y0; y <= y1 + 1e-9; y += dy) {out.push_back(vcol("c" + std::to_string(i++), y, 0.2, 0.6));}
  return out;
}

BasePlacementOptions wallOptions(const BaseModel & base)
{
  BasePlacementOptions o;
  o.search.min_clearance = 0.3;
  o.search.max_standoff = 1.1;
  o.search.grid_step = 0.1;
  o.search.lateral_margin = 0.3;
  o.search.lift_min = 0.0;
  o.search.lift_max = 0.6;
  o.search.lift_step = 0.1;
  o.search.base = base;
  o.sequence.standoff_m = 0.10;
  o.sequence.approach_extra_m = 0.10;
  o.sequence.press_depth_m = 0.02;
  o.margin.min_score = 0.0;
  return o;
}

SurfaceFrame wallFrame() {return surfaceFrame(Vec3(-1.0, 0.0, 0.0), Vec3(2.0, 0.0, 0.0));}

// Horizontal ceiling patch: tool up, free side down. Columns run along y, one per x.
constexpr double kCeil = 2.01;
Map4D ringMap(double r_lo, double r_hi)
{
  Map4D map;
  for (double z = 0.76; z <= 1.07; z += 0.05) {
    for (double r = r_lo; r <= r_hi + 1e-9; r += 0.05) {
      Pose T = Pose::Identity();
      T.translation() = Vec3(r + 0.025, 0.0, z);
      for (int i = 0; i < 8; ++i) {map.record(T);}
    }
  }
  return map;
}

std::vector<ColumnPlan> ceilingColumns(double x0, double y0, int nx, int ny)
{
  std::vector<ColumnPlan> out;
  for (int i = 0; i < nx; ++i) {
    ColumnPlan c;
    c.label = "x" + std::to_string(i);
    for (int j = 0; j < ny; ++j) {
      Pose T = Pose::Identity();
      T.translation() = Vec3(x0 + 0.1 * i, y0 + 0.1 * j, kCeil);
      c.strokes_world.push_back(T);
    }
    out.push_back(c);
  }
  return out;
}

BasePlacementOptions ceilingOptions(const BaseModel & base)
{
  BasePlacementOptions o;
  o.search.grid_step = 0.1;
  o.search.lateral_margin = 0.9;
  o.search.lift_min = 0.0;
  o.search.lift_max = 0.0;
  o.search.lift_step = 0.1;
  o.search.arm_mount_offset_z = 1.0;
  o.search.base = base;
  o.sequence.standoff_m = 0.10;
  o.margin.min_score = 0.0;
  return o;
}

SurfaceFrame ceilingFrame() {return surfaceFrame(Vec3(0.0, 0.0, -1.0), Vec3(0.0, 0.0, kCeil));}

// Invariants every result has to satisfy.
void checkInvariants(const BasePlacement & p, const FreeSpace & fs, const BasePlacementOptions & o, const std::string & what)
{
  SCOPED_TRACE(what);
  ASSERT_EQ(p.whole + p.split + p.unreachable, p.columns.size());
  size_t covered = 0;
  for (const auto & c : p.columns) {
    EXPECT_LE(c.strokes_covered, c.strokes_total);
    covered += c.strokes_covered;
    if (c.status != ColumnStatus::kWhole) {
      EXPECT_NE(c.reason, BpoReason::kOk);                 // never dropped: always a code
      EXPECT_STRNE(toString(c.reason), "unknown");
    } else {
      EXPECT_EQ(c.reason, BpoReason::kOk);
      EXPECT_EQ(c.pieces.size(), 1u);
    }
    if (c.status == ColumnStatus::kUnreachable) {EXPECT_EQ(c.strokes_covered, 0u);}
  }
  EXPECT_EQ(covered, p.strokes_covered);
  for (const auto & s : p.stands) {
    ASSERT_TRUE(o.search.base.has_value());
    EXPECT_TRUE(fs.footprintFree(s.stand, *o.search.base, o.search.free_space_margin - 1e-9));
    EXPECT_GE(s.clearance_m, o.search.free_space_margin - 1e-3);
  }
}

BasePlacement runWall(const std::string & name, const std::vector<ColumnPlan> & cols, const FreeSpace & fs,
  const BasePlacementOptions & o, const std::optional<Stand> & start, const Map4D & map)
{
  BasePlacement p = planBasePlacement(map, wallFrame(), cols, fs, start, o);
  record(name, p);
  checkInvariants(p, fs, o, name);
  return p;
}

bool sameStands(const BasePlacement & a, const BasePlacement & b)
{
  if (a.stands.size() != b.stands.size() || a.columns.size() != b.columns.size()) {return false;}
  for (size_t i = 0; i < a.stands.size(); ++i) {
    if ((a.stands[i].stand.arm_xy - b.stands[i].stand.arm_xy).norm() > 0.0 || a.stands[i].stand.yaw != b.stands[i].stand.yaw) {
      return false;
    }
  }
  for (size_t k = 0; k < a.columns.size(); ++k) {
    if (a.columns[k].reason != b.columns[k].reason || a.columns[k].strokes_covered != b.columns[k].strokes_covered) {
      return false;
    }
  }
  return true;
}
}  // namespace

// ---------------------------------------------------------------- navigation grid

TEST(NavGrid, ConnectedBlockedAndDisconnected)
{
  FreeSpace fs;
  fs.obstacles.push_back(FreeSpace::box(-5.0, 0.0, 5.0, 0.1));   // a partition across the room
  const BaseModel base = rect(0.4, 0.4, true);
  GridNavFeasibility nav(fs, base, V2(-3, -3), V2(3, 3));
  const Stand s{V2(0.0, -1.5), 0.0};
  EXPECT_TRUE(nav.feasible(s, Stand{V2(1.0, -1.0), 0.0}).ok);
  const NavVerdict across = nav.feasible(s, Stand{V2(0.0, 1.5), 0.0});
  EXPECT_FALSE(across.ok);
  EXPECT_EQ(across.reason, NavReason::kDisconnected);
  EXPECT_EQ(nav.feasible(s, Stand{V2(0.0, 0.05), 0.0}).reason, NavReason::kGoalBlocked);
  EXPECT_EQ(nav.feasible(s, Stand{V2(9.0, 0.0), 0.0}).reason, NavReason::kOutsideMap);
  EXPECT_EQ(nav.feasible(Stand{V2(0.0, 0.05), 0.0}, s).reason, NavReason::kStartBlocked);
}

TEST(NavGrid, AGapTooNarrowForTheFootprintDisconnects)
{
  FreeSpace fs;
  fs.obstacles.push_back(FreeSpace::box(-5.0, 0.0, -0.3, 0.1));
  fs.obstacles.push_back(FreeSpace::box(0.3, 0.0, 5.0, 0.1));   // a 0.6 m door
  const Stand a{V2(0.0, -1.0), 0.0}, b{V2(0.0, 1.0), 0.0};
  EXPECT_TRUE(GridNavFeasibility(fs, rect(0.2, 0.2, true), V2(-3, -3), V2(3, 3)).feasible(a, b).ok);
  EXPECT_FALSE(GridNavFeasibility(fs, rect(0.9, 0.7, true), V2(-3, -3), V2(3, 3)).feasible(a, b).ok);
}

// ---------------------------------------------------------------- plain wall (regression)

TEST(BasePlacement, PlainWallMatchesTheColumnPlanner)
{
  const Map4D map = syntheticMap();
  const auto cols = vcols(-1.2, 1.2);
  BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  const FreeSpace none;
  const BasePlacement p = runWall("plain wall 13 columns", cols, none, o, std::nullopt, map);
  EXPECT_TRUE(p.success) << p.message;
  EXPECT_EQ(p.split_count, 0u);
  EXPECT_DOUBLE_EQ(p.covered_fraction, 1.0);

  StationRequest st = o.search;
  st.wall_point = Vec3(2.0, 0.0, 0.0);
  st.wall_normal = Vec3(-1.0, 0.0, 0.0);
  ColumnStandOptions co;
  co.sequence = o.sequence;
  co.margin = o.margin;
  const ColumnStandPlan plain = planColumnStands(map, st, cols, std::nullopt, co);
  EXPECT_EQ(plain.stands.size(), p.stand_count);
  EXPECT_EQ(plain.whole, p.whole);
  ASSERT_TRUE(p.exact_min_stands.has_value());
  EXPECT_LE(p.stand_count, *p.exact_min_stands + 1);
  EXPECT_GE(p.stand_count, *p.exact_min_stands);
  // Every column is whole at the stand the map says; the nearest-stand escalation is reported.
  for (const auto & c : p.columns) {EXPECT_NE(c.escalation, BpoEscalation::kUnreachable);}
}

TEST(BasePlacement, EmptyOrInvalidInputIsReportedNotInvented)
{
  const Map4D map = syntheticMap();
  const BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  BasePlacement p = planBasePlacement(map, SurfaceFrame{}, vcols(0.0, 0.2), FreeSpace{}, std::nullopt, o);
  EXPECT_FALSE(p.success);
  EXPECT_EQ(p.unreachable, 2u);
  EXPECT_TRUE(p.stands.empty());
  p = planBasePlacement(map, wallFrame(), {}, FreeSpace{}, std::nullopt, o);
  EXPECT_TRUE(p.success);
  EXPECT_TRUE(p.stands.empty());
}

TEST(BasePlacement, Deterministic)
{
  const Map4D map = syntheticMap();
  const auto cols = vcols(-1.0, 1.0);
  FreeSpace fs;
  fs.walls.push_back({V2(0.0, 1.2), V2(0.0, -1.0)});
  const BasePlacementOptions o = wallOptions(rect(0.6, 0.5, false));
  const Stand start{V2(0.5, 0.0), 0.0};
  const BasePlacement a = planBasePlacement(map, wallFrame(), cols, fs, start, o);
  const BasePlacement b = planBasePlacement(map, wallFrame(), cols, fs, start, o);
  EXPECT_TRUE(sameStands(a, b));
  EXPECT_DOUBLE_EQ(a.travel_m, b.travel_m);
}

// ---------------------------------------------------------------- escalation

TEST(BasePlacement, ALiftHeightEscalationIsReported)
{
  // The arm sees a slab 0.2..0.5 m above its base. The nominal lift puts the base level with the column's
  // middle, which is outside the slab, so the column is only whole after the lift height changes.
  SyntheticReach r;
  r.z_lo = 0.2;
  r.z_hi = 0.5;
  const Map4D map = syntheticMap(r);
  BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  o.search.lift_max = 1.5;
  o.search.arm_mount_offset_z = 0.2;
  const std::vector<ColumnPlan> cols = {vcol("a", 0.0, 0.8, 0.9)};
  const BasePlacement p = runWall("lift escalation", cols, FreeSpace{}, o, std::nullopt, map);
  ASSERT_TRUE(p.success) << p.message;
  EXPECT_EQ(p.columns[0].escalation, BpoEscalation::kLiftHeight);
  ASSERT_EQ(p.stands[0].columns.size(), 1u);
  EXPECT_NEAR(p.stands[0].columns[0].lift_height, 0.3, 0.16);
}

TEST(BasePlacement, SplitOnlyWhenNoStandDoesTheColumnWhole)
{
  // A column 2.4 m long along the wall: wider than the +-0.8 m window, so no stand does it whole.
  const Map4D map = syntheticMap();
  ColumnPlan wide;
  wide.label = "wide";
  for (double y = -1.2; y <= 1.2 + 1e-9; y += 0.1) {wide.strokes_world.push_back(towardPlusX(2.0, y, 0.3));}
  BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  BasePlacement p = runWall("split last resort", {wide}, FreeSpace{}, o, std::nullopt, map);
  EXPECT_EQ(p.columns[0].status, ColumnStatus::kSplit);
  EXPECT_EQ(p.columns[0].reason, BpoReason::kSplitNoWholeStand);
  EXPECT_EQ(p.columns[0].escalation, BpoEscalation::kSplit);
  EXPECT_EQ(p.split_count, 1u);
  EXPECT_GE(p.stand_count, 2u);
  EXPECT_NEAR(p.covered_fraction, 1.0, 1e-9);

  o.allow_splits = false;
  p = runWall("split refused", {wide}, FreeSpace{}, o, std::nullopt, map);
  EXPECT_EQ(p.split_count, 0u);
  EXPECT_EQ(p.columns[0].reason, BpoReason::kSkippedToAvoidSplit);
  EXPECT_EQ(p.columns[0].escalation, BpoEscalation::kSkipped);
  EXPECT_EQ(p.columns[0].strokes_covered, 0u);
  EXPECT_TRUE(p.stands.empty());
}

TEST(BasePlacement, TheStandCapIsACode)
{
  const Map4D map = syntheticMap();
  BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  o.max_stands = 1;
  const BasePlacement p = runWall("stand cap 1", vcols(-1.6, 1.6, 0.4), FreeSpace{}, o, std::nullopt, map);
  EXPECT_EQ(p.stand_count, 1u);
  EXPECT_GT(p.unreachable, 0u);
  bool cap = false;
  for (const auto & c : p.columns) {cap = cap || c.reason == BpoReason::kMaxStands;}
  EXPECT_TRUE(cap);
}

// ---------------------------------------------------------------- verifier loop

TEST(BasePlacement, AStandTheVerifierRefusesIsAvoidedAndTheColumnsReplanned)
{
  const Map4D map = syntheticMap();
  const BasePlacementOptions base_o = wallOptions(rect(0.5, 0.5, true));
  const std::vector<ColumnPlan> cols = {vcol("a", 0.0, 0.2, 0.6)};
  const BasePlacement plain = planBasePlacement(map, wallFrame(), cols, FreeSpace{}, std::nullopt, base_o);
  ASSERT_EQ(plain.stands.size(), 1u);
  const V2 bad = plain.stands[0].stand.arm_xy;
  FunctionStandVerifier ver([&](const Stand & s, double, const Pose &) {
      TargetVerdict v;
      v.reason = (s.arm_xy - bad).norm() < 1e-6 ? VerifyReason::kInCollision : VerifyReason::kOk;
      return v;
    });
  BasePlacementOptions o = base_o;
  o.verifier = &ver;
  const BasePlacement p = runWall("verifier avoids refused stand", cols, FreeSpace{}, o, std::nullopt, map);
  EXPECT_TRUE(p.verified);
  ASSERT_EQ(p.stands.size(), 1u);
  EXPECT_GT((p.stands[0].stand.arm_xy - bad).norm(), 1e-6);
  EXPECT_TRUE(p.success) << p.message;
  EXPECT_GE(p.rounds, 2u);
  EXPECT_FALSE(p.refused_stands.empty());

  // Everything refused: reported with the verifier's code, not dropped.
  FunctionStandVerifier none([](const Stand &, double, const Pose &) {
      TargetVerdict v;
      v.reason = VerifyReason::kIkFailed;
      return v;
    });
  o.verifier = &none;
  o.verifier_rounds = 3;
  const BasePlacement q = runWall("verifier refuses all", cols, FreeSpace{}, o, std::nullopt, map);
  EXPECT_FALSE(q.success);
  EXPECT_EQ(q.columns[0].strokes_covered, 0u);
  EXPECT_NE(q.columns[0].reason, BpoReason::kOk);
}

// ---------------------------------------------------------------- navigation and yaw

TEST(BasePlacement, UnreachableByNavigationIsReported)
{
  const Map4D map = syntheticMap();
  FreeSpace fs;
  fs.obstacles.push_back(FreeSpace::box(-3.0, 0.5, 2.0, 0.55));   // partition from the work wall out into the room
  BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  o.search.lateral_margin = 0.5;
  const auto cols = vcols(1.6, 2.0);
  const Stand start{V2(0.8, -0.5), 0.0};
  const BasePlacement p = runWall("partition, start behind it", cols, fs, o, start, map);
  EXPECT_FALSE(p.success);
  EXPECT_EQ(p.stand_count, 0u);
  EXPECT_GT(p.nav_rejected, 0u);
  for (const auto & c : p.columns) {
    EXPECT_EQ(c.reason, BpoReason::kUnreachableByNav);
    EXPECT_EQ(c.status, ColumnStatus::kUnreachable);
  }
  // Same room, start on the right side: fine.
  const BasePlacement q = runWall("partition, start in front", cols, fs, o, Stand{V2(0.8, 1.0), 0.0}, map);
  EXPECT_TRUE(q.success) << q.message;
  // And without a start nothing is filtered by navigation.
  const BasePlacement r = runWall("partition, no start", cols, fs, o, std::nullopt, map);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.nav_rejected, 0u);
}

TEST(BasePlacement, ACostmapPortReplacesTheGrid)
{
  class Deny : public NavFeasibility
  {
public:
    NavVerdict feasible(const Stand &, const Stand & g) const override
    {
      return g.arm_xy.y() > 0.0 ? NavVerdict{false, NavReason::kDisconnected} : NavVerdict{true, NavReason::kOk};
    }
  } deny;
  const Map4D map = syntheticMap();
  BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  o.nav = &deny;
  const BasePlacement p = runWall("nav port denies y>0", {vcol("a", 0.0, 0.2, 0.6)}, FreeSpace{}, o, Stand{V2(0.0, -1.0), 0.0}, map);
  ASSERT_TRUE(p.success) << p.message;
  EXPECT_LE(p.stands[0].stand.arm_xy.y(), 0.0);
  EXPECT_GT(p.nav_rejected, 0u);
}

TEST(BasePlacement, CorridorLimitsYawForADiffDriveBase)
{
  // A strip 1 m deep between the work wall (x = 2) and a back wall (x = 1): the 0.9 m long base cannot face
  // the wall in it, it has to stand sideways. A base that turns freely does; one limited to +-0.7 rad
  // around "facing" cannot, and the columns say yaw_window.
  const Map4D map = syntheticMap();
  FreeSpace fs;
  fs.walls.push_back({V2(1.0, 0.0), V2(1.0, 0.0)});
  const auto cols = vcols(-0.2, 0.2);
  const BasePlacement omni = runWall("corridor omni", cols, fs, wallOptions(rect(0.9, 0.5, true)), std::nullopt, map);
  ASSERT_TRUE(omni.success) << omni.message;
  for (const auto & s : omni.stands) {EXPECT_GT(std::fabs(std::sin(s.stand.yaw)), 0.9);}   // sideways

  const BasePlacement diff = runWall("corridor diff drive", cols, fs, wallOptions(rect(0.9, 0.5, false)), std::nullopt, map);
  EXPECT_FALSE(diff.success);
  EXPECT_GT(diff.yaw_rejected, 0u);
  EXPECT_TRUE(diff.stands.empty());
  for (const auto & c : diff.columns) {EXPECT_EQ(c.reason, BpoReason::kYawWindow);}

  // A short diff base fits facing the wall; its yaw stays inside the window.
  const BasePlacement small = runWall("corridor short diff", cols, fs, wallOptions(rect(0.4, 0.5, false)), std::nullopt, map);
  ASSERT_TRUE(small.success) << small.message;
  const double facing = std::atan2(0.0, 1.0);
  for (const auto & s : small.stands) {EXPECT_LE(std::fabs(std::remainder(s.stand.yaw - facing, 2 * M_PI)), 0.7 + 1e-9);}
}

// ---------------------------------------------------------------- geometry catalogue

TEST(BasePlacement, InsideCornerCoverageGrowsWithTheDistanceFromTheSecondWall)
{
  // Ceiling patch 0.4 x 0.3 m near an inside corner (walls x = 0 and y = 0, free side x, y > 0): the patch
  // starts at y = d. The base is big and the arm's ring short, so the near-corner points are hard.
  const Map4D map = ringMap(0.30, 0.50);
  FreeSpace fs;
  fs.walls.push_back({V2(0.0, 0.0), V2(1.0, 0.0)});
  fs.walls.push_back({V2(0.0, 0.0), V2(0.0, 1.0)});
  BasePlacementOptions o = ceilingOptions(rect(0.7, 0.6, true));
  double prev = -1.0;
  for (const double d : {0.0, 0.1, 0.3, 0.6}) {
    const auto cols = ceilingColumns(0.1, d, 4, 4);
    const BasePlacement p = planBasePlacement(map, ceilingFrame(), cols, fs, Stand{V2(1.5, 1.5), 0.0}, o);
    record("inside corner d=" + std::to_string(d).substr(0, 3), p);
    checkInvariants(p, fs, o, "inside corner");
    EXPECT_GE(p.covered_fraction, prev - 1e-9) << "d=" << d;
    EXPECT_GE(p.covered_fraction, 0.99 * p.reachable_fraction - 1e-9);
    for (const auto & s : p.stands) {   // never inside the corner
      EXPECT_GE(s.stand.arm_xy.x(), 0.3 + 0.05 - 1e-9);
      EXPECT_GE(s.stand.arm_xy.y(), 0.3 + 0.05 - 1e-9);
    }
    prev = p.covered_fraction;
  }
  EXPECT_GT(prev, 0.0);
}

TEST(BasePlacement, OutsideCornerFacesAreSeparateAndTheStandSitsBeyondTheCorner)
{
  // The work wall x = 2 ends at y = 1 (a block behind it, x in [2, 3], y in [-5, 1]). Columns reach up to the
  // end; with a finite work span the base may stand beyond the end, closer than an infinite wall allows.
  const Map4D map = syntheticMap();
  FreeSpace fs;
  fs.has_work_span = true;
  fs.work_span_lo = -1.0;   // u = -y: the wall covers y <= 1
  fs.work_span_hi = 10.0;
  fs.obstacles.push_back(FreeSpace::box(2.0, -5.0, 3.0, 1.0));
  BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  o.search.lateral_margin = 0.9;
  const auto cols = vcols(0.6, 0.98, 0.2);
  const BasePlacement p = runWall("outside corner / wall end", cols, fs, o, std::nullopt, map);
  EXPECT_TRUE(p.success) << p.message;
  EXPECT_GE(p.covered_fraction, 0.99 * p.reachable_fraction);
  const auto block = FreeSpace::box(2.0, -5.0, 3.0, 1.0);
  for (const auto & s : p.stands) {
    EXPECT_GE(polygonDistance(footprintWorld(*o.search.base, s.stand), block), o.search.free_space_margin - 1e-9);
  }
}

TEST(BasePlacement, OpeningBorderKeepsTheBaseOffTheBenchInFrontOfTheWindow)
{
  const Map4D map = syntheticMap();
  FreeSpace fs;
  fs.obstacles.push_back(FreeSpace::box(1.0, 0.5, 1.95, 1.1));   // a bench under the window
  std::vector<ColumnPlan> cols = vcols(-0.2, 0.4);
  const auto right = vcols(1.2, 1.8);
  cols.insert(cols.end(), right.begin(), right.end());          // columns on both sides of the 0.8 m gap
  const BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  const BasePlacement p = runWall("window border + bench", cols, fs, o, std::nullopt, syntheticMap());
  EXPECT_TRUE(p.success) << p.message;
  EXPECT_EQ(p.split_count, 0u);
  (void)map;
}

TEST(BasePlacement, AProtrusionIsAvoidedAndWhatItHidesIsReportedByCode)
{
  const Map4D map = syntheticMap();
  FreeSpace fs;
  fs.obstacles.push_back(FreeSpace::box(0.9, 0.9, 2.0, 1.3));   // a pillar against the wall
  const auto cols = vcols(-0.4, 2.0, 0.2);
  const BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  const BasePlacement p = runWall("pillar against the wall", cols, fs, o, std::nullopt, map);
  EXPECT_GE(p.covered_fraction, 0.99 * p.reachable_fraction);
  for (const auto & s : p.stands) {
    EXPECT_GE(polygonDistance(footprintWorld(*o.search.base, s.stand), FreeSpace::box(0.9, 0.9, 2.0, 1.3)),
      o.search.free_space_margin - 1e-9);
  }
}

TEST(BasePlacement, ANarrowAlcoveIsReportedNeverInvented)
{
  const Map4D map = syntheticMap();
  FreeSpace fs;   // side walls 0.4 m apart around the targets: no 0.5 m wide base fits
  fs.walls.push_back({V2(0.0, -0.2), V2(0.0, 1.0)});
  fs.walls.push_back({V2(0.0, 0.2), V2(0.0, -1.0)});
  const BasePlacementOptions o = wallOptions(rect(0.5, 0.5, true));
  const auto cols = vcols(-0.1, 0.1);
  const BasePlacement p = runWall("narrow alcove", cols, fs, o, std::nullopt, map);
  EXPECT_FALSE(p.success);
  EXPECT_TRUE(p.stands.empty());
  EXPECT_EQ(p.unreachable, cols.size());
  EXPECT_DOUBLE_EQ(p.covered_fraction, 0.0);
  EXPECT_EQ(p.candidates, 0u);
  for (const auto & c : p.columns) {
    EXPECT_EQ(c.reason, BpoReason::kNoFreeStand);
    EXPECT_GT(c.strokes_total, 0u);
  }
}

// ---------------------------------------------------------------- randomised, fixed seeds

namespace
{
struct Rng
{
  std::mt19937 g;
  explicit Rng(uint32_t seed)
  : g(seed) {}
  double u(double lo, double hi) {return lo + (hi - lo) * (static_cast<double>(g() >> 8) / 16777216.0);}
};
}  // namespace

TEST(BasePlacementProperty, RandomInsideCornersOverARingMap)
{
  constexpr int kCases = 40;
  size_t exact_checked = 0, whole_all = 0;
  for (int seed = 1; seed <= kCases; ++seed) {
    Rng rng(static_cast<uint32_t>(seed));
    const double angle = rng.u(60.0, 120.0) * M_PI / 180.0;
    const double r_lo = rng.u(0.25, 0.4), r_hi = rng.u(0.55, 0.9);
    const double len = rng.u(0.45, 0.8), wid = rng.u(0.4, 0.6);
    const double s = rng.u(0.5, 1.4), wall_len = rng.u(1.5, 4.0);
    const bool finite = rng.u(0.0, 1.0) < 0.5;
    const bool strafe = rng.u(0.0, 1.0) < 0.6;
    const double ox = rng.u(-0.3, 0.3), oy = rng.u(-0.3, 0.3);

    // Wedge between a = +y and b at 90 deg - angle, apex at the origin.
    const V2 a(0.0, 1.0), b(std::sin(angle), std::cos(angle));
    const V2 na(1.0, 0.0), nb(-std::cos(angle), std::sin(angle));
    const V2 bis = (a + b).normalized();
    FreeSpace fs;
    if (finite) {
      for (const auto & w : {std::make_pair(a, na), std::make_pair(b, nb)}) {
        const V2 p1 = w.first * wall_len, off = -w.second * 0.02;
        fs.obstacles.push_back({off, p1 + off, p1, V2(0.0, 0.0)});
      }
    } else {
      fs.walls.push_back({V2(0.0, 0.0), na});
      fs.walls.push_back({V2(0.0, 0.0), nb});
    }
    const V2 c = bis * s + V2(ox, oy) * 0.0;
    const auto cols = ceilingColumns(c.x() - 0.15 + ox * 0.2, c.y() - 0.15 + oy * 0.2, 4, 4);
    const Map4D map = ringMap(r_lo, r_hi);
    BasePlacementOptions o = ceilingOptions(rect(len, wid, strafe));
    const Stand start{bis * 2.6, 0.0};
    const std::string name = "random corner seed " + std::to_string(seed);
    const BasePlacement p = planBasePlacement(map, ceilingFrame(), cols, fs, start, o);
    record(name, p);
    SCOPED_TRACE(name);
    checkInvariants(p, fs, o, name);

    EXPECT_GE(p.covered_fraction, 0.99 * p.reachable_fraction - 1e-9);
    const size_t coverable = p.whole_coverable;
    if (coverable == cols.size()) {
      ++whole_all;
      EXPECT_EQ(p.split_count, 0u);          // a whole cover exists: no splits
      EXPECT_EQ(p.whole, cols.size());
    }
    if (p.exact_min_stands && p.split_count == 0 && p.whole == coverable) {
      ++exact_checked;
      EXPECT_LE(p.stand_count, *p.exact_min_stands + 1);
      EXPECT_GE(p.stand_count, *p.exact_min_stands);
    }
    // Determinism.
    const BasePlacement q = planBasePlacement(map, ceilingFrame(), cols, fs, start, o);
    EXPECT_TRUE(sameStands(p, q));
    for (const auto & st : p.stands) {   // inside the wedge (a finite wall can be walked around)
      if (finite) {break;}
      EXPECT_GE(st.stand.arm_xy.dot(na), -1e-9);
      EXPECT_GE(st.stand.arm_xy.dot(nb), -1e-9);
    }
  }
  EXPECT_GT(whole_all, 0u);
  EXPECT_GT(exact_checked, 0u);
}

TEST(BasePlacementProperty, RandomWallsWithObstaclesAndPartitions)
{
  for (int seed = 101; seed <= 130; ++seed) {
    Rng rng(static_cast<uint32_t>(seed));
    SyntheticReach r;
    r.x_lo = rng.u(0.25, 0.4);
    r.x_hi = rng.u(0.8, 1.2);
    r.y_half = rng.u(0.5, 0.9);
    const Map4D map = syntheticMap(r);
    FreeSpace fs;
    const int nobs = static_cast<int>(rng.u(0.0, 3.99));
    for (int i = 0; i < nobs; ++i) {
      const double x0 = rng.u(0.8, 1.6), y0 = rng.u(-1.5, 1.5);
      fs.obstacles.push_back(FreeSpace::box(x0, y0, x0 + rng.u(0.1, 0.4), y0 + rng.u(0.1, 0.5)));
    }
    fs.walls.push_back({V2(0.0, rng.u(1.2, 2.2)), V2(0.0, -1.0)});
    const double wy = rng.u(-0.6, 0.6);
    const auto cols = vcols(wy - 0.8, wy + 0.8, 0.2);
    BasePlacementOptions o = wallOptions(rect(rng.u(0.4, 0.8), rng.u(0.4, 0.6), rng.u(0.0, 1.0) < 0.5));
    const Stand start{V2(0.0, -1.0), 0.0};
    const std::string name = "random wall seed " + std::to_string(seed);
    const BasePlacement p = planBasePlacement(map, wallFrame(), cols, fs, start, o);
    record(name, p);
    SCOPED_TRACE(name);
    checkInvariants(p, fs, o, name);
    EXPECT_GE(p.covered_fraction, 0.99 * p.reachable_fraction - 1e-9);
    if (p.whole_coverable == cols.size()) {EXPECT_EQ(p.split_count, 0u);}
    if (p.exact_min_stands && p.split_count == 0 && p.whole == p.whole_coverable) {
      EXPECT_LE(p.stand_count, *p.exact_min_stands + 1);
    }
    EXPECT_TRUE(sameStands(p, planBasePlacement(map, wallFrame(), cols, fs, start, o)));
  }
}
