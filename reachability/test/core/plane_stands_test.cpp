// Plane-generic stand search (ceiling, floor) and base free space (corners, wall ends, alcoves).
// Uses hand-made maps (synthetic_map.hpp), so no arm is built.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "reachability/core/free_space.hpp"
#include "reachability/core/global_stands.hpp"
#include "reachability/core/surface_frame.hpp"
#include "reachability/core/station.hpp"
#include "synthetic_map.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

namespace
{
BaseModel box08x05()
{
  BaseModel b;
  b.footprint = {{-0.4, -0.25}, {0.4, -0.25}, {0.4, 0.25}, {-0.4, 0.25}};   // 0.8 x 0.5 m
  return b;
}

// Vertical wall at x = 3 (normal -x), tool pointing into it, at the given y positions and heights.
StationRequest wallRequest(const std::vector<double> & ys, const std::vector<double> & zs)
{
  StationRequest r;
  for (double y : ys) {
    for (double z : zs) {r.targets_world.push_back(towardPlusX(3.0, y, z));}
  }
  r.wall_point = Vec3(3.0, 0.0, 0.0);
  r.wall_normal = Vec3(-1.0, 0.0, 0.0);
  r.arm_mount_offset_z = 0.2;
  r.lift_max = 1.5;
  r.grid_step = 0.1;
  r.lateral_margin = 0.3;
  r.base = box08x05();
  return r;
}

// Horizontal patch at height z with the tool pointing up (ceiling) or down (floor / table top).
StationRequest patchRequest(double z, bool tool_up)
{
  StationRequest r;
  for (double x = 1.0; x <= 1.4 + 1e-9; x += 0.1) {
    for (double y = -0.2; y <= 0.2 + 1e-9; y += 0.1) {
      Pose T = Pose::Identity();
      T.translation() = Vec3(x, y, z);
      if (!tool_up) {T.linear() = Eigen::AngleAxisd(M_PI, Vec3::UnitX()).toRotationMatrix();}
      r.targets_world.push_back(T);
    }
  }
  r.wall_point = Vec3(1.2, 0.0, z);
  r.wall_normal = tool_up ? Vec3(0.0, 0.0, -1.0) : Vec3(0.0, 0.0, 1.0);   // into the free side
  r.arm_mount_offset_z = 0.2;
  r.lift_min = 0.0;
  r.lift_max = 1.5;
  r.grid_step = 0.1;
  r.lateral_margin = 0.2;
  return r;
}

Stand standOf(const Station & s) {return Stand{Eigen::Vector2d(s.x, s.y), s.yaw};}

double minY(const BaseModel & b, const Stand & s)
{
  double m = 1e9;
  for (const auto & p : footprintWorld(b, s)) {m = std::min(m, p.y());}
  return m;
}
double maxY(const BaseModel & b, const Stand & s)
{
  double m = -1e9;
  for (const auto & p : footprintWorld(b, s)) {m = std::max(m, p.y());}
  return m;
}
}  // namespace

// ---- plane frame ----

TEST(SurfaceFrame, VerticalWallKeepsTheOldAxisBitForBit)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> a(-3.14, 3.14);
  for (int i = 0; i < 50; ++i) {
    const double th = a(rng);
    const Vec3 raw(std::cos(th) * 2.0, std::sin(th) * 2.0, 0.0);
    const SurfaceFrame f = surfaceFrame(raw);
    ASSERT_TRUE(f.ok);
    EXPECT_FALSE(f.horizontal);
    const Vec3 n = raw.normalized();
    Vec3 u = Vec3::UnitZ().cross(n);
    u.normalize();
    EXPECT_EQ(f.u.x(), u.x());
    EXPECT_EQ(f.u.y(), u.y());
    EXPECT_EQ(f.u.z(), u.z());
    EXPECT_EQ(f.n.x(), n.x());
  }
}

TEST(SurfaceFrame, EveryNormalGivesAnOrthonormalFrame)
{
  for (const Vec3 & raw : {Vec3(0, 0, 1), Vec3(0, 0, -1), Vec3(0, 0, -5), Vec3(1, 1, 1), Vec3(-1, 0, 0.3), Vec3(0.2, -0.5, 0.0)}) {
    const SurfaceFrame f = surfaceFrame(raw);
    ASSERT_TRUE(f.ok);
    EXPECT_NEAR(f.u.norm(), 1.0, 1e-12);
    EXPECT_NEAR(f.v.norm(), 1.0, 1e-12);
    EXPECT_NEAR(f.u.dot(f.n), 0.0, 1e-12);
    EXPECT_NEAR(f.u.dot(f.v), 0.0, 1e-12);
    EXPECT_NEAR(f.v.dot(f.n), 0.0, 1e-12);
  }
  const SurfaceFrame ceil = surfaceFrame(Vec3(0, 0, -1));
  EXPECT_TRUE(ceil.horizontal);
  EXPECT_NEAR((ceil.u - Vec3::UnitX()).norm(), 0.0, 1e-12);   // +x projected into the plane
  EXPECT_FALSE(surfaceFrame(Vec3::Zero()).ok);
  EXPECT_FALSE(surfaceFrame(Vec3(std::nan(""), 0, 1)).ok);
}

// ---- free space geometry ----

TEST(FreeSpace, WallsObstaclesAndMargin)
{
  FreeSpace fs;
  fs.walls.push_back({Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.0, -1.0)});   // wall y = 1, free side y < 1
  fs.obstacles.push_back(FreeSpace::box(2.0, -1.0, 2.5, 0.0));
  const auto fp = [](double x0, double y0, double x1, double y1) {return FreeSpace::box(x0, y0, x1, y1);};
  EXPECT_TRUE(fs.footprintFree(fp(0.0, 0.0, 0.5, 0.5), 0.1));
  EXPECT_FALSE(fs.footprintFree(fp(0.0, 0.6, 0.5, 0.95), 0.1));   // 0.05 from the wall
  EXPECT_TRUE(fs.footprintFree(fp(0.0, 0.6, 0.5, 0.95), 0.0));
  EXPECT_FALSE(fs.footprintFree(fp(1.5, -0.5, 2.1, 0.5), 0.0));   // overlaps the obstacle
  EXPECT_FALSE(fs.footprintFree(fp(1.0, -0.5, 1.95, 0.5), 0.1));  // inside the margin
  EXPECT_TRUE(fs.footprintFree(fp(1.0, -0.5, 1.8, 0.5), 0.1));
  EXPECT_FALSE(fs.footprintFree(fp(2.1, -0.2, 2.2, -0.1), 0.0));  // fully inside the obstacle
  EXPECT_NEAR(polygonDistance(fp(0, 0, 1, 1), fp(2, 0, 3, 1)), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(polygonDistance(fp(0, 0, 1, 1), fp(0.5, 0.5, 3, 3)), 0.0);
  EXPECT_TRUE(FreeSpace{}.empty());
}

// ---- ceiling and floor ----

TEST(PlaneStands, CeilingGivesAStationUnderThePatch)
{
  const Map4D map = syntheticRingMap(0.0, 0.9, 0.3, true);
  const auto r = patchRequest(2.0, true);
  const Station s = chooseStation(map, r);
  ASSERT_TRUE(s.found);
  EXPECT_EQ(s.code, StationCode::kOk);
  EXPECT_TRUE(s.complete());
  EXPECT_GE(s.x, 1.0 - 0.2 - 1e-9);
  EXPECT_LE(s.x, 1.4 + 0.2 + 1e-9);
  EXPECT_GE(s.y, -0.4 - 1e-9);
  EXPECT_LE(s.y, 0.4 + 1e-9);
  EXPECT_DOUBLE_EQ(s.standoff, 0.0);
  EXPECT_NEAR(s.x, 1.2, 0.15);   // near the middle of the patch
  EXPECT_NEAR(s.y, 0.0, 0.15);
}

TEST(PlaneStands, FloorGivesAStationOverThePatch)
{
  const Map4D map = syntheticRingMap(0.0, 0.9, 0.3, false);
  const auto r = patchRequest(0.5, false);
  const Station s = chooseStation(map, r);
  ASSERT_TRUE(s.found);
  EXPECT_TRUE(s.complete());
  EXPECT_NEAR(s.x, 1.2, 0.15);
  EXPECT_NEAR(s.y, 0.0, 0.15);
}

TEST(PlaneStands, HorizontalCandidatesAndSelectionAreNonEmpty)
{
  const Map4D map = syntheticRingMap(0.0, 0.9, 0.3, true);
  auto r = patchRequest(2.0, true);
  r.base = box08x05();
  const auto cands = makeCandidateStands(r);
  ASSERT_FALSE(cands.empty());
  for (const auto & c : cands) {EXPECT_DOUBLE_EQ(c.standoff, 0.0);}
  const auto sel = selectStands(map, r);
  EXPECT_TRUE(sel.success);
  EXPECT_EQ(sel.code, StandSelectionCode::kOk);
  EXPECT_FALSE(sel.stands.empty());
  EXPECT_TRUE(sel.unreachable.empty());
}

TEST(PlaneStands, ACeilingColumnPushesTheBaseOutOfTheWay)
{
  const Map4D map = syntheticRingMap(0.0, 1.0, 0.3, true);
  auto r = patchRequest(2.0, true);
  r.base = box08x05();
  const Station free = chooseStation(map, r);
  ASSERT_TRUE(free.found);

  FreeSpace fs;
  fs.obstacles.push_back(FreeSpace::box(1.0, -0.15, 1.4, 0.15));   // a column under the middle of the patch
  r.free_space = fs;
  const Station s = chooseStation(map, r);
  ASSERT_TRUE(s.found);
  EXPECT_GE(polygonDistance(footprintWorld(*r.base, standOf(s)), fs.obstacles[0]), r.free_space_margin - 1e-9);
  EXPECT_GT((Eigen::Vector2d(s.x, s.y) - Eigen::Vector2d(free.x, free.y)).norm(), 0.1);
}

// ---- vertical wall results ----

TEST(PlaneStands, AnEmptyFreeSpaceChangesNothingOnAWall)
{
  const Map4D map = syntheticMap();
  auto r = wallRequest({0.0}, {0.3, 0.6});
  const Station a = chooseStation(map, r);
  ASSERT_TRUE(a.found);
  r.free_space = FreeSpace{};
  const Station b = chooseStation(map, r);
  EXPECT_EQ(a.x, b.x);
  EXPECT_EQ(a.y, b.y);
  EXPECT_EQ(a.yaw, b.yaw);
  EXPECT_EQ(a.standoff, b.standoff);
  EXPECT_EQ(a.evaluated, b.evaluated);
  EXPECT_EQ(a.footprint_clearance, b.footprint_clearance);

  // The same rules as the existing wall tests: free side only, footprint clearance, short side to the wall.
  EXPECT_LT(a.x, 3.0);
  EXPECT_GE(a.footprint_clearance, 0.3 - 1e-9);
  EXPECT_GE(a.standoff, 0.3 + 0.25 - 1e-9);
  EXPECT_NEAR(std::abs(std::sin(a.yaw)), 1.0, 0.3);
  EXPECT_NEAR(a.y, 0.0, 0.35);
}

// ---- inside corner ----

TEST(PlaneStands, InsideCornerKeepsClearanceToBothWalls)
{
  const Map4D map = syntheticMap();
  auto r = wallRequest({0.9}, {0.3});   // patch against the perpendicular wall y = 1
  const Station open = chooseStation(map, r);
  ASSERT_TRUE(open.found);
  EXPECT_GT(maxY(*r.base, standOf(open)), 1.0);   // the plain search would run through the corner wall

  FreeSpace fs;
  fs.walls.push_back({Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(0.0, -1.0)});
  r.free_space = fs;
  const Station s = chooseStation(map, r);
  ASSERT_TRUE(s.found);
  const Stand st = standOf(s);
  EXPECT_LE(maxY(*r.base, st), 1.0 - r.free_space_margin + 1e-9);
  EXPECT_GE(s.footprint_clearance, r.min_clearance - 1e-9);   // and the work wall

  for (const auto & c : makeCandidateStands(r)) {
    EXPECT_LE(maxY(*r.base, c.stand), 1.0 - r.free_space_margin + 1e-9);
    EXPECT_GE(c.footprint_clearance, r.min_clearance - 1e-9);
  }
}

// ---- outside corner / wall end ----

TEST(PlaneStands, AStandBeyondAWallEndIsAllowed)
{
  auto r = wallRequest({0.8}, {0.3});
  r.lateral_margin = 0.9;   // the grid runs past the wall end at y = 1
  const double margin = r.free_space_margin;
  const auto wall_poly = FreeSpace::box(3.0, -5.0, 3.2, 1.0);   // the wall itself ends at y = 1
  const double shallow = r.min_clearance + 0.25;                // the closest an infinite wall allows

  auto closest = [](const std::vector<CandidateStand> & cs) {
      double m = 1e9;
      for (const auto & c : cs) {m = std::min(m, c.standoff);}
      return m;
    };
  const auto infinite = makeCandidateStands(r);
  ASSERT_FALSE(infinite.empty());
  EXPECT_GE(closest(infinite), shallow - 1e-9);

  FreeSpace fs;
  fs.has_work_span = true;
  fs.work_span_lo = -1.0;     // u = -y: the wall covers y <= 1
  fs.work_span_hi = 10.0;
  fs.obstacles.push_back(wall_poly);
  r.free_space = fs;
  const auto finite = makeCandidateStands(r);
  ASSERT_FALSE(finite.empty());
  bool beyond = false;
  for (const auto & c : finite) {
    EXPECT_GE(polygonDistance(footprintWorld(*r.base, c.stand), wall_poly), margin - 1e-9);
    if (c.standoff < shallow - 1e-9 && minY(*r.base, c.stand) > 1.0) {beyond = true;}
  }
  EXPECT_TRUE(beyond);   // a stand past the end, closer to the wall plane than an infinite wall allows
  EXPECT_LT(closest(finite), shallow - 1e-9);
}

// ---- alcove ----

TEST(PlaneStands, AnAlcoveTooNarrowForTheBaseReportsNoStand)
{
  const Map4D map = syntheticMap();
  auto r = wallRequest({0.0}, {0.3});
  FreeSpace fs;
  fs.walls.push_back({Eigen::Vector2d(0.0, 0.25), Eigen::Vector2d(0.0, -1.0)});
  fs.walls.push_back({Eigen::Vector2d(0.0, -0.25), Eigen::Vector2d(0.0, 1.0)});   // 0.5 m wide, base is 0.5 m + margins
  r.free_space = fs;

  EXPECT_TRUE(makeCandidateStands(r).empty());
  const Station s = chooseStation(map, r);
  EXPECT_FALSE(s.found);
  EXPECT_EQ(s.code, StationCode::kNoFreeStand);
  EXPECT_STREQ(toString(s.code), "no_free_stand");

  const auto sel = selectStands(map, r);
  EXPECT_FALSE(sel.success);
  EXPECT_EQ(sel.code, StandSelectionCode::kNoFreeStand);
  EXPECT_STREQ(toString(sel.code), "no_free_stand");
  EXPECT_TRUE(sel.stands.empty());
  EXPECT_EQ(sel.unreachable.size(), r.targets_world.size());

  // wide enough: a stand exists and fits
  r.free_space->walls[0].point = Eigen::Vector2d(0.0, 1.0);
  r.free_space->walls[1].point = Eigen::Vector2d(0.0, -1.0);
  const Station wide = chooseStation(map, r);
  ASSERT_TRUE(wide.found);
  EXPECT_EQ(wide.code, StationCode::kOk);
  EXPECT_GE(minY(*r.base, standOf(wide)), -1.0 + r.free_space_margin - 1e-9);
  EXPECT_LE(maxY(*r.base, standOf(wide)), 1.0 - r.free_space_margin + 1e-9);
}

TEST(PlaneStands, NothingReachableIsNotNoFreeStand)
{
  const Map4D map = syntheticMap();
  auto r = wallRequest({0.0}, {5.0});   // far above any reach
  const Station s = chooseStation(map, r);
  EXPECT_FALSE(s.found);
  EXPECT_EQ(s.code, StationCode::kNoReachableStand);
  EXPECT_EQ(chooseStation(map, wallRequest({}, {})).code, StationCode::kInvalidRequest);
}

// ---- determinism ----

TEST(PlaneStands, ResultsAreDeterministic)
{
  const Map4D map = syntheticMap();
  auto r = wallRequest({0.3, 0.9}, {0.3});
  FreeSpace fs;
  fs.walls.push_back({Eigen::Vector2d(0.0, 1.2), Eigen::Vector2d(0.0, -1.0)});
  fs.obstacles.push_back(FreeSpace::box(1.0, -0.2, 1.4, 0.2));
  r.free_space = fs;
  const Station a = chooseStation(map, r), b = chooseStation(map, r);
  EXPECT_EQ(a.x, b.x);
  EXPECT_EQ(a.y, b.y);
  EXPECT_EQ(a.yaw, b.yaw);
  EXPECT_EQ(a.evaluated, b.evaluated);
  const auto ca = makeCandidateStands(r), cb = makeCandidateStands(r);
  ASSERT_EQ(ca.size(), cb.size());
  for (size_t i = 0; i < ca.size(); ++i) {
    EXPECT_EQ(ca[i].stand.arm_xy, cb[i].stand.arm_xy);
    EXPECT_EQ(ca[i].stand.yaw, cb[i].stand.yaw);
  }
  const auto sa = selectStands(map, r), sb = selectStands(map, r);
  ASSERT_EQ(sa.stands.size(), sb.stands.size());
  for (size_t i = 0; i < sa.stands.size(); ++i) {EXPECT_EQ(sa.stands[i].stand.arm_xy, sb.stands[i].stand.arm_xy);}
}
