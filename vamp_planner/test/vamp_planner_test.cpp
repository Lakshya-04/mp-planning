#include <gtest/gtest.h>

#include <algorithm>

#include "vamp_planner/vamp_planner.hpp"

using namespace vamp_planner;  // NOLINT
namespace mp = motion_planner;

namespace
{
// UR5 arm folded up in front of its base; the two ends differ only in the base joint.
std::vector<double> pose(double base) {return {base, -1.2, 1.4, -1.8, -1.57, 0.0};}

mp::WorldGeometryBox box(double x, double y, double z, double sx, double sy, double sz)
{
  mp::WorldGeometryBox b;
  b.id = "b";
  b.center_x = x;
  b.center_y = y;
  b.center_z = z;
  b.size_x = sx;
  b.size_y = sy;
  b.size_z = sz;
  return b;
}

mp::PlanningRequest request(double a, double b)
{
  mp::PlanningRequest r;
  r.start_joint_state = pose(a);
  r.target_joint_state = pose(b);
  return r;
}
}  // namespace

TEST(VampPlanner, KnowsItsRobots)
{
  const auto names = VampPlanner::availableRobots();
  EXPECT_NE(std::find(names.begin(), names.end(), "ur5"), names.end());
  VampPlanner p{Options{}};
  ASSERT_TRUE(p.ok());
  EXPECT_EQ(p.dimension(), 6U);
  EXPECT_EQ(p.jointNames().size(), 6U);
  Options bad;
  bad.robot = "no_such_robot";
  EXPECT_FALSE(VampPlanner(bad).ok());
}

TEST(VampPlanner, FreeSpaceMoveIsAStraightLine)
{
  VampPlanner p{Options{}};
  const auto r = p.plan(request(-0.8, 0.8));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.trajectory.points.size(), 2U);
  EXPECT_EQ(r.trajectory.joint_names.size(), 6U);
  EXPECT_GT(r.trajectory.points.back().time_from_start_sec, 0.0);
  EXPECT_NEAR(r.trajectory.points.back().positions[0], 0.8, 1e-5);
}

TEST(VampPlanner, PlansAroundABoxAndTheResultIsCollisionFree)
{
  VampPlanner p{Options{}};
  mp::WorldGeometryUpdate w;
  // The UR5 tool sweeps an arc (radius ~0.64 m, z ~1.2 m) between the two base angles; this box sits
  // on the arc where the base joint is 0, so the direct joint-space motion is blocked.
  w.boxes.push_back(box(-0.11, 0.64, 0.9, 0.2, 0.2, 0.8));
  p.setWorld(w);

  mp::CollisionRequest straight;
  straight.trajectory_joint_values = {pose(-0.8), pose(0.8)};
  const auto blocked = p.collision(straight);
  ASSERT_TRUE(blocked.success);
  ASSERT_FALSE(blocked.collision_free) << "test setup: the box must block the direct motion";

  const auto r = p.plan(request(-0.8, 0.8));
  ASSERT_TRUE(r.success) << r.failure_reason_code << ": " << r.message;
  EXPECT_GT(r.trajectory.points.size(), 2U);

  mp::CollisionRequest check;
  for (const auto & pt : r.trajectory.points) {
    check.trajectory_joint_values.push_back(pt.positions);
  }
  const auto c = p.collision(check);
  ASSERT_TRUE(c.success);
  EXPECT_TRUE(c.collision_free);
  EXPECT_NEAR(r.trajectory.points.front().positions[0], -0.8, 1e-5);
  EXPECT_NEAR(r.trajectory.points.back().positions[0], 0.8, 1e-5);
}

TEST(VampPlanner, PointCloudWorldBlocksTheArm)
{
  VampPlanner p{Options{}};
  mp::WorldGeometryUpdate w;
  for (int i = -4; i <= 4; ++i) {        // a 0.4 m x 0.8 m plate of points where the tool is at base angle 0
    for (int j = -4; j <= 4; ++j) {
      w.points.push_back({-0.11 + i * 0.05, 0.64, 1.2 + j * 0.05});
    }
  }
  p.setWorld(w);
  mp::CollisionRequest probe;
  probe.joint_values = pose(0.0);
  const auto r = p.collision(probe);
  ASSERT_TRUE(r.success);
  EXPECT_FALSE(r.collision_free);  // the tool is inside the plate at base angle 0
  EXPECT_TRUE(p.hasWorld());
  p.clearWorld();
  EXPECT_TRUE(p.collision(probe).collision_free);
}

TEST(VampPlanner, RefusesBadRequestsWithStructuredFailures)
{
  VampPlanner p{Options{}};
  mp::PlanningRequest wrong;
  wrong.start_joint_state = {0, 0};
  wrong.target_joint_state = {0, 0};
  auto r = p.plan(wrong);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "invalid_request");
  EXPECT_EQ(r.failure_stage, mp::FailureStage::kPrecheck);

  Options strict;
  strict.require_world = true;
  VampPlanner blind(strict);
  EXPECT_EQ(blind.plan(request(-0.5, 0.5)).failure_reason_code, "no_world");

  EXPECT_EQ(p.ik(mp::IkRequest{}).failure_reason_code, "unsupported");
  EXPECT_FALSE(p.collision(mp::CollisionRequest{}).success);
}

TEST(VampPlanner, ReportsStartAndGoalInCollision)
{
  VampPlanner p{Options{}};
  mp::WorldGeometryUpdate w;
  w.boxes.push_back(box(0.0, 0.0, 0.0, 6.0, 6.0, 6.0));  // swallows the whole arm
  p.setWorld(w);
  EXPECT_EQ(p.plan(request(-0.5, 0.5)).failure_reason_code, "start_in_collision");
}

// Only runs when a Cricket-generated robot named "genericarm" was compiled in
// (-DVAMP_PLANNER_ROBOT_DIR=<dir with GenericArm.hh>, from docker/tools/generate_vamp_robot.sh).
TEST(VampPlannerGenerated, PlansOnTheGeneratedMobileManipulatorArm)
{
  const auto names = VampPlanner::availableRobots();
  if (std::find(names.begin(), names.end(), "genericarm") == names.end()) {
    GTEST_SKIP() << "no generated robot compiled in";
  }
  Options o;
  o.robot = "genericarm";
  VampPlanner p(o);
  ASSERT_TRUE(p.ok());
  EXPECT_EQ(p.dimension(), 6U);
  ASSERT_EQ(p.jointNames().size(), 6U);
  EXPECT_EQ(p.jointNames()[0], "arm_joint1");

  mp::PlanningRequest r;
  r.start_joint_state = {-0.8, -0.5, 0.8, 0.0, 0.5, 0.0};
  r.target_joint_state = {0.8, -0.5, 0.8, 0.0, 0.5, 0.0};
  auto res = p.plan(r);
  ASSERT_TRUE(res.success) << res.failure_reason_code << ": " << res.message;
  EXPECT_NEAR(res.trajectory.points.back().positions[0], 0.8, 1e-5);

  // A huge obstacle puts the start in collision, so the generated spheres really are being checked.
  mp::WorldGeometryUpdate w;
  w.boxes.push_back(box(0.0, 0.0, 1.5, 8.0, 8.0, 8.0));
  p.setWorld(w);
  EXPECT_EQ(p.plan(r).failure_reason_code, "start_in_collision");
}
