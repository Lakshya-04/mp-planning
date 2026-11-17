#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <moveit/robot_model/robot_model.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include "motion_planner_moveit/moveit_time_parameterizer.hpp"

namespace motion_planner
{
namespace
{

// A lift (prismatic) and two revolute joints carrying a 0.5 m link: a mixed-type chain.
std::shared_ptr<moveit::core::RobotModel> makeModel()
{
  const char * urdf_xml = R"(
<robot name="r">
  <link name="base"/><link name="a"/><link name="b"/><link name="tip"/>
  <joint name="lift" type="prismatic"><parent link="base"/><child link="a"/>
    <axis xyz="0 0 1"/><limit lower="0.0" upper="0.5" effort="1" velocity="0.2"/></joint>
  <joint name="j1" type="revolute"><parent link="a"/><child link="b"/>
    <axis xyz="0 0 1"/><limit lower="-3.0" upper="3.0" effort="1" velocity="1.0"/></joint>
  <joint name="j2" type="revolute"><parent link="b"/><child link="tip"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="1" velocity="1.0"/></joint>
</robot>)";
  const char * srdf_xml = R"(
<robot name="r"><group name="g"><joint name="lift"/><joint name="j1"/><joint name="j2"/></group></robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  EXPECT_TRUE(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  EXPECT_TRUE(srdf_model->initString(*urdf_model, srdf_xml));
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

core_types::Trajectory line(size_t n, double j1_from, double j1_to)
{
  core_types::Trajectory t;
  t.joint_names = {"lift", "j1", "j2"};
  for (size_t i = 0; i < n; ++i) {
    const double f = static_cast<double>(i) / static_cast<double>(n - 1);
    core_types::TrajectoryPoint p;
    p.positions = {0.1, j1_from + f * (j1_to - j1_from), 0.3};
    t.points.push_back(p);
  }
  return t;
}

TimingRequest request(SegmentRole role, const MoveItTimeParameterizer & r, double speed)
{
  TimingRequest req;
  req.role = role;
  req.limits = r.modelLimits(2.0, role == SegmentRole::kContactSeam ? 20.0 : 0.0);
  req.stroke_speed = speed;
  return req;
}

TEST(MoveItTimeParameterizerTest, ModelLimitsFollowTheGroup)
{
  MoveItTimeParameterizer retimer(makeModel(), "g");
  const auto limits = retimer.modelLimits(2.0, 5.0);
  ASSERT_EQ(limits.size(), 3u);
  EXPECT_DOUBLE_EQ(limits[0].velocity, 0.2);
  EXPECT_DOUBLE_EQ(limits[1].velocity, 1.0);
  EXPECT_DOUBLE_EQ(limits[2].acceleration, 2.0);
  EXPECT_DOUBLE_EQ(limits[2].jerk, 5.0);
}

TEST(MoveItTimeParameterizerTest, TransitionIsTimeOptimalWithinLimits)
{
  MoveItTimeParameterizer retimer(makeModel(), "g");
  const auto r = parameterizeSegment(
    retimer, line(8, -1.0, 1.0), request(SegmentRole::kTransition, retimer, 0.0));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_TRUE(r.violations.empty());
  ASSERT_EQ(r.pieces.size(), 1u);
  const auto & pts = r.pieces[0].points;
  EXPECT_NEAR(pts.front().positions[1], -1.0, 1e-6);
  EXPECT_NEAR(pts.back().positions[1], 1.0, 1e-6);
  EXPECT_NEAR(pts.back().velocities[1], 0.0, 1e-6);
  // 2 rad at 1 rad/s and 2 rad/s^2: at least 2 s; trapezoid is about 3 s
  EXPECT_GT(r.total_duration_sec, 2.0);
  EXPECT_LT(r.total_duration_sec, 4.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    EXPECT_GT(pts[i].time_from_start_sec, pts[i - 1].time_from_start_sec);
  }
}

TEST(MoveItTimeParameterizerTest, SeamRunsAtTheStrokeSpeedWithinJerk)
{
  MoveItTimeParameterizer retimer(makeModel(), "g");
  const auto r = parameterizeSegment(
    retimer, line(20, -0.3, 0.3), request(SegmentRole::kContactSeam, retimer, 0.05));
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_EQ(r.pieces.size(), 1u);
  const auto & pts = r.pieces[0].points;
  const size_t m = pts.size() / 2;
  // the tip is 0.5 m from the j1 axis: 0.05 m/s is 0.1 rad/s on j1
  EXPECT_NEAR(std::fabs(pts[m].velocities[1]), 0.1, 0.03);
  EXPECT_NEAR(pts.back().velocities[1], 0.0, 1e-3);
  EXPECT_TRUE(r.violations.empty());
}

TEST(MoveItTimeParameterizerTest, SeamTooFastForTheJointsFails)
{
  MoveItTimeParameterizer retimer(makeModel(), "g");
  // the lift moves 0.4 m while the tool is asked for 1 m/s: well above the 0.2 m/s bound
  core_types::Trajectory t = line(6, 0.0, 0.0);
  for (size_t i = 0; i < t.points.size(); ++i) {
    t.points[i].positions[0] = 0.4 * static_cast<double>(i) / 5.0;
  }
  const auto r = parameterizeSegment(
    retimer, t, request(SegmentRole::kContactSeam, retimer, 1.0));
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.error == TimingError::kLimitViolation || r.error == TimingError::kBackendFailed);
}

TEST(MoveItTimeParameterizerTest, ReversalSplitsAndEachPieceRestsAtTheCut)
{
  MoveItTimeParameterizer retimer(makeModel(), "g");
  auto t = line(5, 0.0, 0.8);
  const auto back = line(5, 0.8, 0.0);
  t.points.insert(t.points.end(), back.points.begin() + 1, back.points.end());
  const auto r = parameterizeSegment(
    retimer, t, request(SegmentRole::kTransition, retimer, 0.0));
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_EQ(r.pieces.size(), 2u);
  EXPECT_NEAR(r.pieces[0].points.back().velocities[1], 0.0, 1e-6);
  EXPECT_NEAR(r.pieces[1].points.front().velocities[1], 0.0, 5e-3);
  const auto joined = concatenate(r.pieces);
  EXPECT_NEAR(joined.points.back().time_from_start_sec, r.total_duration_sec, 1e-9);
}

TEST(MoveItTimeParameterizerTest, MismatchedJointsAreRefusedNotCrashed)
{
  MoveItTimeParameterizer retimer(makeModel(), "g");
  auto t = line(4, 0.0, 0.5);
  t.joint_names[1] = "other";
  const auto r = parameterizeSegment(
    retimer, t, request(SegmentRole::kTransition, retimer, 0.0));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.error, TimingError::kBackendFailed);
}

TEST(MoveItTimeParameterizerTest, SeamWithoutJerkLimitIsRefused)
{
  MoveItTimeParameterizer retimer(makeModel(), "g");
  auto req = request(SegmentRole::kContactSeam, retimer, 0.05);
  req.limits[1].jerk = 0.0;
  const auto r = parameterizeSegment(retimer, line(6, 0.0, 0.3), req);
  EXPECT_EQ(r.error, TimingError::kBackendFailed);
}

}  // namespace
}  // namespace motion_planner
