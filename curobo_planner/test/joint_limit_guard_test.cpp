// The joint limit guard with hand-made trajectories (no ROS, no cuRobo).
#include <gtest/gtest.h>

#include <cmath>

#include "curobo_planner/joint_limit_guard.hpp"

namespace cp = curobo_planner;
namespace mp = motion_planner;

namespace
{
mp::JointLimit lim(double v, double a)
{
  mp::JointLimit l;
  l.velocity = v;
  l.acceleration = a;
  return l;
}

// n points, dt apart, joint 0 moving `step` per point, joint 1 still
core_types::Trajectory ramp(int n, double dt, double step)
{
  core_types::Trajectory t;
  t.joint_names = {"a", "b"};
  for (int i = 0; i < n; ++i) {
    core_types::TrajectoryPoint p;
    p.positions = {i * step, 0.3};
    p.time_from_start_sec = i * dt;
    t.points.push_back(p);
  }
  return t;
}

cp::JointLimitPolicy policy(double factor = 1.0)
{
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(1.0, 100.0)}, {"b", lim(1.0, 100.0)}};
  p.safety_factor = factor;
  return p;
}
}  // namespace

TEST(JointLimitGuard, SlowTrajectoryIsUntouched)
{
  auto t = ramp(5, 0.1, 0.05);  // 0.5 rad/s, constant
  const auto before = t;
  const auto c = cp::enforceJointLimits(t, policy(0.9));
  EXPECT_TRUE(c.ok) << c.message;
  EXPECT_DOUBLE_EQ(c.time_scale, 1.0);
  EXPECT_NEAR(c.max_velocity_ratio, 0.5, 1e-9);
  for (size_t i = 0; i < t.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(t.points[i].time_from_start_sec, before.points[i].time_from_start_sec);
  }
}

TEST(JointLimitGuard, FastTrajectoryIsRetimedToTheSafetyFactor)
{
  auto t = ramp(5, 0.025, 0.05);  // 2 rad/s against 1 rad/s
  const auto c = cp::enforceJointLimits(t, policy(0.9));
  ASSERT_TRUE(c.ok) << c.message;
  EXPECT_NEAR(c.time_scale, 2.0 / 0.9, 1e-9);
  EXPECT_NEAR(t.points.back().time_from_start_sec, 0.1 * 2.0 / 0.9, 1e-9);
  EXPECT_NEAR(c.duration_after_sec, t.points.back().time_from_start_sec, 1e-12);
  // positions untouched: only the time axis moves
  EXPECT_DOUBLE_EQ(t.points[4].positions[0], 0.2);
  // re-checking the result: it now sits at the factor, nothing more to do
  const auto again = cp::enforceJointLimits(t, policy(0.9));
  EXPECT_TRUE(again.ok);
  EXPECT_DOUBLE_EQ(again.time_scale, 1.0);
  EXPECT_NEAR(again.max_velocity_ratio, 0.9, 1e-9);
}

TEST(JointLimitGuard, AccelerationLimitsTheScaleAsASquareRoot)
{
  // rest, then a jump of velocity: v = 0, 1, 1 over dt 0.1 => a = 10 rad/s^2 against 4
  core_types::Trajectory t;
  t.joint_names = {"a"};
  const double q[] = {0.0, 0.0, 0.1, 0.2};
  for (int i = 0; i < 4; ++i) {
    core_types::TrajectoryPoint p;
    p.positions = {q[i]};
    p.time_from_start_sec = i * 0.1;
    t.points.push_back(p);
  }
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(5.0, 4.0)}};
  p.safety_factor = 1.0;
  const auto c = cp::enforceJointLimits(t, p);
  ASSERT_TRUE(c.ok) << c.message;
  EXPECT_NEAR(c.max_acceleration_ratio, 10.0 / 4.0, 1e-9);
  EXPECT_NEAR(c.time_scale, std::sqrt(2.5), 1e-9);
  // the stretched trajectory meets the acceleration limit exactly
  const auto after = cp::enforceJointLimits(t, p);
  EXPECT_NEAR(after.max_acceleration_ratio, 1.0, 1e-9);
}

TEST(JointLimitGuard, RejectModeFailsWithAReasonCodeAndKeepsTheTrajectory)
{
  auto t = ramp(5, 0.025, 0.05);
  const auto before = t;
  auto p = policy(0.9);
  p.mode = cp::LimitMode::kReject;
  const auto c = cp::enforceJointLimits(t, p);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "limit_violation");
  EXPECT_DOUBLE_EQ(t.points[4].time_from_start_sec, before.points[4].time_from_start_sec);
  // reject mode lets a conforming trajectory through
  auto slow = ramp(5, 0.1, 0.05);
  EXPECT_TRUE(cp::enforceJointLimits(slow, p).ok);
}

TEST(JointLimitGuard, LimitsAreMatchedByNameNotByPosition)
{
  auto t = ramp(5, 0.025, 0.05);  // joint "a" moves at 2 rad/s
  t.joint_names = {"b", "a"};     // the moving joint is now listed second... and is named "b" at index 0
  // index 0 (moving) is "b": give "b" a generous limit and "a" the tight one => untouched
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(0.1, 100.0)}, {"b", lim(5.0, 100.0)}};
  p.safety_factor = 1.0;
  auto c = cp::enforceJointLimits(t, p);
  EXPECT_TRUE(c.ok);
  EXPECT_DOUBLE_EQ(c.time_scale, 1.0);
  // swap the limits: now the moving joint is the tight one
  p.limits = {{"a", lim(5.0, 100.0)}, {"b", lim(1.0, 100.0)}};
  c = cp::enforceJointLimits(t, p);
  EXPECT_NEAR(c.time_scale, 2.0, 1e-9);
}

TEST(JointLimitGuard, AJointWithoutALimitIsAnError)
{
  auto t = ramp(5, 0.1, 0.01);
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(1.0, 1.0)}};
  const auto c = cp::enforceJointLimits(t, p);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "limit_missing");
  EXPECT_NE(c.message.find("'b'"), std::string::npos);
}

TEST(JointLimitGuard, BadInputsAreReported)
{
  auto t = ramp(3, 0.1, 0.01);
  auto p = policy();
  p.safety_factor = 0.0;
  EXPECT_EQ(cp::enforceJointLimits(t, p).reason_code, "limits_invalid");
  p = policy();
  p.limits["a"].velocity = 0.0;
  EXPECT_EQ(cp::enforceJointLimits(t, p).reason_code, "limits_invalid");
  t.points[2].time_from_start_sec = t.points[1].time_from_start_sec;
  EXPECT_EQ(cp::enforceJointLimits(t, policy()).reason_code, "invalid_trajectory");
  auto ragged = ramp(3, 0.1, 0.01);
  ragged.points[1].positions.pop_back();
  EXPECT_EQ(cp::enforceJointLimits(ragged, policy()).reason_code, "invalid_trajectory");
}

TEST(JointLimitGuard, NoLimitsIsANoOp)
{
  auto t = ramp(5, 0.025, 0.5);
  cp::JointLimitPolicy none;
  EXPECT_FALSE(none.enabled());
  const auto c = cp::enforceJointLimits(t, none);
  EXPECT_TRUE(c.ok);
  EXPECT_DOUBLE_EQ(t.points[4].time_from_start_sec, 0.1);
}

TEST(JointLimitGuard, ProvidedVelocityVectorsAreScaledWithTime)
{
  auto t = ramp(3, 0.1, 0.01);  // slow by differences
  for (auto & p : t.points) {
    p.velocities = {3.0, 0.0};  // but the planner says 3 rad/s
    p.accelerations = {0.0, 0.0};
  }
  const auto c = cp::enforceJointLimits(t, policy(1.0));
  ASSERT_TRUE(c.ok);
  EXPECT_NEAR(c.time_scale, 3.0, 1e-9);
  EXPECT_NEAR(t.points[1].velocities[0], 1.0, 1e-9);
}

TEST(MoveItJointLimits, ParsesNamesAndSkipsUnlimitedJoints)
{
  const std::string yaml =
    "default_velocity_scaling_factor: 0.5\n"
    "default_acceleration_scaling_factor: 0.25\n"
    "joint_limits:\n"
    "  lift_joint: {has_velocity_limits: true, max_velocity: 0.2, has_acceleration_limits: true, max_acceleration: 0.5}\n"
    "  arm_1: &f {has_velocity_limits: true, max_velocity: 2.0, has_acceleration_limits: true, max_acceleration: 3.0}\n"
    "  arm_2: *f\n"
    "  half: {has_velocity_limits: true, max_velocity: 2.0, has_acceleration_limits: false}\n";
  std::map<std::string, mp::JointLimit> limits;
  std::string error;
  double vs = 0, as = 0;
  ASSERT_TRUE(cp::parseMoveItJointLimits(yaml, limits, error, &vs, &as)) << error;
  EXPECT_EQ(limits.size(), 3U);
  EXPECT_DOUBLE_EQ(limits.at("lift_joint").velocity, 0.2);
  EXPECT_DOUBLE_EQ(limits.at("arm_2").acceleration, 3.0);
  EXPECT_EQ(limits.count("half"), 0U);  // not unlimited: absent, so the guard reports it
  EXPECT_DOUBLE_EQ(vs, 0.5);
  EXPECT_DOUBLE_EQ(as, 0.25);
  EXPECT_FALSE(cp::parseMoveItJointLimits("foo: 1\n", limits, error));
  EXPECT_FALSE(cp::loadMoveItJointLimits("/nonexistent/joint_limits.yaml", limits, error));
}

TEST(MoveItJointLimits, ShippedDescriptionFileCoversTheLiftArmJoints)
{
  std::map<std::string, mp::JointLimit> limits;
  std::string error;
  ASSERT_TRUE(cp::loadMoveItJointLimits(SHIPPED_JOINT_LIMITS_YAML, limits, error)) << error;
  for (const char * j : {"lift_joint", "arm_shoulder_pan_joint", "arm_wrist_3_joint"}) {
    EXPECT_EQ(limits.count(j), 1U) << j;
  }
}

TEST(InlineJointLimits, ParsesAndRejectsMalformedEntries)
{
  std::map<std::string, mp::JointLimit> limits;
  std::string error;
  ASSERT_TRUE(cp::parseInlineJointLimits("j1:1.5:3, j2 : 0.2 : 0.5", limits, error)) << error;
  EXPECT_DOUBLE_EQ(limits.at("j2").velocity, 0.2);
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:1.5", limits, error));
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:abc:3", limits, error));
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:0:3", limits, error));
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:1:3:4", limits, error));
}
