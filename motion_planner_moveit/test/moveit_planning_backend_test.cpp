#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "motion_planner_moveit/cartesian_planner.hpp"
#include "motion_planner_moveit/ompl_planner.hpp"

namespace motion_planner
{
namespace
{

// The actual bug fix, made testable with no new production-only API: a
// shared_ptr refcount of 3 (this test's own copy + CartesianPlanner's +
// OmplPlanner's) is only reachable if both planners stored the literal same
// MoveItPlanningBackend instance -- directly proving the single-ground-truth
// property this class exists to provide, without needing to reach into
// either planner's private state.
TEST(MoveItPlanningBackendTest, BothPlannersShareTheSameBackendInstance)
{
  auto backend = std::make_shared<MoveItPlanningBackend>();
  EXPECT_EQ(backend.use_count(), 1);

  CartesianPlanner cartesian(backend);
  EXPECT_EQ(backend.use_count(), 2);

  OmplPlanner ompl(backend);
  EXPECT_EQ(backend.use_count(), 3);
}

TEST(MoveItPlanningBackendTest, CollisionRequestRequiresExplicitInput)
{
  MoveItPlanningBackend backend;
  const CollisionRequest request;

  const CollisionResult result = backend.collision(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.collision_free);
  EXPECT_EQ(result.failure_reason_code, "missing_collision_input");
}

TEST(MoveItPlanningBackendTest, CollisionTrajectoryRejectsInvalidJointVector)
{
  MoveItPlanningBackend backend;
  CollisionRequest request;
  request.trajectory_joint_values = {{}};

  const CollisionResult result = backend.collision(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.collision_free);
  EXPECT_EQ(result.failure_reason_code, "invalid_joint_values");
}

TEST(MoveItPlanningBackendTest, IkRequiresTargetPose)
{
  MoveItPlanningBackend backend;
  const IkRequest request;

  const IkResult result = backend.ik(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason_code, "missing_target_pose");
  EXPECT_EQ(result.failure_stage, FailureStage::kPrecheck);
}

TEST(MoveItPlanningBackendTest, EnsureMoveGroupFailsWithoutRclcppInit)
{
  MoveItPlanningBackend backend;
  std::string error_message;

  const bool ok = backend.ensureMoveGroup("arm_7dof", error_message);

  EXPECT_FALSE(ok);
  EXPECT_EQ(error_message, "rclcpp_not_initialized");
}

TEST(MoveItPlanningBackendTest, AccessorsAreEmptyBeforeEnsureMoveGroup)
{
  MoveItPlanningBackend backend;

  EXPECT_EQ(backend.node(), nullptr);
  EXPECT_EQ(backend.moveGroup(), nullptr);
  EXPECT_TRUE(backend.activeMoveGroupName().empty());
}

TEST(MoveItPlanningBackendTest, ParseTargetPoseAcceptsSevenValueQuaternionPassthrough)
{
  geometry_msgs::msg::Pose pose;
  std::string error_message;

  const bool ok = MoveItPlanningBackend::parseTargetPose(
    {1.0, 2.0, 3.0, 0.0, 0.0, 0.70710678, 0.70710678}, pose, error_message);

  ASSERT_TRUE(ok);
  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.70710678);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 0.70710678);
}

TEST(MoveItPlanningBackendTest, ParseTargetPoseConvertsSixValueRpyToQuaternion)
{
  geometry_msgs::msg::Pose pose;
  std::string error_message;

  // roll=0, pitch=0, yaw=pi/2 -- a pure yaw rotation, whose quaternion is known exactly
  // (z=sin(yaw/2), w=cos(yaw/2)) rather than needing tf2 itself to check tf2's own conversion.
  const bool ok = MoveItPlanningBackend::parseTargetPose(
    {0.1, 0.2, 0.3, 0.0, 0.0, M_PI_2}, pose, error_message);

  ASSERT_TRUE(ok);
  EXPECT_DOUBLE_EQ(pose.position.x, 0.1);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.2);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.3);
  EXPECT_NEAR(pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.z, std::sin(M_PI_4), 1e-9);
  EXPECT_NEAR(pose.orientation.w, std::cos(M_PI_4), 1e-9);
}

TEST(MoveItPlanningBackendTest, ParseTargetPoseRejectsWrongSizedVector)
{
  geometry_msgs::msg::Pose pose;
  std::string error_message;

  const bool ok = MoveItPlanningBackend::parseTargetPose({1.0, 2.0, 3.0}, pose, error_message);

  EXPECT_FALSE(ok);
  EXPECT_EQ(
    error_message, "target_pose must contain 6 values (xyz+rpy) or 7 values (xyz+quaternion)");
}

// A lift (prismatic, bounded), a limited revolute and a continuous joint, in one chain.
TEST(MoveItPlanningBackendTest, SeedingConfigFromGroupReadsLimitsAndRevoluteMask)
{
  const char * urdf_xml = R"(
<robot name="r">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/>
  <joint name="lift" type="prismatic"><parent link="base"/><child link="a"/>
    <axis xyz="0 0 1"/><limit lower="0.0" upper="0.5" effort="1" velocity="1"/></joint>
  <joint name="shoulder" type="revolute"><parent link="a"/><child link="b"/>
    <axis xyz="0 0 1"/><limit lower="-2.0" upper="3.0" effort="1" velocity="1"/></joint>
  <joint name="wrist" type="continuous"><parent link="b"/><child link="c"/>
    <axis xyz="0 0 1"/></joint>
</robot>)";
  const char * srdf_xml = R"(
<robot name="r"><group name="g"><joint name="lift"/><joint name="shoulder"/>
  <joint name="wrist"/></group></robot>)";
  auto urdf_model = std::make_shared<urdf::Model>();
  ASSERT_TRUE(urdf_model->initString(urdf_xml));
  auto srdf_model = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf_model->initString(*urdf_model, srdf_xml));
  moveit::core::RobotModel model(urdf_model, srdf_model);

  SeedPreferringIkConfig config;
  ASSERT_TRUE(
    MoveItPlanningBackend::seedingConfigFromGroup(model.getJointModelGroup("g"), config));
  ASSERT_EQ(config.wrap_joint.size(), 3u);
  EXPECT_FALSE(config.wrap_joint[0]);
  EXPECT_TRUE(config.wrap_joint[1]);
  EXPECT_TRUE(config.wrap_joint[2]);
  EXPECT_DOUBLE_EQ(config.lower_limits[0], 0.0);
  EXPECT_DOUBLE_EQ(config.upper_limits[0], 0.5);
  EXPECT_DOUBLE_EQ(config.lower_limits[1], -2.0);
  EXPECT_DOUBLE_EQ(config.upper_limits[1], 3.0);
  EXPECT_TRUE(std::isinf(config.lower_limits[2]));
  EXPECT_TRUE(std::isinf(config.upper_limits[2]));
  // distance weights follow the joint type: 1 m of the prismatic joint counts as 6 rad by default
  ASSERT_EQ(config.weights.size(), 3u);
  EXPECT_DOUBLE_EQ(config.weights[0], 36.0);
  EXPECT_DOUBLE_EQ(config.weights[1], 1.0);
  EXPECT_DOUBLE_EQ(config.weights[2], 1.0);
  SeedPreferringIkConfig custom;
  ASSERT_TRUE(MoveItPlanningBackend::seedingConfigFromGroup(model.getJointModelGroup("g"), custom, 2.0));
  EXPECT_DOUBLE_EQ(custom.weights[0], 4.0);

  EXPECT_FALSE(MoveItPlanningBackend::seedingConfigFromGroup(nullptr, config));
}

}  // namespace
}  // namespace motion_planner
