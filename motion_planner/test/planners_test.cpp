#include <memory>

#include <gtest/gtest.h>

#include "motion_planner/moveit_planning_backend.hpp"
#include "motion_planner/planners/cartesian_planner.hpp"
#include "motion_planner/planners/ompl_planner.hpp"

namespace motion_planner
{
namespace
{

TEST(OmplPlannerTest, CollisionRequestRequiresExplicitInput)
{
  OmplPlanner planner(std::make_shared<MoveItPlanningBackend>());
  const CollisionRequest request;

  const CollisionResult result = planner.collision(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.collision_free);
  EXPECT_EQ(result.failure_reason_code, "missing_collision_input");
}

TEST(OmplPlannerTest, CollisionTrajectoryRejectsInvalidJointVector)
{
  OmplPlanner planner(std::make_shared<MoveItPlanningBackend>());
  CollisionRequest request;
  request.trajectory_joint_values = {{}};

  const CollisionResult result = planner.collision(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.collision_free);
  EXPECT_EQ(result.failure_reason_code, "invalid_joint_values");
}

TEST(OmplPlannerTest, PlanRequiresTargetPosesOrJointState)
{
  OmplPlanner planner(std::make_shared<MoveItPlanningBackend>());
  const PlanningRequest request;

  const PlanningResult result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason_code, "missing_target_poses");
  EXPECT_EQ(result.failure_stage, FailureStage::kPrecheck);
}

TEST(OmplPlannerTest, PlanAcceptsTargetJointStateWithoutTargetPoses)
{
  OmplPlanner planner(std::make_shared<MoveItPlanningBackend>());
  PlanningRequest request;
  request.target_joint_state = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};

  const PlanningResult result = planner.plan(request);

  EXPECT_FALSE(result.success);
  // Bypasses the missing_target_poses precheck now that target_joint_state
  // is set -- fails at the next stage instead (no live rclcpp node in this
  // offline gtest), proving the precheck logic actually changed rather than
  // still requiring target_poses under the hood.
  EXPECT_NE(result.failure_reason_code, "missing_target_poses");
  EXPECT_EQ(result.failure_reason_code, "planner_init");
}

TEST(CartesianPlannerTest, PlanRequiresTargetPoses)
{
  CartesianPlanner planner(std::make_shared<MoveItPlanningBackend>());
  const PlanningRequest request;

  const PlanningResult result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason_code, "missing_target_poses");
  EXPECT_EQ(result.failure_stage, FailureStage::kPrecheck);
}

TEST(CartesianPlannerTest, PlanWithTargetPosesFailsAtInitWithoutLiveNode)
{
  // Bypasses the missing_target_poses precheck (target_poses is set), reaching
  // the robot-model/local-CartesianInterpolator computation this session added
  // -- proving that code path doesn't crash offline, just fails cleanly at
  // planner_init (no live rclcpp node in this test), same as OmplPlanner's own
  // PlanAcceptsTargetJointStateWithoutTargetPoses. Genuine jump/collision/
  // ik_failure disambiguation -- and the consistency-limited collision-recovery
  // retry built on top of it (also nested inside this same live-backend-only
  // path, unreachable past planner_init) -- needs a real robot model + planning
  // scene, which this offline suite doesn't have -- verified live instead (see
  // cartesian_test.launch.py / motion_planner_test.launch.py scenario_group:=cartesian).
  CartesianPlanner planner(std::make_shared<MoveItPlanningBackend>());
  PlanningRequest request;
  request.target_poses.push_back(core_types::Pose{});

  const PlanningResult result = planner.plan(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.failure_reason_code, "missing_target_poses");
  EXPECT_EQ(result.failure_reason_code, "planner_init");
  EXPECT_EQ(result.truncation_reason, CartesianTruncationReason::kNone);
}

TEST(CartesianPlannerTest, IkRequiresTargetPose)
{
  CartesianPlanner planner(std::make_shared<MoveItPlanningBackend>());
  const IkRequest request;

  const IkResult result = planner.ik(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_reason_code, "missing_target_pose");
  EXPECT_EQ(result.failure_stage, FailureStage::kPrecheck);
}

TEST(CartesianPlannerTest, CollisionRequestRequiresExplicitInput)
{
  CartesianPlanner planner(std::make_shared<MoveItPlanningBackend>());
  const CollisionRequest request;

  const CollisionResult result = planner.collision(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.collision_free);
  EXPECT_EQ(result.failure_reason_code, "missing_collision_input");
}

TEST(CartesianPlannerTest, CollisionTrajectoryRejectsInvalidJointVector)
{
  CartesianPlanner planner(std::make_shared<MoveItPlanningBackend>());
  CollisionRequest request;
  request.trajectory_joint_values = {{}};

  const CollisionResult result = planner.collision(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.collision_free);
  EXPECT_EQ(result.failure_reason_code, "invalid_joint_values");
}

}  // namespace
}  // namespace motion_planner
