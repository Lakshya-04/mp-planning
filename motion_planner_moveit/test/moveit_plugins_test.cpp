#include <gtest/gtest.h>

#include <algorithm>

#include "motion_planner/ros/planning_config.hpp"

namespace motion_planner
{
namespace
{

TEST(MoveItPluginsTest, EveryMoveItPluginLoadsByName)
{
  auto registry = makePluginlibRegistry();
  PluginContext context;
  PluginStatus status;

  auto cartesian = registry->createPlanner("moveit_cartesian", status);
  ASSERT_TRUE(cartesian != nullptr) << status.message;
  EXPECT_TRUE(cartesian->initialize(context, {}).ok);
  EXPECT_EQ(cartesian->name(), "cartesian");

  auto ompl = registry->createPlanner("moveit_ompl", status);
  ASSERT_TRUE(ompl != nullptr) << status.message;
  EXPECT_TRUE(ompl->initialize(context, {}).ok);
  EXPECT_EQ(ompl->name(), "ompl");

  auto ik = registry->createIk("moveit_ik", status);
  ASSERT_TRUE(ik != nullptr) << status.message;
  EXPECT_TRUE(ik->initialize(context, {}).ok);

  auto collision = registry->createCollision("moveit_collision", status);
  ASSERT_TRUE(collision != nullptr) << status.message;
  EXPECT_TRUE(collision->initialize(context, {}).ok);

  auto scene = registry->createScene("moveit_scene", status);
  ASSERT_TRUE(scene != nullptr) << status.message;
  EXPECT_TRUE(scene->initialize(context, {}).ok);
}

TEST(MoveItPluginsTest, DefaultSelectionBuildsThroughPluginlibAndKeepsCartesianFirst)
{
  auto registry = makePluginlibRegistry();
  PluginContext context;
  PlanningPipeline pipeline;
  PipelineError error;
  ASSERT_TRUE(pipeline.build(PlanningSelection{}, *registry, context, error)) << error.message;
  auto planner = pipeline.makeMotionPlanner();
  ASSERT_TRUE(planner != nullptr);
  EXPECT_EQ(planner->cartesianPlanner().name(), "cartesian");
  EXPECT_EQ(planner->selectedPlanner(), "ompl");
}

TEST(MoveItPluginsTest, UnknownNameIsReportedWithTheLoadedNames)
{
  auto registry = makePluginlibRegistry();
  PluginContext context;
  PlanningSelection selection;
  selection.free_space = "no_such_planner";
  PlanningPipeline pipeline;
  PipelineError error;
  ASSERT_FALSE(pipeline.build(selection, *registry, context, error));
  EXPECT_EQ(error.code, "unknown_plugin");
  EXPECT_EQ(error.role, "free_space");
  EXPECT_NE(error.message.find("moveit_ompl"), std::string::npos);
}

}  // namespace
}  // namespace motion_planner
