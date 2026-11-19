// The VAMP plugins load by name; a robot that is not compiled in is reported as "unavailable".
#include <gtest/gtest.h>

#include "motion_planner/ros/planning_config.hpp"

namespace mp = motion_planner;

TEST(VampPlugins, LoadByNameAndInitializeWithACompiledRobot)
{
  auto registry = mp::makePluginlibRegistry();
  mp::PluginStatus status;
  auto planner = registry->createPlanner("vamp", status);
  ASSERT_TRUE(planner != nullptr) << status.message;
  EXPECT_NE(registry->createCollision("vamp_collision", status), nullptr);
  EXPECT_NE(registry->createScene("vamp_scene", status), nullptr);

  mp::PluginContext context;
  const auto init = planner->initialize(context, {{"robot", "ur5"}});
  ASSERT_TRUE(init.ok) << init.message;
  EXPECT_EQ(planner->name(), "vamp");
  mp::IkRequest ik;
  EXPECT_EQ(planner->ik(ik).failure_reason_code, "unsupported");  // VAMP has no IK
}

TEST(VampPlugins, RobotThatIsNotCompiledInIsUnavailable)
{
  auto registry = mp::makePluginlibRegistry();
  mp::PluginStatus status;
  auto planner = registry->createPlanner("vamp", status);
  ASSERT_TRUE(planner != nullptr) << status.message;
  mp::PluginContext context;
  const auto init = planner->initialize(context, {{"robot", "no_such_robot"}});
  EXPECT_FALSE(init.ok);
  EXPECT_EQ(init.code, "unavailable");
  EXPECT_NE(init.message.find("no_such_robot"), std::string::npos);
}

TEST(VampPlugins, BaseFrameParamRejectsAWorldInAnotherFrameThroughThePluginPath)
{
  auto registry = mp::makePluginlibRegistry();
  mp::PluginStatus status;
  mp::PluginContext context;
  auto planner = registry->createPlanner("vamp", status);      // builds the shared planner, no base_frame
  ASSERT_TRUE(planner->initialize(context, {{"robot", "ur5"}}).ok);
  auto scene = registry->createScene("vamp_scene", status);
  ASSERT_TRUE(scene != nullptr) << status.message;
  ASSERT_TRUE(scene->initialize(context, {{"robot", "ur5"}, {"base_frame", "base_link"}}).ok);
  mp::WorldGeometryUpdate world;
  world.frame_id = "map";
  std::string error;
  EXPECT_FALSE(scene->updateWorldGeometry(world, error));
  EXPECT_NE(error.find("differs from the planning base frame"), std::string::npos) << error;
  world.frame_id = "base_link";
  EXPECT_TRUE(scene->updateWorldGeometry(world, error)) << error;
}

