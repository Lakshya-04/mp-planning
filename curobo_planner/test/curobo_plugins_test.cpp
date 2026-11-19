// The cuRobo plugins load by name and fail cleanly when the cuRobo node is not running. No GPU and
// no cuRobo node are involved: the service calls simply time out.
#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "motion_planner/ros/planning_config.hpp"

namespace mp = motion_planner;

TEST(CuroboPlugins, LoadByNameAndReportUnavailableWithoutTheNode)
{
  rclcpp::init(0, nullptr);
  {
    auto registry = mp::makePluginlibRegistry();
    mp::PluginStatus status;
    auto planner = registry->createPlanner("curobo", status);
    ASSERT_TRUE(planner != nullptr) << status.message;
    EXPECT_NE(registry->createIk("curobo_ik", status), nullptr);
    EXPECT_NE(registry->createCollision("curobo_collision", status), nullptr);
    EXPECT_NE(registry->createScene("curobo_scene", status), nullptr);

    // No node in the context: a structured error, nothing created behind the caller's back.
    mp::PluginContext without_node;
    const auto missing = planner->initialize(without_node, {});
    EXPECT_FALSE(missing.ok);
    EXPECT_EQ(missing.code, "missing_dependency");

    // With a node but no cuRobo node behind it: plugins initialize, calls report "unavailable".
    mp::PluginContext context;
    context.set<rclcpp::Node>("node", std::make_shared<rclcpp::Node>("curobo_plugin_test"));
    const mp::PluginParams params = {{"wait_for_service_sec", "0.1"}, {"timeout_sec", "0.2"}};
    auto planner2 = registry->createPlanner("curobo", status);
    ASSERT_TRUE(planner2->initialize(context, params).ok);
    EXPECT_EQ(planner2->name(), "curobo");
    mp::PlanningRequest request;
    request.start_joint_state = {0, 0, 0, 0, 0, 0};
    request.target_joint_state = {0.1, 0, 0, 0, 0, 0};
    const auto result = planner2->plan(request);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.failure_reason_code, "unavailable");

    auto ik = registry->createIk("curobo_ik", status);
    ASSERT_TRUE(ik->initialize(context, params).ok);  // reuses the same client
    mp::IkRequest ik_request;
    ik_request.target_pose = {0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0};
    EXPECT_FALSE(ik->ik(ik_request).success);
  }
  rclcpp::shutdown();
}

TEST(CuroboPlugins, BaseFrameParamRejectsAWorldInAnotherFrameThroughThePluginPath)
{
  rclcpp::init(0, nullptr);
  {
    auto registry = mp::makePluginlibRegistry();
    mp::PluginStatus status;
    mp::PluginContext context;
    context.set<rclcpp::Node>("node", std::make_shared<rclcpp::Node>("curobo_plugin_base_frame_test"));
    // The planner plugin initialises first and builds the shared client without a base_frame; the
    // scene plugin's base_frame must still take effect.
    auto planner = registry->createPlanner("curobo", status);
    ASSERT_TRUE(planner->initialize(context, {{"wait_for_service_sec", "0.1"}, {"timeout_sec", "0.2"}}).ok);
    auto scene = registry->createScene("curobo_scene", status);
    ASSERT_TRUE(scene != nullptr) << status.message;
    ASSERT_TRUE(scene->initialize(
        context, {{"wait_for_service_sec", "0.1"}, {"timeout_sec", "0.2"}, {"base_frame", "base"}}).ok);
    mp::WorldGeometryUpdate world;
    world.frame_id = "map";
    std::string error;
    EXPECT_FALSE(scene->updateWorldGeometry(world, error));
    // rejected for its frame, before any service call (there is no node to answer one)
    EXPECT_NE(error.find("differs from the robot base frame"), std::string::npos) << error;
  }
  rclcpp::shutdown();
}

