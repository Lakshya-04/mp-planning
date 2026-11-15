#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "core_types/trajectory.hpp"
#include "motion_planner/core/planning_pipeline.hpp"
#include "motion_planner/ros/planning_config.hpp"

namespace motion_planner
{
namespace
{

/// Fake planner plugin: records calls in a shared counter, succeeds or fails on demand.
struct Calls
{
  int cartesian{0};
  int free_space{0};
};

class FakePlanner : public PlannerPlugin
{
public:
  FakePlanner(std::string name, int * counter, bool succeeds, bool available = true)
  : name_(std::move(name)), counter_(counter), succeeds_(succeeds), available_(available) {}

  PluginStatus initialize(PluginContext & context, const PluginParams & params) override
  {
    if (!available_) {
      return PluginStatus::fail("unavailable", "fake backend is not running");
    }
    seen_param = paramOr(params, "tag", std::string{});
    saw_ik_port = context.get<IkPort>("ik_port") != nullptr;
    return PluginStatus::success();
  }
  std::string name() const override {return name_;}
  PlanningResult plan(const PlanningRequest & request) override
  {
    ++*counter_;
    PlanningResult r;
    r.success = succeeds_;
    r.path_fraction = succeeds_ ? 1.0 : 0.0;
    r.failure_reason_code = succeeds_ ? "none" : "fake_failure";
    if (succeeds_) {
      core_types::TrajectoryPoint pt;
      pt.positions = {1.0};
      r.trajectory.points = {pt};
      r.waypoints_completed = request.waypoint_indices.size();
    }
    return r;
  }
  IkResult ik(const IkRequest &) override {return IkResult{};}
  CollisionResult collision(const CollisionRequest &) override {return CollisionResult{};}

  std::string seen_param;
  bool saw_ik_port{false};

private:
  std::string name_;
  int * counter_;
  bool succeeds_;
  bool available_;
};

class FakeIk : public IkPlugin
{
public:
  PluginStatus initialize(PluginContext &, const PluginParams &) override
  {
    return PluginStatus::success();
  }
  IkResult ik(const IkRequest &) override {return IkResult{};}
};

class FakeCollision : public CollisionPlugin
{
public:
  PluginStatus initialize(PluginContext &, const PluginParams &) override
  {
    return PluginStatus::success();
  }
  CollisionResult collision(const CollisionRequest &) override {return CollisionResult{};}
};

struct Fixture
{
  Fixture()
  {
    registry.addIk("fake_ik", [] {return std::make_unique<FakeIk>();});
    registry.addCollision("fake_collision", [] {return std::make_unique<FakeCollision>();});
    registry.addPlanner(
      "fake_cartesian", [this] {return std::make_unique<FakePlanner>("cart", &calls.cartesian, true);});
    registry.addPlanner(
      "fake_cartesian_failing",
      [this] {return std::make_unique<FakePlanner>("cart", &calls.cartesian, false);});
    registry.addPlanner(
      "fake_free", [this] {return std::make_unique<FakePlanner>("free", &calls.free_space, true);});
    registry.addPlanner(
      "fake_down", [this] {return std::make_unique<FakePlanner>("down", &calls.free_space, true, false);});
    selection.ik = "fake_ik";
    selection.collision = "fake_collision";
    selection.cartesian = "fake_cartesian";
    selection.free_space = "fake_free";
  }

  FactoryRegistry registry;
  PluginContext context;
  Calls calls;
  PlanningSelection selection;
};

PlanningRequest request(bool contact)
{
  PlanningRequest r;
  r.waypoint_indices = {0};
  r.min_path_fraction = 0.5;
  r.allow_target_contact = contact;
  return r;
}

TEST(PlanningPipelineTest, SelectsThePluginsNamedInTheConfig)
{
  Fixture f;
  f.selection.plugin_params["fake_free"] = {{"tag", "hello"}};
  PlanningPipeline pipeline;
  PipelineError error;
  ASSERT_TRUE(pipeline.build(f.selection, f.registry, f.context, error)) << error.message;
  EXPECT_TRUE(pipeline.ik() != nullptr);
  EXPECT_TRUE(pipeline.collision() != nullptr);
  EXPECT_TRUE(pipeline.scene() == nullptr);
  auto planner = pipeline.makeMotionPlanner();
  ASSERT_TRUE(planner != nullptr);
  EXPECT_EQ(planner->cartesianPlanner().name(), "cart");
  EXPECT_TRUE(pipeline.makeMotionPlanner() == nullptr);  // planners were handed over
  // A cartesian-routed segment goes to the cartesian planner only.
  SegmentRoutingSpec routing;
  routing.mode_hint = "cartesian";
  const auto result = planner->planSegment(request(false), routing);
  EXPECT_TRUE(result.planning_result.success);
  EXPECT_EQ(f.calls.cartesian, 1);
  EXPECT_EQ(f.calls.free_space, 0);
}

TEST(PlanningPipelineTest, PlannersSeeTheSelectedIkAndTheirOwnParams)
{
  Fixture f;
  PlanningPipeline pipeline;
  PipelineError error;
  ASSERT_TRUE(pipeline.build(f.selection, f.registry, f.context, error));
  EXPECT_TRUE(f.context.get<IkPort>("ik_port") != nullptr);
  EXPECT_TRUE(f.context.get<CollisionPort>("collision_port") != nullptr);
}

TEST(PlanningPipelineTest, PerSegmentOverrideBeatsTheSegmentHint)
{
  Fixture f;
  f.selection.cartesian = "fake_cartesian_failing";
  f.selection.segments = {{"contact", "cartesian"}, {"transition", "free_space"}};
  PlanningPipeline pipeline;
  PipelineError error;
  ASSERT_TRUE(pipeline.build(f.selection, f.registry, f.context, error)) << error.message;
  auto planner = pipeline.makeMotionPlanner();
  ASSERT_TRUE(planner != nullptr);

  // A contact segment hinted "fsp" is forced to cartesian: no free-space call even though the
  // cartesian planner fails (contact segments never fall back to free space).
  SegmentRoutingSpec fsp_hint;
  fsp_hint.mode_hint = "fsp";
  const auto contact = planner->planSegment(request(true), fsp_hint);
  EXPECT_EQ(contact.mode_used, "cartesian");
  EXPECT_EQ(f.calls.free_space, 0);

  // A transition hinted "cartesian" is forced to free space: cartesian is tried first, fails,
  // and the free-space planner produces the result.
  SegmentRoutingSpec cart_hint;
  cart_hint.mode_hint = "cartesian";
  const auto transition = planner->planSegment(request(false), cart_hint);
  EXPECT_EQ(transition.mode_used, "fsp");
  EXPECT_TRUE(transition.planning_result.success);
  EXPECT_EQ(f.calls.free_space, 1);
}

TEST(PlanningPipelineTest, UnknownPluginIsAStructuredError)
{
  Fixture f;
  f.selection.free_space = "no_such_planner";
  PlanningPipeline pipeline;
  PipelineError error;
  EXPECT_FALSE(pipeline.build(f.selection, f.registry, f.context, error));
  EXPECT_EQ(error.code, "unknown_plugin");
  EXPECT_EQ(error.role, "free_space");
  EXPECT_EQ(error.plugin, "no_such_planner");
  EXPECT_NE(error.message.find("fake_free"), std::string::npos);  // lists what is available
  EXPECT_TRUE(pipeline.makeMotionPlanner() == nullptr);
}

TEST(PlanningPipelineTest, UnavailableBackendIsReportedWithItsRole)
{
  Fixture f;
  f.selection.free_space = "fake_down";
  PlanningPipeline pipeline;
  PipelineError error;
  EXPECT_FALSE(pipeline.build(f.selection, f.registry, f.context, error));
  EXPECT_EQ(error.code, "unavailable");
  EXPECT_EQ(error.role, "free_space");
}

TEST(PlanningPipelineTest, BadSegmentOverrideIsRejected)
{
  Fixture f;
  f.selection.segments = {{"contact", "vamp"}};
  PlanningPipeline pipeline;
  PipelineError error;
  EXPECT_FALSE(pipeline.build(f.selection, f.registry, f.context, error));
  EXPECT_EQ(error.code, "invalid_selection");
}

TEST(PlanningConfigTest, ParsesTheDocumentedSchemaAndKeepsDefaults)
{
  PlanningSelection s;
  std::string error;
  ASSERT_TRUE(parsePlanningSelection(
      "planning:\n  free_space: vamp\n  segments: {contact: cartesian}\n"
      "  plugin_params:\n    vamp: {robot: ur5}\n", s, error)) << error;
  EXPECT_EQ(s.free_space, "vamp");
  EXPECT_EQ(s.cartesian, "moveit_cartesian");
  EXPECT_EQ(s.ik, "moveit_ik");
  EXPECT_EQ(s.segments.at("contact"), "cartesian");
  EXPECT_EQ(s.plugin_params.at("vamp").at("robot"), "ur5");
  // Also accepted under pipeline:, as in the architecture example.
  ASSERT_TRUE(parsePlanningSelection("pipeline:\n  planning:\n    ik: curobo_ik\n", s, error));
  EXPECT_EQ(s.ik, "curobo_ik");
}

TEST(PlanningConfigTest, RejectsUnknownKeysAndMissingMap)
{
  PlanningSelection s;
  std::string error;
  EXPECT_FALSE(parsePlanningSelection("planning:\n  free_spcae: vamp\n", s, error));
  EXPECT_NE(error.find("free_spcae"), std::string::npos);
  EXPECT_FALSE(parsePlanningSelection("other: 1\n", s, error));
}

}  // namespace
}  // namespace motion_planner
