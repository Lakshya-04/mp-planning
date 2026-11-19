// In-process test: a real ReachabilityNode serving a real (small) map over real services.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "reachability/core/builder.hpp"
#include "reachability/core/io.hpp"
#include "reachability/core/stand_verifier.hpp"
#include "reachability/ros/reachability_node.hpp"
#include "support.hpp"

using namespace std::chrono_literals;
using reachability_msgs::srv::GetBaseCandidates;
using reachability_msgs::srv::IsReachable;
using reachability_msgs::srv::PlanLiftBands;
using reachability_msgs::srv::PlanStands;

namespace
{
std::string mapFile()
{
  static const std::string path = [] {
      const char * dir = std::getenv("TEST_TMPDIR");
      std::string p = std::string(dir ? dir : "/tmp") + "/reachability_node_test_" +
        std::to_string(::getpid()) + ".rm4d";
      reachability::BuildOptions o;
      o.samples = 1'000'000;
      o.threads = 4;
      reachability::saveMap(
        p, reachability::buildMap(reachability_test::genericArm(), reachability_test::coarse(), o));
      return p;
    }();
  return path;
}

struct Fixture
{
  explicit Fixture(const std::string & base_profile = "", const std::string & strategy = "greedy")
  {
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"map_file", mapFile()}, {"base_profile", base_profile}, {"stand_strategy", strategy}});
    node = std::make_shared<reachability::ReachabilityNode>(o);
    client_node = std::make_shared<rclcpp::Node>("client");
    exec.add_node(node);
    exec.add_node(client_node);
  }

  template<typename Srv>
  typename Srv::Response::SharedPtr call(typename Srv::Request::SharedPtr req)
  {
    auto client = client_node->create_client<Srv>("/reachability/" + name<Srv>());
    EXPECT_TRUE(client->wait_for_service(5s));
    auto fut = client->async_send_request(req);
    EXPECT_EQ(exec.spin_until_future_complete(fut, 10s), rclcpp::FutureReturnCode::SUCCESS);
    return fut.get();
  }

  template<typename Srv>
  static std::string name();

  PlanStands::Response::SharedPtr callVerified(PlanStands::Request::SharedPtr req)
  {
    auto client = client_node->create_client<PlanStands>("/reachability/plan_verified_stands");
    EXPECT_TRUE(client->wait_for_service(5s));
    auto fut = client->async_send_request(req);
    EXPECT_EQ(exec.spin_until_future_complete(fut, 30s), rclcpp::FutureReturnCode::SUCCESS);
    return fut.get();
  }

  std::shared_ptr<reachability::ReachabilityNode> node;
  std::shared_ptr<rclcpp::Node> client_node;
  rclcpp::executors::SingleThreadedExecutor exec;
};

template<> std::string Fixture::name<IsReachable>() {return "is_reachable";}
template<> std::string Fixture::name<GetBaseCandidates>() {return "get_base_candidates";}
template<> std::string Fixture::name<PlanLiftBands>() {return "plan_lift_bands";}
template<> std::string Fixture::name<PlanStands>() {return "plan_stands";}


geometry_msgs::msg::Pose wallPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  const double h = M_PI / 4;  // +90 deg about y: tool z axis points along +x (into the wall)
  p.orientation.y = std::sin(h);
  p.orientation.w = std::cos(h);
  return p;
}
}  // namespace

TEST(ReachabilityNode, LoadsTheMap)
{
  Fixture f;
  EXPECT_TRUE(f.node->ready());
}

TEST(ReachabilityNode, ForwardQueryAnswersNearAndFar)
{
  Fixture f;
  auto near = std::make_shared<IsReachable::Request>();
  near->tool_in_arm_base = wallPose(0.6, 0.0, 0.3);
  EXPECT_TRUE(f.call<IsReachable>(near)->reachable);
  auto far = std::make_shared<IsReachable::Request>();
  far->tool_in_arm_base = wallPose(2.1, 0.0, 0.3);  // beyond the arm's reach
  EXPECT_FALSE(f.call<IsReachable>(far)->reachable);
}

TEST(ReachabilityNode, BaseCandidatesAreWithinReachAndIncludeTheNearSide)
{
  Fixture f;
  auto req = std::make_shared<GetBaseCandidates::Request>();
  req->tool_in_world = wallPose(3.0, 0.0, 1.0);
  req->arm_base_z = 0.8;
  const auto res = f.call<GetBaseCandidates>(req);
  ASSERT_TRUE(res->success) << res->message;
  // The map is side-agnostic (the wrist can fold back), so candidates exist on both sides of the
  // target plane; choosing the free side is the caller's job. All must be within reach.
  bool near_side = false;
  for (const auto & c : res->candidates) {
    EXPECT_LT(std::hypot(3.0 - c.x, c.y), 1.6);
    near_side = near_side || c.x < 3.0;
  }
  EXPECT_TRUE(near_side);
}

TEST(ReachabilityNode, LiftBandsCoverAWallOrReportWhatIsLeft)
{
  Fixture f;
  auto req = std::make_shared<PlanLiftBands::Request>();
  for (double z : {0.5, 1.0, 1.5, 2.0}) {
    req->targets_world.push_back(wallPose(0.9, 0.0, z));
  }
  req->base_x = 0.0;
  req->base_y = 0.0;
  req->lift_min = 0.0;
  req->lift_max = 1.5;
  req->step = 0.05;
  req->arm_mount_offset_z = 0.2;
  const auto res = f.call<PlanLiftBands>(req);
  EXPECT_TRUE(res->success) << res->message;
  EXPECT_FALSE(res->bands.empty());
  size_t covered = res->unreachable.size();
  for (const auto & b : res->bands) {
    covered += b.pose_indices.size();
  }
  EXPECT_EQ(covered, req->targets_world.size());

  req->step = 0.0;
  EXPECT_FALSE(f.call<PlanLiftBands>(req)->success);  // invalid request is rejected, not served
}

namespace
{
std::shared_ptr<PlanStands::Request> wideWall()
{
  auto req = std::make_shared<PlanStands::Request>();
  for (double y : {-3.0, -1.5, 0.0, 1.5, 3.0}) {
    for (double z : {0.8, 1.4}) {
      req->targets_world.push_back(wallPose(3.0, y, z));
    }
  }
  req->wall_point.x = 3.0;
  req->wall_normal.x = -1.0;
  req->grid_step = 0.2;
  req->lateral_margin = 0.2;
  req->lift_max = 1.5;
  req->arm_mount_offset_z = 0.2;
  return req;
}

std::string baseProfile(bool strafe)
{
  const char * dir = std::getenv("TEST_TMPDIR");
  const std::string path = std::string(dir ? dir : "/tmp") + "/reachability_stands_base_" + std::to_string(::getpid()) +
    (strafe ? "_omni.yaml" : "_diff.yaml");
  std::ofstream(path) << "base:\n  name: t\n  drive_type: " << (strafe ? "omni" : "diff")
                      << "\n  can_strafe: " << (strafe ? "true" : "false")
                      << "\n  footprint: [[-0.4, -0.25], [0.4, -0.25], [0.4, 0.25], [-0.4, 0.25]]\n";
  return path;
}
}  // namespace

TEST(ReachabilityNode, PlanStandsCoversAWideWallAndAssignsEveryTargetOnce)
{
  Fixture f;
  const auto res = f.call<PlanStands>(wideWall());
  EXPECT_FALSE(f.node->hasBase());
  ASSERT_GE(res->stands.size(), 2U) << res->message;
  std::multiset<uint32_t> seen(res->unreachable.begin(), res->unreachable.end());
  seen.insert(res->not_planned.begin(), res->not_planned.end());
  for (const auto & s : res->stands) {seen.insert(s.target_indices.begin(), s.target_indices.end());}
  EXPECT_EQ(seen.size(), 10U);
  for (uint32_t i = 0; i < 10; ++i) {EXPECT_EQ(seen.count(i), 1U) << i;}
  EXPECT_EQ(res->total_travel_s, 0.0);   // no base profile: no travel time
  for (const auto & s : res->stands) {    // lift band indices refer to the request's targets
    const std::set<uint32_t> mine(s.target_indices.begin(), s.target_indices.end());
    for (const auto & b : s.bands) {
      for (const auto k : b.pose_indices) {EXPECT_EQ(mine.count(k), 1U);}
    }
  }
}

TEST(ReachabilityNode, PlanStandsSetCoverStrategyCoversTheWallWithNoMoreStandsThanGreedy)
{
  Fixture greedy;
  Fixture cover("", "set_cover");
  const auto g = greedy.call<PlanStands>(wideWall());
  const auto c = cover.call<PlanStands>(wideWall());
  EXPECT_TRUE(c->success) << c->message;
  EXPECT_LE(c->stands.size(), g->stands.size());
  std::multiset<uint32_t> seen;
  for (const auto & s : c->stands) {seen.insert(s.target_indices.begin(), s.target_indices.end());}
  EXPECT_EQ(seen.size(), 10U);
}

TEST(ReachabilityNode, PlanStandsReportsWhatTheStandCapLeftUnplanned)
{
  Fixture f;
  auto req = wideWall();
  req->max_stands = 1;
  const auto res = f.call<PlanStands>(req);
  ASSERT_EQ(res->stands.size(), 1U) << res->message;
  EXPECT_FALSE(res->success);   // a capped plan never claims full coverage
  EXPECT_FALSE(res->not_planned.empty());
  EXPECT_NE(res->message.find("not planned"), std::string::npos) << res->message;
  std::multiset<uint32_t> seen(res->unreachable.begin(), res->unreachable.end());
  seen.insert(res->not_planned.begin(), res->not_planned.end());
  for (const auto & s : res->stands) {seen.insert(s.target_indices.begin(), s.target_indices.end());}
  EXPECT_EQ(seen.size(), 10U);   // every target is placed or reported exactly once
}

TEST(ReachabilityNode, PlanStandsUsesTheBaseProfile)
{
  const std::string profile = baseProfile(false);
  Fixture f(profile);
  std::remove(profile.c_str());      // the node has read it
  ASSERT_TRUE(f.node->hasBase());
  auto req = wideWall();
  req->has_start = true;
  req->start_x = 1.0;
  req->start_y = -3.0;
  const auto res = f.call<PlanStands>(req);
  ASSERT_GE(res->stands.size(), 2U) << res->message;
  EXPECT_GT(res->total_travel_s, 0.0);
  for (const auto & s : res->stands) {EXPECT_GE(s.footprint_clearance, 0.3 - 1e-9);}
}

TEST(ReachabilityNode, PlanStandsRejectsABadRequest)
{
  Fixture f;
  auto req = wideWall();
  req->wall_normal.x = 0.0;
  EXPECT_FALSE(f.call<PlanStands>(req)->success);
  EXPECT_FALSE(f.call<PlanStands>(std::make_shared<PlanStands::Request>())->success);
}

TEST(ReachabilityNode, UnloadableBaseProfileLeavesTheNodeNotReady)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides({{"map_file", mapFile()}, {"base_profile", std::string("/nonexistent/base.yaml")}});
  reachability::ReachabilityNode node(o);
  EXPECT_FALSE(node.ready());   // would otherwise plan stands without the footprint check
}
TEST(ReachabilityNode, PlanStandsRejectsStepsThatWouldRunForever)
{
  Fixture f;
  auto req = wideWall();
  req->grid_step = 0.01;
  auto res = f.call<PlanStands>(req);
  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("grid_step"), std::string::npos) << res->message;

  req = wideWall();
  req->lift_step = 0.001;
  res = f.call<PlanStands>(req);
  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("lift_step"), std::string::npos) << res->message;

  req = wideWall();
  req->max_standoff = 1e6;      // huge search area
  res = f.call<PlanStands>(req);
  EXPECT_FALSE(res->success);
  EXPECT_TRUE(res->stands.empty());
}
TEST(ReachabilityNode, PlanStandsRejectsAnEmptyLiftRange)
{
  Fixture f;
  auto req = wideWall();
  req->lift_min = 1.8;
  req->lift_max = 0.0;          // default 1.5 is below lift_min
  const auto res = f.call<PlanStands>(req);
  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("lift_max"), std::string::npos) << res->message;
}
TEST(ReachabilityNode, MissingMapFileLeavesTheNodeNotReady)
{
  rclcpp::NodeOptions o;
  o.parameter_overrides({{"map_file", std::string("/nonexistent.rm4d")}});
  reachability::ReachabilityNode node(o);
  EXPECT_FALSE(node.ready());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}

TEST(ReachabilityNode, VerifiedStandsNeedAVerifier)
{
  Fixture f;
  const auto res = f.callVerified(wideWall());
  EXPECT_FALSE(res->success);
  EXPECT_TRUE(res->stands.empty());
  EXPECT_NE(res->message.find("no stand verifier"), std::string::npos) << res->message;
}

TEST(ReachabilityNode, VerifiedStandsWithAnAcceptingVerifierCoverTheWallLikePlanStands)
{
  Fixture f;
  f.node->setVerifier(std::make_shared<reachability::FunctionStandVerifier>(
      [](const reachability::Stand &, double, const reachability::Pose &) {
        reachability::TargetVerdict v;
        v.reason = reachability::VerifyReason::kOk;
        return v;
      }));
  const auto res = f.callVerified(wideWall());
  ASSERT_GE(res->stands.size(), 2U) << res->message;
  EXPECT_TRUE(res->success) << res->message;
  std::multiset<uint32_t> seen(res->unreachable.begin(), res->unreachable.end());
  for (const auto & s : res->stands) {seen.insert(s.target_indices.begin(), s.target_indices.end());}
  for (uint32_t i = 0; i < 10; ++i) {EXPECT_EQ(seen.count(i), 1U) << i;}
  EXPECT_NE(res->message.find("verified:"), std::string::npos) << res->message;
}

TEST(ReachabilityNode, VerifiedStandsReportTargetsTheVerifierRefusedEverywhere)
{
  Fixture f;
  // the two y = -3 targets are refused from every stand (IK fails), the rest are fine
  f.node->setVerifier(std::make_shared<reachability::FunctionStandVerifier>(
      [](const reachability::Stand &, double, const reachability::Pose & t) {
        reachability::TargetVerdict v;
        const bool first = t.translation().y() < -2.5;   // the two y = -3 targets of wideWall()
        v.reason = first ? reachability::VerifyReason::kIkFailed : reachability::VerifyReason::kOk;
        return v;
      }));
  const auto res = f.callVerified(wideWall());
  EXPECT_FALSE(res->success);
  EXPECT_FALSE(res->unreachable.empty()) << res->message;
  EXPECT_FALSE(res->stands.empty());
}
