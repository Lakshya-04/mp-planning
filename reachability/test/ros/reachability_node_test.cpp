// In-process test: a real ReachabilityNode serving a real (small) map over real services.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "reachability/core/builder.hpp"
#include "reachability/core/io.hpp"
#include "reachability/ros/reachability_node.hpp"
#include "support.hpp"

using namespace std::chrono_literals;
using reachability_msgs::srv::GetBaseCandidates;
using reachability_msgs::srv::IsReachable;
using reachability_msgs::srv::PlanLiftBands;

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
  Fixture()
  {
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"map_file", mapFile()}});
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

  std::shared_ptr<reachability::ReachabilityNode> node;
  std::shared_ptr<rclcpp::Node> client_node;
  rclcpp::executors::SingleThreadedExecutor exec;
};

template<> std::string Fixture::name<IsReachable>() {return "is_reachable";}
template<> std::string Fixture::name<GetBaseCandidates>() {return "get_base_candidates";}
template<> std::string Fixture::name<PlanLiftBands>() {return "plan_lift_bands";}

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
